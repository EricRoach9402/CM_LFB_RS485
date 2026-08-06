#include "cmos.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <netinet/tcp.h>
#include <signal.h>

#define CM_RX_BUF        2048 // 接收緩衝區大小，必須足夠大以容納 publisher 發來的訊息，否則可能會被截斷
#define CM_LINE_BUF      1024 // 單行命令緩衝區大小，從 rxbuf 切出的單行最大長度
#define CM_MAX_PUB_CONN  256 // 訂閱者最多同時連線的 publisher 數量，超過就拒絕新連線
#define CM_MAX_SUB_SLOTS  256 // 訂閱槽的最大數量，每個訂閱槽對應一組過濾條件和 callback，當收到訊息時會檢查每個槽的過濾條件，如果符合就呼叫對應的 callback

/* -------------------------------------------------------
 * 訂閱槽：每個 topic 訂閱有自己的過濾條件與 callback
 * ------------------------------------------------------- */
typedef struct {
    char              topic[64];
    char              filter_state[64]; /* 空字串 = 不過濾 */
    char              filter_type [64]; /* 空字串 = 不過濾 */
    char              filter_key  [64]; /* 空字串 = 不過濾 */
    cmos_callback_t cb;
    int               used;
} cm_sub_slot_t;

/* -------------------------------------------------------
 * 內部 per-connection 狀態
 * ------------------------------------------------------- */
typedef struct {
    int  sock; // socket descriptor 與 publisher 的連線 socket fd
    char node_name[64]; // 對應 publisher 的 node_name，從 master 那邊註冊來的
    char rxbuf[CM_RX_BUF]; // 接收緩衝區
    int  rxlen; // 接收緩衝區的長度
    int  active; // 是否有效連線
    int  sub_slot;   /* 對應 ctx->sub_slots[] 的索引 */
} cm_pub_conn_t;

/* -------------------------------------------------------
 * Context 結構
 * ------------------------------------------------------- */
struct cmos_sub_ctx {
    int             master_sock; // 連接到 master 的 socket fd
    char            master_ip[64]; // 連接到 master 的 IP 位址，通常是 "
    int             master_port; // 連接到 master 的 port 號，通常是 5555
    char            node_name[64]; // 訂閱者的節點名稱，註冊到 master 後就不會變了
    cm_sub_slot_t   sub_slots[CM_MAX_SUB_SLOTS]; // 訂閱槽陣列，每個槽對應一組過濾條件和 callback，當收到訊息時會檢查每個槽的過濾條件，如果符合就呼叫對應的 callback
    int             sub_slot_count; // 已使用的訂閱槽數量，sub_slots[0..sub_slot_count-1] 是有效的訂閱槽
    cm_pub_conn_t   pub_conns[CM_MAX_PUB_CONN]; // 與 publisher 的連線陣列，每個連線包含 socket fd、對應的 publisher node_name、接收緩衝區和長度、是否有效，以及對應的訂閱槽索引 sub_slot
    int             pub_count;// 與 publisher 的有效連線數量，pub_conns[0..pub_count-1] 是有效的連線
    int             epfd;     // epoll fd，監聽所有 pub 連線是否有資料進來
};

/* -------------------------------------------------------
 * 內部 helper
 * ------------------------------------------------------- */
static void init_sigpipe_once(void) // 忽略 SIGPIPE，確保只設置一次，保護層
{
    static int done = 0;
    if (!done) { signal(SIGPIPE, SIG_IGN); done = 1; }
}

static int connect_tcp(const char *ip, int port) // 連線到 TCP 伺服器，成功回傳 socket fd，失敗回傳 -1
{
    struct sockaddr_in addr;
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (inet_pton(AF_INET, ip, &addr.sin_addr) <= 0) { close(sock); return -1; }
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) { close(sock); return -1; }
    return sock;
}

static int send_line(int sock, const char *line) // 發送一行文字給 socket，使用 send()，如果系統支援 MSG_NOSIGNAL 就加上這個 flag 以避免 SIGPIPE，否則就正常發送。成功回傳發送的字節數，失敗回傳 -1
{
#ifdef MSG_NOSIGNAL
    return send(sock, line, strlen(line), MSG_NOSIGNAL);
#else
    return send(sock, line, strlen(line), 0);
#endif
}

/* 設定 master socket 的接收超時，避免 ctx_lookup_slot 的 recv 永久阻塞 */
static void set_recv_timeout(int sock, int seconds)
{
    struct timeval tv;
    tv.tv_sec  = seconds;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

/* -------------------------------------------------------
 * Connection 管理
 * ------------------------------------------------------- */
static void ctx_init_conns(struct cmos_sub_ctx *ctx)  // 初始化 context 中的連線陣列，將所有連線的 socket 設為 -1，node_name 和 rxbuf 清空，rxlen 設為 0，active 設為 0，sub_slot 設為 -1，pub_count 設為 0
{
    for (int i = 0; i < CM_MAX_PUB_CONN; i++) {
        ctx->pub_conns[i].sock         = -1; // -1 代表沒有連線
        ctx->pub_conns[i].node_name[0] = '\0'; // 清空 node_name 字串
        ctx->pub_conns[i].rxbuf[0]     = '\0'; // 清空 rxbuf 字串
        ctx->pub_conns[i].rxlen        = 0; // 接收緩衝區長度設為 0
        ctx->pub_conns[i].active       = 0; // 連線不活躍
        ctx->pub_conns[i].sub_slot     = -1; // 沒有對應的訂閱槽
    }
    ctx->pub_count = 0; // 有效連線數量設為 0
}

static void ctx_close_conn(struct cmos_sub_ctx *ctx, int idx) // 關閉 context 中索引為 idx 的連線，如果 idx 不合法或連線不活躍就直接返回，否則關閉 socket，將 active 設為 0，清空 node_name 和 rxbuf，rxlen 設為 0，sub_slot 設為 -1，pub_count 減 1
{
    if (idx < 0 || idx >= CM_MAX_PUB_CONN) return; // idx 不合法
    if (ctx->pub_conns[idx].active) ctx->pub_count--; // 如果連線活躍，pub_count 減 1
    if (ctx->pub_conns[idx].sock >= 0) {
        if (ctx->epfd >= 0)
            epoll_ctl(ctx->epfd, EPOLL_CTL_DEL, ctx->pub_conns[idx].sock, NULL);
        close(ctx->pub_conns[idx].sock); // 關閉 socket
    }
    ctx->pub_conns[idx].sock         = -1; // 將 socket 設為 -1 代表沒有連線
    ctx->pub_conns[idx].node_name[0] = '\0'; // 清空 node_name 字串
    ctx->pub_conns[idx].rxbuf[0]     = '\0'; // 清空 rxbuf 字串
    ctx->pub_conns[idx].rxlen        = 0;  // 接收緩衝區長度設為 0
    ctx->pub_conns[idx].active       = 0; // 連線不活躍
    ctx->pub_conns[idx].sub_slot     = -1; // 沒有對應的訂閱槽
}

static void ctx_close_all_conns(struct cmos_sub_ctx *ctx) // 關閉 context 中的所有連線，呼叫 ctx_close_conn() 關閉每一個連線
{
    for (int i = 0; i < CM_MAX_PUB_CONN; i++) ctx_close_conn(ctx, i);
}

/* 找同一 (node_name, sub_slot) 的連線，避免重複連接 */
static int ctx_find_conn(struct cmos_sub_ctx *ctx, const char *node, int slot) // 在 context 的連線陣列中尋找對應 node_name 和 sub_slot 的連線，如果找到就回傳索引，否則回傳 -1
{
    for (int i = 0; i < CM_MAX_PUB_CONN; i++)
        if (ctx->pub_conns[i].active &&
            ctx->pub_conns[i].sub_slot == slot &&
            strcmp(ctx->pub_conns[i].node_name, node) == 0)
            return i;
    return -1;
}

static int ctx_add_pub(struct cmos_sub_ctx *ctx, int sub_slot,
                       const char *node_name, const char *ip, int port) // 在 context 中新增一個與 publisher 的連線，對應到訂閱槽 sub_slot，publisher 的 node_name 是 node_name，IP 是 ip，port 是 port，如果已經有同一個 (node_name, sub_slot) 的連線就不重複新增，成功回傳 0，失敗回傳 -1（例如沒有空位或連線失敗）
{
    if (!node_name || !ip) return -1;
    if (ctx_find_conn(ctx, node_name, sub_slot) >= 0) return 0;  /* 已存在 */ // 如果已經有同一個 (node_name, sub_slot) 的連線就不重複新增，直接回傳 0 表示成功

    int slot = -1;
    for (int i = 0; i < CM_MAX_PUB_CONN; i++)
        if (!ctx->pub_conns[i].active) { slot = i; break; }
    if (slot < 0) return -1;

    int sock = connect_tcp(ip, port);
    if (sock < 0) return -1;

    int ka = 1, idle = 10, interval = 5, count = 3; // 設置 TCP keepalive，確保 publisher 斷線時能夠被偵測到並清理連線，避免 pub_conns 中充斥著已斷線但還沒被清理的連線，導致 pub_count 不準確和資源浪費 
    /*
    ka = 1 → 啟用 keepalive 功能。不開的話後面三個參數設了也沒用。
    idle = 10 → 連線閒置 10 秒後，系統開始發送探測封包。
    interval = 5 → 如果對方沒回應，每隔 5 秒再發一次。
    count = 3 → 連續 3 次沒回應，系統就認定對方死了，自動把連線關掉。
    */
    setsockopt(sock, SOL_SOCKET,  SO_KEEPALIVE,  &ka,       sizeof(ka));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE,  &idle,     sizeof(idle));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT,   &count,    sizeof(count));

    ctx->pub_conns[slot].sock     = sock;
    ctx->pub_conns[slot].sub_slot = sub_slot;
    strncpy(ctx->pub_conns[slot].node_name, node_name,
            sizeof(ctx->pub_conns[slot].node_name) - 1);
    ctx->pub_conns[slot].node_name[sizeof(ctx->pub_conns[slot].node_name) - 1] = '\0';
    ctx->pub_conns[slot].rxbuf[0] = '\0';
    ctx->pub_conns[slot].rxlen    = 0;
    ctx->pub_conns[slot].active   = 1;
    ctx->pub_count++;

    if (ctx->epfd >= 0) {
        struct epoll_event ev;
        ev.events   = EPOLLIN;
        ev.data.u32 = (uint32_t)slot; /* 用索引當識別，收到事件就知道是哪條連線 */
        epoll_ctl(ctx->epfd, EPOLL_CTL_ADD, sock, &ev);
    }
    return 0;
}

static int ctx_active_count(struct cmos_sub_ctx *ctx) // 回傳 context 中目前有多少條有效的 publisher 連線
{
    int n = 0;
    for (int i = 0; i < CM_MAX_PUB_CONN; i++)
        if (ctx->pub_conns[i].active && ctx->pub_conns[i].sock >= 0) n++;
    return n;
}

/* -------------------------------------------------------
 * Master 重連：重新連接並重新登錄所有 topic
 * ------------------------------------------------------- */
static int ctx_reconnect_master(struct cmos_sub_ctx *ctx) // 重新連接到 master，並重新註冊 node name 和訂閱的 topic，如果成功回傳 0，失敗回傳 -1
{
    if (ctx->master_sock >= 0) { close(ctx->master_sock); ctx->master_sock = -1; }
    ctx->master_sock = connect_tcp(ctx->master_ip, ctx->master_port);
    if (ctx->master_sock < 0) return -1;
    set_recv_timeout(ctx->master_sock, 3); /* 問題四修正：避免 recv 永久阻塞 */

    char buf[256]; // 重新註冊 node name 和訂閱的 topic，確保 master 知道這個訂閱者的存在和訂閱的 topic，讓 master 能夠正確地把 publisher 的資訊發給這個訂閱者
    snprintf(buf, sizeof(buf), "NODE %s\n", ctx->node_name);
    if (send_line(ctx->master_sock, buf) <= 0) {
        close(ctx->master_sock); ctx->master_sock = -1; return -1;
    }
    for (int i = 0; i < ctx->sub_slot_count; i++) {
        if (!ctx->sub_slots[i].used) continue;
        snprintf(buf, sizeof(buf), "REGISTER_SUB %s\n", ctx->sub_slots[i].topic);
        if (send_line(ctx->master_sock, buf) <= 0) {
            close(ctx->master_sock); ctx->master_sock = -1; return -1;
        }
    }
    printf("[SUB] reconnected to master\n");
    return 0;
}

/* -------------------------------------------------------
 * Lookup & 連線到 Publisher（per slot）
 * ------------------------------------------------------- */
static int ctx_lookup_slot(struct cmos_sub_ctx *ctx, int s)  // 對訂閱槽 s 執行 lookup，向 master 詢問這個 topic 的 publisher 資訊，然後對每個 publisher 執行 ctx_add_pub() 連線，如果成功找到至少一個 publisher 就回傳 0，否則回傳 -1
{
    char buf[256], rxbuf[CM_RX_BUF];
    int  rxlen = 0, found = 0;

    snprintf(buf, sizeof(buf), "LOOKUP_TOPIC %s\n", ctx->sub_slots[s].topic);
    if (send_line(ctx->master_sock, buf) <= 0) {
        close(ctx->master_sock); ctx->master_sock = -1; return -1;
    }

    while (1) {  // 從 master 那邊接收回應，直到收到 "END\n" 為止，回應的格式是 "PUBINFO topic ip port node\n"，例如 "PUBINFO test_topic
        int n = recv(ctx->master_sock, rxbuf + rxlen,
                     sizeof(rxbuf) - 1 - rxlen, 0);
        if (n <= 0) { close(ctx->master_sock); ctx->master_sock = -1; return -1; }
        rxlen += n;
        rxbuf[rxlen] = '\0';

        int start = 0;
        for (int i = 0; i < rxlen; i++) {
            if (rxbuf[i] != '\n') continue;
            char line[CM_LINE_BUF];
            int  ll = i - start + 1;
            if (ll >= (int)sizeof(line)) ll = (int)sizeof(line) - 1;
            memcpy(line, rxbuf + start, ll);
            line[ll] = '\0';

            if (strcmp(line, "END\n") == 0)
                return found ? 0 : -1;

            if (strncmp(line, "PUBINFO ", 8) == 0) {
                char *cmd  = strtok(line, " \r\n"); (void)cmd;
                char *rtop = strtok(NULL, " \r\n"); (void)rtop;
                char *ip   = strtok(NULL, " \r\n");
                char *port = strtok(NULL, " \r\n");
                char *node = strtok(NULL, " \r\n");
                if (ip && port && node &&
                    ctx_add_pub(ctx, s, node, ip, atoi(port)) == 0)
                    found = 1;
            }
            start = i + 1;
        }
        if (start > 0) { memmove(rxbuf, rxbuf + start, rxlen - start); rxlen -= start; }
        if (rxlen >= (int)sizeof(rxbuf) - 1) rxlen = 0;
    }
}


static void ctx_reconnect_all_pubs(struct cmos_sub_ctx *ctx) // 重新連接到 master 並對所有訂閱槽執行 lookup，確保所有訂閱的 publisher 都是連線狀態
{
    if (ctx->master_sock < 0 && ctx_reconnect_master(ctx) != 0) return;
    for (int i = 0; i < ctx->sub_slot_count; i++)
        if (ctx->sub_slots[i].used)
            ctx_lookup_slot(ctx, i);
}

/* -------------------------------------------------------
 * 訊息處理：解析 → 過濾 → 呼叫 callback
 * ------------------------------------------------------- */
static void ctx_handle_line(struct cmos_sub_ctx *ctx,
                             cm_pub_conn_t *conn, char *line) // 處理從 publisher 連線收到的一行訊息，解析出 topic 和 data，檢查是否符合訂閱槽的過濾條件，如果符合就呼叫對應的 callback
{
    char *cmd   = strtok(line, " \r\n");
    if (!cmd || strcmp(cmd, "MSG") != 0) return;
    char *topic = strtok(NULL, " \r\n");
    char *data  = strtok(NULL, "\r\n");
    if (!topic || !data) return;

    int s = conn->sub_slot;
    if (s < 0 || s >= ctx->sub_slot_count || !ctx->sub_slots[s].used) return;

    cm_sub_slot_t *slot = &ctx->sub_slots[s];
    if (strcmp(topic, slot->topic) != 0) return;
    if (!slot->cb) return;

    /* 無任何 filter：直接傳整個 data 字串（向下相容） */
    if (!slot->filter_state[0] && !slot->filter_type[0] && !slot->filter_key[0]) {
        slot->cb(topic, data);
        return;
    }

    /* 有 filter：解析 state=X type=Y key=Z value=W，對比後只傳 value */
    char tmp[1024]; // 複製 data 到 tmp 中，確保不會修改原始 data 字串，並且有足夠的空間解析出 state、type、key 和 value
    strncpy(tmp, data, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';
    char *p_state = NULL, *p_type = NULL, *p_key = NULL, *p_value = NULL;
    char *t = strtok(tmp, " ");
    while (t) {
        if      (strncmp(t, "state=", 6) == 0) p_state = t + 6;
        else if (strncmp(t, "type=",  5) == 0) p_type  = t + 5;
        else if (strncmp(t, "key=",   4) == 0) p_key   = t + 4;
        else if (strncmp(t, "value=", 6) == 0) p_value = t + 6;
        t = strtok(NULL, " ");
    }
    if (slot->filter_state[0] && (!p_state || strcmp(p_state, slot->filter_state) != 0)) return;
    if (slot->filter_type[0]  && (!p_type  || strcmp(p_type,  slot->filter_type)  != 0)) return;
    if (slot->filter_key[0]   && (!p_key   || strcmp(p_key,   slot->filter_key)   != 0)) return;

    /*
     * filter_key 有指定 → key 已確定，cb 只收 value 字串
     * filter_key 是 NULL → key 不限，cb 收完整 data（讓 cb 自行 parse key）
     */
    if (slot->filter_key[0])
        slot->cb(topic, p_value ? p_value : "");
    else
        slot->cb(topic, data);
}

/* -------------------------------------------------------
 * 公開 Context API
 * ------------------------------------------------------- */
cmos_sub_ctx_t *cmos_sub_create(const char *master_ip, int master_port,
                                     const char *node_name) // 建立訂閱者 context，連接到 master_ip:master_port，註冊 node_name，成功回傳 context 指標，失敗回傳 NULL
{
    init_sigpipe_once(); // 確保 SIGPIPE 被忽略，保護層

    cmos_sub_ctx_t *ctx = calloc(1, sizeof(cmos_sub_ctx_t)); // 分配並初始化 context 結構，master_sock 設為 -1 代表尚未連線
    if (!ctx) return NULL; // 分配失敗

    ctx->epfd = -1; /* 先設 -1，避免 calloc 零初始化誤把 fd=0（stdin）當有效 */
    ctx_init_conns(ctx); // 初始化 context 中的連線陣列，確保所有連線的狀態都是初始值

    ctx->epfd = epoll_create1(0);
    if (ctx->epfd < 0) { free(ctx); return NULL; }
    strncpy(ctx->master_ip,  master_ip,  sizeof(ctx->master_ip)  - 1); //   將 master_ip 複製到 context 中，確保字串結尾
    strncpy(ctx->node_name,  node_name,  sizeof(ctx->node_name)  - 1);//  將 node_name 複製到 context 中，確保字串結尾
    ctx->master_port    = master_port; // 將 master_port 設定到 context 中
    ctx->sub_slot_count = 0;// 初始化訂閱槽數量為 0

    ctx->master_sock = connect_tcp(master_ip, master_port); // 連接到 master，成功回傳 socket fd，失敗回傳 -1
    if (ctx->master_sock < 0) { free(ctx); return NULL; } // 連接失敗
    set_recv_timeout(ctx->master_sock, 3); /* 問題四修正：避免 ctx_lookup_slot recv 永久阻塞 */

    char buf[256]; // 向 master 註冊 node name，格式是 NODE node_name\n，如果發送失敗就關閉 socket 並釋放 context 後回傳 NULL
    snprintf(buf, sizeof(buf), "NODE %s\n", node_name); // 將註冊命令格式化到 buf 中
    if (send_line(ctx->master_sock, buf) <= 0) { // 發送註冊命令到 master，如果失敗就關閉 socket 並釋放 context 後回傳 NULL
        close(ctx->master_sock); free(ctx); return NULL;
    }

    printf("[%s] connected to master\n", node_name);
    return ctx;
}

/*
 * cmos_sub_add(ctx, topic, filter_state, filter_type, filter_key, cb)
 *
 * 新增一個 topic 訂閱，三個 filter 任一填 NULL 表示該欄位不過濾（萬用）：
 *
 *   filter_state = "state"  → 只收有快取的訊息
 *   filter_type  = "sensor" → 只收 type=sensor 的訊息
 *   filter_key   = "rpm"    → 只收 key=rpm 的訊息
 *
 * 有任何 filter：cb(topic, value)  — 只傳解析後的 value 字串
 * 全部 NULL：   cb(topic, data)   — 傳完整原始 data（向下相容）
 */
int cmos_sub_add(cmos_sub_ctx_t *ctx,
                   const char *topic,
                   const char *filter_state,
                   const char *filter_type,
                   const char *filter_key,
                   cmos_callback_t cb)
{
    if (!ctx || !topic || ctx->sub_slot_count >= CM_MAX_SUB_SLOTS) return -1; // 參數檢查：context 和 topic 必須存在，訂閱槽數量不能超過上限

    int s = ctx->sub_slot_count++;
    cm_sub_slot_t *slot = &ctx->sub_slots[s];
    strncpy(slot->topic,        topic,                          sizeof(slot->topic)        - 1);
    strncpy(slot->filter_state, filter_state ? filter_state : "", sizeof(slot->filter_state) - 1);
    strncpy(slot->filter_type,  filter_type  ? filter_type  : "", sizeof(slot->filter_type)  - 1);
    strncpy(slot->filter_key,   filter_key   ? filter_key   : "", sizeof(slot->filter_key)   - 1);
    slot->topic       [sizeof(slot->topic)        - 1] = '\0';
    slot->filter_state[sizeof(slot->filter_state) - 1] = '\0';
    slot->filter_type [sizeof(slot->filter_type)  - 1] = '\0';
    slot->filter_key  [sizeof(slot->filter_key)   - 1] = '\0';
    slot->cb   = cb;
    slot->used = 1;

    /* 向 master 登錄，然後嘗試一次連線 */
    char buf[256]; // 向 master 登錄這個訂閱的 topic，格式是 REGISTER_SUB topic\n，讓 master 知道這個訂閱者對這個 topic 有興趣，當 publisher 註冊或 lookup 時 master 就會把這個訂閱者的資訊發給 publisher，讓 publisher 知道要連線給這個訂閱者
    snprintf(buf, sizeof(buf), "REGISTER_SUB %s\n", topic);
    send_line(ctx->master_sock, buf);

    /* 非阻塞：試一次，成功最好；失敗由 spin 的 timeout 週期補連 */
    if (ctx_lookup_slot(ctx, s) == 0) // 嘗試一次連線到這個訂閱槽對應的 publisher，如果成功就回傳 0，失敗就回傳 -1（例如 master 沒有回應或沒有找到 publisher），不論成功與否都繼續執行，讓 spin 的 timeout 週期補連
        printf("[SUB:%s] connected publisher(s)\n", slot->topic);
    else
        printf("[SUB:%s] publisher 未上線，spin 中持續等待...\n", slot->topic);

    return 0;
}

void cmos_sub_spin_ctx(cmos_sub_ctx_t *ctx) // 進入 spin loop，持續監聽 publisher 的訊息，並處理連線和斷線事件，當 publisher 發來訊息時會呼叫對應訂閱槽的 callback
{
    printf("[%s] spinning\n", ctx->node_name);
    time_t last_reconnect = 0; /* 問題一修正：用時間戳獨立控制補連週期 */

    while (1) {
        if (ctx_active_count(ctx) == 0) ctx_reconnect_all_pubs(ctx);

        /* 完全沒有連線：sleep 後重試 */
        if (ctx_active_count(ctx) == 0) {
            sleep(2);
            ctx_reconnect_all_pubs(ctx);
            continue;
        }

        /* timeout 2000ms */
        struct epoll_event events[CM_MAX_PUB_CONN];
        int r = epoll_wait(ctx->epfd, events, CM_MAX_PUB_CONN, 2000);

        /*
         * 問題一修正：不再依賴 r==0 判斷。
         * 每 2 秒用 time() 獨立觸發補連，即使其他 topic 持續有資料也能執行。
         * 問題二/三修正：ctx_lookup_slot 內部 recv 已設 SO_RCVTIMEO=3s，
         * 不會永久阻塞；最多凍住 slot_count × 3 秒，不再無限掛住。
         */
        time_t now = time(NULL);
        if (now - last_reconnect >= 2) {
            last_reconnect = now;

            /*
             * 修正:master_sock 的重連原本完全依賴外層的
             * "ctx_active_count(ctx) == 0" 判斷 —— 但 ctx_lookup_slot() 內部
             * 的 recv() 設有 SO_RCVTIMEO=3s,單純「Master 一時忙、3 秒內沒回
             * 應」也會被當成連線斷掉,直接把 master_sock 關閉、設成 -1。
             * 只要當下還有任一個 Publisher 連線是活著的(active_count > 0),
             * 外層就永遠不會重連 master,導致 master_sock 一旦因逾時被關閉,
             * 就再也連不回去、永遠發現不了新上線的 Publisher(即使既有連線
             * 收資料完全正常,問題也不會被發現)。改成每輪都獨立檢查、必要
             * 就重連,不再依賴 active_count。
             */
            if (ctx->master_sock < 0) ctx_reconnect_master(ctx);

            /*
             * 修正:同一個 topic 可能同時有多個 Publisher(例如本地測試用的
             * Publisher 跟 Redfish 端的 Publisher 都在 publish 同一個
             * topic)。原本只在「這個 slot 目前 0 條連線」時才重新查詢
             * Master,導致只要已經連上任一個 Publisher,之後才上線的其他
             * Publisher 永遠不會被追加連線進來。改成每個 slot 都定期重新
             * 查詢,ctx_add_pub() 內部本來就有 ctx_find_conn() 去重,不會
             * 造成同一個 Publisher 被重複連線。
             */
            if (ctx->master_sock >= 0) {
                for (int s = 0; s < ctx->sub_slot_count; s++) {
                    if (!ctx->sub_slots[s].used) continue;
                    ctx_lookup_slot(ctx, s);
                }
            }
        }

        if (r <= 0) continue;

        for (int e = 0; e < r; e++) {
            int i = (int)events[e].data.u32;
            if (i < 0 || i >= CM_MAX_PUB_CONN) continue;
            cm_pub_conn_t *c = &ctx->pub_conns[i];
            if (!c->active || c->sock < 0) continue;

            int n = recv(c->sock, c->rxbuf + c->rxlen,
                         CM_RX_BUF - 1 - c->rxlen, 0);
            if (n <= 0) {
                int s = c->sub_slot;
                printf("[SUB:%s] publisher disconnected: %s\n",
                       (s >= 0 ? ctx->sub_slots[s].topic : "?"), c->node_name);
                ctx_close_conn(ctx, i);
                continue;
            }

            c->rxlen += n;
            c->rxbuf[c->rxlen] = '\0';

            int start = 0;
            for (int k = 0; k < c->rxlen; k++) {
                if (c->rxbuf[k] != '\n') continue;
                char line[CM_LINE_BUF];
                int  ll = k - start + 1;
                if (ll >= (int)sizeof(line)) ll = (int)sizeof(line) - 1;
                memcpy(line, c->rxbuf + start, ll);
                line[ll] = '\0';
                ctx_handle_line(ctx, c, line);
                start = k + 1;
            }

            if (start > 0) {
                memmove(c->rxbuf, c->rxbuf + start, c->rxlen - start);
                c->rxlen -= start;
            }
            if (c->rxlen >= CM_RX_BUF - 1) { c->rxlen = 0; c->rxbuf[0] = '\0'; }
        }
    }
}

void cmos_sub_destroy(cmos_sub_ctx_t *ctx) // 銷毀訂閱者 context，關閉所有連線並釋放資源
{
    if (!ctx) return;
    ctx_close_all_conns(ctx);
    if (ctx->master_sock >= 0) close(ctx->master_sock);
    if (ctx->epfd >= 0) close(ctx->epfd);
    free(ctx);
}

