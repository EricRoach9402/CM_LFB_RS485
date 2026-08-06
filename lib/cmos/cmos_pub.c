#include "cmos.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <netinet/tcp.h>
#include <signal.h>

#define CM_MAX_SUB_CLIENT  256 // publisher 最多同時連線的 subscriber 數量，超過就拒絕新連線

static int master_sock = -1;
static int listen_sock = -1;  // publisher 的 listen socket fd，等待 subscriber 連線用的 socket，如果沒有就設為 -1
static int sub_socks[CM_MAX_SUB_CLIENT];  // 與 subscriber 的連線 socket 陣列，最多 CM_MAX_SUB_CLIENT 個 subscriber，每個元素是與 subscriber 的連線 socket fd，如果沒有連線就設為 -1
static int sub_count  = 0;
static char pub_topic[64]; // publisher 的 topic 名稱，最多 63 個字元，後續發佈訊息時會用到
static int sigpipe_inited = 0;
static int pub_epfd       = -1; // epoll fd，用來非阻塞監聽 listen_sock 是否有 subscriber 連進來

static void init_sigpipe_ignore(void)  // 忽略 SIGPIPE，跟 master 一樣。防止 subscriber 斷線時 send() 把整個程式殺掉。用 sigpipe_inited 旗標確保只設一次。
{
    if (!sigpipe_inited) {
        signal(SIGPIPE, SIG_IGN);
        sigpipe_inited = 1;
    }
}

static int connect_tcp(const char *ip, int port)  // 連線到 TCP 伺服器，成功回傳 socket fd，失敗回傳 -1
{
    int sock;
    struct sockaddr_in addr;

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    if (inet_pton(AF_INET, ip, &addr.sin_addr) <= 0) { close(sock); return -1; }
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) { close(sock); return -1; }

    return sock;
}

static int send_line(int sock, const char *line)
{
#ifdef MSG_NOSIGNAL
    return send(sock, line, strlen(line), MSG_NOSIGNAL);
#else
    return send(sock, line, strlen(line), 0);
#endif
}

int cmos_pub_init(const char *master_ip, int master_port,
                    const char *node_name,
                    const char *topic,
                    int listen_port) // 初始化 publisher，連線到 master_ip:master_port，註冊 node_name 和 topic，開 listen socket 等待 subscriber 連線，成功回傳 0，失敗回傳 -1
{
    int opt = 1; // socket 選項，SO_REUSEADDR 需要這個變數來設定，值為 1 代表開啟這個選項
    struct sockaddr_in addr;
    char buf[256];  // 用來組合註冊訊息的緩衝區，確保足夠大以容納註冊訊息，避免被截斷

    init_sigpipe_ignore(); // 確保 SIGPIPE 已經被忽略，保護層

    strncpy(pub_topic, topic, sizeof(pub_topic) - 1); // 設定 publisher 的 topic 名稱，後續發佈訊息時會用到
    pub_topic[sizeof(pub_topic) - 1] = '\0'; // 確保字串結尾，避免 pub_topic 沒有 null terminator 而導致後續使用時出錯

    for (int i = 0; i < CM_MAX_SUB_CLIENT; i++) sub_socks[i] = -1; // 初始化 subscriber socket 陣列
    sub_count = 0;

    master_sock = connect_tcp(master_ip, master_port); // 連線到 master，註冊節點和 publisher
    if (master_sock < 0) return -1;

    snprintf(buf, sizeof(buf), "NODE %s\n", node_name); // 告訴 master 這個 publisher 的節點名稱
    if (send_line(master_sock, buf) <= 0) return -1; // 發送註冊訊息給 master，格式是 "NODE node_name\n"

    
    /*
    開自己的 listen socket
    跟 master 建立 server socket 的流程一模一樣：socket → SO_REUSEADDR → bind → listen。差別只是 port 不同（例如 6001）。這個 socket 是給 subscriber 連進來用的。
    */
    listen_sock = socket(AF_INET, SOCK_STREAM, 0); // 建立 TCP socket，成功回傳 socket fd，失敗回傳 -1
    if (listen_sock < 0) return -1;

    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(listen_port);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) return -1;
    if (listen(listen_sock, 10) < 0) return -1;

    /* 建立 epoll 並把 listen_sock 加進去 */
    pub_epfd = epoll_create1(0);
    if (pub_epfd < 0) return -1;
    {
        struct epoll_event ev;
        ev.events  = EPOLLIN;
        ev.data.fd = listen_sock;
        epoll_ctl(pub_epfd, EPOLL_CTL_ADD, listen_sock, &ev);
    }

     /* 向 master 註冊：我在 listen_port 上發布 topic */
     // 註冊訊息格式是 "REGISTER_PUB topic ip port\n"，例如 "REGISTER_PUB test_topic


    // 向 master 註冊 送 REGISTER_PUB test_topic 127.0.0.1 6001\n 給 master，意思是：「我在 127.0.0.1:6001 上發布 test_topic，subscriber 來找我就對了。」
    snprintf(buf, sizeof(buf), "REGISTER_PUB %s 127.0.0.1 %d\n", topic, listen_port);
    if (send_line(master_sock, buf) <= 0) return -1;

    return 0;
}

// 這個函式要定期被呼叫，檢查是否有 subscriber 連線進來，並把它們加入 sub_socks 陣列。
void cmos_pub_poll(void)
{
    if (listen_sock < 0 || pub_epfd < 0) return;

    struct epoll_event ev;
    if (epoll_wait(pub_epfd, &ev, 1, 0) <= 0) return; /* timeout=0：非阻塞 */

    int c = accept(listen_sock, NULL, NULL);
    if (c >= 0 && sub_count < CM_MAX_SUB_CLIENT) {
        int ka = 1, idle = 10, interval = 5, count = 3;
        setsockopt(c, SOL_SOCKET,  SO_KEEPALIVE,  &ka,       sizeof(ka));
        setsockopt(c, IPPROTO_TCP, TCP_KEEPIDLE,  &idle,     sizeof(idle));
        setsockopt(c, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
        setsockopt(c, IPPROTO_TCP, TCP_KEEPCNT,   &count,    sizeof(count));
        sub_socks[sub_count++] = c;
        printf("[PUB] subscriber connected, total=%d\n", sub_count);
    } else if (c >= 0) {
        close(c);
    }
}

int cmos_publish(const char *state,
                   const char *type, const char *key, const char *value)
{
    char data[896];  /* 組出 data 部分 */ // 896 是 1024 減去 "MSG topic " 的長度 128，確保整行訊息不會超過 1024 字元，避免被截斷。
    int  pos = 0;

    /* state 寫進 wire format，讓 sub 端可以過濾 */
    if (state) pos += snprintf(data + pos, sizeof(data) - pos, "state=%s", state);
    if (type)  pos += snprintf(data + pos, sizeof(data) - pos, "%stype=%s",
                               pos ? " " : "", type);
    if (key)   pos += snprintf(data + pos, sizeof(data) - pos, "%skey=%s",
                               pos ? " " : "", key);
    if (value) {
        if (pos)  /* 有 state/type/key → 加 value= 前綴 */
            pos += snprintf(data + pos, sizeof(data) - pos, " value=%s", value);
        else      /* 純數值，直接送，不加前綴 */
            pos += snprintf(data + pos, sizeof(data) - pos, "%s", value);
    }

    char buf[1024]; // 組出整行訊息，格式是 "MSG topic data\n"，例如 "MSG test_topic state=ok type=temperature value=25\n"，然後發送給所有 subscriber。
    snprintf(buf, sizeof(buf), "MSG %s %s\n", pub_topic, data);
    int len = (int)strlen(buf);

    for (int i = 0; i < sub_count; ) {
#ifdef MSG_NOSIGNAL
        int n = send(sub_socks[i], buf, len, MSG_NOSIGNAL);
#else
        int n = send(sub_socks[i], buf, len, 0); // 發送訊息給 subscriber，使用 send() 的 MSG_NOSIGNAL 選項來避免 SIGPIPE，如果系統不支援 MSG_NOSIGNAL 就用 0，然後靠之前設的 signal(SIGPIPE, SIG_IGN) 來忽略 SIGPIPE 信號，確保 subscriber 斷線時不會殺掉整個 publisher 程式。
#endif
        if (n <= 0) {
            close(sub_socks[i]);
            for (int k = i; k < sub_count - 1; k++) sub_socks[k] = sub_socks[k + 1];
            sub_socks[--sub_count] = -1;
            printf("[PUB] subscriber disconnected, total=%d\n", sub_count);
        } else {
            i++;
        }
    }
    return 0;
}

void cmos_pub_close(void)  // 關閉所有 subscriber 連線，關閉 listen socket 和 master socket，清空 sub_socks 陣列
{
    for (int i = 0; i < sub_count; i++) {
        if (sub_socks[i] >= 0) close(sub_socks[i]);
        sub_socks[i] = -1;
    }
    sub_count = 0;
    if (pub_epfd   >= 0) { close(pub_epfd);   pub_epfd   = -1; }
    if (listen_sock >= 0) { close(listen_sock); listen_sock = -1; }
    if (master_sock >= 0) { close(master_sock); master_sock = -1; }
}
