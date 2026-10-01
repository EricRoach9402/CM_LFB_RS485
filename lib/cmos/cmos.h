#ifndef CMOS_H
#define CMOS_H

typedef void (*cmos_callback_t)(const char *topic, const char *data);

/* -------------------------------------------------------
 * Publisher API
 * ------------------------------------------------------- */
int  cmos_pub_init(const char *master_ip, int master_port,
                     const char *node_name, const char *topic, int listen_port);
void cmos_pub_poll(void);

/*
 * cmos_publish(state, type, key, value)
 *
 * 單一發送 API，NULL 表示省略該欄位：
 *
 *   cmos_publish(NULL,    "sensor", "temperature", "25.3");
 *   cmos_publish(NULL,    NULL,     NULL,          "hello"); → 純字串
 *   cmos_publish("state", "fan",    "rpm",         "1200"); → 快取並廣播
 *
 * state != NULL：Publisher 端快取最新值；
 *               新 Subscriber 連線時自動重播所有 state，
 *               讓晚上線的訂閱者立即取得最新狀態。
 * state == NULL：正常廣播，不快取。
 */
int  cmos_publish(const char *state,
                    const char *type, const char *key, const char *value);


void cmos_pub_close(void);

/* -------------------------------------------------------
 * Subscriber API
 * ------------------------------------------------------- */
typedef struct cmos_sub_ctx cmos_sub_ctx_t;

/* 建立 Subscriber Node，連線至 Master */
cmos_sub_ctx_t *cmos_sub_create(const char *master_ip, int master_port,
                                     const char *node_name);

/*
 * cmos_sub_add(ctx, topic, filter_state, filter_type, filter_key, cb)
 *
 * 訂閱一個 topic，可同時指定 type/key 過濾：
 *   filter_type / filter_key 填 NULL 表示不過濾該欄位。
 *
 * 有 filter：cb(topic, value)  — 只傳解析後的 value
 * 無 filter：cb(topic, data)   — 傳完整原始 data（向下相容）
 *
 * 可多次呼叫以訂閱不同 topic（上限 CM_MAX_SUB_SLOTS = 256）。
 */
/*
 * filter_state / filter_type / filter_key 任一填 NULL = 該欄位不過濾
 *
 * 有任何 filter → cb(topic, value)   只傳解析後的 value
 * 全部 NULL    → cb(topic, data)    傳完整原始 data（向下相容）
 *
 * 範例：
 *   cmos_sub_add(ctx, "sensors", NULL,    "sensor", NULL,  cb); // 所有 sensor type
 *   cmos_sub_add(ctx, "sensors", "state", "fan",    "rpm", cb); // state + fan + rpm
 *   cmos_sub_add(ctx, "sensors", NULL,    NULL,     NULL,  cb); // 全收
 */
int  cmos_sub_add(cmos_sub_ctx_t *ctx,
                    const char *topic,
                    const char *filter_state,
                    const char *filter_type,
                    const char *filter_key,
                    cmos_callback_t cb);

/* 進入接收迴圈（阻塞，單一 select 處理所有 topic） */
void cmos_sub_spin_ctx(cmos_sub_ctx_t *ctx);

/* 釋放所有資源 */
void cmos_sub_destroy(cmos_sub_ctx_t *ctx);

/* -------------------------------------------------------
 * Service API  (ROS-style request / response)
 * ------------------------------------------------------- */

/*
 * Service handler (server side)：
 *   service  — service 名稱
 *   request  — client 送來的請求字串
 *   response — 填入回應內容（最多 response_len-1 個字元）
 */
typedef void (*cmos_srv_handler_t)(const char *service,
                                   const char *request,
                                   char *response, int response_len);

/* --- Service Server --- */

/*
 * 初始化 Service Server：
 *   向 master 註冊 node_name，開 listen socket 等待 client 連線，
 *   並向 master 登錄 service_name @ 127.0.0.1:listen_port。
 *   成功回傳 0，失敗回傳 -1。
 */
int  cmos_srv_init(const char *master_ip, int master_port,
                   const char *node_name, const char *service_name,
                   int listen_port, cmos_srv_handler_t handler);

/* 進入 Service spin loop（阻塞）：接受 client 連線、呼叫 handler、回傳結果 */
void cmos_srv_spin(void);

/* 釋放 Service Server 資源 */
void cmos_srv_close(void);

/* --- Service Client --- */

/*
 * 同步呼叫 service（阻塞直到收到回應或逾時）：
 *   service_name — 要呼叫的 service 名稱
 *   request      — 請求字串
 *   response     — 填入回應內容的緩衝區
 *   response_len — response 緩衝區大小
 *   timeout_ms   — 逾時（毫秒）
 *
 *   成功回傳 0，失敗（service 不存在 / 逾時 / 連線錯誤）回傳 -1。
 */
int  cmos_srv_call(const char *master_ip, int master_port,
                   const char *service_name,
                   const char *request,
                   char *response, int response_len,
                   int timeout_ms);

/*
 * 等待 service 出現（類似 ros::service::waitForService()）：
 *   每 500ms 嘗試一次，直到成功或 timeout_ms 到期。
 *   成功回傳 0，timeout 回傳 -1。
 */
int  cmos_srv_wait(const char *master_ip, int master_port,
                   const char *service_name,
                   int timeout_ms);

#endif
