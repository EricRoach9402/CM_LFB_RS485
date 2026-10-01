/*
 * src/cmos_srv.c
 *
 * CMOS Service — ROS-style request/response
 *
 * Wire protocol (plain text, line-based):
 *
 *   Master commands:
 *     REGISTER_SRV  <service> 127.0.0.1 <port>   (server → master)
 *     LOOKUP_SRV    <service>                      (client → master)
 *       → SRVINFO <service> <ip> <port> <node>\n
 *       → END\n
 *
 *   Direct server ↔ client:
 *     REQ <service> <request_data>\n              (client → server)
 *     RES <service> <response_data>\n             (server → client)
 */

#include "cmos.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <signal.h>

#define SRV_LINE_BUF  1024

/* -------------------------------------------------------
 * shared helpers
 * ------------------------------------------------------- */

static int srv_sigpipe_inited = 0;

static void srv_init_sigpipe(void)
{
    if (!srv_sigpipe_inited) {
        signal(SIGPIPE, SIG_IGN);
        srv_sigpipe_inited = 1;
    }
}

static int srv_connect_tcp(const char *ip, int port)
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

static int srv_send_line(int sock, const char *line)
{
#ifdef MSG_NOSIGNAL
    return send(sock, line, strlen(line), MSG_NOSIGNAL);
#else
    return send(sock, line, strlen(line), 0);
#endif
}

/*
 * 逐字元收取一行（含 '\n'），回傳讀取字元數，失敗回傳 -1。
 * 呼叫前應先用 SO_RCVTIMEO 設定 socket 逾時。
 */
static int srv_recv_line(int sock, char *buf, int buf_len)
{
    int pos = 0;
    while (pos < buf_len - 1) {
        char c;
        int n = recv(sock, &c, 1, 0);
        if (n <= 0) return -1;
        buf[pos++] = c;
        if (c == '\n') break;
    }
    buf[pos] = '\0';
    return pos;
}

static void set_sock_timeout(int sock, int ms)
{
    struct timeval tv;
    tv.tv_sec  = ms / 1000;
    tv.tv_usec = (ms % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

/* -------------------------------------------------------
 * Service Server globals
 * ------------------------------------------------------- */

static int                 srv_master_sock = -1;
static int                 srv_listen_sock = -1;
static char                srv_service[64];
static cmos_srv_handler_t  srv_handler     = NULL;

/* -------------------------------------------------------
 * Service Server API
 * ------------------------------------------------------- */

int cmos_srv_init(const char *master_ip, int master_port,
                  const char *node_name, const char *service_name,
                  int listen_port, cmos_srv_handler_t handler)
{
    srv_init_sigpipe();

    strncpy(srv_service, service_name, sizeof(srv_service) - 1);
    srv_service[sizeof(srv_service) - 1] = '\0';
    srv_handler = handler;

    /* 連線到 master */
    srv_master_sock = srv_connect_tcp(master_ip, master_port);
    if (srv_master_sock < 0) {
        fprintf(stderr, "[SRV] connect master failed\n");
        return -1;
    }

    char buf[256];
    snprintf(buf, sizeof(buf), "NODE %s\n", node_name);
    if (srv_send_line(srv_master_sock, buf) <= 0) {
        close(srv_master_sock); srv_master_sock = -1; return -1;
    }

    /* 開 listen socket */
    int opt = 1;
    struct sockaddr_in addr;
    srv_listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (srv_listen_sock < 0) { close(srv_master_sock); srv_master_sock = -1; return -1; }

    setsockopt(srv_listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(listen_port);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    if (bind(srv_listen_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[SRV] bind"); close(srv_listen_sock); srv_listen_sock = -1; close(srv_master_sock); srv_master_sock = -1; return -1;
    }
    if (listen(srv_listen_sock, 5) < 0) {
        perror("[SRV] listen"); close(srv_listen_sock); srv_listen_sock = -1; close(srv_master_sock); srv_master_sock = -1; return -1;
    }

    /* 向 master 登錄 */
    snprintf(buf, sizeof(buf), "REGISTER_SRV %s 127.0.0.1 %d\n", service_name, listen_port);
    if (srv_send_line(srv_master_sock, buf) <= 0) {
        close(srv_listen_sock); srv_listen_sock = -1; close(srv_master_sock); srv_master_sock = -1; return -1;
    }

    printf("[SRV] /%s ready on port %d\n", service_name, listen_port);
    return 0;
}

void cmos_srv_spin(void)
{
    printf("[SRV] /%s spinning\n", srv_service);

    while (1) {
        int client = accept(srv_listen_sock, NULL, NULL);
        if (client < 0) continue;

        /* 5 秒逾時接收 REQ */
        set_sock_timeout(client, 5000);

        char line[SRV_LINE_BUF];
        if (srv_recv_line(client, line, sizeof(line)) <= 0) {
            close(client); continue;
        }

        /*
         * 解析 REQ <service> <request_data>
         *   strtok(line, " \r\n") → cmd
         *   strtok(NULL, " \r\n") → service
         *   strtok(NULL, "\r\n")  → request_data（可含空格）
         */
        char *cmd      = strtok(line, " \r\n");
        char *svc      = strtok(NULL, " \r\n");
        char *req_data = strtok(NULL, "\r\n");

        if (!cmd || strcmp(cmd, "REQ") != 0 || !svc) {
            close(client); continue;
        }

        /* req_data 可能開頭有一個空格（strtok 殘留），跳過 */
        if (req_data && req_data[0] == ' ') req_data++;

        char response[SRV_LINE_BUF] = {0};
        if (srv_handler)
            srv_handler(svc, req_data ? req_data : "", response, sizeof(response));

        /* 送出 RES */
        char out[SRV_LINE_BUF + 128];
        snprintf(out, sizeof(out), "RES %s %s\n", srv_service, response);
        srv_send_line(client, out);
        close(client);
    }
}

void cmos_srv_close(void)
{
    if (srv_listen_sock >= 0) { close(srv_listen_sock); srv_listen_sock = -1; }
    if (srv_master_sock >= 0) { close(srv_master_sock); srv_master_sock = -1; }
}

/* -------------------------------------------------------
 * Service Client API
 * ------------------------------------------------------- */

int cmos_srv_call(const char *master_ip, int master_port,
                  const char *service_name,
                  const char *request,
                  char *response, int response_len,
                  int timeout_ms)
{
    srv_init_sigpipe();

    /* 1. 向 master 查詢 service 位址 */
    int msock = srv_connect_tcp(master_ip, master_port);
    if (msock < 0) return -1;

    set_sock_timeout(msock, timeout_ms);

    char buf[256];
    snprintf(buf, sizeof(buf), "NODE cmos_srv_client\n");
    srv_send_line(msock, buf);

    snprintf(buf, sizeof(buf), "LOOKUP_SRV %s\n", service_name);
    if (srv_send_line(msock, buf) <= 0) { close(msock); return -1; }

    /* 收 SRVINFO ... END */
    char srv_ip[64] = {0};
    int  srv_port   = -1;

    char line[SRV_LINE_BUF];
    while (1) {
        if (srv_recv_line(msock, line, sizeof(line)) <= 0) break;
        if (strcmp(line, "END\n") == 0) break;

        if (strncmp(line, "SRVINFO ", 8) == 0) {
            /* SRVINFO <service> <ip> <port> <node> */
            char tmp[SRV_LINE_BUF];
            strncpy(tmp, line, sizeof(tmp) - 1);
            tmp[sizeof(tmp) - 1] = '\0';

            char *cmd  = strtok(tmp, " \r\n"); (void)cmd;
            char *svc  = strtok(NULL, " \r\n"); (void)svc;
            char *ip   = strtok(NULL, " \r\n");
            char *port = strtok(NULL, " \r\n");
            if (ip && port) {
                strncpy(srv_ip, ip, sizeof(srv_ip) - 1);
                srv_port = atoi(port);
            }
        }
    }
    close(msock);

    if (srv_port < 0) return -1;  /* service 不存在 */

    /* 2. 直連 service server */
    int ssock = srv_connect_tcp(srv_ip, srv_port);
    if (ssock < 0) return -1;

    set_sock_timeout(ssock, timeout_ms);

    /* 3. 送 REQ */
    snprintf(buf, sizeof(buf), "REQ %s %s\n", service_name,
             request ? request : "");
    if (srv_send_line(ssock, buf) <= 0) { close(ssock); return -1; }

    /* 4. 收 RES */
    if (srv_recv_line(ssock, line, sizeof(line)) <= 0) { close(ssock); return -1; }
    close(ssock);

    /*
     * 解析 RES <service> <response_data>
     *   跳過 "RES " 和 service 名稱，取剩餘部分為回應資料
     */
    char *p = line;
    /* 跳過 cmd */
    while (*p && *p != ' ') p++;
    while (*p == ' ') p++;
    /* 跳過 service name */
    while (*p && *p != ' ' && *p != '\r' && *p != '\n') p++;
    while (*p == ' ') p++;
    /* 去掉尾端 \r\n */
    int len = (int)strlen(p);
    while (len > 0 && (p[len - 1] == '\n' || p[len - 1] == '\r'))
        p[--len] = '\0';

    if (response && response_len > 0) {
        strncpy(response, p, response_len - 1);
        response[response_len - 1] = '\0';
    }
    return 0;
}

/* -------------------------------------------------------
 * cmos_srv_wait — 等待 service 上線
 * ------------------------------------------------------- */

/* 只查 master 有沒有登記這個 service，不實際呼叫 server */
static int srv_exists(const char *master_ip, int master_port,
                      const char *service_name)
{
    int msock = srv_connect_tcp(master_ip, master_port);
    if (msock < 0) return -1;

    set_sock_timeout(msock, 500);

    char buf[256];
    snprintf(buf, sizeof(buf), "NODE cmos_srv_wait\n");
    srv_send_line(msock, buf);

    snprintf(buf, sizeof(buf), "LOOKUP_SRV %s\n", service_name);
    if (srv_send_line(msock, buf) <= 0) { close(msock); return -1; }

    char line[SRV_LINE_BUF];
    int found = 0;
    while (1) {
        if (srv_recv_line(msock, line, sizeof(line)) <= 0) break;
        if (strcmp(line, "END\n") == 0) break;
        if (strncmp(line, "SRVINFO ", 8) == 0) found = 1;
    }
    close(msock);
    return found ? 0 : -1;
}

int cmos_srv_wait(const char *master_ip, int master_port,
                  const char *service_name,
                  int timeout_ms)
{
    int elapsed = 0;
    while (elapsed < timeout_ms) {
        /* 只檢查 master 登記表，不觸發 handler */
        if (srv_exists(master_ip, master_port, service_name) == 0)
            return 0;
        usleep(500 * 1000);
        elapsed += 500;
    }
    return -1;
}