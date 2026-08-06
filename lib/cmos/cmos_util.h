#ifndef CMOS_UTIL_H
#define CMOS_UTIL_H

#include "cmos.h"
#include <string.h>
#include <stdlib.h>

/* -------------------------------------------------------
 * Publisher — 批次發送
 *
 * 單筆直接用 cmos_publish(state, type, key, value)
 * 多筆用 cmos_publish_batch()：
 *
 *   cmos_msg_t msgs[] = {
 *       { "state", "sensor", "temperature", "25.3" },
 *       { "state", "sensor", "humidity",    "60.1" },
 *       { "state", "fan",    "rpm",         "1200" },
 *       { NULL,    "sys",    "event",       "42"   },
 *   };
 *   cmos_publish_batch(msgs, 4);
 * ------------------------------------------------------- */
typedef struct {
    const char *state;
    const char *type;
    const char *key;
    const char *value;
} cmos_msg_t;

static inline int cmos_publish_batch(const cmos_msg_t *msgs, int count)
{
    for (int i = 0; i < count; i++) {
        if (cmos_publish(msgs[i].state, msgs[i].type, msgs[i].key, msgs[i].value) < 0)
            return -1;
    }
    return 0;
}

/* -------------------------------------------------------
 * Subscriber — 解析收到的 data 字串
 *
 * void on_msg(const char *topic, const char *data) {
 *     cmos_parsed_t msg;
 *     cmos_parse(data, &msg);
 *
 *     if (strcmp(msg.type, "sensor") == 0 &&
 *         strcmp(msg.key,  "temperature") == 0)
 *         printf("%.1f\n", msg.value_f);
 * }
 * ------------------------------------------------------- */
typedef struct {
    char  type[64];
    char  key[64];
    char  value[256];
    float value_f;
    int   value_i;
} cmos_parsed_t;

/* 從 data 取出指定欄位，回傳 0 成功 / -1 找不到 */
static inline int cmos_get_field(const char *data, const char *field,
                                   char *out, int out_size)
{
    char search[128];
    int  slen;

    /* 組出 "field=" 搜尋字串 */
    slen = 0;
    while (field[slen] && slen < 126) { search[slen] = field[slen]; slen++; }
    search[slen++] = '=';
    search[slen]   = '\0';

    const char *p = strstr(data, search);
    if (!p) { if (out_size > 0) out[0] = '\0'; return -1; }

    p += slen;
    int i = 0;
    while (*p && *p != ' ' && *p != '\r' && *p != '\n' && i < out_size - 1)
        out[i++] = *p++;
    out[i] = '\0';
    return 0;
}

/* 解析 data → cmos_parsed_t，回傳 0 成功 / -1 無 type */
static inline int cmos_parse(const char *data, cmos_parsed_t *msg)
{
    memset(msg, 0, sizeof(*msg));
    cmos_get_field(data, "type",  msg->type,  sizeof(msg->type));
    cmos_get_field(data, "key",   msg->key,   sizeof(msg->key));
    cmos_get_field(data, "value", msg->value, sizeof(msg->value));
    if (msg->value[0]) {
        msg->value_f = (float)atof(msg->value);
        msg->value_i = atoi(msg->value);
    }
    return msg->type[0] ? 0 : -1;
}

#endif /* CM_MINI_ROS_UTIL_H */
