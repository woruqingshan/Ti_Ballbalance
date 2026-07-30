#include "protocol.h"
#include "crc16.h"
#include <string.h>

uint16_t protocol_get_u16_le(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
uint32_t protocol_get_u32_le(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
int16_t protocol_get_i16_le(const uint8_t *p) { return (int16_t)protocol_get_u16_le(p); }
void protocol_put_u16_le(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
void protocol_put_i16_le(uint8_t *p, int16_t v) { protocol_put_u16_le(p, (uint16_t)v); }
void protocol_put_u32_le(uint8_t *p, uint32_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }

void protocol_parser_init(ProtocolParser *p) { memset(p, 0, sizeof(*p)); }

static void reset_keep_errors(ProtocolParser *p)
{
    uint32_t c=p->crc_errors, l=p->length_errors, v=p->version_errors;
    memset(p,0,sizeof(*p)); p->crc_errors=c; p->length_errors=l; p->version_errors=v;
}

bool protocol_parser_push(ProtocolParser *p, uint8_t byte, ProtocolFrame *out)
{
    if (p->index == 0U) { if (byte != PROTOCOL_SOF0) return false; p->buffer[p->index++] = byte; return false; }
    if (p->index == 1U) { if (byte != PROTOCOL_SOF1) { p->index = (byte == PROTOCOL_SOF0) ? 1U : 0U; p->buffer[0]=byte; return false; } p->buffer[p->index++]=byte; return false; }
    if (p->index >= PROTOCOL_MAX_FRAME) { p->length_errors++; reset_keep_errors(p); return false; }
    p->buffer[p->index++] = byte;
    if (p->index == 6U) {
        uint16_t n = protocol_get_u16_le(&p->buffer[4]);
        if (n > PROTOCOL_MAX_PAYLOAD) { p->length_errors++; reset_keep_errors(p); return false; }
        p->expected = (uint16_t)(16U + n);
    }
    if (p->expected != 0U && p->index == p->expected) {
        uint16_t payload_len = protocol_get_u16_le(&p->buffer[4]);
        uint16_t received_crc = protocol_get_u16_le(&p->buffer[14U + payload_len]);
        uint16_t calculated_crc = crc16_ccitt_false(&p->buffer[2], (size_t)(12U + payload_len));
        if (p->buffer[2] != PROTOCOL_VERSION) { p->version_errors++; reset_keep_errors(p); return false; }
        if (received_crc != calculated_crc) { p->crc_errors++; reset_keep_errors(p); return false; }
        out->type = p->buffer[3]; out->payload_len = payload_len;
        out->sequence = protocol_get_u32_le(&p->buffer[6]);
        out->timestamp_ms = protocol_get_u32_le(&p->buffer[10]);
        if (payload_len) memcpy(out->payload, &p->buffer[14], payload_len);
        reset_keep_errors(p); return true;
    }
    return false;
}

size_t protocol_encode(uint8_t type, uint32_t sequence, uint32_t timestamp_ms,
                       const uint8_t *payload, uint16_t payload_len,
                       uint8_t *out, size_t cap)
{
    size_t total = (size_t)16U + payload_len;
    uint16_t crc;
    if (payload_len > PROTOCOL_MAX_PAYLOAD || cap < total) return 0U;
    out[0]=PROTOCOL_SOF0; out[1]=PROTOCOL_SOF1; out[2]=PROTOCOL_VERSION; out[3]=type;
    protocol_put_u16_le(&out[4], payload_len); protocol_put_u32_le(&out[6], sequence);
    protocol_put_u32_le(&out[10], timestamp_ms);
    if (payload_len && payload) memcpy(&out[14], payload, payload_len);
    crc = crc16_ccitt_false(&out[2], (size_t)(12U + payload_len));
    protocol_put_u16_le(&out[14U + payload_len], crc);
    return total;
}
