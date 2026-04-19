/**
 * allstar_firefly.c  --  Allstar Firefly 318ALD31K native SubGHz protocol
 *
 * Implements the SubGhzProtocol vtable for the Supertex ED-9 based gate remote.
 * Both the decoder state machine and encoder TX buffer are ported directly from
 * the proven FAP implementation; only the framing (alloc/free/serialize/etc.)
 * changes to match the native SubGHz plugin interface.
 *
 * See allstar_firefly.h for full protocol documentation.
 */

#include "allstar_firefly.h"

#include <lib/subghz/protocols/base.h>

#include <furi.h>
#include <furi_hal.h>
#include <flipper_format/flipper_format.h>
#include <string.h>

#define TAG "AllstarFirefly"

/* --- Decoder instance ------------------------------------------------------ */

typedef enum {
    AfRxState_WaitGap,
    AfRxState_Receiving,
} AfRxState;

typedef struct {
    SubGhzProtocolDecoderBase base;
    AfRxState rx_state;
    uint8_t   rx_syms[AF_SYM_COUNT];
    uint8_t   rx_count;
    char dip[AF_BIT_COUNT + 1];
} SubGhzProtocolDecoderAllstarFirefly;

typedef struct {
    SubGhzProtocolEncoderBase base;
    char     dip[AF_BIT_COUNT + 1];
    uint32_t tx_buf[AF_TX_BUF_SIZE];
    uint32_t tx_size;
    uint32_t tx_pos;
} SubGhzProtocolEncoderAllstarFirefly;

static bool af_decode_symbols(const uint8_t* syms, char* dip) {
    for(uint8_t i = 0; i < AF_BIT_COUNT; i++) {
        uint8_t a = syms[i * 2];
        uint8_t b = syms[i * 2 + 1];
        if      (a == 1 && b == 1) dip[i] = '+';
        else if (a == 0 && b == 0) dip[i] = '-';
        else if (a == 1 && b == 0) dip[i] = '0';
        else return false;
    }
    dip[AF_BIT_COUNT] = '\0';
    return true;
}

static bool af_dip_valid(const char* dip) {
    if(!dip) return false;
    for(uint8_t i = 0; i < AF_BIT_COUNT; i++) {
        if(dip[i] != '+' && dip[i] != '-' && dip[i] != '0') return false;
    }
    return (dip[AF_BIT_COUNT] == '\0');
}

static uint32_t af_dip_to_uint32(const char* dip) {
    uint32_t val = 0;
    for(uint8_t i = 0; i < AF_BIT_COUNT; i++) {
        val *= 3;
        if     (dip[i] == '+') val += 2;
        else if(dip[i] == '0') val += 1;
    }
    return val;
}

static uint32_t af_build_tx_buf(const char* dip, uint32_t* buf) {
    uint32_t pos = 0;
    for(uint32_t rep = 0; rep < AF_TX_REPEAT; rep++) {
        for(uint32_t bit = 0; bit < AF_BIT_COUNT; bit++) {
            bool     last = (bit == AF_BIT_COUNT - 1);
            char     c    = dip[bit];
            uint32_t p0, g0, p1, g1;
            if(c == '+') {
                p0 = AF_LONG_PULSE_US;  g0 = AF_SHORT_GAP_US;
                p1 = AF_LONG_PULSE_US;  g1 = AF_SHORT_GAP_US;
            } else if(c == '-') {
                p0 = AF_SHORT_PULSE_US; g0 = AF_LONG_GAP_US;
                p1 = AF_SHORT_PULSE_US; g1 = AF_LONG_GAP_US;
            } else {
                p0 = AF_LONG_PULSE_US;  g0 = AF_SHORT_GAP_US;
                p1 = AF_SHORT_PULSE_US; g1 = AF_LONG_GAP_US;
            }
            if(last) g1 = AF_INTERFRAME_US;
            buf[pos++] = p0; buf[pos++] = g0;
            buf[pos++] = p1; buf[pos++] = g1;
        }
    }
    return pos;
}
static void* subghz_protocol_decoder_allstar_firefly_alloc(SubGhzEnvironment* environment) {
    UNUSED(environment);
    SubGhzProtocolDecoderAllstarFirefly* instance =
        malloc(sizeof(SubGhzProtocolDecoderAllstarFirefly));
    instance->base.protocol = &subghz_protocol_allstar_firefly;
    instance->rx_state      = AfRxState_WaitGap;
    instance->rx_count      = 0;
    memset(instance->dip, '-', AF_BIT_COUNT);
    instance->dip[AF_BIT_COUNT] = '\0';
    return instance;
}

static void subghz_protocol_decoder_allstar_firefly_free(void* context) {
    furi_assert(context); free(context);
}

static void subghz_protocol_decoder_allstar_firefly_reset(void* context) {
    furi_assert(context);
    SubGhzProtocolDecoderAllstarFirefly* instance = context;
    instance->rx_state = AfRxState_WaitGap;
    instance->rx_count = 0;
}

static void subghz_protocol_decoder_allstar_firefly_feed(
    void* context, bool level, uint32_t duration) {

    furi_assert(context);
    SubGhzProtocolDecoderAllstarFirefly* instance = context;

    if(level) {
        if(instance->rx_state == AfRxState_Receiving) {
            uint8_t sym;
            if(duration >= AF_LONG_PULSE_MIN && duration <= AF_LONG_PULSE_MAX) {
                sym = 1u;
            } else if(duration >= AF_SHORT_PULSE_MIN && duration <= AF_SHORT_PULSE_MAX) {
                sym = 0u;
            } else {
                instance->rx_state = AfRxState_WaitGap;
                instance->rx_count = 0;
                return;
            }
            if(instance->rx_count < AF_SYM_COUNT) {
                instance->rx_syms[instance->rx_count++] = sym;
            }
        }
    } else {
        if(duration >= AF_FRAME_THRESH_US) {
            if(instance->rx_state == AfRxState_Receiving &&
               instance->rx_count == AF_SYM_COUNT) {
                char decoded[AF_BIT_COUNT + 1];
                if(af_decode_symbols(instance->rx_syms, decoded)) {
                    memcpy(instance->dip, decoded, AF_BIT_COUNT + 1);
                    if(instance->base.callback) {
                        instance->base.callback(
                            &instance->base,
                            instance->base.context);
                    }
                }
                instance->rx_state = AfRxState_WaitGap;
                instance->rx_count = 0;
            } else if(instance->rx_state == AfRxState_WaitGap) {
                instance->rx_state = AfRxState_Receiving;
                instance->rx_count = 0;
            } else {
                instance->rx_state = AfRxState_WaitGap;
                instance->rx_count = 0;
            }
        }
    }
}

static uint8_t subghz_protocol_decoder_allstar_firefly_get_hash_data(void* context) {
    furi_assert(context);
    SubGhzProtocolDecoderAllstarFirefly* instance = context;
    uint8_t hash = 0;
    for(uint8_t i = 0; i < AF_BIT_COUNT; i++) hash ^= (uint8_t)instance->dip[i];
    return hash;
}
static SubGhzProtocolStatus subghz_protocol_decoder_allstar_firefly_serialize(
    void* context, FlipperFormat* flipper_format, SubGhzRadioPreset* preset) {

    furi_assert(context);
    SubGhzProtocolDecoderAllstarFirefly* instance = context;
    if(!flipper_format_write_uint32(flipper_format, "Frequency", &preset->frequency, 1)) {
        FURI_LOG_E(TAG, "Failed to write Frequency");
        return SubGhzProtocolStatusErrorParserOthers;
    }
    if(!flipper_format_write_string(flipper_format, "Preset", preset->name)) {
        FURI_LOG_E(TAG, "Failed to write Preset");
        return SubGhzProtocolStatusErrorParserOthers;
    }
    if(!flipper_format_write_string_cstr(
           flipper_format, "Protocol", SUBGHZ_PROTOCOL_ALLSTAR_FIREFLY_NAME)) {
        FURI_LOG_E(TAG, "Failed to write Protocol");
        return SubGhzProtocolStatusErrorParserOthers;
    }
    if(!flipper_format_write_string_cstr(flipper_format, "Key", instance->dip)) {
        FURI_LOG_E(TAG, "Failed to write Key");
        return SubGhzProtocolStatusErrorParserOthers;
    }
    return SubGhzProtocolStatusOk;
}

static SubGhzProtocolStatus subghz_protocol_decoder_allstar_firefly_deserialize(
    void* context, FlipperFormat* flipper_format) {

    furi_assert(context);
    SubGhzProtocolDecoderAllstarFirefly* instance = context;
    FuriString* key_str = furi_string_alloc();
    if(!flipper_format_read_string(flipper_format, "Key", key_str)) {
        FURI_LOG_E(TAG, "Missing Key field");
        furi_string_free(key_str);
        return SubGhzProtocolStatusErrorParserOthers;
    }
    const char* key_cstr = furi_string_get_cstr(key_str);
    if(strlen(key_cstr) != AF_BIT_COUNT || !af_dip_valid(key_cstr)) {
        FURI_LOG_E(TAG, "Invalid Key value: %s", key_cstr);
        furi_string_free(key_str);
        return SubGhzProtocolStatusErrorParserOthers;
    }
    memcpy(instance->dip, key_cstr, AF_BIT_COUNT + 1);
    furi_string_free(key_str);
    return SubGhzProtocolStatusOk;
}

static void subghz_protocol_decoder_allstar_firefly_get_string(
    void* context, FuriString* output) {

    furi_assert(context);
    SubGhzProtocolDecoderAllstarFirefly* instance = context;
    uint32_t code = af_dip_to_uint32(instance->dip);
    furi_string_cat_printf(
        output,
        "%s\r\n0x%04lX\r\n"
        "Freq: 318MHz  OOK\r\n"
        "1 2 3 4 5 6 7 8 9\r\n"
        "%c %c %c %c %c %c %c %c %c",
        SUBGHZ_PROTOCOL_ALLSTAR_FIREFLY_NAME,
        (unsigned long)code,
        instance->dip[0], instance->dip[1], instance->dip[2],
        instance->dip[3], instance->dip[4], instance->dip[5],
        instance->dip[6], instance->dip[7], instance->dip[8]);
}
static void* subghz_protocol_encoder_allstar_firefly_alloc(SubGhzEnvironment* environment) {
    UNUSED(environment);
    SubGhzProtocolEncoderAllstarFirefly* instance =
        malloc(sizeof(SubGhzProtocolEncoderAllstarFirefly));
    instance->base.protocol = &subghz_protocol_allstar_firefly;
    memset(instance->dip, '-', AF_BIT_COUNT);
    instance->dip[AF_BIT_COUNT] = '\0';
    instance->tx_size = 0;
    instance->tx_pos  = 0;
    return instance;
}

static void subghz_protocol_encoder_allstar_firefly_free(void* context) {
    furi_assert(context); free(context);
}

static void subghz_protocol_encoder_allstar_firefly_stop(void* context) {
    UNUSED(context);
}

static SubGhzProtocolStatus subghz_protocol_encoder_allstar_firefly_deserialize(
    void* context, FlipperFormat* flipper_format) {

    furi_assert(context);
    SubGhzProtocolEncoderAllstarFirefly* instance = context;
    FuriString* key_str = furi_string_alloc();
    if(!flipper_format_read_string(flipper_format, "Key", key_str)) {
        FURI_LOG_E(TAG, "Encoder: missing Key field");
        furi_string_free(key_str);
        return SubGhzProtocolStatusErrorParserOthers;
    }
    const char* key_cstr = furi_string_get_cstr(key_str);
    if(strlen(key_cstr) != AF_BIT_COUNT || !af_dip_valid(key_cstr)) {
        FURI_LOG_E(TAG, "Encoder: invalid Key: %s", key_cstr);
        furi_string_free(key_str);
        return SubGhzProtocolStatusErrorParserOthers;
    }
    memcpy(instance->dip, key_cstr, AF_BIT_COUNT + 1);
    furi_string_free(key_str);
    instance->tx_size = af_build_tx_buf(instance->dip, instance->tx_buf);
    instance->tx_pos  = 0;
    return SubGhzProtocolStatusOk;
}

static LevelDuration subghz_protocol_encoder_allstar_firefly_yield(void* context) {
    furi_assert(context);
    SubGhzProtocolEncoderAllstarFirefly* instance = context;
    if(instance->tx_pos >= instance->tx_size) {
        return level_duration_reset();
    }
    bool     lv  = (instance->tx_pos % 2 == 0);
    uint32_t dur = instance->tx_buf[instance->tx_pos++];
    return level_duration_make(lv, dur);
}
const SubGhzProtocolDecoder subghz_protocol_decoder_allstar_firefly = {
    .alloc          = subghz_protocol_decoder_allstar_firefly_alloc,
    .free           = subghz_protocol_decoder_allstar_firefly_free,
    .feed           = subghz_protocol_decoder_allstar_firefly_feed,
    .reset          = subghz_protocol_decoder_allstar_firefly_reset,
    .get_hash_data  = subghz_protocol_decoder_allstar_firefly_get_hash_data,
    .serialize      = subghz_protocol_decoder_allstar_firefly_serialize,
    .deserialize    = subghz_protocol_decoder_allstar_firefly_deserialize,
    .get_string     = subghz_protocol_decoder_allstar_firefly_get_string,
};

const SubGhzProtocolEncoder subghz_protocol_encoder_allstar_firefly = {
    .alloc       = subghz_protocol_encoder_allstar_firefly_alloc,
    .free        = subghz_protocol_encoder_allstar_firefly_free,
    .deserialize = subghz_protocol_encoder_allstar_firefly_deserialize,
    .stop        = subghz_protocol_encoder_allstar_firefly_stop,
    .yield       = subghz_protocol_encoder_allstar_firefly_yield,
};

const SubGhzProtocol subghz_protocol_allstar_firefly = {
    .name    = SUBGHZ_PROTOCOL_ALLSTAR_FIREFLY_NAME,
    .type    = SubGhzProtocolTypeStatic,
    .flag    = SubGhzProtocolFlag_AM |
               SubGhzProtocolFlag_Decodable |
               SubGhzProtocolFlag_Load |
               SubGhzProtocolFlag_Save |
               SubGhzProtocolFlag_Send,
    .decoder = &subghz_protocol_decoder_allstar_firefly,
    .encoder = &subghz_protocol_encoder_allstar_firefly,
};
