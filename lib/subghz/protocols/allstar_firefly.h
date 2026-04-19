#pragma once

/**
 * allstar_firefly.h  -  Allstar Firefly 318ALD31K native SubGHz protocol
 *
 * Registers the Allstar Firefly gate remote as a first-class SubGHz protocol,
 * supporting decode from air, save/load of .sub files, and retransmit.
 *
 * Protocol summary (measured from captured remotes):
 *   Carrier   : 318 MHz OOK  (FuriHalSubGhzPresetOok650Async)
 *   Code type : Static 9-bit trinary  (Supertex ED-9 encoder IC)
 *   Frame     : 18 symbols (2 per bit), no separate sync pulse
 *   Repeats   : 20 frames per keypress
 *   Inter-frame gap: ~30 440 us
 *
 *   Symbol encoding - each bit = two (pulse, gap) pairs:
 *     '+'  ON    : H H  = [4045us HIGH, 607us LOW] x2
 *     '-'  OFF   : L L  = [530us HIGH, 4139us LOW] x2
 *     '0'  FLOAT : H L  = [4045us HIGH, 607us LOW, 530us HIGH, 4139us LOW]
 *
 * Save file format:
 *   Filetype: Flipper SubGhz Key File
 *   Version: 1
 *   Frequency: 318000000
 *   Preset: FuriHalSubGhzPresetOok650Async
 *   Protocol: Allstar Firefly
 *   Key: +--000++-
 *
 * To register with the SubGHz app, in protocol_list.c add:
 *   #include "allstar_firefly.h"
 *   &subghz_protocol_allstar_firefly,   (in the protocol array)
 */

#include <lib/subghz/protocols/base.h>

/* Protocol name (must match what is written to .sub files) */
#define SUBGHZ_PROTOCOL_ALLSTAR_FIREFLY_NAME  "Allstar Firefly"

/* Timing constants (from 37-frame analysis of two captures) */
#define AF_FREQ              318000000UL
#define AF_BIT_COUNT         9u
#define AF_SYM_COUNT         18u

#define AF_LONG_PULSE_US     4045u
#define AF_SHORT_PULSE_US    530u
#define AF_SHORT_GAP_US      607u
#define AF_LONG_GAP_US       4139u
#define AF_INTERFRAME_US     30440u

#define AF_SHORT_PULSE_MIN   300u
#define AF_SHORT_PULSE_MAX   1200u
#define AF_LONG_PULSE_MIN    2500u
#define AF_LONG_PULSE_MAX    5500u
#define AF_FRAME_THRESH_US   20000u
#define AF_TX_REPEAT         20u
#define AF_TX_BUF_SIZE       (AF_TX_REPEAT * AF_SYM_COUNT * 2u + 8u)

/* Protocol vtable entries */
extern const SubGhzProtocolDecoder subghz_protocol_decoder_allstar_firefly;
extern const SubGhzProtocolEncoder subghz_protocol_encoder_allstar_firefly;
extern const SubGhzProtocol        subghz_protocol_allstar_firefly;
