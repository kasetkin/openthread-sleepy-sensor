#pragma once

#include <stdint.h>
#include "openthread/ip6.h"

// Thread specification §5.5.2 — Anycast Locator (ALOC) and Routing Locator (RLOC) addressing.
//
// All Thread locator addresses share the same IID pattern: 0000:00ff:fe00:xxxx
// where xxxx is the RLOC16 or ALOC16 value.
//
// In terms of otIp6Address.mFields.m8, the address layout is:
//   [0..7]  — mesh-local prefix (8 bytes from the active dataset)
//   [8..9]  — 0x00 0x00
//   [10..11]— 0x00 0xff
//   [12..13]— 0xfe 0x00
//   [14..15]— locator16 in big-endian (RLOC16 or ALOC16)

// Fixed bytes [8..13] common to every Thread locator address (RLOC and ALOC).
static constexpr uint8_t OT_LOCATOR_IID_PREFIX[] = {0x00, 0x00, 0x00, 0xff, 0xfe, 0x00};

// ALOC16 values — Thread spec Table 5-7.
static constexpr uint16_t OT_ALOC16_LEADER = 0xfc00; ///< Current partition leader.
