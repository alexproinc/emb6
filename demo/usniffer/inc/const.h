/* Copyright (c) 2007 - 2013
    Marco Arena
    Axel Wachtler

   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of the authors nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE. */

/* $Id$ */
/**
 * @file
 * @brief General Definitions.
 *
 */
/*====================================================================*/
#ifndef URACOLI_CONST_H
#define URACOLI_CONST_H
#include <stdint.h>
#include <stdbool.h>

/*=== board type constants ===================================================*/
/**
 * @addtogroup grpBoard
 * @{
 */

#define BOARD_NONE (0)

/* the legacy boards are sorted alphabetically */
#define BOARD_ANY2400 (1)
#define BOARD_ANY2400ST (2)
#define BOARD_ANY900 (3)
#define BOARD_ANY900ST (4)
#define BOARD_BAT (5)
#define BOARD_BITBEAN (6)
#define BOARD_CBB212 (7)
#define BOARD_CBB230 (8)
#define BOARD_CBB230B (9)
#define BOARD_CBB231 (10)
#define BOARD_CBB232 (11)
#define BOARD_CBB233 (12)
#define BOARD_DERFA1 (13)
#define BOARD_DERFTORCBRFA1 (14)
#define BOARD_DRACULA (15)
#define BOARD_IBDT212 (16)
#define BOARD_IBDT231 (17)
#define BOARD_IBDT232 (18)
#define BOARD_ICM230_11 (19)
#define BOARD_ICM230_12A (20)
#define BOARD_ICM230_12B (21)
#define BOARD_ICM230_12C (22)
#define BOARD_ICS230_11 (23)
#define BOARD_ICS230_12 (24)
#define BOARD_ICT230 (25)
#define BOARD_IM240A (26)
#define BOARD_IM240A_EVAL (27)
#define BOARD_LGEE231 (28)
#define BOARD_LGEE231_V2 (29)
#define BOARD_MIDGEE (30)
#define BOARD_MNB900 (31)
#define BOARD_MUSE231 (32)
#define BOARD_MUSEII232 (33)
#define BOARD_MUSEIIRFA (34)
#define BOARD_PINOCCIO (35)
#define BOARD_PSK212 (36)
#define BOARD_PSK230 (37)
#define BOARD_PSK230B (38)
#define BOARD_PSK231 (39)
#define BOARD_PSK232 (40)
#define BOARD_PSK233 (41)
#define BOARD_RADIOFARO (42)
#define BOARD_RADIOFARO_V1 (43)
#define BOARD_RAVRF230A (44)
#define BOARD_RAVRF230B (45)
#define BOARD_RBB128RFA1 (46)
#define BOARD_RBB212 (47)
#define BOARD_RBB230 (48)
#define BOARD_RBB230B (49)
#define BOARD_RBB231 (50)
#define BOARD_RBB232 (51)
#define BOARD_RBB233 (52)
#define BOARD_RDK212 (53)
#define BOARD_RDK230 (54)
#define BOARD_RDK230B (55)
#define BOARD_RDK231 (56)
#define BOARD_RDK232 (57)
#define BOARD_RDK233 (58)
#define BOARD_ROSE231 (59)
#define BOARD_RZUSB (60)
#define BOARD_STB128RFA1 (61)
#define BOARD_STB212 (62)
#define BOARD_STB230 (63)
#define BOARD_STB230B (64)
#define BOARD_STB231 (65)
#define BOARD_STB232 (66)
#define BOARD_STB233 (67)
#define BOARD_STKM16 (68)
#define BOARD_STKM8 (69)
#define BOARD_TINY230 (70)
#define BOARD_TINY231 (71)
#define BOARD_WDBA1281 (72)
#define BOARD_WPROG (73)
#define BOARD_XXO (74)
#define BOARD_ZGBH212 (75)
#define BOARD_ZGBH230 (76)
#define BOARD_ZGBH231 (77)
#define BOARD_ZGBL212 (78)
#define BOARD_ZGBL230 (79)
#define BOARD_ZGBL231 (80)
#define BOARD_ZGBT1281A2NOUART (81)
#define BOARD_ZGBT1281A2UART0 (82)
#define BOARD_ZGBT1281A2UART1 (83)
#define BOARD_ZIGDUINO (84)

/* beyond this point the boards are added chronologically */
#define BOARD_STB256RFR2 (85)
#define BOARD_RASPBEE (86)
// leave a gap for boards on mainline.
#define BOARD_DERFA2 (91)
#define BOARD_DERFA2P (92)

#define BOARD_LAST (BOARD_RASPBEE)
/** @} */

/**
 * @addtogroup grpRadio
 * @{
 */

/*=== radio constants ================================================*/
#define  RADIO_AT86RF230 (1) /**< Identifier for radio AT86RF230 */
#define  RADIO_AT86RF230A (RADIO_AT86RF230) /**< Identifier for radio AT86RF230 Rev A */
#define  RADIO_AT86RF230B (2) /**< Identifier for radio AT86RF230 Rev B */
#define  RADIO_AT86RF231 (3) /**< Identifier for radio AT86RF231 */
#define  RADIO_AT86RF212 (4) /**< Identifier for radio AT86RF212 */
#define  RADIO_ATMEGA128RFA1_A (5) /**< Identifier for radio ATmega128RFA1 Rev. A */
#define  RADIO_ATMEGA128RFA1_B (6) /**< Identifier for radio ATmega128RFA1 Rev. B */
#define  RADIO_ATMEGA128RFA1_C (7) /**< Identifier for radio ATmega128RFA1 Rev. C */
#define  RADIO_ATMEGA128RFA1_D (8) /**< Identifier for radio ATmega128RFA1 Rev. D */
#define  RADIO_AT86RF232 (9) /**< Identifier for radio AT86RF232 */
#define  RADIO_AT86RF233 (10) /**< Identifier for radio AT86RF233 */
#define  RADIO_ATMEGA256RFR2 (11) /**< Identifier for radio ATmega128RFR2 */


#if defined (DOXYGEN)
/** macro that forces an enumeration to use 8 bit instead of 16 bit integers. */
#define SHORTENUM
#else
#define SHORTENUM  __attribute__((packed))
#endif

/**
 * @addtogroup grpTrx
 * @{
 */

/** transceiver channel type */
typedef int8_t  channel_t;

/** @} */
#endif  /* #ifndef URACOLI_CONST_H */
