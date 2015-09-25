/*
 * emb6 is licensed under the 3-clause BSD license. This license gives everyone
 * the right to use and distribute the code, either in binary or source code
 * format, as long as the copyright license is retained in the source code.
 *
 * The emb6 is derived from the Contiki OS platform with the explicit approval
 * from Adam Dunkels. However, emb6 is made independent from the OS through the
 * removal of protothreads. In addition, APIs are made more flexible to gain
 * more adaptivity during run-time.
 *
 * The license text is:
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*============================================================================*/
/**
 *      \addtogroup emb6
 *      @{
 *   \addtogroup demo
 *   @{
 *   \addtogroup demo_sniffer
 *
 *   Sniffer application [should be in promiscuous mode].
 *   @{
*/
/*! \file   demo_sniffer.h

    \author Artem Yushev, 

    \brief  This is the header file of the sniffer application

    \version 0.0.1
*/

#include "emb6.h"
#include "emb6_conf.h"
#include "bsp.h"
#include "evproc.h"
#include "etimer.h"

#include "transceiver.h"
#include "ioutil.h"
#include "const.h"

#ifndef DEMO_USNIFFER_H_
#define DEMO_USNIFFER_H_



#define SCAN_PERIOD_MS  (2000)
#define NL "\n"
#define CHANNEL_OFFSET(x)  (x > TRX_MAX_CHANNEL ? TRX_MIN_CHANNEL : (x - TRX_MIN_CHANNEL))
#define CHANNEL_MAX_OFFSET (TRX_NB_CHANNELS-1)
#define VERSION "0.1"
/**
 * Increment channel number and wraps back to lowest channel, if
 * upper channel + 1 is reached.
 */
#define CHANNEL_NEXT_CIRCULAR(x) \
    do \
    { \
        (x)++; \
        if ((x) > TRX_MAX_CHANNEL) \
        { \
            (x) = TRX_MIN_CHANNEL; \
        } \
    } \
    while(0)


#ifndef MAX_PACKET_BUFFERS
# define MAX_PACKET_BUFFERS (8)
#endif




/**
 * Data type for time values (measured in number of system ticks).
 */
typedef uint32_t time_t;


/**
 * Hi resolution time stamp in seconds and micro seconds
 */
typedef struct time_stamp_tag
{
    time_t  time_sec;
    time_t  time_usec;
} time_stamp_t;




/**
 * @brief Appication States.
 */
typedef enum
{
    /** Application is in idle mode, receiving commands from HIF. */
    IDLE,
    /** Application is in scanning mode. */
    EDSCAN,
    /** Application is in scanning mode. */
    SCAN,
    /** Application is in scanning mode. */
    SCAN_DONE,
    /** Application is in sniffing mode. */
    SNIFF
} SHORTENUM sniffer_state_t;

/**
 * @brief Data structure for scan results.
 */
typedef struct scan_result_tag
{
    /** total number of received frames */
    uint16_t framecnt;
    /** total number of frames with valid crc */
    uint16_t crc_ok;
    uint16_t edsum;
    uint16_t lqisum;
    uint16_t ftypes[8];
} scan_result_t;

/**
 * @brief Data structure for internal state variables of
 * the application.
 */
typedef struct sniffer_context_tag
{
    /** state */
    volatile sniffer_state_t state;
    /** current channel */
    channel_t cchan;
    /** current channel page */
    uint8_t cpage;
    /** mask for scanned channels */
    uint32_t cmask;

    /** upload only frames with CRC ok if set to 1*/
    bool chkcrc;

    /** timer handle */
    //--- timer_hdl_t     thdl;
    /** scan period */
    //--- time_t          scanper;
    /** table for scan results */
    scan_result_t   scanres[TRX_NB_CHANNELS];
    uint8_t         scanres_reset;
    uint16_t frames;
    uint16_t irq_ur;
    uint16_t missed_frames;
} sniffer_context_t;

typedef struct pcap_packet_tag
{
    /** length value (frame length + sizeof(time_stamp_t)) */
    uint8_t len;
    /** time stamp storage */
    time_stamp_t ts;
    /** buffer that holds a frame with maximum length */
    uint8_t frame[MAX_FRAME_SIZE];
} pcap_packet_t;


typedef struct pcap_pool_tag
{
    volatile uint8_t ridx;
    volatile uint8_t widx;
    pcap_packet_t packet[MAX_PACKET_BUFFERS];
} pcap_pool_t;



extern sniffer_context_t ctx;



/*==============================================================================
                         FUNCTION PROTOTYPES OF THE API
==============================================================================*/

/*============================================================================*/
/*!
   \brief Initialization of the sniffer application.

*/
/*============================================================================*/
int8_t demo_usniffInit(void);

/*============================================================================*/
/*!
    \brief Configuration of the sniffer application.

    \return 0 - error, 1 - success
*/
/*============================================================================*/
uint8_t demo_usniffConf(s_ns_t* pst_netStack);

void demo_usniffer_start(sniffer_state_t state);
void demo_usniffer_stop(void);


void demo_usniffer_et_callback(c_event_t c_event, p_data_t p_data);
void demo_usniffer_input_frame(const uint8_t* pc_data, uint8_t c_len,
                                int8_t status);
void demo_usniffer_input_byte(void * chr);
void demo_usniffProcessInput(void * chr);

#endif /* DEMO_COAP_SRV_H_ */
/** @} */
/** @} */
/** @} */
