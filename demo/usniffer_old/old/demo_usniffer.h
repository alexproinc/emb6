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
 *   \addtogroup emb6
 *   @{
 *   \addtogroup demo
 *   @{
 *   \addtogroup demo_usniffer
 *
 *   Uracoli Sniffer application [should be in promiscuous mode].
 *   @{
*/
/*! \file   demo_usniffer.h

    \author Aleksei Turtsevich

    \brief  This is the header file of the uracoli sniffer application

    \version 0.0.1
*/
/*==============================================================================
                                   INCLUDES
==============================================================================*/
#include "emb6.h"
#include "emb6_conf.h"
#include "bsp.h"
#include "rime.h"  
#include "evproc.h"
#include "etimer.h"

#include <avr/pgmspace.h>
#include <util/crc16.h>
#include <avr/interrupt.h>

//#include "transceiver.h"
//#include "ioutil.h"
//#include "utimer.h"
#ifndef DEMO_USNIFFER_H_
#define DEMO_USNIFFER_H_


/*==============================================================================
                                    MACROS
==============================================================================*/



#define F_CPU 8000000UL




/** macro that forces an enumeration to use 8 bit instead of 16 bit integers. */
#define SHORTENUM  __attribute__((packed))

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
 * Data type for a timer handle
 * (a reference number to identify a running timer).
 */
typedef uint16_t timer_hdl_t;

/** transceiver channel type */
typedef int8_t  channel_t;

/**
 * Data type for the argument of a timer handler function.
 */
typedef uint32_t timer_arg_t;

/**
 * Data type for timer expiration action function.
 * This function is called, when the expiration time is over.
 * When luanched, the function is called with a parameter p of
 * type .@ref timer_arg_t. If the function returns a value,
 * which is greate then 0, the timer is restarted again.
 */
typedef time_t (timer_handler_t)(timer_arg_t p);







//--- For at86rf212
#define BOARD_NAME "any900st"
#define RADIO_TYPE (RADIO_AT86RF212)

/** lowest supported channel number */
#define TRX_MIN_CHANNEL (0)

/** highest supported channel number */
#define TRX_MAX_CHANNEL (10)

/** number of channels */
#define TRX_NB_CHANNELS (11)

/**
 * @brief Mask for supported channels of this radio.
 * The AT86RF212 supports channels 0 ... 10 of IEEE 802.15.4
 * (currently no support for free configurable frequencies here)
 */
#define TRX_SUPPORTED_CHANNELS  (0x00007ffUL)







#define SCAN_PERIOD_MS  (2000)
#define NL "\n"
#define CHANNEL_OFFSET(x)  (x > TRX_MAX_CHANNEL ? TRX_MIN_CHANNEL : (x - TRX_MIN_CHANNEL))
#define CHANNEL_MAX_OFFSET (TRX_NB_CHANNELS-1)
#define VERSION "0.1"



#ifndef MAX_PACKET_BUFFERS
# define MAX_PACKET_BUFFERS (8)
#endif

/** Maximum size in bytes of an IEEE 802.15.4 frame */
#ifndef MAX_FRAME_SIZE
# define MAX_FRAME_SIZE (127)
#endif



/*==============================================================================
                                    TYPES
==============================================================================*/
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
    timer_hdl_t     thdl;
    /** scan period */
    time_t          scanper;
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

/*==============================================================================
                                  EXTERNALS
==============================================================================*/
extern sniffer_context_t ctx;

/*==============================================================================
                               INLINE FUNCTIONS
==============================================================================*/
/**
 * @brief update the scan table for a channel.
 */
static inline void scan_update_frame(uint8_t flen, bool crc_ok, uint8_t lqi, uint8_t ed, uint8_t *rxbuf)
{
scan_result_t *scres;

     scres = &ctx.scanres[(ctx.cchan - TRX_MIN_CHANNEL)];
     scres->framecnt ++;
     scres->edsum +=ed;
     scres->lqisum += lqi;

     if (flen < 0x80)
     {
         /* process valid frame length */
         if (crc_ok == true)
         {
             scres->crc_ok++;
             scres->ftypes[rxbuf[0]&7] ++;
         }
         /* parse beacon */
     }
}

/*==============================================================================
                         FUNCTION PROTOTYPES OF THE API
==============================================================================*/
#ifdef __cplusplus
extern "C" {
#endif
/*============================================================================*/
/*!
    \brief Get Instance Of Stack used by the uracoli sniffer application.

*/
/*============================================================================*/
s_ns_t* demo_usniffGetStackInstance(void);

/*============================================================================*/
/*!
    \brief Initialization of the uracoli sniffer application.

*/
/*============================================================================*/
int8_t demo_usniffInit(void);



/*============================================================================*/
/*!
    \brief Start the uracoli sniffer application.

*/
/*============================================================================*/
void demo_usniffStart(sniffer_state_t state);


/*============================================================================*/
/*!
    \brief Stop the uracoli sniffer application.

*/
/*============================================================================*/
void demo_usniffStop(void);

/*============================================================================*/
/*!
    \brief The input data for the uracoli sniffer application.

*/
/*============================================================================*/
void demo_usniffInput(void);

/*============================================================================*/
/*!
    \brief The output data for the uracoli sniffer application.

*/
/*============================================================================*/
void demo_usniffOutput(int mac_status);

/*============================================================================*/
/*!
    \brief Configuration of the uracoli sniffer application.

    \return 0 - error, 1 - success
*/
/*============================================================================*/
uint8_t demo_usniffConf(s_ns_t* pst_netStack);


/*============================================================================*/
/*!
    \brief Control process input of the uracoli sniffer application.

*/
/*============================================================================*/
void demo_usniffMain(c_event_t c_event, p_data_t p_data);


//--- void scan_init(void);
void ctrl_process_init(void);
void ctrl_process_input(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* DEMO_USNIFFER_H_ */
/** @} */
/** @} */
/** @} */
