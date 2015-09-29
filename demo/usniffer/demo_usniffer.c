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
 *      \addtogroup demo
 *      @{
 *      \addtogroup demo_sniffer
 *      @{
*/
/*! \file   demo_sniffer.c

 \author Aleksei Turtsevich

 \brief  Just Print application

 \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/
#include "demo_usniffer.h"
//#include "const.h"

#define     LOGGER_ENABLE        FALSE
#include    "logger.h"

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
struct etimer print_et;
sniffer_context_t ctx;
pcap_pool_t PcapPool;
/*==============================================================================
                                         API FUNCTIONS
 =============================================================================*/
/*----------------------------------------------------------------------------*/
/*    demo_usnifferInit()                                                     */
/*----------------------------------------------------------------------------*/
int8_t demo_usniffInit(void)
{

    s_ns_t* pst_ns = NULL;
    pst_ns = emb6_get();

    if ((pst_ns == NULL) || (pst_ns->inif == NULL))
        return 0;

    /* init memory locations */
    memset(&ctx, 0, sizeof(ctx));
    PcapPool.ridx = 0;
    PcapPool.widx = 0;
    ctx.state = IDLE;
    ctx.cchan = TRX_MIN_CHANNEL;
    ctx.cmask = TRX_SUPPORTED_CHANNELS;

    if ((pst_ns->inif != NULL) &&
        (pst_ns->inif->ioctl != NULL)) {
        pst_ns->inif->ioctl(E_RADIO_PROMISC, 1);
    }

    if (pst_ns->lmac != NULL) {
        pst_ns->lmac->regcallb(demo_usniffer_input_frame);
    }
    
    /* Register callback for U(S)ART Int */
    bsp_extIntInit(E_TARGET_USART_INT, demo_usniffer_input_byte);

    /* Register ETimer callback */
    etimer_set(&print_et, 0.3 * bsp_get(E_BSP_GET_TRES), demo_usniffer_et_callback);

    printf(NL"Sniffer V%s [%s]"NL, VERSION, BOARD_NAME);

    return 1;
} /* demo_usnifferInit() */

/*----------------------------------------------------------------------------*/
/*    demo_usnifferConf()                                                     */
/*----------------------------------------------------------------------------*/
uint8_t demo_usniffConf(s_ns_t* pst_netStack)
{
    uint8_t c_ret = 1;

    /*
     * By default stack
     */
    if (pst_netStack != NULL) {
        if (!pst_netStack->c_configured) {
            pst_netStack->hc     = &sicslowpan_driver;
            pst_netStack->llsec  = &nullsec_driver;
            pst_netStack->hmac   = &nullmac_driver;
            pst_netStack->lmac   = &sniffer_driver;
            pst_netStack->frame  = &no_framer;
            pst_netStack->c_configured = 1;
            /* Transceiver interface is defined by @ref board_conf function*/
            /* pst_netStack->inif   = $<some_transceiver>;*/
        } else {
            if ((pst_netStack->hc == &sicslowpan_driver)   &&
                (pst_netStack->llsec == &nullsec_driver)   &&
                (pst_netStack->hmac == &nullmac_driver)    &&
                (pst_netStack->lmac == &sniffer_driver) &&
                (pst_netStack->frame == &no_framer)) {
                /* right configuration */
            }
            else {
                pst_netStack = NULL;
                c_ret = 0;
            }
        }
    }
    return (c_ret);
} /*  demo_usnifferConf() */


/*----------------------------------------------------------------------------*/
/*    demo_usniffer_et_callback()                                             */
/*----------------------------------------------------------------------------*/
void demo_usniffer_et_callback(c_event_t c_event, p_data_t p_data)
{
    if (!etimer_expired(&print_et))
        return;
    
    LOG_INFO("Timeout!\r");

    #if 1
    if ((ctx.state == SNIFF) && (PcapPool.widx != PcapPool.ridx))
    {
        uint8_t tmp, len, *p;
        pcap_packet_t *ppcap = &PcapPool.packet[PcapPool.ridx];
        
        /* start frame delimiter */
        printf("%c", 1);

        len = ppcap->len+1;
        p = (uint8_t*)ppcap;
        do
        {
            printf("%c", *p);
            p++;
            len--;
        }
        while(len>0);

        /* end frame delimiter */
        printf("%c", 4);

        /* mark buffer as processed */
        ppcap->len = 0;
        PcapPool.ridx++;
        PcapPool.ridx &= (MAX_PACKET_BUFFERS-1);
    }
    #endif

    etimer_restart(&print_et);
    
} /* demo_usniffer_et_callback() */


/*----------------------------------------------------------------------------*/
/*    demo_usniffer_input_frame()                                             */
/*----------------------------------------------------------------------------*/
void demo_usniffer_input_frame(const uint8_t* pc_data, uint8_t c_len,
                                int8_t status)
{
    static pcap_packet_t *ppcap;

    ppcap = &PcapPool.packet[PcapPool.widx];
    if (ppcap->len != 0)
    {
        /* drop packet, no free buffers*/
        ppcap = NULL;
        return;
    }

    ppcap->ts.time_usec = TRX_TSTAMP_REG; // temporary
    ppcap->ts.time_sec = bsp_get(E_BSP_GET_TICK);

    memcpy(&ppcap->frame[0], pc_data, c_len);
    if (ctx.state == SNIFF)
    {
        ppcap->len = c_len + sizeof(time_stamp_t);
        PcapPool.widx++;
        PcapPool.widx &= (MAX_PACKET_BUFFERS-1);
    }

    #if 0
    /* Upload frame at TRX_END IRQ */
    if (ppcap != NULL)
    {
        memcpy(&ppcap->frame[0], pc_data, c_len);
        if (ctx.state == SNIFF)
        {
            ppcap->len = c_len + sizeof(time_stamp_t);
            PcapPool.widx++;
            PcapPool.widx &= (MAX_PACKET_BUFFERS-1);
        }
    }
    #endif
    ctx.frames++;

} /* demo_usniffer_input_frame() */


/*----------------------------------------------------------------------------*/
/*    demo_usniffer_input_byte()                                              */
/*----------------------------------------------------------------------------*/
void demo_usniffer_input_byte(void * chr)
{
    //--- char c = *(char*)chr;
    demo_usniffProcessInput(chr);
} /* demo_usniffer_input_byte() */


/**
 * @brief Start a new operating state.
 */
void demo_usniffer_start(sniffer_state_t state)
{
    switch (state)
    {
        case IDLE:
            //trx_reg_write(RG_TRX_STATE, CMD_FORCE_TRX_OFF);
            ctx.state = IDLE;
            //LED_SET_VALUE(0);
            break;
        case SCAN:
            ctx.state = SCAN;
            //scan_init();
            break;
        case SNIFF:
            //trx_reg_write(RG_TRX_STATE, CMD_RX_ON);
            ctx.state = SNIFF;
            break;

        default:
            break;
    }
}

/**
 * @brief Halt current operation and enter state IDLE.
 */
void demo_usniffer_stop(void)
{
sniffer_state_t curr_state;

    //cli();
    curr_state = ctx.state;
    ctx.state = IDLE;
    //sei();

    switch(curr_state)
    {
        case SCAN:
            ctx.cchan = TRX_MIN_CHANNEL;
            //ctx.thdl = timer_stop(ctx.thdl);
            break;
        case SNIFF:
        case IDLE:
            break;
        default:
            printf("Unknown state %d"NL,ctx.state);
            break;

    }
}


/** @} */
/** @} */
/** @} */
