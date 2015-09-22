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
 *      \addtogroup demo_usniffer
 *      @{
*/
/*! \file   demo_usniffer.c

    \author Aleksei Turtsevich

    \brief  Uracoli Sniffer application

    \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
                                INCLUDE FILES
 =============================================================================*/
#include "demo_usniffer.h"
#include "tcpip.h"
#include "uip.h" 

//#define     LOGGER_ENABLE        LOGGER_DEMO_USNIFFER
#define     LOGGER_ENABLE        TRUE
#include    "logger.h"

/*==============================================================================
                            LOCAL FUNCTIONS
 =============================================================================*/

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
//static struct rime_sniffer usniffer = { NULL, demo_usniffInput, demo_usniffOutput };
sniffer_context_t   ctx;
struct etimer       ctrl_et;
pcap_pool_t         PcapPool;
static s_ns_t*      pst_ns = NULL;
RIME_SNIFFER(usniffer, demo_usniffInput, demo_usniffOutput);

/*==============================================================================
                                API FUNCTIONS
 =============================================================================*/
/*----------------------------------------------------------------------------*/
/*    demo_usniffInit()                                                       */
/*----------------------------------------------------------------------------*/

s_ns_t* demo_usniffGetStackInstance(void)
{
    return pst_ns;
}

int8_t demo_usniffInit(void)
{
//uint8_t val;

    LOG_INFO("Starting Uracoli Sniffer");

    if ((pst_ns->inif != NULL) &&
        (pst_ns->inif->set_promisc != NULL)) {
        pst_ns->inif->set_promisc(1);
    }


    /* Phase 1: initialize MCU peripherals */
    //--- LED_INIT();

    /* init memory locations */
    memset(&ctx, 0, sizeof(ctx));
    PcapPool.ridx = 0;
    PcapPool.widx = 0;
    ctx.state = IDLE;
    ctx.cchan = TRX_MIN_CHANNEL;
    ctx.cmask = TRX_SUPPORTED_CHANNELS;

    //--- LED_SET_VALUE(1);

    /* initialize MCU ressources */
    //--- timer_init();
    //--- hif_init(HIF_DEFAULT_BAUDRATE);

    //--- TODO: It is in demo_main.c
    //--- bsp_init();
    //--- hal_init();
    //--- _hal_usartInit();
    //--- _hal_tim0Init();
    //--- _hal_ledsInit();

    //--- LED_SET_VALUE(1);

    //--- it is in transceiver initialization
    /* initialize transceiver */
    //--- trx_io_init(DEFAULT_SPI_RATE);
    //--- trx_init();

    /* done with init */
    printf(NL"Sniffer V%s [%s]"NL, VERSION, BOARD_NAME);


    //--- TODO: Init


    /* emb6 running usniff main function */
    etimer_set(&ctrl_et, 1 * bsp_get(E_BSP_GET_TRES), demo_usniffMain);

    /* add uracoli sniffer to a sniffers queue */
    rime_sniffer_add(&usniffer);

    /* TODO: */
    ctrl_process_init();

    return 1;
} /* demo_usniffInit() */

/*----------------------------------------------------------------------------*/
/*    demo_usniffConf()                                                       */
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
            pst_netStack->lmac   = &nullrdc_driver;
            pst_netStack->frame  = &no_framer;
            pst_netStack->c_configured = 1;
            /* Transceiver interface is defined by @ref board_conf function*/
            /* pst_netStack->inif   = $<some_transceiver>;*/
        } else {
            if ((pst_netStack->hc == &sicslowpan_driver)   &&
                (pst_netStack->llsec == &nullsec_driver)   &&
                (pst_netStack->hmac == &nullmac_driver)    &&
                (pst_netStack->lmac == &nullrdc_driver) &&
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

} /*  demo_usniffConf() */


void demo_usniffMain(c_event_t c_event, p_data_t p_data)
{

    if (!etimer_expired(&ctrl_et))
        return;

    ctrl_process_input();

    if(ctx.state == SCAN_DONE)
    {
        //scan_update_status();
    }
    if ((ctx.state == SNIFF) && (PcapPool.widx != PcapPool.ridx))
    {
        uint8_t tmp, len, *p;
        pcap_packet_t *ppcap = &PcapPool.packet[PcapPool.ridx];
        //--- hif_putc(1);
        putchar(1);
        #if 0
            hif_put_blk((uint8_t*)ppcap, ppcap->len+1);
        #else
            len = ppcap->len+1;
            p = (uint8_t*)ppcap;
            do
            {
                //--- TODO: change hif_put_blk
                //tmp = hif_put_blk(p, len);
                tmp = 10;
                p += tmp;
                len -= tmp;
            }
            while(len>0);
        #endif
        //--- hif_putc(4);
        putchar(4);
        /* mark buffer as processed */
        ppcap->len = 0;
        PcapPool.ridx++;
        PcapPool.ridx &= (MAX_PACKET_BUFFERS-1);
    }

    etimer_restart(&ctrl_et);
    return;
} /* demo_usniffMain() */



void demo_usniffStart(sniffer_state_t state)
{
    switch (state)
    {
        case IDLE:
            // TODO: the same thing that function below does
            // trx_reg_write(RG_TRX_STATE, CMD_FORCE_TRX_OFF);
            //--- TODO: RemoveSniffer
            ctx.state = IDLE;
            // LED_SET_VALUE(0);
            break;
        case SCAN:
            ctx.state = SCAN;
            //--- TODO: SCAN
            //--- scan_init();
            break;
        case SNIFF:
            // trx_reg_write(RG_TRX_STATE, CMD_RX_ON);
            rime_sniffer_add(&usniffer);
            ctx.state = SNIFF;
            break;

        default:
            break;
    }
} /* demo_usniffStart() */

void demo_usniffStop(void)
{
sniffer_state_t curr_state;

    //trx_reg_write(RG_TRX_STATE, CMD_FORCE_TRX_OFF);
    rime_sniffer_remove(&usniffer);
    cli();
    curr_state = ctx.state;
    ctx.state = IDLE;
    sei();

    switch(curr_state)
    {
        case SCAN:
            ctx.cchan = TRX_MIN_CHANNEL;
            //--- ctx.thdl = utimer_stop(ctx.thdl);
            break;
        case SNIFF:
        case IDLE:
            break;
        default:
            //--- PRINTF("Unknown state %d"NL,ctx.state);
            printf("Unknown state %d"NL,ctx.state);
            break;
    }
} /* demo_usniffStop() */


void demo_usniffInput(void)
{
    static pcap_packet_t *ppcap;
    uint8_t ed, flen, lqi = 0;
    bool crc_ok = 1; // crc_ok = 0

    ppcap = &PcapPool.packet[PcapPool.widx];
    if (ppcap->len != 0)
    {
        /* drop packet, no free buffers*/
        // ppcap = NULL;
        return;
    }
    ppcap->ts.time_usec = 0; // TRX_TSTAMP_REG;
    ppcap->ts.time_sec = 0;  // systime;


    ed = 0;                  // trx_reg_read(RG_PHY_ED_LEVEL);
    flen = uip_datalen();    // trx_frame_read_crc(&ppcap->frame[0], MAX_FRAME_SIZE, &crc_ok);
    lqi = 0;                 // trx_sram_read(flen, 1, &lqi);
    
    memcpy(&ppcap->frame[0], uip_appdata, MAX_FRAME_SIZE); // TODO: perhaps size could be more
    if (ctx.state == SCAN)
    {
        scan_update_frame(flen, crc_ok, lqi, ed, ppcap->frame);
    }
    if (ctx.state == SNIFF)
    {
        ppcap->len = flen + sizeof(time_stamp_t);
        PcapPool.widx++;
        PcapPool.widx &= (MAX_PACKET_BUFFERS-1);
    }

    ctx.frames++;
    //LED_SET_VALUE(ctx.frames);

    return;
} /* demo_usniffIntput() */



void demo_usniffOutput(int mac_status)
{
    /* TODO: a blank function */
    return;
} /* demo_usniffOutput() */



/** @} */
/** @} */
/** @} */
