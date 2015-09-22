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

#define     LOGGER_ENABLE        TRUE
#include    "logger.h"

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
s_ns_t* pst_ns = NULL;
struct etimer print_et;
/*==============================================================================
                                         API FUNCTIONS
 =============================================================================*/
/*----------------------------------------------------------------------------*/
/*    demo_usnifferInit()                                                           */
/*----------------------------------------------------------------------------*/

int8_t demo_usniffInit(void)
{

    LOG_INFO("Starting Sniffer");

    if ((pst_ns->inif != NULL) &&
        (pst_ns->inif->set_promisc != NULL)) {
        pst_ns->inif->set_promisc(1);
    }

    if (pst_ns->lmac != NULL) {
        pst_ns->lmac->regcallb(demo_usniffer_input_frame);
    }
    /*  */
    //bsp_extIntInit(E_TARGET_RADIO_INT, demo_usniffer_input_frame);
    
    /*  */
    bsp_extIntInit(E_TARGET_USART_INT, demo_usniffer_input_byte);

    /*  */
    etimer_set(&print_et, 1 * bsp_get(E_BSP_GET_TRES), demo_usniffer_et_callback);


    return 1;
} /* demo_usnifferInit() */

/*----------------------------------------------------------------------------*/
/*    demo_usnifferConf()                                                           */
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
    pst_ns = pst_netStack;
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

    etimer_restart(&print_et);
    
} /* demo_usniffer_et_callback() */


/*----------------------------------------------------------------------------*/
/*    demo_usniffer_input_frame()                                             */
/*----------------------------------------------------------------------------*/
void demo_usniffer_input_frame(const uint8_t* pc_data, uint8_t c_len,
                                int8_t status)
{
    int i;
    LOG_INFO("I've got a frame!\r");
    for (i=0; i<c_len; i++)
    {
        printf("%d ", *pc_data);
        pc_data++;
    }


} /* demo_usniffer_input_frame() */


/*----------------------------------------------------------------------------*/
/*    demo_usniffer_input_byte()                                              */
/*----------------------------------------------------------------------------*/
void demo_usniffer_input_byte(void * chr)
{
    char c = *(char*)chr;
    printf("\rI've got the char: %c\r\n", c);
} /* demo_usniffer_input_byte() */

/** @} */
/** @} */
/** @} */
