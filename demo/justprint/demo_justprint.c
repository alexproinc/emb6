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
#include "demo_justprint.h"


#define     LOGGER_ENABLE        FALSE
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
/*    demo_justprintInit()                                                           */
/*----------------------------------------------------------------------------*/

int8_t demo_justprintInit(void)
{

    LOG_INFO("Starting Sniffer");

    printf("printf test\n");

    putchar('p');
    putchar('u');
    putchar('t');
    putchar('c');
    putchar('h');
    putchar('a');
    putchar('r');
    putchar(' ');
    putchar('t');
    putchar('e');
    putchar('s');
    putchar('t');
    putchar('\n');

    etimer_set(&print_et, 1 * bsp_get(E_BSP_GET_TRES), demo_print_et_callback);


    return 1;
} /* demo_justprintInit() */

/*----------------------------------------------------------------------------*/
/*    demo_justprintConf()                                                           */
/*----------------------------------------------------------------------------*/

uint8_t demo_justprintConf(s_ns_t* pst_netStack)
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
} /*  demo_justprintConf() */


/*----------------------------------------------------------------------------*/
/*    demo_printing()                                                           */
/*----------------------------------------------------------------------------*/
void demo_print_et_callback(c_event_t c_event, p_data_t p_data)
{
    if (etimer_expired(&print_et))
    {
        
        printf("printf test\n");

        putchar('p');
        putchar('u');
        putchar('t');
        putchar('c');
        putchar('h');
        putchar('a');
        putchar('r');
        putchar(' ');
        putchar('t');
        putchar('e');
        putchar('s');
        putchar('t');
        putchar('\n');


        etimer_restart(&print_et);
    }
} /* demo_print_et_callback() */


/** @} */
/** @} */
/** @} */
