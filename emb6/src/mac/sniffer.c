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
 * \file
 *         A sniffer mac layer
 * \author
 *         Artem Yushev
 */

#include "emb6.h"
#include "packetbuf.h"


static s_ns_t*            p_ns = NULL;
static sniff_callback_t   f_sncallb = NULL;

static void   _sniffer_init(s_ns_t* p_netStack);
static void   _sniffer_sendpck(mac_callback_t sent, void *ptr);
static void   _sniffer_sendlst(mac_callback_t sent, void *ptr, struct lmac_buf_list *buf_list);
static void   _sniffer_input(void);
static int8_t _sniffer_on(void);
static int8_t _sniffer_off(int keep_radio_on);
static unsigned short _sniffer_cci(void);
static void   _sniffer_regcallb(sniff_callback_t f_sniffCallb);



/*---------------------------------------------------------------------------*/
static void _sniffer_init(s_ns_t* p_netStack)
{
    if ((p_netStack != NULL) && (p_netStack->inif != NULL)) {
        p_ns = p_netStack;
        p_ns->inif->on();
    }
}

/*---------------------------------------------------------------------------*/
static void _sniffer_sendpck(mac_callback_t sent, void *ptr)
{

}

/*---------------------------------------------------------------------------*/
static void _sniffer_sendlst(mac_callback_t sent, void *ptr, struct lmac_buf_list *buf_list)
{
}
/*---------------------------------------------------------------------------*/
static void _sniffer_input(void)
{
    if ((p_ns != NULL) && (f_sncallb != NULL)) 
    {
        f_sncallb(packetbuf_dataptr(),packetbuf_datalen(),0);
    }
}
/*---------------------------------------------------------------------------*/
static int8_t _sniffer_on(void)
{
    if ((p_ns != NULL) && (p_ns->inif != NULL))
        return p_ns->inif->on();
    else
        return 0;
}
/*---------------------------------------------------------------------------*/
static int8_t _sniffer_off(int keep_radio_on)
{
    if ((p_ns != NULL) && (p_ns->inif != NULL)) {
        if(keep_radio_on) {
            return p_ns->inif->on();
        } else {
            return p_ns->inif->off();
        }
    } else
        return 0;
}

/*---------------------------------------------------------------------------*/
static unsigned short _sniffer_cci(void)
{
  return 0;
}

static void _sniffer_regcallb(sniff_callback_t f_sniffCallb)
{
    f_sncallb = f_sniffCallb;
}
/*---------------------------------------------------------------------------*/
const s_nsLowMac_t sniffer_driver = {
  .name                   = "nullrdc",
  .init                   = &_sniffer_init,
  .send                   = &_sniffer_sendpck,
  .send_list              = &_sniffer_sendlst,
  .input                  = &_sniffer_input,
  .on                     = &_sniffer_on,
  .off                    = &_sniffer_off,
  .channel_check_interval = &_sniffer_cci,
  .regcallb               = &_sniffer_regcallb
};
/*---------------------------------------------------------------------------*/
