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
 *      \addtogroup demo_sniffer_control
 *      @{
*/
/*! \file   demo_sniffer_control.c

 \author Aleksei Turtsevich

 \brief  Just Print application

 \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/
#include "demo_usniffer.h"

#define     LOGGER_ENABLE        FALSE
#include    "logger.h"


/** supported command codes for sniffer application
 *
 * This enumeration is generated by:
 * Tools/cmdhash.py parms cmset cmclr cmask chan cpage ed scan sniff idle
 */
typedef enum {
     /** Hashvalue for command 'chkcrc' */
     CMD_CHKCRC = 0x23,
     /** Hashvalue for command 'timeset' */
     CMD_TIMESET = 0x35,
     /** Hashvalue for command 'scan' */
     CMD_SCAN = 0x4d,
     /** Hashvalue for command 'chan' */
     CMD_CHAN = 0x4e,
     /** Hashvalue for command 'cmclr' */
     CMD_CMCLR = 0x52,
     /** Hashvalue for command 'sniff' */
     CMD_SNIFF = 0x64,
     /** Hashvalue for command 'cmset' */
     CMD_CMSET = 0x74,
     /** Hashvalue for command 'cpage' */
     CMD_CPAGE = 0x87,
     /** Hashvalue for command 'drate' */
     CMD_DRATE = 0xa0,
     /** Hashvalue for command 'idle' */
     CMD_IDLE = 0xa1,
     /** Hashvalue for command 'cmask' */
     CMD_CMASK = 0xa9,
     /** Hashvalue for command 'ed' */
     CMD_ED = 0xc7,
     /** Hashvalue for command 'parms' */
     CMD_PARMS = 0xf1,
     /** Hashvalue for command 'sfd' */
     CMD_SFD = 0xc4,
     /** Hashvalue for empty command  */
     CMD_EMPTY = 0x00,
} SHORTENUM cmd_hash_t;


static bool demo_usniffProcessCommand(char * cmd);
static cmd_hash_t demo_usniffGetCmdHash(char *cmd);
static bool demo_usniffProcessHotkey(char cmdkey);

/**
 * @brief Split a null terminated string.
 *
 * This function creates argc,argv style data from a null
 * terminated string. The splitting is done on the base of
 * spaces (ASCII 32).
 *
 * @param  txtline  string to split
 * @param  maxargs  maximum number of arguments to split
 * @retval argv     array of pointers, that store the arguments
 * @return number of arguments splitted (argc)
 */
static inline int demo_usniffSplitArgs(char *txtline, int maxargs, char **argv)
{
uint8_t argc = 0, nextarg = 1;

    while((*txtline !=0) && (argc < maxargs))
    {
        if (*txtline == ' ')
        {
            *txtline = 0;
            nextarg = 1;
        }
        else
        {
            if(nextarg)
            {
                argv[argc] = txtline;
                argc++;
                nextarg = 0;
            }
        }
        txtline++;
    }

    return argc;
} /* demo_usniffSplitArgs() */

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
s_ns_t* pst_ns = NULL;

/*==============================================================================
                                         API FUNCTIONS
 =============================================================================*/

void demo_usniffProcessInput(void * chr)
{
static  char     cmdline[16];
static  uint8_t  cmdidx = 0;
        char     inchar = *(char*)chr;
        bool     success;

    /* command processing */
    if(EOF != inchar)
    {
        cmdline[cmdidx++] = inchar;
        if (inchar == '\n' || inchar == '\r')
        {
            cmdline[cmdidx-1] = 0;
            if (cmdidx > 2)
            {
                demo_usniffProcessCommand(cmdline);
            }
            cmdidx = 0;
        }
        else if (cmdidx == 1)
        {
            success = demo_usniffProcessHotkey(cmdline[0]);
            if (success == true)
            {
                printf("\r");
                cmdidx = 0;
            }
        }
    }
    if (cmdidx >= sizeof(cmdline)-1)
    {
        cmdidx = 0;
        cmdline[0] = 0;
    }
} /* demo_usniffProcessInput() */




uint8_t cnt_active_channels(uint32_t cmask)
{
uint8_t ret = 0;

    while (cmask != 0)
    {
        ret += (uint8_t)(cmask & 1UL);
        cmask >>= 1;
    }
    return ret;
} /* cnt_active_channels() */



/**
 * @brief Command processing.
 */
static bool demo_usniffProcessCommand(char * cmd)
{
    char *argv[4];
    uint8_t argc;
    uint8_t ch, tmp;
    volatile uint8_t i;
    bool cmdok;
    sniffer_state_t next_state;
    uint32_t tv;
    next_state = ctx.state;

    s_ns_t* pst_ns = NULL;
    pst_ns = emb6_get();

    if ((pst_ns == NULL) || (pst_ns->inif == NULL))
        return false;

    printf("> %s"NL, cmd);
    argc = demo_usniffSplitArgs(cmd, sizeof(argv), argv);
    ch = demo_usniffGetCmdHash(argv[0]);
    cmdok = true;
    switch (ch)
    {
        case CMD_TIMESET:
            printf("\n\rct1=%ld\n\r", bsp_get(E_BSP_GET_TICK));
            tv = strtol(argv[1],NULL,10);
            printf("tv=%ld %s\n\r", tv , argv[1]);
            //timer_set_systime(tv);
            printf("ct2=%ld\n\r", bsp_get(E_BSP_GET_TICK));
            break;
        case CMD_CHAN:
            tmp = atoi(argv[1]);
            if (tmp>= TRX_MIN_CHANNEL && tmp <= TRX_MAX_CHANNEL)
            {
                ctx.cchan = tmp;
                pst_ns->inif->ioctl(E_RADIO_CHANNEL, ctx.cchan);
            }
            else
            {
                cmdok = false;
            }
            break;
        case CMD_CPAGE:
            ctx.cpage = atoi(argv[1]);
            break;
        case CMD_CMASK:
            ctx.cmask = (strtol(argv[1],NULL,0) & TRX_SUPPORTED_CHANNELS);
            break;
        case CMD_CMCLR:
            if (argc < 2)
            {
                ctx.cmask &= ~TRX_SUPPORTED_CHANNELS;
            }
            else
            {
                ctx.cmask &= ~(uint32_t)(1UL << (atoi(argv[1])));
            }
            break;
        case CMD_CMSET:
            if (argc < 2)
            {
                ctx.cmask |= TRX_SUPPORTED_CHANNELS;
            }
            else
            {
                ctx.cmask |= (uint32_t)((1UL << (atoi(argv[1]))) & TRX_SUPPORTED_CHANNELS);
            }
            break;
        case CMD_PARMS:
            printf(NL"PLATFORM: %s V%s"NL, BOARD_NAME, VERSION);
            printf("SUPP_CMSK: 0x%08lx"NL, (unsigned long)TRX_SUPPORTED_CHANNELS);
            printf("CURR_CMSK: 0x%08lx"NL, (unsigned long)ctx.cmask);
            printf("CURR_CHAN: %d"NL, ctx.cchan);
            printf("CURR_PAGE: %d"NL"CURR_RATE: BPSK20", ctx.cpage);
            //hif_puts_p(trx_decode_datarate_p(trx_get_datarate()));
            printf(NL"SUPP_RATES: ");
            //for (i=0;i<trx_get_number_datarates();i++)
            //{
            //    hif_puts_p(trx_get_datarate_str_p(i));
            //    printf(" ");
            //}
            printf(" BPSK20 QPSK100");
            printf(NL);
            printf("TIMER_SCALE: %d.0/%ld"NL, HWTMR_PRESCALE, F_CPU);
            printf("TICK_NUMBER: %ld"NL, HWTIMER_TICK_NB);
            printf("CHKCRC: %d"NL, ctx.chkcrc);
            printf("MISSED_FRAMES: %d"NL,ctx.missed_frames);
            break;
        case CMD_SCAN:
            next_state = SCAN;
            break;
        case CMD_SNIFF:
            next_state = SNIFF;
            break;
        case CMD_IDLE:
            next_state = IDLE;
            break;
        case CMD_DRATE:
            /* Experimental Hunk */
            // TODO: more suitable
            if (strcmp (argv[1],"BPSK20") == 0)
            {
                pst_ns->inif->ioctl(E_RADIO_MOD, E_BSP_RMODE_BPSK20);
            }
            else if (strcmp (argv[1],"QPSK100") == 0)
            {
                pst_ns->inif->ioctl(E_RADIO_MOD, E_BSP_RMODE_QPSK100);
            }
            break;
        case CMD_CHKCRC:
            if (argc < 2)
            {
                ctx.chkcrc ^= 1;
            }
            else if (atoi(argv[1]) != 0)
            {
                ctx.chkcrc = 1;
            }
            else
            {
                ctx.chkcrc = 0;
            }
            printf("crc_ok=%d"NL,ctx.chkcrc );
            break;
        case CMD_EMPTY:
            break;
        default:
            cmdok = false;
            break;
    }

    if(cmdok != true)
    {
        printf("invalid cmd, argc=%d, argv[0]=%s, hash=0x%02x"NL,
                argc, argv[0], ch);
    }
    else
    {
        printf("OK\r\n"); // del \r\n
    }

    if (next_state != ctx.state)
    {
        demo_usniffer_stop();
        demo_usniffer_start(next_state);
    }

    return false;
} /* demo_usniffProcessCommand() */


/**
 * @brief Key processing.
 */
static bool demo_usniffProcessHotkey(char cmdkey)
{
    bool ret = true;
    sniffer_state_t next_state;
    next_state = ctx.state;

    s_ns_t* pst_ns = NULL;
    pst_ns = emb6_get();

    if ((pst_ns == NULL) || (pst_ns->inif == NULL))
        return false;

    switch (cmdkey)
    {
        case 'I':
        case ' ': /* panic mode switch off: hit space bar*/
            next_state = IDLE;
            printf("IDLE\n");
            break;

        case '+':
            if (ctx.state != SCAN)
            {
                ctx.cchan = (ctx.cchan >= TRX_MAX_CHANNEL) ? TRX_MIN_CHANNEL : ctx.cchan + 1;
                pst_ns->inif->ioctl(E_RADIO_CHANNEL, ctx.cchan);

                printf("Channel %u"NL, ctx.cchan);
            }
            break;

        case '-':
            if (ctx.state != SCAN)
            {
                ctx.cchan = (ctx.cchan <= TRX_MIN_CHANNEL) ? TRX_MAX_CHANNEL : ctx.cchan - 1;
                pst_ns->inif->ioctl(E_RADIO_CHANNEL, ctx.cchan);
                printf("Channel %u"NL, ctx.cchan);
            }
            break;

        case 'r':
            if (ctx.scanres_reset != 0)
            {
                ctx.scanres_reset = 0;
            }
            else
            {
                ctx.scanres_reset = cnt_active_channels(ctx.cmask);
            }
            break;

        case 'R':
            if (ctx.scanres_reset != 0)
            {
                ctx.scanres_reset = 0;
            }
            else
            {
                ctx.scanres_reset = TRX_NB_CHANNELS + 1;
            }
            break;

        default:
            ret = false;
            break;
    }

    if (next_state != ctx.state)
    {
        demo_usniffer_stop();
        demo_usniffer_start(next_state);
    }

    return ret;
} /* demo_usniffProcessHotkey() */



static cmd_hash_t demo_usniffGetCmdHash(char *cmd)
{
cmd_hash_t h, accu;

    h = 0;
    while (*cmd != 0)
    {
        accu = h & 0xe0;
        h = h << 5;
        h = h ^ (accu >> 5);
        h = h ^ *cmd;
        h &= 0xff;
        cmd ++;
    }
    return h;
} /* demo_usniffGetCmdHash() */


/** @} */
/** @} */
/** @} */
