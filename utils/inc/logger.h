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
/**
 * \file   logger.h
 *
 * \author Artem Yushev 
 *
 * \brief  There are 3 levels of LOG macros. Please try to follow advices
 *         LOG_XXXX  - to output "MUST HAVE" information. For example errors.
 *         LOG1_XXXX - to output supplementary information which is usefull
 *                     for debugging; e.g. entry points to the functions,
 *                     report about function process execution
 *         LOG2_XXXX - to output data which normally consumes a lot of space;
 *                     e.g., to printout packets or raw data and to annotate
 *                     this output
 *
 *         You can control logger behavior combining two methods
 *         INDIVIDUAL module:
 *              Following construction for your SOURCE file should be defined:
 *              #define     LOGGER_ENABLE        LOGGER_MODULENAME
 *              #include    "logger.h"
 *              Where <LOGGER_MODULENAME> can be defined externally
 *              Moreover in that external include you may specify LOGGER_LEVEL,
 *              which instructs logger
 *         GLOBAL:
 *              LOGGER_LEVEL specify up to what level messages will be shown
 *              for example if in your module you've written
 *              LOG2_HEXDUMP(<data to output>) and global LOGGER_LEVEL is set
 *              to 2 then you data won't be shown, but if you used
 *              LOG1_HEXDUMP(<data to output>) instead data will be printed out
 */
#ifndef LOGGER_H_
#define LOGGER_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>

/*!
\brief   Print out IPv6 address

\param    pc_addr      Pointer to address (16 bytes).

\return   None
*/
void log_ip6addr( const uint8_t* pc_addr);

/*!
\brief   Printout data in a standard hex view

\param    p_buf        Pointer to data which should be printed out.
\param    l_len        Length of a data

\return   None
\example
0x000000: 2e 2f 68 65 78 64 75 6d ./hexdum
0x000008: 70 00 53 53 48 5f 41 47 p.SSH_AG
0x000010: 45 4e 54 5f             ENT_
*/
void log_hexdump(const void* p_buf, uint32_t l_len);


#define LOG2_OK(msg, ...)       LOGGER_OK(2,msg, ##__VA_ARGS__)
#define LOG2_ERR(msg, ...)      LOGGER_ERR(2,msg, ##__VA_ARGS__)
#define LOG2_INFO(msg, ...)     LOGGER_INFO(2,msg, ##__VA_ARGS__)
#define LOG2_WARN(msg, ...)     LOGGER_WARN(2,msg, ##__VA_ARGS__)
#define LOG2_FAIL(msg, ...)     LOGGER_FAIL(2,msg, ##__VA_ARGS__)
#define LOG2_DBG(msg, ...)      LOGGER_DBG(2,msg, ##__VA_ARGS__)
#define LOG2_RAW(...)           LOGGER_RAW(2, __VA_ARGS__)
#define LOG2_HEXDUMP(buf,len)   LOGGER_HEXDUMP(2, buf,len)
#define LOG2_IP6ADDR(addr)     LOGGER_IP6ADDR(2, addr)

#define LOG1_OK(msg, ...)       LOGGER_OK(1,msg, ##__VA_ARGS__)
#define LOG1_ERR(msg, ...)      LOGGER_ERR(1,msg, ##__VA_ARGS__)
#define LOG1_INFO(msg, ...)     LOGGER_INFO(1,msg, ##__VA_ARGS__)
#define LOG1_WARN(msg, ...)     LOGGER_WARN(1,msg, ##__VA_ARGS__)
#define LOG1_FAIL(msg, ...)     LOGGER_FAIL(1,msg, ##__VA_ARGS__)
#define LOG1_DBG(msg, ...)      LOGGER_DBG(1,msg, ##__VA_ARGS__)
#define LOG1_RAW(...)           LOGGER_RAW(1, __VA_ARGS__)
#define LOG1_HEXDUMP(buf,len)   LOGGER_HEXDUMP(1, buf,len)
#define LOG1_IP6ADDR(addr)     LOGGER_IP6ADDR(1, addr)

#define LOG_OK(msg, ...)        LOGGER_OK(0,msg, ##__VA_ARGS__)
#define LOG_ERR(msg, ...)       LOGGER_ERR(0,msg, ##__VA_ARGS__)
#define LOG_INFO(msg, ...)      LOGGER_INFO(0,msg, ##__VA_ARGS__)
#define LOG_WARN(msg, ...)      LOGGER_WARN(0,msg, ##__VA_ARGS__)
#define LOG_FAIL(msg, ...)      LOGGER_FAIL(0,msg, ##__VA_ARGS__)
#define LOG_DBG(msg, ...)       LOGGER_DBG(0,msg, ##__VA_ARGS__)
#define LOG_RAW(msg, ...)       LOGGER_RAW(0,msg, ##__VA_ARGS__)
#define LOG_HEXDUMP(buf,len)    LOGGER_HEXDUMP(0, buf,len)
#define LOG_IP6ADDR(addr)      LOGGER_IP6ADDR(0, (const uint8_t* )addr)

#define LOGGER_OK(log_lvl, msg, ...)        \
    do { if ((LOGGER_ENABLE) && (LOGGER_LEVEL > log_lvl)) printf("%lu |   ok | %5s (%d)| " msg "\n\r", bsp_getSec(), __FILE__, __LINE__, ##__VA_ARGS__); }while (0)

#define LOGGER_ERR(log_lvl, msg, ...)       \
    do { if ((LOGGER_ENABLE) && (LOGGER_LEVEL > log_lvl)) printf("%lu |  err | %5s (%d)| " msg "\n\r", bsp_getSec(), __FILE__, __LINE__, ##__VA_ARGS__); }while (0)

#define LOGGER_INFO(log_lvl, msg, ...)      \
    do { if ((LOGGER_ENABLE) && (LOGGER_LEVEL > log_lvl)) printf("%lu | info | %5s (%d)| " msg "\n\r", bsp_getSec(), __FILE__, __LINE__, ##__VA_ARGS__); }while (0)

#define LOGGER_WARN(log_lvl, msg, ...)      \
    do { if ((LOGGER_ENABLE) && (LOGGER_LEVEL > log_lvl)) printf("%lu | warn | %5s (%d)| " msg "\n\r", bsp_getSec(), __FILE__, __LINE__, ##__VA_ARGS__); }while (0)

#define LOGGER_FAIL(log_lvl, msg, ...)      \
    do { if ((LOGGER_ENABLE) && (LOGGER_LEVEL > log_lvl)) printf("%lu | fail | %5s (%d)| " msg "\n\r", bsp_getSec(), __FILE__, __LINE__, ##__VA_ARGS__); }while (0)

#define LOGGER_DBG(log_lvl, msg, ...)       \
    do { if ((LOGGER_ENABLE) && (LOGGER_LEVEL > log_lvl)) printf("%lu | dbg  | %5s (%d)| " msg "\n\r", bsp_getSec(), __FILE__, __LINE__, ##__VA_ARGS__); }while (0)

#define LOGGER_RAW(log_lvl, msg, ...)       \
    do { if ((LOGGER_ENABLE) && (LOGGER_LEVEL > log_lvl)) printf(msg , ##__VA_ARGS__); }while (0)

#define LOGGER_HEXDUMP(log_lvl, buf,len)        \
    do { if ((LOGGER_ENABLE) && (LOGGER_LEVEL > log_lvl)) log_hexdump(buf,len); }while (0)

#define LOGGER_IP6ADDR(log_lvl, addr)          \
    do { if ((LOGGER_ENABLE) && (LOGGER_LEVEL > log_lvl)) log_ip6addr(addr); }while (0)

#endif /* LOGGER_H_ */
