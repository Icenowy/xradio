/*
 * DebugFS code for XRadio drivers
 *
 * Copyright (c) 2013, XRadio
 * Author: XRadio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef XRADIO_DEBUG_H_INCLUDED
#define XRADIO_DEBUG_H_INCLUDED

#define XRADIO_DBG_ALWY   0x01    /* Message always need to be present even in release version. */
#define XRADIO_DBG_ERROR  0x02    /* Error message to report an error, it can hardly works. */
#define XRADIO_DBG_WARN   0x04    /* Warning message to inform us of something unnormal or 
                                   * something very important, but it still work. */
#define XRADIO_DBG_NIY    0x08    /* Important message we need to know in unstable version. */
#define XRADIO_DBG_MSG    0x10    /* Normal message just for debug in developing stage. */
#define XRADIO_DBG_TRC    0x20    /* Trace of functions, for sequence of functions called. Normally,
                                   * don't set this level because there are too more print. */
#define XRADIO_DBG_LEVEL	0xFF


#define WSM_DUMP_MAX_SIZE 20


/****************************** release version *******************************/
#define SYS_BUG(c)  BUG_ON(c)
#define SYS_WARN(c) WARN_ON(c)

#define xradio_dbg(level, ...)
#define txrx_printk(level, ...)
#define wsm_printk(level, ...)
#define sta_printk(level, ...)
#define scan_printk(level, ...)
#define ap_printk(level, ...)
#define pm_printk(level, ...)
#define itp_printk(level, ...)

#define DBG_FUN_LINE
#define PARAM_CHECK_FALSE
#define PARAM_CHECK_TRUE


#endif /* XRADIO_DEBUG_H_INCLUDED */
