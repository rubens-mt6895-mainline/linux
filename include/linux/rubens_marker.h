/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_RUBENS_MARKER_H
#define _LINUX_RUBENS_MARKER_H

#include <linux/stdarg.h>
#include <linux/types.h>

#ifdef CONFIG_RUBENS_MARKER_WRITER
void rubens_marker_early_init(void);
void rubens_marker_stage(u32 stage);
void rubens_marker_early_printk(const char *fmt, va_list args);
void rubens_marker_put(const char *fmt, ...) __printf(1, 2);
#else
static inline void rubens_marker_early_init(void) { }
static inline void rubens_marker_stage(u32 stage) { }
static inline void rubens_marker_early_printk(const char *fmt, va_list args) { }
#endif

#endif /* _LINUX_RUBENS_MARKER_H */
