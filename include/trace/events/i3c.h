/* SPDX-License-Identifier: GPL-2.0-or-later */
/* I3C message transfer tracepoints
 *
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM i3c

#if !defined(_TRACE_I3C_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_I3C_H

#include <linux/i3c/device.h>
#include <linux/i3c/master.h>
#include <linux/tracepoint.h>

TRACE_EVENT(i3c_priv_xfer,
       TP_PROTO(const struct i3c_master_controller *ctrl,
		const struct i3c_device *dev,
		const struct i3c_priv_xfer *xfer),
       TP_ARGS(ctrl, dev, xfer),
       TP_STRUCT__entry(
	       __field(int,	bus_id			)
	       __field(u8,	dev_addr		)
	       __field(__u8,	rnw			)
	       __field(__u16,	len			)
	       __field(__u16,	actual_len		)
	       __field(int,	res			)
	       __dynamic_array(__u8, buf, xfer->len)
       ),
       TP_fast_assign(
	       __entry->bus_id = ctrl->bus.id;
	       __entry->dev_addr = dev->desc->info.dyn_addr;
	       __entry->rnw = xfer->rnw;
	       __entry->len = xfer->len;
	       __entry->actual_len = xfer->actual_len;
	       __entry->len = xfer->len;
	       __entry->res = xfer->err;
	       memcpy(__get_dynamic_array(buf), xfer->data.out, xfer->len);
       ),
       TP_printk("i3c-%d %02x %c l=%u/%u r=%d [%*phD]",
		 __entry->bus_id,
		 __entry->dev_addr,
		 __entry->rnw ? 'r' : 'w',
		 __entry->len,
		 __entry->actual_len,
		 __entry->res,
		 __entry->len, __get_dynamic_array(buf)
	)
)

#endif

#include <trace/define_trace.h>
