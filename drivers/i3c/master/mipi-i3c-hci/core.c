// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2020, MIPI Alliance, Inc.
 *
 * Author: Nicolas Pitre <npitre@baylibre.com>
 *
 * Core driver code with main interface to the I3C subsystem.
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/i3c/master.h>
#include <linux/i3c/target.h>
#include <linux/i3c/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <dt-bindings/i3c/i3c.h>

#include "hci.h"
#include "ext_caps.h"
#include "cmd.h"
#include "dat.h"
#include "vendor_aspeed.h"


/*
 * Host Controller Capabilities and Operation Registers
 */

#define reg_read(r)		readl(hci->base_regs + (r))
#define reg_write(r, v)		writel(v, hci->base_regs + (r))
#define reg_set(r, v)		reg_write(r, reg_read(r) | (v))
#define reg_clear(r, v)		reg_write(r, reg_read(r) & ~(v))

#define HCI_VERSION			0x00	/* HCI Version (in BCD) */

#define HC_CONTROL			0x04
#define HC_CONTROL_BUS_ENABLE		BIT(31)
#define HC_CONTROL_RESUME		BIT(30)
#define HC_CONTROL_ABORT		BIT(29)
#define HC_CONTROL_HALT_ON_CMD_TIMEOUT	BIT(12)
#define HC_CONTROL_HOT_JOIN_CTRL	BIT(8)	/* Hot-Join ACK/NACK Control */
#define HC_CONTROL_I2C_TARGET_PRESENT	BIT(7)
#define HC_CONTROL_PIO_MODE		BIT(6)	/* DMA/PIO Mode Selector */
#define HC_CONTROL_DATA_BIG_ENDIAN	BIT(4)
#define HC_CONTROL_IBA_INCLUDE		BIT(0)	/* Include I3C Broadcast Address */

#define MASTER_DEVICE_ADDR		0x08	/* Master Device Address */
#define MASTER_DYNAMIC_ADDR_VALID	BIT(31)	/* Dynamic Address is Valid */
#define MASTER_DYNAMIC_ADDR(v)		FIELD_PREP(GENMASK(22, 16), v)

#define HC_CAPABILITIES			0x0c
#define HC_CAP_SG_DC_EN			BIT(30)
#define HC_CAP_SG_IBI_EN		BIT(29)
#define HC_CAP_SG_CR_EN			BIT(28)
#define HC_CAP_MAX_DATA_LENGTH		GENMASK(24, 22)
#define HC_CAP_CMD_SIZE			GENMASK(21, 20)
#define HC_CAP_DIRECT_COMMANDS_EN	BIT(18)
#define HC_CAP_MULTI_LANE_EN		BIT(15)
#define HC_CAP_CMD_CCC_DEFBYTE		BIT(10)
#define HC_CAP_HDR_BT_EN		BIT(8)
#define HC_CAP_HDR_TS_EN		BIT(7)
#define HC_CAP_HDR_DDR_EN		BIT(6)
#define HC_CAP_NON_CURRENT_MASTER_CAP	BIT(5)	/* master handoff capable */
#define HC_CAP_DATA_BYTE_CFG_EN		BIT(4)	/* endian selection possible */
#define HC_CAP_AUTO_COMMAND		BIT(3)
#define HC_CAP_COMBO_COMMAND		BIT(2)

#define RESET_CONTROL			0x10
#define BUS_RESET			BIT(31)
#define BUS_RESET_TYPE			GENMASK(30, 29)
#define IBI_QUEUE_RST			BIT(5)
#define RX_FIFO_RST			BIT(4)
#define TX_FIFO_RST			BIT(3)
#define RESP_QUEUE_RST			BIT(2)
#define CMD_QUEUE_RST			BIT(1)
#define SOFT_RST			BIT(0)	/* Core Reset */

#define PRESENT_STATE			0x14
#define STATE_CURRENT_MASTER		BIT(2)

#define INTR_STATUS			0x20
#define INTR_STATUS_ENABLE		0x24
#define INTR_SIGNAL_ENABLE		0x28
#define INTR_FORCE			0x2c
#define INTR_HC_CMD_SEQ_UFLOW_STAT	BIT(12)	/* Cmd Sequence Underflow */
#define INTR_HC_RESET_CANCEL		BIT(11)	/* HC Cancelled Reset */
#define INTR_HC_INTERNAL_ERR		BIT(10)	/* HC Internal Error */
#define INTR_HC_PIO			BIT(8)	/* cascaded PIO interrupt */
#define INTR_HC_RINGS			GENMASK(7, 0)

#define DAT_SECTION			0x30	/* Device Address Table */
#define DAT_ENTRY_SIZE			GENMASK(31, 28)
#define DAT_TABLE_SIZE			GENMASK(18, 12)
#define DAT_TABLE_OFFSET		GENMASK(11, 0)

#define DCT_SECTION			0x34	/* Device Characteristics Table */
#define DCT_ENTRY_SIZE			GENMASK(31, 28)
#define DCT_TABLE_INDEX			GENMASK(23, 19)
#define DCT_TABLE_SIZE			GENMASK(18, 12)
#define DCT_TABLE_OFFSET		GENMASK(11, 0)

#define RING_HEADERS_SECTION		0x38
#define RING_HEADERS_OFFSET		GENMASK(15, 0)

#define PIO_SECTION			0x3c
#define PIO_REGS_OFFSET			GENMASK(15, 0)	/* PIO Offset */

#define EXT_CAPS_SECTION		0x40
#define EXT_CAPS_OFFSET			GENMASK(15, 0)

#define IBI_NOTIFY_CTRL			0x58	/* IBI Notify Control */
#define IBI_NOTIFY_SIR_REJECTED		BIT(3)	/* Rejected Target Interrupt Request */
#define IBI_NOTIFY_MR_REJECTED		BIT(1)	/* Rejected Master Request Control */
#define IBI_NOTIFY_HJ_REJECTED		BIT(0)	/* Rejected Hot-Join Control */

#define DEV_CTX_BASE_LO			0x60
#define DEV_CTX_BASE_HI			0x64

static inline struct i3c_hci *to_i3c_hci(struct i3c_master_controller *m)
{
	return container_of(m, struct i3c_hci, master);
}

static int i3c_hci_bus_init(struct i3c_master_controller *m)
{
	struct i3c_hci *hci = to_i3c_hci(m);
	struct i3c_device_info info;
	int ret;

	DBG("");
	dev_info(&hci->master.dev, "Master Mode");

#ifdef CONFIG_ARCH_ASPEED
	ast_inhouse_write(ASPEED_I3C_CTRL,
			  ASPEED_I3C_CTRL_INIT |
				  FIELD_PREP(ASPEED_I3C_CTRL_INIT_MODE,
					     INIT_MST_MODE));
#endif

	if (hci->cmd == &mipi_i3c_hci_cmd_v1) {
		ret = mipi_i3c_hci_dat_v1.init(hci);
		if (ret)
			return ret;
	}

	ret = i3c_master_get_free_addr(m, 0);
	if (ret < 0)
		return ret;
	reg_write(MASTER_DEVICE_ADDR,
		  MASTER_DYNAMIC_ADDR(ret) | MASTER_DYNAMIC_ADDR_VALID);
	memset(&info, 0, sizeof(info));
	info.dyn_addr = ret;
	if (hci->caps & HC_CAP_HDR_DDR_EN)
		info.hdr_cap |= BIT(I3C_HDR_DDR);
	if (hci->caps & HC_CAP_HDR_TS_EN) {
		if (reg_read(HC_CONTROL) & HC_CONTROL_I2C_TARGET_PRESENT)
			info.hdr_cap |= BIT(I3C_HDR_TSL);
		else
			info.hdr_cap |= BIT(I3C_HDR_TSP);
	}
	if (hci->caps & HC_CAP_HDR_BT_EN)
		info.hdr_cap |= BIT(I3C_HDR_BT);
	ret = i3c_master_set_info(m, &info);
	if (ret)
		return ret;

	ret = hci->io->init(hci);
	if (ret)
		return ret;

	reg_set(HC_CONTROL, HC_CONTROL_BUS_ENABLE);
	DBG("HC_CONTROL = %#x", reg_read(HC_CONTROL));

	return 0;
}

static void i3c_hci_bus_cleanup(struct i3c_master_controller *m)
{
	struct i3c_hci *hci = to_i3c_hci(m);
	struct platform_device *pdev = to_platform_device(m->dev.parent);

	DBG("");

	reg_clear(HC_CONTROL, HC_CONTROL_BUS_ENABLE);
	synchronize_irq(platform_get_irq(pdev, 0));
	hci->io->cleanup(hci);
	if (hci->cmd == &mipi_i3c_hci_cmd_v1)
		mipi_i3c_hci_dat_v1.cleanup(hci);
}

static int i3c_hci_bus_reset(struct i3c_master_controller *m)
{
	struct i3c_hci *hci = to_i3c_hci(m);
	struct hci_xfer *xfer;
	DECLARE_COMPLETION_ONSTACK(done);
	int ret;

	xfer = hci_alloc_xfer(1);
	if (!xfer)
		return -ENOMEM;
	if (hci->master.bus.context == I3C_BUS_CONTEXT_JESD403)
		hci->cmd->prep_internal(hci, xfer, M_SUB_CMD_REC_RST_PROC,
					REC_PROC_TIMED_RST);
	else
		hci->cmd->prep_internal(hci, xfer, M_SUB_CMD_TARGET_RST_PATTERN,
					RST_OP_TARGET_RST);
	xfer[0].completion = &done;

	ret = hci->io->queue_xfer(hci, xfer, 1);
	if (ret)
		goto out;
	if (!wait_for_completion_timeout(&done, HZ) &&
	    hci->io->dequeue_xfer(hci, xfer, 1)) {
		ret = -ETIME;
		goto out;
	}
out:
	hci_free_xfer(xfer, 1);
	return ret;
}

void mipi_i3c_hci_iba_ctrl(struct i3c_hci *hci, bool enable)
{
	DBG("%s IBA\n", enable ? "ENABLE" : "DISABLE");
	reg_write(HC_CONTROL,
		  enable ? reg_read(HC_CONTROL) | HC_CONTROL_IBA_INCLUDE :
			   reg_read(HC_CONTROL) & ~HC_CONTROL_IBA_INCLUDE);
}

void mipi_i3c_hci_hj_ctrl(struct i3c_hci *hci, bool ack_nack)
{
	DBG("%s Hot-join requeset\n", ack_nack ? "ACK" : "NACK");
	reg_write(HC_CONTROL,
		  ack_nack ? reg_read(HC_CONTROL) & ~HC_CONTROL_HOT_JOIN_CTRL :
			     reg_read(HC_CONTROL) | HC_CONTROL_HOT_JOIN_CTRL);
}

void mipi_i3c_hci_resume(struct i3c_hci *hci)
{
	reg_set(HC_CONTROL, HC_CONTROL_RESUME);
}

/* located here rather than pio.c because needed bits are in core reg space */
void mipi_i3c_hci_pio_reset(struct i3c_hci *hci)
{
	reg_write(RESET_CONTROL,
		  RX_FIFO_RST | TX_FIFO_RST | RESP_QUEUE_RST | CMD_QUEUE_RST);
}

/* located here rather than dct.c because needed bits are in core reg space */
void mipi_i3c_hci_dct_index_reset(struct i3c_hci *hci)
{
	reg_write(DCT_SECTION, FIELD_PREP(DCT_TABLE_INDEX, 0));
}

static int i3c_hci_enable_hotjoin(struct i3c_master_controller *m)
{
	struct i3c_hci *hci = to_i3c_hci(m);
	int ret;

	if (hci->io->request_hj)
		ret = hci->io->request_hj(hci);
	mipi_i3c_hci_hj_ctrl(hci, true);

	return ret;
}

static int i3c_hci_disable_hotjoin(struct i3c_master_controller *m)
{
	struct i3c_hci *hci = to_i3c_hci(m);

	if (hci->io->free_hj)
		hci->io->free_hj(hci);
	mipi_i3c_hci_hj_ctrl(hci, false);
	return 0;
}

static int i3c_hci_send_ccc_cmd(struct i3c_master_controller *m,
				struct i3c_ccc_cmd *ccc)
{
	struct i3c_hci *hci = to_i3c_hci(m);
	struct hci_xfer *xfer;
	bool raw = !!(hci->quirks & HCI_QUIRK_RAW_CCC);
	bool prefixed = raw && !!(ccc->id & I3C_CCC_DIRECT);
	unsigned int nxfers = ccc->ndests + prefixed;
	DECLARE_COMPLETION_ONSTACK(done);
	int i, last, ret = 0;

	DBG("cmd=%#x rnw=%d dbp=%d db=%#x ndests=%d data[0].len=%d", ccc->id,
	    ccc->rnw, ccc->dbp, ccc->db, ccc->ndests,
	    ccc->dests[0].payload.len);

	xfer = hci_alloc_xfer(nxfers);
	if (!xfer)
		return -ENOMEM;

	if (prefixed) {
		xfer->data = NULL;
		xfer->data_len = 0;
		xfer->rnw = false;
		hci->cmd->prep_ccc(hci, xfer, I3C_BROADCAST_ADDR, ccc->id,
				   ccc->dbp, ccc->db, true);
		xfer++;
	}

	for (i = 0; i < nxfers - prefixed; i++) {
		xfer[i].data = ccc->dests[i].payload.data;
		xfer[i].data_len = ccc->dests[i].payload.len;
		xfer[i].rnw = ccc->rnw;
		ret = hci->cmd->prep_ccc(hci, &xfer[i], ccc->dests[i].addr,
					 ccc->id, ccc->dbp, ccc->db, raw);
		if (ret)
			goto out;
		xfer[i].cmd_desc[0] |= CMD_0_ROC;
	}
	last = i - 1;
	xfer[last].cmd_desc[0] |= CMD_0_TOC;
	xfer[last].completion = &done;

	if (prefixed)
		xfer--;

	ret = hci->io->queue_xfer(hci, xfer, nxfers);
	if (ret)
		goto out;
	if (!wait_for_completion_timeout(&done, HZ) &&
	    hci->io->dequeue_xfer(hci, xfer, nxfers)) {
		ret = -ETIME;
		goto out;
	}
	for (i = prefixed; i < nxfers; i++) {
		if (ccc->rnw)
			ccc->dests[i - prefixed].payload.len =
				RESP_DATA_LENGTH(xfer[i].response);
		if (RESP_STATUS(xfer[i].response) != RESP_SUCCESS) {
			DBG("resp status = %lx", RESP_STATUS(xfer[i].response));
			if (RESP_STATUS(xfer[i].response) ==
			    RESP_ERR_ADDR_HEADER)
				ret = I3C_ERROR_M2;
			else
				ret = -EIO;
			goto out;
		}
	}

	if (ccc->rnw)
		DBG("got: %*ph",
		    ccc->dests[0].payload.len, ccc->dests[0].payload.data);

out:
	hci_free_xfer(xfer, nxfers);
	return ret;
}

static int i3c_hci_daa(struct i3c_master_controller *m)
{
	struct i3c_hci *hci = to_i3c_hci(m);

	DBG("");

	return hci->cmd->perform_daa(hci);
}

static int i3c_hci_priv_xfers(struct i3c_dev_desc *dev,
			      struct i3c_priv_xfer *i3c_xfers,
			      int nxfers)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	struct hci_xfer *xfer;
	DECLARE_COMPLETION_ONSTACK(done);
	unsigned int size_limit;
	int i, last, ret = 0;

	DBG("nxfers = %d", nxfers);

	xfer = hci_alloc_xfer(nxfers);
	if (!xfer)
		return -ENOMEM;

	size_limit = 1U << (16 + FIELD_GET(HC_CAP_MAX_DATA_LENGTH, hci->caps));

	for (i = 0; i < nxfers; i++) {
		xfer[i].data_len = i3c_xfers[i].len;
		ret = -EFBIG;
		if (xfer[i].data_len >= size_limit)
			goto out;
		xfer[i].rnw = i3c_xfers[i].rnw;
		if (i3c_xfers[i].rnw) {
			xfer[i].data = i3c_xfers[i].data.in;
		} else {
			/* silence the const qualifier warning with a cast */
			xfer[i].data = (void *) i3c_xfers[i].data.out;
		}
		hci->cmd->prep_i3c_xfer(hci, dev, &xfer[i]);
		xfer[i].cmd_desc[0] |= CMD_0_ROC;
	}
	last = i - 1;
	xfer[last].cmd_desc[0] |= CMD_0_TOC;
	xfer[last].completion = &done;

	ret = hci->io->queue_xfer(hci, xfer, nxfers);
	if (ret)
		goto out;
	if (!wait_for_completion_timeout(&done, HZ) &&
	    hci->io->dequeue_xfer(hci, xfer, nxfers)) {
		ret = -ETIME;
		goto out;
	}
	for (i = 0; i < nxfers; i++) {
		if (i3c_xfers[i].rnw)
			i3c_xfers[i].len = RESP_DATA_LENGTH(xfer[i].response);
		if (RESP_STATUS(xfer[i].response) != RESP_SUCCESS) {
			dev_err(&hci->master.dev, "resp status = %lx",
				RESP_STATUS(xfer[i].response));
			ret = -EIO;
			goto out;
		}
	}

out:
	hci_free_xfer(xfer, nxfers);
	return ret;
}

static int i3c_hci_send_hdr_cmds(struct i3c_master_controller *m,
				 struct i3c_hdr_cmd *cmds, int ncmds)
{
	struct i3c_hci *hci = to_i3c_hci(m);
	struct hci_xfer *xfer;
	DECLARE_COMPLETION_ONSTACK(done);
	int i, last, ret = 0, ntxwords = 0, nrxwords = 0;

	DBG("ncmds = %d", ncmds);

	for (i = 0; i < ncmds; i++) {
		DBG("cmds[%d] mode = %x", i, cmds[i].mode);
		if (!(BIT(cmds[i].mode) & m->this->info.hdr_cap))
			return -EOPNOTSUPP;
		if (cmds[i].code & 0x80)
			nrxwords += DIV_ROUND_UP(cmds[i].ndatawords, 2);
		else
			ntxwords += DIV_ROUND_UP(cmds[i].ndatawords, 2);
	}

	xfer = hci_alloc_xfer(ncmds);
	if (!xfer)
		return -ENOMEM;

	for (i = 0; i < ncmds; i++) {
		xfer[i].data_len = cmds[i].ndatawords << 1;

		xfer[i].rnw = cmds[i].code & 0x80 ? 1 : 0;
		if (xfer[i].rnw)
			xfer[i].data = cmds[i].data.in;
		else
			xfer[i].data = (void *)cmds[i].data.out;
		hci->cmd->prep_hdr(hci, xfer, cmds[i].addr, cmds[i].code,
				   cmds[i].mode);

		xfer[i].cmd_desc[0] |= CMD_0_ROC;
	}
	last = i - 1;
	xfer[last].cmd_desc[0] |= CMD_0_TOC;
	xfer[last].completion = &done;

	ret = hci->io->queue_xfer(hci, xfer, ncmds);
	if (ret)
		goto hdr_out;
	if (!wait_for_completion_timeout(&done, HZ) &&
	    hci->io->dequeue_xfer(hci, xfer, ncmds)) {
		ret = -ETIME;
		goto hdr_out;
	}
	for (i = 0; i < ncmds; i++) {
		if (RESP_STATUS(xfer[i].response) != RESP_SUCCESS) {
			dev_err(&hci->master.dev, "resp status = %lx",
				RESP_STATUS(xfer[i].response));
			ret = -EIO;
			goto hdr_out;
		}
		if (cmds[i].code & 0x80)
			cmds[i].ndatawords = DIV_ROUND_UP(RESP_DATA_LENGTH(xfer[i].response), 2);
	}

hdr_out:
	hci_free_xfer(xfer, ncmds);
	return ret;
}

static int i3c_hci_i2c_xfers(struct i2c_dev_desc *dev,
			     const struct i2c_msg *i2c_xfers, int nxfers)
{
	struct i3c_master_controller *m = i2c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	struct hci_xfer *xfer;
	DECLARE_COMPLETION_ONSTACK(done);
	int i, last, ret = 0;

	DBG("nxfers = %d", nxfers);

	xfer = hci_alloc_xfer(nxfers);
	if (!xfer)
		return -ENOMEM;

	for (i = 0; i < nxfers; i++) {
		xfer[i].data = i2c_xfers[i].buf;
		xfer[i].data_len = i2c_xfers[i].len;
		xfer[i].rnw = i2c_xfers[i].flags & I2C_M_RD;
		hci->cmd->prep_i2c_xfer(hci, dev, &xfer[i]);
		xfer[i].cmd_desc[0] |= CMD_0_ROC;
	}
	last = i - 1;
	xfer[last].cmd_desc[0] |= CMD_0_TOC;
	xfer[last].completion = &done;

	ret = hci->io->queue_xfer(hci, xfer, nxfers);
	if (ret)
		goto out;
	if (!wait_for_completion_timeout(&done, HZ) &&
	    hci->io->dequeue_xfer(hci, xfer, nxfers)) {
		ret = -ETIME;
		goto out;
	}
	for (i = 0; i < nxfers; i++) {
		if (RESP_STATUS(xfer[i].response) != RESP_SUCCESS) {
			dev_err(&hci->master.dev, "resp status = %lx",
				RESP_STATUS(xfer[i].response));
			ret = -EIO;
			goto out;
		}
	}

out:
	hci_free_xfer(xfer, nxfers);
	return ret;
}

static int i3c_hci_attach_i3c_dev(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	struct i3c_hci_dev_data *dev_data;
	int ret;

	DBG("");

	dev_data = kzalloc(sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data)
		return -ENOMEM;
	if (hci->cmd == &mipi_i3c_hci_cmd_v1) {
#ifdef CONFIG_ARCH_ASPEED
		ret = mipi_i3c_hci_dat_v1.alloc_entry(hci, dev->info.dyn_addr);
#else
		ret = mipi_i3c_hci_dat_v1.alloc_entry(hci);
#endif
		if (ret < 0) {
			kfree(dev_data);
			return ret;
		}
		mipi_i3c_hci_dat_v1.set_dynamic_addr(hci, ret, dev->info.dyn_addr);
		dev_data->dat_idx = ret;
	}
	i3c_dev_set_master_data(dev, dev_data);
	return 0;
}

static int i3c_hci_reattach_i3c_dev(struct i3c_dev_desc *dev, u8 old_dyn_addr)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	struct i3c_hci_dev_data *dev_data = i3c_dev_get_master_data(dev);

	DBG("");

	if (hci->cmd == &mipi_i3c_hci_cmd_v1)
		mipi_i3c_hci_dat_v1.set_dynamic_addr(hci, dev_data->dat_idx,
					     dev->info.dyn_addr);
	return 0;
}

static void i3c_hci_detach_i3c_dev(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	struct i3c_hci_dev_data *dev_data = i3c_dev_get_master_data(dev);

	DBG("");

	i3c_dev_set_master_data(dev, NULL);
	if (hci->cmd == &mipi_i3c_hci_cmd_v1)
		mipi_i3c_hci_dat_v1.free_entry(hci, dev_data->dat_idx);
	kfree(dev_data);
}

static int i3c_hci_attach_i2c_dev(struct i2c_dev_desc *dev)
{
	struct i3c_master_controller *m = i2c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	struct i3c_hci_dev_data *dev_data;
	int ret;

	DBG("");

	if (hci->cmd != &mipi_i3c_hci_cmd_v1)
		return 0;
	dev_data = kzalloc(sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data)
		return -ENOMEM;
	#ifdef CONFIG_ARCH_ASPEED
		ret = mipi_i3c_hci_dat_v1.alloc_entry(hci, dev->addr);
	#else
		ret = mipi_i3c_hci_dat_v1.alloc_entry(hci);
	#endif
	if (ret < 0) {
		kfree(dev_data);
		return ret;
	}
	mipi_i3c_hci_dat_v1.set_static_addr(hci, ret, dev->addr);
	mipi_i3c_hci_dat_v1.set_flags(hci, ret, DAT_0_I2C_DEVICE, 0);
	dev_data->dat_idx = ret;
	i2c_dev_set_master_data(dev, dev_data);
	return 0;
}

static void i3c_hci_detach_i2c_dev(struct i2c_dev_desc *dev)
{
	struct i3c_master_controller *m = i2c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	struct i3c_hci_dev_data *dev_data = i2c_dev_get_master_data(dev);

	DBG("");

	if (dev_data) {
		i2c_dev_set_master_data(dev, NULL);
		if (hci->cmd == &mipi_i3c_hci_cmd_v1)
			mipi_i3c_hci_dat_v1.free_entry(hci, dev_data->dat_idx);
		kfree(dev_data);
	}
}

static int i3c_hci_request_ibi(struct i3c_dev_desc *dev,
			       const struct i3c_ibi_setup *req)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	struct i3c_hci_dev_data *dev_data = i3c_dev_get_master_data(dev);
	unsigned int dat_idx = dev_data->dat_idx;

	if (req->max_payload_len != 0)
		mipi_i3c_hci_dat_v1.set_flags(hci, dat_idx, DAT_0_IBI_PAYLOAD, 0);
	else
		mipi_i3c_hci_dat_v1.clear_flags(hci, dat_idx, DAT_0_IBI_PAYLOAD, 0);
	return hci->io->request_ibi(hci, dev, req);
}

static void i3c_hci_free_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);

	hci->io->free_ibi(hci, dev);
}

static int i3c_hci_enable_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	struct i3c_hci_dev_data *dev_data = i3c_dev_get_master_data(dev);

	mipi_i3c_hci_dat_v1.clear_flags(hci, dev_data->dat_idx, DAT_0_SIR_REJECT, 0);
	return i3c_master_enec_locked(m, dev->info.dyn_addr, I3C_CCC_EVENT_SIR);
}

static int i3c_hci_disable_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	struct i3c_hci_dev_data *dev_data = i3c_dev_get_master_data(dev);

	mipi_i3c_hci_dat_v1.set_flags(hci, dev_data->dat_idx, DAT_0_SIR_REJECT, 0);
	return i3c_master_disec_locked(m, dev->info.dyn_addr, I3C_CCC_EVENT_SIR);
}

static void i3c_hci_recycle_ibi_slot(struct i3c_dev_desc *dev,
				     struct i3c_ibi_slot *slot)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);

	hci->io->recycle_ibi_slot(hci, dev, slot);
}

static const struct i3c_master_controller_ops i3c_hci_ops = {
	.bus_init		= i3c_hci_bus_init,
	.bus_cleanup		= i3c_hci_bus_cleanup,
	.bus_reset		= i3c_hci_bus_reset,
	.do_daa			= i3c_hci_daa,
	.send_ccc_cmd		= i3c_hci_send_ccc_cmd,
	.send_hdr_cmds		= i3c_hci_send_hdr_cmds,
	.priv_xfers		= i3c_hci_priv_xfers,
	.i2c_xfers		= i3c_hci_i2c_xfers,
	.attach_i3c_dev		= i3c_hci_attach_i3c_dev,
	.reattach_i3c_dev	= i3c_hci_reattach_i3c_dev,
	.detach_i3c_dev		= i3c_hci_detach_i3c_dev,
	.attach_i2c_dev		= i3c_hci_attach_i2c_dev,
	.detach_i2c_dev		= i3c_hci_detach_i2c_dev,
	.request_ibi		= i3c_hci_request_ibi,
	.free_ibi		= i3c_hci_free_ibi,
	.enable_ibi		= i3c_hci_enable_ibi,
	.disable_ibi		= i3c_hci_disable_ibi,
	.recycle_ibi_slot	= i3c_hci_recycle_ibi_slot,
	.enable_hotjoin		= i3c_hci_enable_hotjoin,
	.disable_hotjoin	= i3c_hci_disable_hotjoin,
};

static int ast2700_i3c_target_bus_init(struct i3c_master_controller *m)
{
	struct i3c_hci *hci = to_i3c_hci(m);
	struct i3c_dev_desc *desc = hci->master.this;
	u32 reg;
	int ret;

	dev_info(&hci->master.dev, "Secondary master Mode");

	ast_inhouse_write(ASPEED_I3C_SLV_PID_LO, SLV_PID_LO(desc->info.pid));
	ast_inhouse_write(ASPEED_I3C_SLV_PID_HI, SLV_PID_HI(desc->info.pid));

	desc->info.bcr = I3C_BCR_DEVICE_ROLE(I3C_BCR_I3C_MASTER) |
			 I3C_BCR_HDR_CAP | I3C_BCR_IBI_PAYLOAD |
			 I3C_BCR_IBI_REQ_CAP;
	reg = FIELD_PREP(ASPEED_I3C_SLV_CHAR_CTRL_DCR, desc->info.dcr) |
	      FIELD_PREP(ASPEED_I3C_SLV_CHAR_CTRL_BCR, desc->info.bcr);
	if (desc->info.static_addr) {
		reg |= ASPEED_I3C_SLV_CHAR_CTRL_STATIC_ADDR_EN |
		       FIELD_PREP(ASPEED_I3C_SLV_CHAR_CTRL_STATIC_ADDR,
				  desc->info.static_addr);
	}
	ast_inhouse_write(ASPEED_I3C_SLV_CHAR_CTRL, reg);
	reg = ast_inhouse_read(ASPEED_I3C_SLV_CAP_CTRL);
	/* Make slave will sned the ibi when bus idle */
	ast_inhouse_write(ASPEED_I3C_SLV_CAP_CTRL,
			  reg | ASPEED_I3C_SLV_CAP_CTRL_IBI_WAIT |
				  ASPEED_I3C_SLV_CAP_CTRL_HJ_WAIT);
	if (hci->caps & HC_CAP_HDR_DDR_EN)
		desc->info.hdr_cap |= BIT(I3C_HDR_DDR);
	if (hci->caps & HC_CAP_HDR_TS_EN) {
		if (reg_read(HC_CONTROL) & HC_CONTROL_I2C_TARGET_PRESENT)
			desc->info.hdr_cap |= BIT(I3C_HDR_TSL);
		else
			desc->info.hdr_cap |= BIT(I3C_HDR_TSP);
	}
	if (hci->caps & HC_CAP_HDR_BT_EN)
		desc->info.hdr_cap |= BIT(I3C_HDR_BT);
	ast_inhouse_write(ASPEED_I3C_SLV_STS8_GETCAPS_TGT, desc->info.hdr_cap);
	ast_inhouse_write(ASPEED_I3C_CTRL,
			  ASPEED_I3C_CTRL_INIT |
				  FIELD_PREP(ASPEED_I3C_CTRL_INIT_MODE,
					     INIT_SEC_MST_MODE));

	init_completion(&hci->ibi_comp);
	init_completion(&hci->pending_r_comp);
	ret = hci->io->init(hci);
	if (ret)
		return ret;

	return 0;
}

static void ast2700_i3c_target_bus_cleanup(struct i3c_master_controller *m)
{
	struct i3c_hci *hci = to_i3c_hci(m);

	DBG("");

	reg_clear(HC_CONTROL, HC_CONTROL_BUS_ENABLE);
	hci->io->cleanup(hci);
	kfree(hci->target_rx.buf);
}

static struct hci_xfer *
ast2700_i3c_target_priv_xfers(struct i3c_dev_desc *dev,
			      struct i3c_priv_xfer *i3c_xfers, int nxfers,
			      unsigned int tid)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	struct hci_xfer *xfer;
	unsigned int size_limit;
	int i, ret = 0;

	DBG("nxfers = %d", nxfers);

	xfer = hci_alloc_xfer(nxfers);
	if (!xfer)
		return xfer;

	size_limit = 1U << (16 + FIELD_GET(HC_CAP_MAX_DATA_LENGTH, hci->caps));

	for (i = 0; i < nxfers; i++) {
		if (!i3c_xfers[i].rnw) {
			xfer[i].data_len = i3c_xfers[i].len;
			xfer[i].rnw = i3c_xfers[i].rnw;
			xfer[i].data = (void *)i3c_xfers[i].data.out;
			xfer[i].cmd_tid = tid;
			hci->cmd->prep_i3c_xfer(hci, dev, &xfer[i]);
		} else {
			dev_err(&hci->master.dev,
				"target mode can't do priv_read command\n");
		}
	}
	ret = hci->io->queue_xfer(hci, xfer, nxfers);
	if (ret) {
		dev_err(&hci->master.dev, "queue xfer error %d", ret);
		hci_free_xfer(xfer, nxfers);
		return NULL;
	}

	return xfer;
}

static int ast2700_i3c_target_generate_ibi(struct i3c_dev_desc *dev, const u8 *data, int len)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	u32 reg;

	if (data || len != 0)
		return -EOPNOTSUPP;

	DBG("");

	reg = ast_inhouse_read(ASPEED_I3C_SLV_STS1);
	if ((reg & ASPEED_I3C_SLV_STS1_IBI_EN) == 0)
		return -EPERM;

	reinit_completion(&hci->ibi_comp);
	reg = ast_inhouse_read(ASPEED_I3C_SLV_CAP_CTRL);
	ast_inhouse_write(ASPEED_I3C_SLV_CAP_CTRL,
			  reg | ASPEED_I3C_SLV_CAP_CTRL_IBI_REQ);

	if (!wait_for_completion_timeout(&hci->ibi_comp,
					 msecs_to_jiffies(1000))) {
		dev_warn(&hci->master.dev, "timeout waiting for completion\n");
		return -EINVAL;
	}

	return 0;
}

static int ast2700_i3c_target_hj_req(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	u32 reg;
	int ret;

	DBG("");

	reg = ast_inhouse_read(ASPEED_I3C_STS);
	if ((reg & ASPEED_I3C_STS_SLV_DYNAMIC_ADDRESS_VALID))
		return -EINVAL;

	reg = ast_inhouse_read(ASPEED_I3C_SLV_STS1);
	if (!(reg & ASPEED_I3C_SLV_STS1_HJ_EN))
		return -EINVAL;

	reg = ast_inhouse_read(ASPEED_I3C_SLV_CAP_CTRL);
	ast_inhouse_write(ASPEED_I3C_SLV_CAP_CTRL,
			  reg | ASPEED_I3C_SLV_CAP_CTRL_HJ_REQ);
	ret = readx_poll_timeout(ast_inhouse_read, ASPEED_I3C_SLV_CAP_CTRL, reg,
				 !(reg & ASPEED_I3C_SLV_CAP_CTRL_HJ_REQ), 0,
				 1000000);
	if (ret) {
		dev_warn(&hci->master.dev, "timeout waiting for completion\n");
		return ret;
	}

	return 0;
}

static int
ast2700_i3c_target_pending_read_notify(struct i3c_dev_desc *dev,
				       struct i3c_priv_xfer *pending_read,
				       struct i3c_priv_xfer *ibi_notify)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	struct hci_xfer *ibi_xfer, *pending_read_xfer;
	u32 reg;

	if (!pending_read || !ibi_notify)
		return -EINVAL;

	reg = ast_inhouse_read(ASPEED_I3C_SLV_STS1);
	if ((reg & ASPEED_I3C_SLV_STS1_IBI_EN) == 0)
		return -EPERM;
	reinit_completion(&hci->pending_r_comp);
	ibi_xfer = ast2700_i3c_target_priv_xfers(dev, ibi_notify, 1,
						 TID_TARGET_IBI);
	if (!ibi_xfer)
		return -EINVAL;
	pending_read_xfer = ast2700_i3c_target_priv_xfers(dev, pending_read, 1,
							  TID_TARGET_RD_DATA);
	if (!pending_read_xfer)
		return -EINVAL;
	ast2700_i3c_target_generate_ibi(dev, NULL, 0);
	hci_free_xfer(ibi_xfer, 1);
	if (!wait_for_completion_timeout(&hci->pending_r_comp,
					 msecs_to_jiffies(1000))) {
		dev_warn(&hci->master.dev, "timeout waiting for master read\n");
		mipi_i3c_hci_pio_reset(hci);
		return -EINVAL;
	}
	hci_free_xfer(pending_read_xfer, 1);

	return 0;
}

static bool ast2700_i3c_target_is_ibi_enabled(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	u32 reg;

	reg = ast_inhouse_read(ASPEED_I3C_SLV_STS1);
	return !!(reg & ASPEED_I3C_SLV_STS1_IBI_EN);
}

static bool ast2700_i3c_target_is_hj_enabled(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct i3c_hci *hci = to_i3c_hci(m);
	u32 reg;

	reg = ast_inhouse_read(ASPEED_I3C_SLV_STS1);
	return !!(reg & ASPEED_I3C_SLV_STS1_HJ_EN);
}

static const struct i3c_target_ops ast2700_i3c_target_ops = {
	.bus_init = ast2700_i3c_target_bus_init,
	.bus_cleanup = ast2700_i3c_target_bus_cleanup,
	.hj_req = ast2700_i3c_target_hj_req,
	.priv_xfers = NULL,
	.generate_ibi = ast2700_i3c_target_generate_ibi,
	.pending_read_notify = ast2700_i3c_target_pending_read_notify,
	.is_ibi_enabled = ast2700_i3c_target_is_ibi_enabled,
	.is_hj_enabled = ast2700_i3c_target_is_hj_enabled,
};

static irqreturn_t i3c_hci_irq_handler(int irq, void *dev_id)
{
	struct i3c_hci *hci = dev_id;
	irqreturn_t result = IRQ_NONE;
	u32 val;

	val = reg_read(INTR_STATUS);
	DBG("INTR_STATUS = %#x", val);

	if (val) {
		reg_write(INTR_STATUS, val);
	} else {
		/* v1.0 does not have PIO cascaded notification bits */
		val |= INTR_HC_PIO;
	}

	if (val & INTR_HC_RESET_CANCEL) {
		DBG("cancelled reset");
		val &= ~INTR_HC_RESET_CANCEL;
	}
	if (val & INTR_HC_INTERNAL_ERR) {
		dev_err(&hci->master.dev, "Host Controller Internal Error\n");
		val &= ~INTR_HC_INTERNAL_ERR;
	}
	if (val)
		dev_err(&hci->master.dev, "unexpected INTR_STATUS %#x\n", val);
	else
		result = IRQ_HANDLED;

	return result;
}

static irqreturn_t i3c_aspeed_irq_handler(int irqn, void *dev_id)
{
	struct i3c_hci *hci = dev_id;
	u32 val, inhouse_val;
	int result = -1;

	val = ast_inhouse_read(ASPEED_I3C_INTR_SUM_STATUS);
	DBG("Global INTR_STATUS = %#x\n", val);

	if (val & ASPEED_INTR_SUM_CAP) {
		i3c_hci_irq_handler(irqn, dev_id);
		val &= ~ASPEED_INTR_SUM_CAP;
	}
	if (val & ASPEED_INTR_SUM_PIO) {
		hci->io->irq_handler(hci, 0);
		val &= ~ASPEED_INTR_SUM_PIO;
	}
	if (val & ASPEED_INTR_SUM_RHS) {
		/*
		 * ASPEED only has one ring, and HCI v1.2 doesn't have a register to indicate which
		 * ring has the interrupt.
		 */
		hci->io->irq_handler(hci, 1);
		val &= ~ASPEED_INTR_SUM_RHS;
	}
	if (val & ASPEED_INTR_SUM_INHOUSE) {
		inhouse_val = ast_inhouse_read(ASPEED_I3C_INTR_STATUS);
		DBG("Inhouse INTR_STATUS = %#x/%#x\n", inhouse_val,
		    ast_inhouse_read(ASPEED_I3C_INTR_SIGNAL_ENABLE));
		ast_inhouse_write(ASPEED_I3C_INTR_STATUS, inhouse_val);
		val &= ~ASPEED_INTR_SUM_INHOUSE;
	}

	if (val)
		dev_err(&hci->master.dev, "unexpected INTR_SUN_STATUS %#x\n",
			val);
	else
		result = IRQ_HANDLED;

	return result;
}

static int i3c_hci_init(struct i3c_hci *hci)
{
	u32 regval, offset;
	int ret;

	/* Validate HCI hardware version */
	regval = reg_read(HCI_VERSION);
	hci->version_major = (regval >> 8) & 0xf;
	hci->version_minor = (regval >> 4) & 0xf;
	hci->revision = regval & 0xf;
	printk(KERN_NOTICE "i3c_hci_init");
	dev_notice(&hci->master.dev, "MIPI I3C HCI v%u.%u r%02u\n",
		   hci->version_major, hci->version_minor, hci->revision);
	/* known versions */
	switch (regval & ~0xf) {
	case 0x100:	/* version 1.0 */
	case 0x110:	/* version 1.1 */
	case 0x200:	/* version 2.0 */
		break;
	default:
		dev_err(&hci->master.dev, "unsupported HCI version\n");
		return -EPROTONOSUPPORT;
	}

	hci->caps = reg_read(HC_CAPABILITIES);
	dev_info(&hci->master.dev, "caps = %#x", hci->caps);

	regval = reg_read(DAT_SECTION);
	offset = FIELD_GET(DAT_TABLE_OFFSET, regval);
	hci->DAT_regs = offset ? hci->base_regs + offset : NULL;
	hci->DAT_entries = FIELD_GET(DAT_TABLE_SIZE, regval);
	hci->DAT_entry_size = FIELD_GET(DAT_ENTRY_SIZE, regval) ? 0 : 8;
	dev_info(&hci->master.dev, "DAT: %u %u-bytes entries at offset %#x\n",
		 hci->DAT_entries, hci->DAT_entry_size, offset);

	regval = reg_read(DCT_SECTION);
	offset = FIELD_GET(DCT_TABLE_OFFSET, regval);
	hci->DCT_regs = offset ? hci->base_regs + offset : NULL;
	hci->DCT_entries = FIELD_GET(DCT_TABLE_SIZE, regval);
	hci->DCT_entry_size = FIELD_GET(DCT_ENTRY_SIZE, regval) ? 0 : 16;
	dev_info(&hci->master.dev, "DCT: %u %u-bytes entries at offset %#x\n",
		 hci->DCT_entries, hci->DCT_entry_size, offset);

	regval = reg_read(RING_HEADERS_SECTION);
	offset = FIELD_GET(RING_HEADERS_OFFSET, regval);
	hci->RHS_regs = offset ? hci->base_regs + offset : NULL;
	dev_info(&hci->master.dev, "Ring Headers at offset %#x\n", offset);

	regval = reg_read(PIO_SECTION);
	offset = FIELD_GET(PIO_REGS_OFFSET, regval);
	hci->PIO_regs = offset ? hci->base_regs + offset : NULL;
	dev_info(&hci->master.dev, "PIO section at offset %#x\n", offset);

	regval = reg_read(EXT_CAPS_SECTION);
	offset = FIELD_GET(EXT_CAPS_OFFSET, regval);
	hci->EXTCAPS_regs = offset ? hci->base_regs + offset : NULL;
	dev_info(&hci->master.dev, "Extended Caps at offset %#x\n", offset);

	ret = i3c_hci_parse_ext_caps(hci);
	if (ret)
		return ret;

	/*
	 * Now let's reset the hardware.
	 * SOFT_RST must be clear before we write to it.
	 * Then we must wait until it clears again.
	 */
	ret = readx_poll_timeout(reg_read, RESET_CONTROL, regval,
				 !(regval & SOFT_RST), 1, 10000);
	if (ret)
		return -ENXIO;
	reg_write(RESET_CONTROL, SOFT_RST);
	ret = readx_poll_timeout(reg_read, RESET_CONTROL, regval,
				 !(regval & SOFT_RST), 1, 10000);
	if (ret)
		return -ENXIO;

	/* Disable all interrupts and allow all signal updates */
	reg_write(INTR_SIGNAL_ENABLE, 0x0);
	reg_write(INTR_STATUS_ENABLE, 0xffffffff);
#ifdef CONFIG_ARCH_ASPEED
	ast_inhouse_write(ASPEED_I3C_INTR_SIGNAL_ENABLE, 0);
	ast_inhouse_write(ASPEED_I3C_INTR_STATUS_ENABLE, 0xffffffff);
#endif

	/* Make sure our data ordering fits the host's */
	regval = reg_read(HC_CONTROL);
	if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN)) {
		if (!(regval & HC_CONTROL_DATA_BIG_ENDIAN)) {
			regval |= HC_CONTROL_DATA_BIG_ENDIAN;
			reg_write(HC_CONTROL, regval);
			regval = reg_read(HC_CONTROL);
			if (!(regval & HC_CONTROL_DATA_BIG_ENDIAN)) {
				dev_err(&hci->master.dev, "cannot set BE mode\n");
				return -EOPNOTSUPP;
			}
		}
	} else {
		if (regval & HC_CONTROL_DATA_BIG_ENDIAN) {
			regval &= ~HC_CONTROL_DATA_BIG_ENDIAN;
			reg_write(HC_CONTROL, regval);
			regval = reg_read(HC_CONTROL);
			if (regval & HC_CONTROL_DATA_BIG_ENDIAN) {
				dev_err(&hci->master.dev, "cannot clear BE mode\n");
				return -EOPNOTSUPP;
			}
		}
	}

	/* Select our command descriptor model */
	switch (FIELD_GET(HC_CAP_CMD_SIZE, hci->caps)) {
	case 0:
		hci->cmd = &mipi_i3c_hci_cmd_v1;
		break;
	case 1:
		hci->cmd = &mipi_i3c_hci_cmd_v2;
		break;
	default:
		dev_err(&hci->master.dev, "wrong CMD_SIZE capability value\n");
		return -EINVAL;
	}

	/* Try activating DMA operations first */
	if (hci->RHS_regs) {
		reg_clear(HC_CONTROL, HC_CONTROL_PIO_MODE);
		if (reg_read(HC_CONTROL) & HC_CONTROL_PIO_MODE) {
			dev_err(&hci->master.dev, "PIO mode is stuck\n");
			ret = -EIO;
		} else if (!hci->dma_rst) {
			dev_err(&hci->master.dev,
				"missing or invalid i3c dma reset controller device tree entry\n");
			ret = -EIO;
		} else {
			hci->io = &mipi_i3c_hci_dma;
			reset_control_deassert(hci->dma_rst);
			dev_info(&hci->master.dev, "Using DMA\n");
		}
	}

	/* If no DMA, try PIO */
	if (!hci->io && hci->PIO_regs) {
		reg_set(HC_CONTROL, HC_CONTROL_PIO_MODE);
		if (!(reg_read(HC_CONTROL) & HC_CONTROL_PIO_MODE)) {
			dev_err(&hci->master.dev, "DMA mode is stuck\n");
			ret = -EIO;
		} else {
			hci->io = &mipi_i3c_hci_pio;
			dev_info(&hci->master.dev, "Using PIO\n");
		}
	}

	if (!hci->io) {
		dev_err(&hci->master.dev, "neither DMA nor PIO can be used\n");
		if (!ret)
			ret = -EINVAL;
		return ret;
	}

	return 0;
}

#ifdef CONFIG_ARCH_ASPEED
static int aspeed_i3c_of_populate_bus_timing(struct i3c_hci *hci,
					     struct device_node *np)
{
	u16 hcnt, lcnt;
	unsigned long core_rate, core_period;

	core_rate = clk_get_rate(hci->clk);
	/* core_period is in nanosecond */
	core_period = DIV_ROUND_UP(1000000000, core_rate);

	dev_info(&hci->master.dev, "core rate = %ld core period = %ld ns",
		 core_rate, core_period);

	hcnt = DIV_ROUND_CLOSEST(PHY_I2C_FM_DEFAULT_CAS_NS, core_period) - 1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I2C_FM_DEFAULT_SU_STO_NS, core_period) - 1;
	ast_phy_write(PHY_I2C_FM_CTRL0,
		      FIELD_PREP(PHY_I2C_FM_CTRL0_CAS, hcnt) |
			      FIELD_PREP(PHY_I2C_FM_CTRL0_SU_STO, lcnt));

	hcnt = DIV_ROUND_CLOSEST(PHY_I2C_FM_DEFAULT_SCL_H_NS, core_period) - 1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I2C_FM_DEFAULT_SCL_L_NS, core_period) - 1;
	ast_phy_write(PHY_I2C_FM_CTRL1,
		      FIELD_PREP(PHY_I2C_FM_CTRL1_SCL_H, hcnt) |
			      FIELD_PREP(PHY_I2C_FM_CTRL1_SCL_L, lcnt));
	ast_phy_write(PHY_I2C_FM_CTRL2,
		      FIELD_PREP(PHY_I2C_FM_CTRL2_ACK_H, hcnt) |
			      FIELD_PREP(PHY_I2C_FM_CTRL2_ACK_L, hcnt));
	hcnt = DIV_ROUND_CLOSEST(PHY_I2C_FM_DEFAULT_HD_DAT, core_period) - 1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I2C_FM_DEFAULT_AHD_DAT, core_period) - 1;
	ast_phy_write(PHY_I2C_FM_CTRL3,
		      FIELD_PREP(PHY_I2C_FM_CTRL3_HD_DAT, hcnt) |
			      FIELD_PREP(PHY_I2C_FM_CTRL3_AHD_DAT, lcnt));

	hcnt = DIV_ROUND_CLOSEST(PHY_I2C_FMP_DEFAULT_CAS_NS, core_period) - 1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I2C_FMP_DEFAULT_SU_STO_NS, core_period) - 1;
	ast_phy_write(PHY_I2C_FMP_CTRL0,
		      FIELD_PREP(PHY_I2C_FMP_CTRL0_CAS, hcnt) |
			      FIELD_PREP(PHY_I2C_FMP_CTRL0_SU_STO, lcnt));

	hcnt = DIV_ROUND_CLOSEST(PHY_I2C_FMP_DEFAULT_SCL_H_NS, core_period) - 1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I2C_FMP_DEFAULT_SCL_L_NS, core_period) - 1;
	ast_phy_write(PHY_I2C_FMP_CTRL1,
		      FIELD_PREP(PHY_I2C_FMP_CTRL1_SCL_H, hcnt) |
			      FIELD_PREP(PHY_I2C_FMP_CTRL1_SCL_L, lcnt));
	ast_phy_write(PHY_I2C_FMP_CTRL2,
		      FIELD_PREP(PHY_I2C_FMP_CTRL2_ACK_H, hcnt) |
			      FIELD_PREP(PHY_I2C_FMP_CTRL2_ACK_L, hcnt));
	hcnt = DIV_ROUND_CLOSEST(PHY_I2C_FMP_DEFAULT_HD_DAT, core_period) - 1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I2C_FMP_DEFAULT_AHD_DAT, core_period) - 1;
	ast_phy_write(PHY_I2C_FMP_CTRL3,
		      FIELD_PREP(PHY_I2C_FMP_CTRL3_HD_DAT, hcnt) |
			      FIELD_PREP(PHY_I2C_FMP_CTRL3_AHD_DAT, lcnt));

	hcnt = DIV_ROUND_CLOSEST(PHY_I3C_OD_DEFAULT_CAS_NS, core_period) - 1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I3C_OD_DEFAULT_SU_STO_NS, core_period) - 1;
	ast_phy_write(PHY_I3C_OD_CTRL0,
		      FIELD_PREP(PHY_I3C_OD_CTRL0_CAS, hcnt) |
			      FIELD_PREP(PHY_I3C_OD_CTRL0_SU_STO, lcnt));

	hcnt = DIV_ROUND_CLOSEST(PHY_I3C_OD_DEFAULT_SCL_H_NS, core_period) - 1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I3C_OD_DEFAULT_SCL_L_NS, core_period) - 1;
	ast_phy_write(PHY_I3C_OD_CTRL1,
		      FIELD_PREP(PHY_I3C_OD_CTRL1_SCL_H, hcnt) |
			      FIELD_PREP(PHY_I3C_OD_CTRL1_SCL_L, lcnt));
	ast_phy_write(PHY_I3C_OD_CTRL2,
		      FIELD_PREP(PHY_I3C_OD_CTRL2_ACK_H, hcnt) |
			      FIELD_PREP(PHY_I3C_OD_CTRL2_ACK_L, hcnt));
	hcnt = DIV_ROUND_CLOSEST(PHY_I3C_OD_DEFAULT_HD_DAT, core_period) - 1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I3C_OD_DEFAULT_AHD_DAT, core_period) - 1;
	ast_phy_write(PHY_I3C_OD_CTRL3,
		      FIELD_PREP(PHY_I3C_OD_CTRL3_HD_DAT, hcnt) |
			      FIELD_PREP(PHY_I3C_OD_CTRL3_AHD_DAT, lcnt));

	hcnt = DIV_ROUND_CLOSEST(PHY_I3C_SDR0_DEFAULT_SCL_H_NS, core_period) - 1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I3C_SDR0_DEFAULT_SCL_L_NS, core_period) - 1;
	ast_phy_write(PHY_I3C_SDR0_CTRL0,
		      FIELD_PREP(PHY_I3C_SDR0_CTRL0_SCL_H, hcnt) |
			      FIELD_PREP(PHY_I3C_SDR0_CTRL0_SCL_L, lcnt));
	hcnt = DIV_ROUND_CLOSEST(PHY_I3C_SDR0_DEFAULT_TBIT_H_NS, core_period) - 1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I3C_SDR0_DEFAULT_TBIT_L_NS, core_period) - 1;
	ast_phy_write(PHY_I3C_SDR0_CTRL1,
		      FIELD_PREP(PHY_I3C_SDR0_CTRL1_TBIT_H, hcnt) |
			      FIELD_PREP(PHY_I3C_SDR0_CTRL1_TBIT_L, lcnt));
	hcnt = DIV_ROUND_CLOSEST(PHY_I3C_SDR0_DEFAULT_HD_PP_NS, core_period) -
	       1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I3C_SDR0_DEFAULT_TBIT_HD_PP_NS,
				 core_period) -
	       1;
	ast_phy_write(PHY_I3C_SDR0_CTRL2,
		      FIELD_PREP(PHY_I3C_SDR0_CTRL2_HD_PP, hcnt) |
			      FIELD_PREP(PHY_I3C_SDR0_CTRL2_TBIT_HD_PP, lcnt));

	hcnt = DIV_ROUND_CLOSEST(PHY_I3C_DDR_DEFAULT_SCL_H_NS, core_period) - 1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I3C_DDR_DEFAULT_SCL_L_NS, core_period) - 1;
	ast_phy_write(PHY_I3C_DDR_CTRL0,
		      FIELD_PREP(PHY_I3C_DDR_CTRL0_SCL_H, hcnt) |
			      FIELD_PREP(PHY_I3C_DDR_CTRL0_SCL_L, lcnt));
	hcnt = DIV_ROUND_CLOSEST(PHY_I3C_DDR_DEFAULT_TBIT_H_NS, core_period) - 1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I3C_DDR_DEFAULT_TBIT_L_NS, core_period) - 1;
	ast_phy_write(PHY_I3C_DDR_CTRL1,
		      FIELD_PREP(PHY_I3C_DDR_CTRL1_TBIT_H, hcnt) |
			      FIELD_PREP(PHY_I3C_DDR_CTRL1_TBIT_L, lcnt));
	hcnt = DIV_ROUND_CLOSEST(PHY_I3C_DDR_DEFAULT_HD_PP_NS, core_period) -
	       1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I3C_DDR_DEFAULT_TBIT_HD_PP_NS,
				 core_period) -
	       1;
	ast_phy_write(PHY_I3C_DDR_CTRL2,
		      FIELD_PREP(PHY_I3C_DDR_CTRL2_HD_PP, hcnt) |
			      FIELD_PREP(PHY_I3C_DDR_CTRL2_TBIT_HD_PP, lcnt));

	hcnt = DIV_ROUND_CLOSEST(PHY_I3C_SR_P_DEFAULT_HD_NS, core_period) - 1;
	lcnt = DIV_ROUND_CLOSEST(PHY_I3C_SR_P_DEFAULT_SCL_L_NS, core_period) - 1;
	ast_phy_write(PHY_I3C_SR_P_PREPARE_CTRL,
		      FIELD_PREP(PHY_I3C_SR_P_PREPARE_CTRL_HD, hcnt) |
			      FIELD_PREP(PHY_I3C_SR_P_PREPARE_CTRL_SCL_L,
					 lcnt));
	ast_phy_write(PHY_PULLUP_EN, 0x0);

	return 0;
}
#endif

static void i3c_hci_hj_work(struct work_struct *work)
{
	struct i3c_hci *hci;

	hci = container_of(work, struct i3c_hci, hj_work);
	i3c_master_do_daa(&hci->master);
}

static int i3c_hci_probe(struct platform_device *pdev)
{
	struct i3c_hci *hci;
	int irq, ret;

	hci = devm_kzalloc(&pdev->dev, sizeof(*hci), GFP_KERNEL);
	if (!hci)
		return -ENOMEM;
	hci->base_regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hci->base_regs))
		return PTR_ERR(hci->base_regs);

	platform_set_drvdata(pdev, hci);
	/* temporary for dev_printk's, to be replaced in i3c_master_register */
	hci->master.dev.init_name = dev_name(&pdev->dev);

	hci->rst = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(hci->rst)) {
		dev_err(&pdev->dev,
			"missing or invalid reset controller device tree entry");
		return PTR_ERR(hci->rst);
	}
	reset_control_assert(hci->rst);
	reset_control_deassert(hci->rst);

	hci->dma_rst = devm_reset_control_get_shared_by_index(&pdev->dev, 1);
	if (IS_ERR(hci->dma_rst))
		hci->dma_rst = NULL;

	hci->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(hci->clk)) {
		dev_err(&pdev->dev,
			"missing or invalid clock controller device tree entry");
		return PTR_ERR(hci->clk);
	}

	ret = clk_prepare_enable(hci->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable i3c clock.\n");
		return ret;
	}

	ret = i3c_hci_init(hci);
	if (ret)
		return ret;

#ifdef CONFIG_ARCH_ASPEED
	ret = aspeed_i3c_of_populate_bus_timing(hci, pdev->dev.of_node);
	if (ret)
		return ret;
#endif

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, irq, i3c_aspeed_irq_handler,
			       0, NULL, hci);
	if (ret)
		return ret;

	INIT_WORK(&hci->hj_work, i3c_hci_hj_work);
	ret = i3c_register(&hci->master, &pdev->dev, &i3c_hci_ops,
			   &ast2700_i3c_target_ops, false);
	if (ret)
		return ret;
	if (!hci->master.target && hci->master.bus.context != I3C_BUS_CONTEXT_JESD403)
		mipi_i3c_hci_iba_ctrl(hci, true);

	return 0;
}

static void i3c_hci_remove(struct platform_device *pdev)
{
	struct i3c_hci *hci = platform_get_drvdata(pdev);

	i3c_master_unregister(&hci->master);
}

static const __maybe_unused struct of_device_id i3c_hci_of_match[] = {
	{ .compatible = "mipi-i3c-hci", },
	{ .compatible = "aspeed-i3c-hci", },
	{},
};
MODULE_DEVICE_TABLE(of, i3c_hci_of_match);

static struct platform_driver i3c_hci_driver = {
	.probe = i3c_hci_probe,
	.remove_new = i3c_hci_remove,
	.driver = {
		.name = "mipi-i3c-hci",
		.of_match_table = of_match_ptr(i3c_hci_of_match),
	},
};
module_platform_driver(i3c_hci_driver);
MODULE_ALIAS("platform:mipi-i3c-hci");

MODULE_AUTHOR("Nicolas Pitre <npitre@baylibre.com>");
MODULE_DESCRIPTION("MIPI I3C HCI driver");
MODULE_LICENSE("Dual BSD/GPL");
