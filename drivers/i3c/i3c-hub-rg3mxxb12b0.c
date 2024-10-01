// SPDX-License-Identifier: GPL-2.0
/*
 * Renedas RG3MxxB12A1 hub driver: WIP, SMBus agent mode only
 *
 * Copyright (c) 2024 Code Construct
 */

#define DEBUG

#include <linux/i3c/device.h>
#include <linux/i3c/master.h>
#include <linux/of_address.h>
#include <linux/regmap.h>

#define RG3M_PORT_MAX	8
#define RG3M_BUF_SIZE	80

#define RG3M_REG_DEV_INFO0		0
#define RG3M_REG_DEV_INFO1		1

#define RG3M_REG_UNLOCK_DEV_PROT	16
#define RG3M_REG_UNLOCK_DEV_PROT_CODE	0x69

#define RG3M_REG_AGENT_ENABLE		24

#define RG3M_REG_AGENT_IBI_ENABLE	27

#define RG3M_REG_GPIO_ENABLE		30

#define RG3M_REG_DEV_PORT_STATUS	32
#define RG3M_REG_TARGET_STATUS		33

#define RG3M_REG_AGENT_TX_REQ		80

#define RG3M_REG_AGENT_CNTLR_STATUS	100
#define RG3M_REG_AGENT_CNTRL_STATUS_FINISH		1
#define RG3M_REG_AGENT_CNTRL_STATUS_RX_BUF0		2
#define RG3M_REG_AGENT_CNTRL_STATUS_RX_BUF1		4
#define RG3M_REG_AGENT_CNTRL_STATUS_RX_BUF_OVF		8
#define RG3M_REG_AGENT_CNTRL_STATUS_TXN_SHIFT		4
#define RG3M_REG_AGENT_CNTRL_STATUS_TXN_OK		0
#define RG3M_REG_AGENT_CNTRL_STATUS_TXN_ADDR_NAK	1
#define RG3M_REG_AGENT_CNTRL_STATUS_TXN_DATA_NAK	2
#define RG3M_REG_AGENT_CNTRL_STATUS_TXN_WTR_NAK		3
#define RG3M_REG_AGENT_CNTRL_STATUS_TXN_SYNC_RCV	4
#define RG3M_REG_AGENT_CNTRL_STATUS_TXN_SYNC_RCVCLR	5
#define RG3M_REG_AGENT_CNTRL_STATUS_TXN_FAULT		6
#define RG3M_REG_AGENT_CNTRL_STATUS_TXN_ARB_LOSS	7
#define RG3M_REG_AGENT_CNTRL_STATUS_TXN_SCL_TO		8

#define RG3M_REG_DEV_CMD		124
#define RG3M_REG_DEV_CMD_DEV_RESET	0x96

#define RG3M_REG_PAGE			127

/* page numbers, per port */
#define RG3M_PAGE_AGENT_TX(p)		(16 + (4 * (p)) + 0)
#define RG3M_PAGE_AGENT_ADDRS(p)	(16 + (4 * (p)) + 1)
#define RG3M_PAGE_AGENT_RX0(p)		(16 + (4 * (p)) + 2)
#define RG3M_PAGE_AGENT_RX1(p)		(16 + (4 * (p)) + 3)

static const bool ibi_paranoia = true;

struct rg3m_port {
	enum {
		RG3M_PORT_MODE_I3C,
		RG3M_PORT_MODE_AGENT,
	} mode;
	struct rg3m_i2c_agent *agent;
	struct device_node *of_node;
};

struct rg3m {
	struct i3c_device *i3c;
	struct regmap *regmap;

	unsigned int n_ports;
	struct rg3m_port ports[RG3M_PORT_MAX];

	unsigned int cur_page;
};

/* utils */

static int rg3m_write_paged(struct rg3m *rg3m, unsigned int page,
			    unsigned int addr, const void *data, size_t size)
{
	int rc;

	if (rg3m->cur_page != page) {
		rc = regmap_write(rg3m->regmap, RG3M_REG_PAGE, page);
		if (rc)
			return rc;
		rg3m->cur_page = page;
	}

	rc = regmap_bulk_write(rg3m->regmap, 128 + addr, data, size);

	return rc;
}

static int rg3m_write_paged_u8(struct rg3m *rg3m, unsigned int page,
			       unsigned int addr, u8 data)
{
	return rg3m_write_paged(rg3m, page, addr, &data, 1);
}

static int rg3m_read_paged(struct rg3m *rg3m, unsigned int page,
			   unsigned int addr, void *data, size_t size)
{
	int rc;

	if (rg3m->cur_page != page) {
		rc = regmap_write(rg3m->regmap, RG3M_REG_PAGE, page);
		if (rc)
			return rc;
		rg3m->cur_page = page;
	}

	rc = regmap_bulk_read(rg3m->regmap, 128 + addr, data, size);

	return rc;
}

/* SMBus Agent */
struct rg3m_i2c_agent {
	struct rg3m *rg3m;
	struct i2c_adapter i2c;
	unsigned int port;

	/* target handling */
	struct i2c_client *client;
	u8 target_rx_buf[RG3M_BUF_SIZE];

	/* protects tx_res */
	spinlock_t lock;
	u8 tx_res;

	struct completion completion;
};

struct rg3m_agent_tx_hdr {
	u8 addr_rnw;
	u8 type;
	u8 wr_len;
	u8 rd_len;
};

static int rg3m_agent_i2c_xfer_one(struct rg3m_i2c_agent *agent,
				   struct i2c_msg *msg, bool last)
{
	const unsigned int port_bit = 1u << agent->port;
	const bool read = msg->flags & I2C_M_RD;
	unsigned int page, port_stat, txn_stat;
	struct rg3m_agent_tx_hdr hdr = { 0 };
	struct device *dev = &agent->i2c.dev;
	unsigned int port = agent->port;
	struct rg3m *rg3m = agent->rg3m;
	unsigned long flags, wait_time;
	int rc;

	hdr.addr_rnw = (msg->addr << 1) | (read ? 1 : 0);
	hdr.type = 0; /* todo: other defaults */
	if (!last)
		hdr.type |= 1 << 7;
	if (read)
		hdr.rd_len = msg->len;
	else
		hdr.wr_len = msg->len;

	page = RG3M_PAGE_AGENT_TX(port);

	rc = rg3m_write_paged(rg3m, page, 0, &hdr, sizeof(hdr));
	if (rc) {
		dev_dbg(dev, "write header failed %d\n", rc);
		return rc;
	}

	if (!read && msg->len) {
		rc = rg3m_write_paged(rg3m, page, 4, msg->buf, msg->len);
		if (rc) {
			dev_dbg(dev, "write data failed %d\n", rc);
			return rc;
		}
	}

	/* start transfer */
	rc = regmap_write(rg3m->regmap, RG3M_REG_AGENT_TX_REQ, port_bit);
	if (rc) {
		dev_dbg(dev, "write start failed %d\n", rc);
		return rc;
	}

	wait_time = wait_for_completion_timeout(&agent->completion,
						agent->i2c.timeout);
	if (!wait_time) {
		dev_dbg(&rg3m->i3c->dev, "tx timeout!\n");
		return -EIO;
	}
	spin_lock_irqsave(&agent->lock, flags);
	port_stat = agent->tx_res;
	spin_unlock_irqrestore(&agent->lock, flags);

	txn_stat = port_stat >> RG3M_REG_AGENT_CNTRL_STATUS_TXN_SHIFT;
	switch (txn_stat) {
	case RG3M_REG_AGENT_CNTRL_STATUS_TXN_OK:
		rc = 0;
		break;
	case RG3M_REG_AGENT_CNTRL_STATUS_TXN_ADDR_NAK:
		rc = -ENXIO;
		break;
	case RG3M_REG_AGENT_CNTRL_STATUS_TXN_DATA_NAK:
		rc = -EIO;
		break;
	case RG3M_REG_AGENT_CNTRL_STATUS_TXN_ARB_LOSS:
		rc = -EAGAIN;
		break;
	case RG3M_REG_AGENT_CNTRL_STATUS_TXN_SCL_TO:
		rc = -ETIMEDOUT;
		break;
	case RG3M_REG_AGENT_CNTRL_STATUS_TXN_WTR_NAK:
		/* we don't issue write-then-read transactions */
		fallthrough;
	default:
		dev_dbg(&agent->i2c.dev,
			"unhandled transaction status 0x%0x", txn_stat);
		rc = -EIO;
		break;
	}

	if (!rc && read && msg->len) {
		rc = rg3m_read_paged(rg3m, page, 4, msg->buf, msg->len);
		if (rc) {
			dev_dbg(dev, "read data failed %d\n", rc);
			return rc;
		}
	}

	/* clear status */
	regmap_write(rg3m->regmap, RG3M_REG_AGENT_CNTLR_STATUS + agent->port,
		     RG3M_REG_AGENT_CNTRL_STATUS_FINISH);

	return rc;
}

static void rg3m_agent_target_rx(struct rg3m_i2c_agent *agent, unsigned int n)
{
	struct {
		u8 len;
		u8 addr;
	} hdr;
	struct rg3m *rg3m = agent->rg3m;
	u8 tmp, len, addr;
	unsigned int i, page;
	int rc;

	if (!agent->client)
		goto ack;

	page = n ? RG3M_PAGE_AGENT_RX1(agent->port) :
		RG3M_PAGE_AGENT_RX0(agent->port);

	/* We need the length to figure out the size of our read. But we also
	 * read the first byte of i2c data in the same read; the hardware has
	 * no facility for filtering on incoming local addresses, so we have a
	 * fast-path to aborting the transaction if it's not targeted to us.
	 */
	rc = rg3m_read_paged(rg3m, page, 0, &hdr, sizeof(hdr));
	if (rc)
		goto ack;

	len = min(hdr.len, RG3M_BUF_SIZE);
	if (len == 0)
		goto ack;

	if (hdr.addr & 0x1) {
		dev_dbg(&rg3m->i3c->dev, "unsupported read requested\n");
		goto ack;
	}

	/* not for us? discard and ack */
	addr = hdr.addr >> 1;
	if (addr != (agent->client->addr & 0x7f))
		goto ack;

	memset(agent->target_rx_buf, 0, sizeof(agent->target_rx_buf));
	rc = rg3m_read_paged(rg3m, page, 2, agent->target_rx_buf, len);
	if (rc)
		goto ack;

	/* synthesize i2c target events from the target write */
	tmp = 0;
	rc = i2c_slave_event(agent->client, I2C_SLAVE_WRITE_REQUESTED, &tmp);
	if (rc)
		goto stop;

	/* len includes the address byte, which we have already read */
	for (i = 0; i < len - 1; i++) {
		tmp = agent->target_rx_buf[i];
		i2c_slave_event(agent->client, I2C_SLAVE_WRITE_RECEIVED, &tmp);
	}

stop:
	tmp = 0;
	i2c_slave_event(agent->client, I2C_SLAVE_STOP, &tmp);

ack:
	tmp = n ? RG3M_REG_AGENT_CNTRL_STATUS_RX_BUF1 :
		RG3M_REG_AGENT_CNTRL_STATUS_RX_BUF0;

	regmap_write(rg3m->regmap, RG3M_REG_AGENT_CNTLR_STATUS + agent->port,
		     tmp);
}

static void rg3m_agent_ibi(struct rg3m_i2c_agent *agent)
{
	struct rg3m *rg3m = agent->rg3m;
	unsigned long flags;
	unsigned int stat;
	int rc;

	rc = regmap_read(rg3m->regmap,
			 RG3M_REG_AGENT_CNTLR_STATUS + agent->port,
			 &stat);
	if (rc)
		return;

	if (stat & RG3M_REG_AGENT_CNTRL_STATUS_FINISH) {
		spin_lock_irqsave(&agent->lock, flags);
		agent->tx_res = stat;
		complete(&agent->completion);
		spin_unlock_irqrestore(&agent->lock, flags);
	}

	if (stat & RG3M_REG_AGENT_CNTRL_STATUS_RX_BUF0)
		rg3m_agent_target_rx(agent, 0);

	if (stat & RG3M_REG_AGENT_CNTRL_STATUS_RX_BUF1)
		rg3m_agent_target_rx(agent, 1);

	if (stat & RG3M_REG_AGENT_CNTRL_STATUS_RX_BUF_OVF) {
		dev_warn(&agent->i2c.dev, "rx overflow\n");
		regmap_write(rg3m->regmap,
			     RG3M_REG_AGENT_CNTLR_STATUS + agent->port,
			     RG3M_REG_AGENT_CNTRL_STATUS_RX_BUF_OVF);
	}

}

static int rg3m_agent_i2c_xfer(struct i2c_adapter *i2c, struct i2c_msg *msgs,
			       int n_msgs)
{
	struct rg3m_i2c_agent *agent = i2c_get_adapdata(i2c);
	unsigned int i;

	for (i = 0; i < n_msgs; i++) {
		bool last = i == (n_msgs - 1);
		int rc;

		rc = rg3m_agent_i2c_xfer_one(agent, &msgs[i], last);
		if (rc)
			return rc;
	}

	return n_msgs;
}

static int rg3m_agent_set_target_addr(struct rg3m_i2c_agent *agent, u8 addr)
{
	struct rg3m *rg3m = agent->rg3m;

	rg3m_write_paged_u8(rg3m, RG3M_PAGE_AGENT_ADDRS(agent->port),
			    0, 0);
	return 0;
}

static int rg3m_agent_i2c_reg_target(struct i2c_client *client)
{
	struct rg3m_i2c_agent *agent = i2c_get_adapdata(client->adapter);

	if (agent->client)
		return -EBUSY;

	agent->client = client;

	return rg3m_agent_set_target_addr(agent, (client->addr & 0x7f) << 1);
}

static int rg3m_agent_i2c_unreg_target(struct i2c_client *client)
{
	struct rg3m_i2c_agent *agent = i2c_get_adapdata(client->adapter);

	agent->client = NULL;
	rg3m_agent_set_target_addr(agent, 0);

	return 0;
}

static u32 rg3m_agent_i2c_functionality(struct i2c_adapter *i2c)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA;
}

static const struct i2c_algorithm rg3m_i2c_algo = {
	.xfer = rg3m_agent_i2c_xfer,
	.reg_target = rg3m_agent_i2c_reg_target,
	.unreg_target = rg3m_agent_i2c_unreg_target,
	.functionality = rg3m_agent_i2c_functionality,
};

static struct device_node *rg3m_find_port_node(struct rg3m *rg3m,
					       unsigned int port_nr)
{
	struct device_node *parent, *np;
	u64 addr;
	int rc;

	parent = rg3m->i3c->dev.of_node;
	if (!parent)
		return NULL;

	for_each_available_child_of_node(parent, np) {
		if (!of_device_is_compatible(np, "renesas,rg3mxxb12b0-port"))
			continue;

		rc = of_property_read_reg(np, 0, &addr, NULL);
		if (rc)
			continue;

		if (addr == port_nr)
			return np;
	}

	return NULL;
}

static int rg3m_port_init_smbus_agent(struct rg3m *rg3m, struct rg3m_port *port,
				      unsigned int port_nr)
{
	const unsigned int port_mask = 1u << port_nr;
	struct rg3m_i2c_agent *agent;
	int rc;

	agent = devm_kzalloc(&rg3m->i3c->dev, sizeof(*agent), GFP_KERNEL);
	if (!agent)
		return -ENOMEM;

	rc = regmap_set_bits(rg3m->regmap, RG3M_REG_AGENT_ENABLE, port_mask);
	if (rc)
		return -1;

	rc = regmap_clear_bits(rg3m->regmap, RG3M_REG_GPIO_ENABLE, port_mask);
	if (rc)
		return -1;

	rc = regmap_set_bits(rg3m->regmap, RG3M_REG_AGENT_IBI_ENABLE, port_mask);
	if (rc)
		return -1;

	/* clear pending events */
	rc = regmap_write(rg3m->regmap, RG3M_REG_AGENT_CNTLR_STATUS + port_nr,
			  0x0f);
	if (rc)
		return -1;

	agent->rg3m = rg3m;
	agent->port = port_nr;
	init_completion(&agent->completion);
	spin_lock_init(&agent->lock);
	agent->i2c.owner = THIS_MODULE;
	agent->i2c.algo = &rg3m_i2c_algo;
	agent->i2c.dev.parent = &rg3m->i3c->dev;
	if (port->of_node) {
		strscpy(agent->i2c.name, port->of_node->name,
			sizeof(agent->i2c.name));
		agent->i2c.dev.of_node = port->of_node;
	} else {
		snprintf(agent->i2c.name, sizeof(agent->i2c.name), "port%d", port_nr);
	}
	i2c_set_adapdata(&agent->i2c, agent);

	rc = i2c_add_adapter(&agent->i2c);
	if (rc)
		devm_kfree(&rg3m->i3c->dev, agent);

	port->agent = agent;

	return rc;
}

/* I3C handling */
struct rg3m_ibi_payload {
	uint8_t dev_port_status;
	uint8_t target_agent_status;
} __packed;

static void rg3m_ibi(struct i3c_device *i3c,
		     const struct i3c_ibi_payload *payload)
{
	struct rg3m *rg3m = i3cdev_get_drvdata(i3c);
	const struct rg3m_ibi_payload *p = NULL;
	unsigned int i, dev_stat, target_stat;

	if (payload->len == sizeof(*p))
		p = payload->data;

	if (!p || ibi_paranoia) {
		unsigned char tmp[2];
		int rc;

		/* DEV_PORT_STATUS and TARGET_STATUS are contiguous,
		 * read as a bulk operation */
		BUILD_BUG_ON(RG3M_REG_TARGET_STATUS !=
			     RG3M_REG_DEV_PORT_STATUS + 1);

		rc = regmap_bulk_read(rg3m->regmap, RG3M_REG_DEV_PORT_STATUS,
				      tmp, 2);
		if (rc)
			return;

		dev_stat = tmp[0];
		target_stat = tmp[1];

		if (p && (p->dev_port_status != dev_stat ||
			  p->target_agent_status != target_stat)) {
			dev_warn(&rg3m->i3c->dev,
				 "IBI stat mismatch: dev %02x/%02x target %02x/%02x\n",
				 p->dev_port_status, dev_stat,
				 p->target_agent_status, target_stat);
		}
	} else {
		dev_stat = p->dev_port_status;
		target_stat = p->target_agent_status;
	}

	/* Pass SMBus agent events to each port's agent, if configured. */
	for (i = 0; i < 8; i++) {
		struct rg3m_port *port = &rg3m->ports[i];

		if (!(target_stat & 1 << i))
			continue;

		if (i >= rg3m->n_ports || port->mode != RG3M_PORT_MODE_AGENT) {
			dev_warn(&rg3m->i3c->dev,
				 "IBI for invalid port %d\n", i);
			continue;
		}

		rg3m_agent_ibi(port->agent);
	}
}

/* I3C core init */
static int rg3m_port_init(struct rg3m *rg3m, unsigned int port_nr)
{
	unsigned int port_mask;
	struct rg3m_port *port;

	if (port_nr >= RG3M_PORT_MAX)
		return -1;

	port = &rg3m->ports[port_nr];
	port_mask = 1u << port_nr;
	port->of_node = rg3m_find_port_node(rg3m, port_nr);

	/* everytyhing is an SMBus Agent */
	port->mode = RG3M_PORT_MODE_AGENT;

	if (port->mode == RG3M_PORT_MODE_AGENT) {
		rg3m_port_init_smbus_agent(rg3m, port, port_nr);
	} else {
		dev_err(&rg3m->i3c->dev, "unknown port %d mode %d\n",
			port_nr, port->mode);
	}

	return 0;
}

/* mapping of part_id register to device-specific data */
static const struct rg3m_devdata {
	__u16 part_id;
	unsigned int n_ports;
} rg3m_devs[] = {
	{ 0x4712, 4 },
	{ 0x4812, 4 },
	{ 0x8712, 8 },
	{ 0x8812, 8 },
};

static const struct rg3m_devdata *rg3m_find_device(u16 part_id)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rg3m_devs); i++) {
		const struct rg3m_devdata *d = &rg3m_devs[i];

		if (d->part_id == part_id)
			return d;
	}

	return NULL;
}

static int rg3m_init(struct rg3m *rg3m)
{
	const struct rg3m_devdata *devdata;
	unsigned int i, dev_info[2];
	u16 part_id;
	int rc;

	rc = regmap_read(rg3m->regmap, RG3M_REG_DEV_INFO0, &dev_info[0]);
	if (rc)
		return rc;

	rc = regmap_read(rg3m->regmap, RG3M_REG_DEV_INFO1, &dev_info[1]);
	if (rc)
		return rc;

	part_id = dev_info[0] << 8 | dev_info[1];

	dev_info(&rg3m->i3c->dev, "RG3M device %04x\n", part_id);

	devdata = rg3m_find_device(part_id);
	if (!devdata)
		return -ENODEV;

	rg3m->n_ports = devdata->n_ports;

	/* unlock */
	rc = regmap_write(rg3m->regmap, RG3M_REG_UNLOCK_DEV_PROT,
			  RG3M_REG_UNLOCK_DEV_PROT_CODE);
	if (rc)
		return rc;

	for (i = 0; i < rg3m->n_ports; i++)
		rg3m_port_init(rg3m, i);

	return 0;
}

static const struct i3c_ibi_setup rg3m_ibi_setup = {
	.max_payload_len = 2, /* no MDB, two status registers */
	.num_slots = 3, /* two target buffers, one controller status */
	.handler = rg3m_ibi,
};

static const struct regmap_config rg3m_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 255,
};

static const struct i3c_device_id rg3m_i3c_ids[] = {
	I3C_VENDOR_CLASS(0x0266, I3C_DCR_HUB, NULL),
	{ 0 },
};

static int rg3m_probe(struct i3c_device *i3c)
{
	const struct i3c_device_id *id;
	struct rg3m *rg3m;
	int rc;

	printk(KERN_NOTICE "rg3m_probe");

	id = i3c_device_match_id(i3c, rg3m_i3c_ids);
	if (!id)
		return -ENODEV;

	rg3m = devm_kzalloc(&i3c->dev, sizeof(*rg3m), GFP_KERNEL);
	if (!rg3m)
		return -ENOMEM;

	rg3m->i3c = i3c;
	i3cdev_set_drvdata(i3c, rg3m);

	rg3m->regmap = devm_regmap_init_i3c(i3c, &rg3m_regmap_config);
	if (IS_ERR_OR_NULL(rg3m->regmap)) {
		return PTR_ERR(rg3m->regmap);
	}

	rc = rg3m_init(rg3m);
	if (rc) {
		dev_err(&rg3m->i3c->dev, "device init failed\n");
		return rc;
	}

	rc = i3c_device_request_ibi(rg3m->i3c, &rg3m_ibi_setup);
	if (rc) {
		dev_err(&rg3m->i3c->dev, "ibi init failed\n");
		return rc;
	}

	rc = i3c_device_enable_ibi(rg3m->i3c);
	if (rc) {
		dev_err(&rg3m->i3c->dev, "ibi enable failed\n");
		goto err_free_ibi;
	}

	return 0;

err_free_ibi:
	i3c_device_free_ibi(rg3m->i3c);
	return rc;

}

static void rg3m_remove(struct i3c_device *i3c)
{
}

static struct i3c_driver rg3m_driver = {
	.driver = {
		.name = "rg3mxxb12b0",
	},
	.probe = rg3m_probe,
	.remove = rg3m_remove,
	.id_table = rg3m_i3c_ids,
};

module_i3c_driver(rg3m_driver);

MODULE_AUTHOR("Jeremy Kerr <jk@codeconstruct.com.au>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Renesas RG3MxxB12A1 i3c hub driver");
MODULE_DEVICE_TABLE(i3c, rg3m_i3c_ids);
