#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/uaccess.h>


#include "sx127x.h"

#define SX127X_DRIVERNAME	"sx127x"
#define SX127X_CLASSNAME	"sx127x"
#define SX127X_DEVICENAME	"sx127x%d"

#define SX127X_LORAREG_MASK BIT(8)
#define SX127X_LORAREG(addr) (addr | SX127X_LORAREG_MASK)
#define SX127X_FSKOOKREG_MASK BIT(9)
#define SX127X_FSKOOKREG(addr) (addr | SX127X_FSKOOK_MASK)

#define SX127X_REGADDR(reg) (reg & 0x7f)
#define SX127X_WRITEADDR(addr) (addr | (1 << 7))

#define SX127X_REG_FIFO													0x00

#define SX127X_REG_OPMODE												0x01
#define SX127X_REG_OPMODE_LONGRANGEMODE									BIT(7)
#define SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE							(BIT(6) | BIT(5))
#define SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE_FSK						0
#define SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE_OOK						BIT(5)
#define SX127X_REG_OPMODE_LOWFREQUENCYMODEON							BIT(3)
#define SX127X_REG_OPMODE_MODE											(BIT(2) | BIT(1) | BIT(0))
#define SX127X_REG_OPMODE_MODE_SLEEPMODE								0
#define SX127X_REG_OPMODE_MODE_STDBYMODE								BIT(0)

#define SX127X_REG_FSKOOK_BITRATEMSB									SX127X_FSKOOKREG(0x02)
#define SX127X_REG_FSKOOK_BITRATELSB									SX127X_FSKOOKREG(0x03)
#define SX127X_REG_FSKOOK_FDEVMSB										SX127X_FSKOOKREG(0x04)
#define SX127X_REG_FSKOOK_FDEVLSB										SX127X_FSKOOKREG(0x05)
#define SX127X_REG_FRFMSB												0x06
#define SX127X_REG_FRFMLD												0x07
#define SX127X_REG_FRFLSB												0x08

#define SX127X_REG_PACONFIG												0x09
#define SX127X_REG_PACONFIG_PASELECT									BIT(7)
#define SX127X_REG_PACONFIG_MAXPOWER									(BIT(6) | BIT(5) | BIT(4))
#define SX127X_REG_PACONFIG_MAXPOWER_SHIFT								4
#define SX127X_REG_PACONFIG_OUTPUTPOWER									(BIT(3) | BIT(2) | BIT(1) | BIT(0))

#define SX127X_REG_PARAMP												0x0a
#define SX127X_REG_OCP													0x0b
#define SX127X_REG_LNA													0x0c
#define SX127X_REG_FSKOOK_RXCONFIG										SX127X_FSKOOKREG(0x0d)
#define SX127X_REG_LORA_FIFOADDRPTR										SX127X_LORAREG(0x0d)
#define SX127X_REG_FSKOOK_RSSICONFIG									SX127X_FSKOOKREG(0x0e)
#define SX127X_REG_LORA_FIFOTXBASEADDR									SX127X_LORAREG(0x0e)
#define SX127X_REG_FSKOOK_RSSICOLLISION									SX127X_FSKOOKREG(0x0f)
#define SX127X_REG_LORA_RXBASEADDR										SX127X_LORAREG(0x0f)
#define SX127X_REG_FSKOOK_RSSTHRESH										SX127X_FSKOOKREG(0x10)
#define SX127X_REG_LORA_RXCURRENTADDR									SX127X_LORAREG(0x10)
#define SX127X_REG_FSKOOK_RSSIVALUE										SX127X_FSKOOKREG(0x11)
#define SX127X_REG_LORA_IRQMASKFLAGS									SX127X_LORAREG(0x11)
#define SX127X_REG_FSKOOK_RXBW											SX127X_FSKOOKREG(0x12)

#define SX127X_REG_LORA_IRQFLAGS										SX127X_LORAREG(0x12)
#define SX127X_REG_LORA_IRQFLAGS_RXTIMEOUT								BIT(7)
#define SX127X_REG_LORA_IRQFLAGS_RXDONE									BIT(6)
#define SX127X_REG_LORA_IRQFLAGS_PAYLOADCRCERROR						BIT(5)
#define SX127X_REG_LORA_IRQFLAGS_TXDONE									BIT(3)
#define SX127X_REG_LORA_IRQFLAGS_CADDONE								BIT(2)
#define SX127X_REG_LORA_IRQFLAGS_CADDETECTED							BIT(0)

#define SX127X_REG_FSKOOK_AFCBW											SX127X_FSKOOKREG(0x13)
#define SX127X_REG_LORA_RXNBBYTES										SX127X_LORAREG(0x13)
#define SX127X_REG_FSKOOK_OOKPEAK										SX127X_FSKOOKREG(0x14)
#define SX127X_REG_LORA_RXHEADERCNTVALUEMSB								SX127X_LORAREG(0x14)
#define SX127X_REG_FSKOOK_OOKFIX										SX127X_FSKOOKREG(0x15)
#define SX127X_REG_LORA_RXHEADERCNTVALUELSB								SX127X_LORAREG(0x15)
#define SX127X_REG_FSKOOK_OOKAVG										SX127X_FSKOOKREG(0x16)
#define SX127X_REG_LORA_RXPACKETCNTVALUEMSB								SX127X_LORAREG(0x16)
#define SX127X_REG_LORA_RXPACKETCNTVALUELSB								SX127X_LORAREG(0x17)
#define SX127X_REG_LORA_MODEMSTAT										SX127X_LORAREG(0x18)
#define SX127X_REG_LORA_PKTSNRVALUE										SX127X_LORAREG(0x19)
#define SX127X_REG_FSKOOK_AFCFEI										SX127X_FSKOOKREG(0x1a)
#define SX127X_REG_LORA_PKTRSSIVALUE									SX127X_LORAREG(0x1a)
#define SX127X_REG_FSKOOK_AFCMSB										SX127X_FSKOOKREG(0x1b)
#define SX127X_REG_LORA_RSSIVALUE										SX127X_LORAREG(0x1b)
#define SX127X_REG_FSKOOK_AFCLSB										SX127X_FSKOOKREG(0x1c)
#define SX127X_REG_LORA_HOPCHANNEL										SX127X_LORAREG(0x1c)
#define SX127X_REG_FSKOOK_FEIMSB										SX127X_FSKOOKREG(0x1d)

#define SX127X_REG_LORA_MODEMCONFIG1									SX127X_LORAREG(0x1d)
#define SX127X_REG_LORA_MODEMCONFIG1_BW									(BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define SX127X_REG_LORA_MODEMCONFIG1_BW_SHIFT							4
#define SX127X_REG_LORA_MODEMCONFIG1_BW_MAX								9
#define SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE							(BIT(3) | BIT(2) | BIT(1))
#define SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_SHIFT					1
#define SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_MIN						1
#define SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_MAX						6
#define SX127X_REG_LORA_MODEMCONFIG1_IMPLICITHEADERMODEON				BIT(0)

#define SX127X_REG_FSKOOK_FEILSB										SX127X_FSKOOKREG(0x1e)

#define SX127X_REG_LORA_MODEMCONFIG2									SX127X_LORAREG(0x1e)
#define SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR					(BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR_SHIFT				4

#define SX127X_REG_FSKOOK_PREAMBLEDETECT								SX127X_FSKOOKREG(0x1f)
#define SX127X_REG_LORA_SYMBTIMEOUTLSB									SX127X_LORAREG(0x1f)
#define SX127X_REG_FSKOOK_RXTIMEOUT1									SX127X_FSKOOKREG(0x20)
#define SX127X_REG_LORA_PREAMBLEMSB										SX127X_LORAREG(0x20)
#define SX127X_REG_FSKOOK_RXTIMEOUT2									SX127X_FSKOOKREG(0x21)
#define SX127X_REG_LORA_PREAMBLELSB										SX127X_LORAREG(0x21)
#define SX127X_REG_FSKOOK_RXTIMEOUT3									SX127X_FSKOOKREG(0x22)
#define SX127X_REG_LORA_PAYLOADLENGTH									SX127X_LORAREG(0x22)
#define SX127X_REG_FSKOOK_RXDELAY										SX127X_FSKOOKREG(0x23)
#define SX127X_REG_LORA_MAXPAYLOADLENGTH								SX127X_LORAREG(0x23)
#define SX127X_REG_FSKOOK_OSC											SX127X_FSKOOKREG(0x24)
#define SX127X_REG_LORA_HOPPERIOD										SX127X_LORAREG(0x24)
#define SX127X_REG_FSKOOK_PREAMBLEMSB									SX127X_FSKOOKREG(0x25)
#define SX127X_REG_LORA_FIFORXBYTEADDR									SX127X_LORAREG(0x25)
#define SX127X_REG_FSKOOK_PREAMBLELSB									SX127X_FSKOOKREG(0x26)

#define SX127X_REG_LORA_MODEMCONFIG3									SX127X_LORAREG(0x26)
#define SX127X_REG_LORA_MODEMCONFIG3_LOWDATARATEOPTIMIZE				BIT(3)

#define SX127X_REG_FSKOOK_SYNCCONFIG									SX127X_FSKOOKREG(0x27)
#define SX127X_REG_FSKOOK_SYNCVALUE1									SX127X_FSKOOKREG(0x28)
#define SX127X_REG_LORA_FEIMSB											SX127X_LORAREG(0x28)
#define SX127X_REG_FSKOOK_SYNCVALUE2									SX127X_FSKOOKREG(0x29)
#define SX127X_REG_LORA_FEIMID											SX127X_LORAREG(0x29)
#define SX127X_REG_FSKOOK_SYNCVALUE3									SX127X_FSKOOKREG(0x2a)
#define SX127X_REG_LORA_FEILSB											SX127X_LORAREG(0x29)
#define SX127X_REG_FSKOOK_SYNCVALUE4									SX127X_FSKOOKREG(0x2b)
#define SX127X_REG_FSKOOK_SYNCVALUE5									SX127X_FSKOOKREG(0x2c)
#define SX127X_REG_LORA_RSSIWIDEBAND									SX127X_LORAREG(0x2c)
#define SX127X_REG_FSKOOK_SYNCVALUE6									SX127X_FSKOOKREG(0x2d)
#define SX127X_REG_FSKOOK_SYNCVALUE7									SX127X_FSKOOKREG(0x2e)
#define SX127X_REG_FSKOOK_SYNCVALUE8									SX127X_FSKOOKREG(0x2f)
#define SX127X_REG_FSKOOK_PACKETCONFIG1									SX127X_FSKOOKREG(0x30)
#define SX127X_REG_FSKOOK_PACKETCONFIG2									SX127X_FSKOOKREG(0x31)

#define SX127X_REG_LORA_DETECTOPTIMIZATION								SX127X_LORAREG(0x31)
#define SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE			(BIT(2) | BIT(1) | BIT(0))
#define SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE_SF7SF12	0x03
#define SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE_SF6		0x05

#define SX127X_REG_FSKOOK_PAYLOADLENGTH									SX127X_FSKOOKREG(0x32)
#define SX127X_REG_FSKOOK_NODEADRS										SX127X_FSKOOKREG(0x33)
#define SX127X_REG_LORA_INVERTIQ										SX127X_LORAREG(0x33)
#define SX127X_REG_FSKOOK_BROADCASTADRS									SX127X_FSKOOKREG(0x34)
#define SX127X_REG_FSKOOK_FIFOTHRESH									SX127X_FSKOOKREG(0x35)
#define SX127X_REG_FSKOOK_SEQCONFIG1									SX127X_FSKOOKREG(0x36)
#define SX127X_REG_FSKOOK_SEQCONFIG2									SX127X_FSKOOKREG(0x37)
#define SX127X_REG_LORA_DETECTIONTHRESHOLD								SX127X_LORAREG(0x37)
#define SX127X_REG_FSKOOK_TIMERRESOL									SX127X_FSKOOKREG(0x38)
#define SX127X_REG_FSKOOK_TIMER1COEF									SX127X_FSKOOKREG(0x39)
#define SX127X_REG_LORA_SYNCWORD										SX127X_LORAREG(0x39)
#define SX127X_REG_FSKOOK_TIMER2COEF									SX127X_FSKOOKREG(0x3a)
#define SX127X_REG_FSKOOK_IMAGECAL										SX127X_FSKOOKREG(0x3b)
#define SX127X_REG_FSKOOK_TEMP											SX127X_FSKOOKREG(0x3c)
#define SX127X_REG_FSKOOK_LOWBAT										SX127X_FSKOOKREG(0x3d)
#define SX127X_REG_FSKOOK_IRQFLAGS1										SX127X_FSKOOKREG(0x3e)
#define SX127X_REG_FSKOOK_IRQFLAGS2										SX127X_FSKOOKREG(0x3f)

#define SX127X_REG_DIOMAPPING1											0x40
#define SX127X_REG_DIOMAPPING1_DIO0										(BIT(7) | BIT(6))
#define SX127X_REG_DIOMAPPING1_DIO0_RXDONE								0
#define SX127X_REG_DIOMAPPING1_DIO0_TXDONE								(BIT(6))
#define SX127X_REG_DIOMAPPING1_DIO0_CADDONE								(BIT(7))

#define SX127X_REG_DIOMAPPING2											0x41
#define SX127X_REG_VERSION												0x42
#define SX127X_REG_FSKOOK_PLLHOP										SX127X_FSKOOKREG(0x44)
#define SX127X_REG_TCXO													0x4b
#define SX127X_REG_PADAC												0x4d
#define SX127X_REG_FORMERTEMP											0x5b
#define SX127X_REG_FSKOOK_BITRATEFRAC									SX127X_FSKOOKREG(0x5d)
#define SX127X_REG_AGCREF												0x61
#define SX127X_REG_AGCTHRESH1											0x62
#define SX127X_REG_AGCTHRESH2											0x63
#define SX127X_REG_AGCTHRESH3											0x64
#define SX127X_REG_PLL													0x70

static int devmajor;
static struct class *devclass;

static const char* invalid = "invalid";
static const char* modstr[] = {"fsk", "ook", "lora"};
static const char* opmodestr[] = {"sleep", "standby", "fstx", "tx", "fsrx", "rx", "rxcontinuous", "rxsingle", "cad"};
static unsigned bwmap[] = { 7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000 };
static const char* paoutput[] = {"rfo", "pa_boost"};

struct sx127x {
	struct device *chardevice;
	struct work_struct irq_work;
	struct spi_device* spidevice;
	struct gpio_desc *gpio_reset, *gpio_txen, *gpio_rxen;
	u32 fosc;
	enum sx127x_pa pa;
	struct mutex mutex;

	struct list_head device_entry;
	dev_t devt;
	bool open;

	/* device state */
	bool loraregmap;
	enum sx127x_opmode opmode;
	/* tx */
	wait_queue_head_t writewq;
	int transmitted;
	/* rx */
	wait_queue_head_t readwq;
	struct kfifo out;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static int sx127x_reg_read(struct spi_device *spi, u16 reg, u8* result){
	u8 addr = reg & 0xff;
	int ret = spi_write_then_read(spi,
			&addr, 1,
			result, 1);
	dev_dbg(&spi->dev, "read: @%02x %02x\n", addr, *result);
	return ret;
}

static int sx127x_reg_read16(struct spi_device *spi, u16 reg, u16* result){
	u8 addr = reg & 0xff;
	int ret = spi_write_then_read(spi,
			&addr, 1,
			result, 2);
	dev_dbg(&spi->dev, "read: @%02x %02x\n", addr, *result);
	return ret;
}

static int sx127x_reg_read24(struct spi_device *spi, u16 reg, u32* result){
	u8 addr = reg & 0xff, buf[3];
	int ret = spi_write_then_read(spi,
			&addr, 1,
			buf, 3);
	*result = (buf[0] << 16) | (buf[1] << 8) | buf[0];
	dev_dbg(&spi->dev, "read: @%02x %06x\n", addr, *result);
	return ret;
}

static int sx127x_reg_write(struct spi_device *spi, u16 reg, u8 value){
	u8 addr = SX127X_REGADDR(reg), buff[2], readback;
	int ret;
	buff[0] = SX127X_WRITEADDR(addr);
	buff[1] = value;
	dev_dbg(&spi->dev, "write: @%02x %02x\n", addr, value);
	ret = spi_write(spi, buff, 2);
//	ret = sx127x_reg_read(spi, reg, &readback);
//	if(readback != value){
//		dev_warn(&spi->dev, "read back does not match\n");
//	}
	return ret;
}

static int sx127x_reg_write24(struct spi_device *spi, u16 reg, u32 value){
	u8 addr = SX127X_REGADDR(reg), buff[4];
	int ret;
	buff[0] = SX127X_WRITEADDR(addr);
	buff[1] = (value >> 16) & 0xff;
	buff[2] = (value >> 8) & 0xff;
	buff[3] = value & 0xff;
	dev_dbg(&spi->dev, "write: @%02x %06x\n", addr, value);
	ret = spi_write(spi, buff, sizeof(buff));
	return ret;
}

static int sx127x_fifo_readpkt(struct spi_device *spi, void *buffer, u8 *len){
	u8 addr = SX127X_REG_FIFO, pktstart, rxbytes, off, fifoaddr;
	size_t maxtransfer = spi_max_transfer_size(spi);
	int ret;
	unsigned readlen;
	ret = sx127x_reg_read(spi, SX127X_REG_LORA_RXCURRENTADDR, &pktstart);
	ret = sx127x_reg_read(spi, SX127X_REG_LORA_RXNBBYTES, &rxbytes);
	for(off = 0; off < rxbytes; off += maxtransfer){
		readlen = min(maxtransfer, (size_t) (rxbytes - off));
		fifoaddr = pktstart + off;
		ret = sx127x_reg_write(spi, SX127X_REG_LORA_FIFOADDRPTR, fifoaddr);
		if(ret){
			break;
		}
		dev_warn(&spi->dev, "fifo read: %02x from %02x\n", readlen, fifoaddr);
		ret = spi_write_then_read(spi,
				&addr, 1,
				buffer + off, readlen);
		if(ret){
			break;
		}

	}
	print_hex_dump_bytes("", DUMP_PREFIX_NONE, buffer, rxbytes);
	*len = rxbytes;
	return ret;
}

static int sx127x_fifo_writepkt(struct spi_device *spi, void *buffer, u8 len){
	u8 addr = SX127X_WRITEADDR(SX127X_REGADDR(SX127X_REG_FIFO));
	struct spi_transfer fifotransfers[2];
	int ret;

	memset(fifotransfers, 0, sizeof(fifotransfers));

	fifotransfers[0].tx_buf = &addr;
	fifotransfers[0].len = 1;
	fifotransfers[1].tx_buf = buffer;
	fifotransfers[1].len = len;

	ret = sx127x_reg_write(spi, SX127X_REG_LORA_FIFOTXBASEADDR, 0);
	ret = sx127x_reg_write(spi, SX127X_REG_LORA_FIFOADDRPTR, 0);

	dev_info(&spi->dev, "fifo write: %d\n", len);
	spi_sync_transfer(spi, fifotransfers, ARRAY_SIZE(fifotransfers));

	ret = sx127x_reg_write(spi, SX127X_REG_LORA_PAYLOADLENGTH, len);

	return ret;
}

static enum sx127x_modulation sx127x_getmodulation(u8 opmode) {
	u8 mod;
	if(opmode & SX127X_REG_OPMODE_LONGRANGEMODE) {
		return SX127X_MODULATION_LORA;
	}
	else {
		mod = opmode & SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE;
		if(mod == SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE_FSK) {
			return SX127X_MODULATION_FSK;
		}
		else if(mod == SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE_OOK) {
			return SX127X_MODULATION_OOK;
		}
	}

	return SX127X_MODULATION_INVALID;
}

static int sx127x_indexofstring(const char* str, const char** options, unsigned noptions){
	int i;
	for (i = 0; i < noptions; i++) {
		if (sysfs_streq(str, options[i])) {
			return i;
		}
	}
	return -1;
}

static ssize_t sx127x_modulation_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct sx127x *data = dev_get_drvdata(child);
	u8 opmode;
	enum sx127x_modulation mod;
	int ret;
	mutex_lock(&data->mutex);
	sx127x_reg_read(data->spidevice, SX127X_REG_OPMODE, &opmode);
	mod = sx127x_getmodulation(opmode);
	if(mod == SX127X_MODULATION_INVALID){
		ret = sprintf(buf, "%s\n", invalid);
	}
	else{
		ret = sprintf(buf, "%s\n", modstr[mod]);
	}
	mutex_unlock(&data->mutex);
	return ret;

}

static int sx127x_setmodulation(struct sx127x *data, enum sx127x_modulation modulation){
	u8 opmode;
	sx127x_reg_read(data->spidevice, SX127X_REG_OPMODE, &opmode);

	// LoRa mode bit can only be changed in sleep mode
	if(opmode & SX127X_REG_OPMODE_MODE){
		dev_warn(data->chardevice, "switching to sleep mode before changing modulation\n");
		opmode &= ~SX127X_REG_OPMODE_MODE;
		sx127x_reg_write(data->spidevice, SX127X_REG_OPMODE, opmode);
	}

	dev_warn(data->chardevice, "setting modulation to %s\n", modstr[modulation]);
	switch(modulation){
		case SX127X_MODULATION_FSK:
		case SX127X_MODULATION_OOK:
			opmode &= ~SX127X_REG_OPMODE_LONGRANGEMODE;
			data->loraregmap = 0;
			break;
		case SX127X_MODULATION_LORA:
			opmode |= SX127X_REG_OPMODE_LONGRANGEMODE;
			data->loraregmap = 1;
			break;
	}
	sx127x_reg_write(data->spidevice, SX127X_REG_OPMODE, opmode);
	return 0;
}

static ssize_t sx127x_modulation_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	struct sx127x *data = dev_get_drvdata(dev);
	int idx = sx127x_indexofstring(buf, modstr, ARRAY_SIZE(modstr));
	if(idx == -1){
		dev_warn(dev, "invalid modulation type\n");
		goto out;
	}
	mutex_lock(&data->mutex);
	sx127x_setmodulation(data, idx);
	mutex_unlock(&data->mutex);
out:
	return count;
}

static DEVICE_ATTR(modulation, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_modulation_show, sx127x_modulation_store);

static ssize_t sx127x_opmode_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct sx127x *data = dev_get_drvdata(child);
	u8 opmode, mode;
	int ret;
	mutex_lock(&data->mutex);
	sx127x_reg_read(data->spidevice, SX127X_REG_OPMODE, &opmode);
	mode = opmode & SX127X_REG_OPMODE_MODE;
	if(mode > 4 && (opmode & SX127X_REG_OPMODE_LONGRANGEMODE)){
		ret = sprintf(buf, "%s\n", opmodestr[mode + 1]);
	}
	else {
		ret =sprintf(buf, "%s\n", opmodestr[mode]);
	}
	mutex_unlock(&data->mutex);
	return ret;
}

static int sx127x_toggletxrxen(struct sx127x *data, bool tx){
	if(data->gpio_txen){
		if(tx){
			dev_warn(data->chardevice, "enabling tx\n");
		}
		gpiod_set_value(data->gpio_txen, tx);
	}
	if(data->gpio_rxen){
		if(!tx){
			dev_warn(data->chardevice, "enabling rx\n");
		}
		gpiod_set_value(data->gpio_rxen, !tx);
	}
	return 0;
}

static int sx127x_setopmode(struct sx127x *data, enum sx127x_opmode mode, bool retain){
	u8 opmode, diomapping1;
	int ret = sx127x_reg_read(data->spidevice, SX127X_REG_OPMODE, &opmode);
	if(mode < SX127X_OPMODE_SLEEP || mode > SX127X_OPMODE_CAD){
		ret = -EINVAL;
		dev_err(data->chardevice, "invalid opmode\n");
	}
	else if((opmode & SX127X_REG_OPMODE_LONGRANGEMODE) && (mode == SX127X_OPMODE_RX)){
		dev_err(data->chardevice, "opmode %s not valid in LoRa mode\n", opmodestr[mode]);
	}
	else if(!(opmode & SX127X_REG_OPMODE_LONGRANGEMODE) && (mode > SX127X_OPMODE_RX)) {
		dev_err(data->chardevice, "opmode %s not valid in FSK/OOK mode\n", opmodestr[mode]);
	}
	else {
		if(retain){
					data->opmode = mode;
		}
		dev_warn(data->chardevice, "setting opmode to %s\n", opmodestr[mode]);
		sx127x_reg_read(data->spidevice, SX127X_REG_DIOMAPPING1, &diomapping1);
		diomapping1 &= ~SX127X_REG_DIOMAPPING1_DIO0;
		switch(mode){
		case SX127X_OPMODE_CAD:
			diomapping1 |= SX127X_REG_DIOMAPPING1_DIO0_CADDONE;
			sx127x_toggletxrxen(data, false);
			break;
		case SX127X_OPMODE_TX:
			diomapping1 |= SX127X_REG_DIOMAPPING1_DIO0_TXDONE;
			sx127x_toggletxrxen(data, true);
			break;
		case SX127X_OPMODE_RX:
		case SX127X_OPMODE_RXCONTINUOS:
		case SX127X_OPMODE_RXSINGLE:
			diomapping1 |= SX127X_REG_DIOMAPPING1_DIO0_RXDONE;
			sx127x_toggletxrxen(data, false);
			break;
		default:
			dev_warn(data->chardevice, "disabling rx and tx\n");
			gpiod_set_value(data->gpio_rxen, 0);
			gpiod_set_value(data->gpio_txen, 0);
			break;
		}
		opmode &= ~SX127X_REG_OPMODE_MODE;
		if(mode > SX127X_OPMODE_RX){
			mode -= 1;
		}
		opmode |= mode;
		sx127x_reg_write(data->spidevice, SX127X_REG_DIOMAPPING1, diomapping1);
		sx127x_reg_write(data->spidevice, SX127X_REG_OPMODE, opmode);
	}
	return ret;
}

static ssize_t sx127x_opmode_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	struct sx127x *data = dev_get_drvdata(dev);
	int idx;
	enum sx127x_opmode mode;

	idx = sx127x_indexofstring(buf, opmodestr, ARRAY_SIZE(opmodestr));
	if(idx == -1){
		dev_err(dev, "invalid opmode\n");
		goto out;
	}
	mutex_lock(&data->mutex);
	mode = idx;
	sx127x_setopmode(data, mode, true);
	mutex_unlock(&data->mutex);
out:
	return count;
}

static DEVICE_ATTR(opmode, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_opmode_show, sx127x_opmode_store);

static ssize_t sx127x_carrierfrequency_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx127x *data = dev_get_drvdata(dev);
	u8 msb, mld, lsb;
	u32 frf;
	u32 freq;
	mutex_lock(&data->mutex);
	sx127x_reg_read(data->spidevice, SX127X_REG_FRFMSB, &msb);
	sx127x_reg_read(data->spidevice, SX127X_REG_FRFMLD, &mld);
	sx127x_reg_read(data->spidevice, SX127X_REG_FRFLSB, &lsb);
	frf = (msb << 16) | (mld << 8) | lsb;
	freq = ((u64)data->fosc * frf) / 524288;
	mutex_unlock(&data->mutex);
    return sprintf(buf, "%u\n", freq);
}

static int sx127x_setcarrierfrequency(struct sx127x *data, u64 freq){
	u8 opmode, newopmode;
	sx127x_reg_read(data->spidevice, SX127X_REG_OPMODE, &opmode);
	newopmode = opmode & ~SX127X_REG_OPMODE_LOWFREQUENCYMODEON;
	if(freq < 700000000){
		newopmode |= SX127X_REG_OPMODE_LOWFREQUENCYMODEON;
	}
	if(newopmode != opmode){
		dev_warn(data->chardevice, "toggling LF/HF bit\n");
		sx127x_reg_write(data->spidevice, SX127X_REG_OPMODE, newopmode);
	}

	dev_warn(data->chardevice, "setting carrier frequency to %llu\n", freq);
	freq *= 524288;

	do_div(freq, data->fosc);

	sx127x_reg_write24(data->spidevice, SX127X_REG_FRFMSB, freq);
	return 0;
}

static ssize_t sx127x_carrierfrequency_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	struct sx127x *data = dev_get_drvdata(dev);
	u64 freq;

	if(kstrtou64(buf, 10, &freq)){
		goto out;
	}
	mutex_lock(&data->mutex);
	sx127x_setcarrierfrequency(data, freq);
	mutex_unlock(&data->mutex);
	out:
	return count;
}

static DEVICE_ATTR(carrierfrequency, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_carrierfrequency_show, sx127x_carrierfrequency_store);

static ssize_t sx127x_rssi_show(struct device *child, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", 0);
}

static DEVICE_ATTR(rssi, S_IRUSR | S_IRGRP | S_IROTH, sx127x_rssi_show, NULL);

static ssize_t sx127x_sf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx127x *data = dev_get_drvdata(dev);
	u8 config2;
	int sf;
	mutex_lock(&data->mutex);
	sx127x_reg_read(data->spidevice, SX127X_REG_LORA_MODEMCONFIG2, &config2);
	sf = config2 >> SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR_SHIFT;
	mutex_unlock(&data->mutex);
    return sprintf(buf, "%d\n", sf);
}

static int sx127x_setsf(struct sx127x *data, unsigned sf){
	u8 r;
	dev_info(data->chardevice, "setting spreading factor to %u\n", sf);

	// set the spreading factor
	sx127x_reg_read(data->spidevice, SX127X_REG_LORA_MODEMCONFIG2, &r);
	r &= ~SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR;
	r |= sf << SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR_SHIFT;
	sx127x_reg_write(data->spidevice, SX127X_REG_LORA_MODEMCONFIG2, r);

	// set the detection optimization magic number depending on the spreading factor
	sx127x_reg_read(data->spidevice, SX127X_REG_LORA_DETECTOPTIMIZATION, &r);
	r &= ~SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE;
	if(sf == 6){
		r |= SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE_SF6;
	}
	else{
		r |= SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE_SF7SF12;
	}
	sx127x_reg_write(data->spidevice, SX127X_REG_LORA_DETECTOPTIMIZATION, r);

	// set the low data rate bit
	sx127x_reg_read(data->spidevice, SX127X_REG_LORA_MODEMCONFIG3, &r);
	r |= SX127X_REG_LORA_MODEMCONFIG3_LOWDATARATEOPTIMIZE;
	sx127x_reg_write(data->spidevice, SX127X_REG_LORA_MODEMCONFIG3, r);
	return 0;
}

static ssize_t sx127x_sf_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	struct sx127x *data = dev_get_drvdata(dev);
	int sf;
	if(kstrtoint(buf, 10, &sf)){
		goto out;
	}
	mutex_lock(&data->mutex);
	sx127x_setsf(data, sf);
	mutex_unlock(&data->mutex);
	out:
	return count;
}

static DEVICE_ATTR(sf, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_sf_show, sx127x_sf_store);

static ssize_t sx127x_bw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx127x *data = dev_get_drvdata(dev);
	u8 config1;
	int bw, ret;

	mutex_lock(&data->mutex);
	sx127x_reg_read(data->spidevice, SX127X_REG_LORA_MODEMCONFIG1, &config1);
	bw = config1 >> SX127X_REG_LORA_MODEMCONFIG1_BW_SHIFT;
	if(bw > SX127X_REG_LORA_MODEMCONFIG1_BW_MAX){
		ret = sprintf(buf, "invalid\n");
	}
	else {
		ret = sprintf(buf, "%d\n", bwmap[bw]);
	}
	mutex_unlock(&data->mutex);
	return ret;
}

static ssize_t sx127x_bw_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	struct sx127x *data = dev_get_drvdata(dev);
	return count;
}

static DEVICE_ATTR(bw, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_bw_show, sx127x_bw_store);

static char* crmap[] = { NULL, "4/5", "4/6", "4/7", "4/8" };
static ssize_t sx127x_codingrate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx127x *data = dev_get_drvdata(dev);
	u8 config1;
	int cr, ret;

	mutex_lock(&data->mutex);
	sx127x_reg_read(data->spidevice, SX127X_REG_LORA_MODEMCONFIG1, &config1);
	cr = (config1 & SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE) >> SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_SHIFT;
	if(cr < SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_MIN ||
			cr > SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_MAX){
		ret = sprintf(buf, "invalid\n");
	}
	else {
		ret = sprintf(buf, "%s\n", crmap[cr]);
	}
	mutex_unlock(&data->mutex);
    return ret;
}

static ssize_t sx127x_codingrate_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	struct sx127x *data = dev_get_drvdata(dev);
	return count;
}

static DEVICE_ATTR(codingrate, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_codingrate_show, sx127x_codingrate_store);

static ssize_t sx127x_implicitheadermodeon_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx127x *data = dev_get_drvdata(dev);
	u8 config1;
	int hdrmodeon;

	mutex_lock(&data->mutex);
	sx127x_reg_read(data->spidevice, SX127X_REG_LORA_MODEMCONFIG1, &config1);
	hdrmodeon = config1 & SX127X_REG_LORA_MODEMCONFIG1_IMPLICITHEADERMODEON;
	mutex_unlock(&data->mutex);
    return sprintf(buf, "%d\n", hdrmodeon);
}

static ssize_t sx127x_implicitheadermodeon_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	struct sx127x *data = dev_get_drvdata(dev);
	return count;
}

static DEVICE_ATTR(implicitheadermodeon, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_implicitheadermodeon_show, sx127x_implicitheadermodeon_store);

static ssize_t sx127x_paoutput_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx127x *data = dev_get_drvdata(dev);
	u8 paconfig;
	int idx;
	mutex_lock(&data->mutex);
	sx127x_reg_read(data->spidevice, SX127X_REG_PACONFIG, &paconfig);
	idx = (paconfig & SX127X_REG_PACONFIG_PASELECT) ? 1 : 0;
	mutex_unlock(&data->mutex);
    return sprintf(buf, "%s\n", paoutput[idx]);
}

static int sx127x_setpaoutput(struct sx127x *data, enum sx127x_pa pa){
	int ret = 0;
	u8 paconfig;
	sx127x_reg_read(data->spidevice, SX127X_REG_PACONFIG, &paconfig);
	switch(pa){
		case SX127X_PA_RFO:
			paconfig &= ~SX127X_REG_PACONFIG_PASELECT;
			data->pa = SX127X_PA_RFO;
			break;
		case SX127X_PA_PABOOST:
			paconfig |= SX127X_REG_PACONFIG_PASELECT;
			data->pa = SX127X_PA_PABOOST;
			break;
	}
	sx127x_reg_write(data->spidevice, SX127X_REG_PACONFIG, paconfig);
	return 0;
}

static ssize_t sx127x_paoutput_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	//TODO this needs to take into account non-default values for the "padac".
	struct sx127x *data = dev_get_drvdata(dev);
	int idx = sx127x_indexofstring(buf, paoutput, ARRAY_SIZE(paoutput));
	if(idx == -1)
		goto out;
	mutex_lock(&data->mutex);
	sx127x_setpaoutput(data, idx);
	mutex_unlock(&data->mutex);
out:
	return count;
}

static DEVICE_ATTR(paoutput, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_paoutput_show, sx127x_paoutput_store);

static ssize_t sx127x_outputpower_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx127x *data = dev_get_drvdata(dev);
	u8 paconfig;
	int maxoutputpower = 17;
	int outputpower;
	mutex_lock(&data->mutex);
	sx127x_reg_read(data->spidevice, SX127X_REG_PACONFIG, &paconfig);
	if(!(paconfig & SX127X_REG_PACONFIG_PASELECT)){
		maxoutputpower = ((paconfig & SX127X_REG_PACONFIG_MAXPOWER) >> SX127X_REG_PACONFIG_MAXPOWER_SHIFT);
	}
	outputpower = maxoutputpower - (15 - (paconfig & SX127X_REG_PACONFIG_OUTPUTPOWER));
	mutex_unlock(&data->mutex);
    return sprintf(buf, "%d\n", outputpower);
}

static ssize_t sx127x_outputpower_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	struct sx127x *data = dev_get_drvdata(dev);
	int idx = sx127x_indexofstring(buf, paoutput, ARRAY_SIZE(paoutput));
	u8 paconfig;
	sx127x_reg_read(data->spidevice, SX127X_REG_PACONFIG, &paconfig);
	sx127x_reg_write(data->spidevice, SX127X_REG_PACONFIG, paconfig);
	return count;
}

static DEVICE_ATTR(outputpower, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		sx127x_outputpower_show, sx127x_outputpower_store);

static int sx127x_dev_open(struct inode *inode, struct file *file){
	struct sx127x *data;
	int status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(data, &device_list, device_entry) {
		if (data->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if(status){
		pr_debug("sx127x: nothing for minor %d\n", iminor(inode));
		goto err_notfound;
	}

	mutex_lock(&data->mutex);
	if(data->open){
		pr_debug("sx127x: already open\n");
		status = -EBUSY;
		goto err_open;
	}
	data->open = 1;
	mutex_unlock(&data->mutex);

	mutex_unlock(&device_list_lock);

	file->private_data = data;
	return 0;

	err_open:
	mutex_unlock(&data->mutex);
	err_notfound:
	mutex_unlock(&device_list_lock);
	return status;
}

static ssize_t sx127x_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos){
	struct sx127x *data = filp->private_data;
	unsigned copied;
	ssize_t ret = 0;
	wait_event_interruptible(data->readwq, kfifo_len(&data->out));
	ret = kfifo_to_user(&data->out, buf, count, &copied);
	if(!ret && copied > 0){
		ret = copied;
	}
	return ret;
}

static ssize_t sx127x_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos){
	struct sx127x *data = filp->private_data;
	size_t packetsz, offset, maxpkt = 256;
	u8 kbuf[256];
	dev_info(&data->spidevice->dev, "char device write; %d\n", count);
	for(offset = 0; offset < count; offset += maxpkt){
		packetsz = min((count - offset), maxpkt);
		mutex_lock(&data->mutex);
		copy_from_user(kbuf, buf + offset, packetsz);
		sx127x_fifo_writepkt(data->spidevice, kbuf, packetsz);
		data->transmitted = 0;
		sx127x_setopmode(data, SX127X_OPMODE_TX, false);
		mutex_unlock(&data->mutex);
		wait_event_interruptible_timeout(data->writewq, data->transmitted, 60 * HZ);
	}
	return count;
}

static int sx127x_dev_release(struct inode *inode, struct file *filp){
	struct sx127x *data = filp->private_data;
	mutex_lock(&data->mutex);
	sx127x_setopmode(data, SX127X_OPMODE_STANDBY, true);
	data->open = 0;
	kfifo_reset(&data->out);
	mutex_unlock(&data->mutex);
	return 0;
}

static long sx127x_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){
	struct sx127x *data = filp->private_data;
	int ret;
	enum sx127x_ioctl_cmd ioctlcmd = cmd;
	mutex_lock(&data->mutex);
	switch(ioctlcmd){
		case SX127X_IOCTL_CMD_SETMODULATION:
			ret = sx127x_setmodulation(data, arg);
			break;
		case SX127X_IOCTL_CMD_GETMODULATION:
			ret = 0;
			break;
		case SX127X_IOCTL_CMD_SETCARRIERFREQUENCY:
			ret = sx127x_setcarrierfrequency(data, arg);
			break;
		case SX127X_IOCTL_CMD_GETCARRIERFREQUENCY:
			ret = 0;
			break;
		case SX127X_IOCTL_CMD_SETSF:
			ret = sx127x_setsf(data, arg);
			break;
		case SX127X_IOCTL_CMD_GETSF:
			ret = 0;
			break;
		case SX127X_IOCTL_CMD_SETOPMODE:
			ret = sx127x_setopmode(data, arg, true);
			break;
		case SX127X_IOCTL_CMD_GETOPMODE:
			ret = 0;
			break;
		case SX127X_IOCTL_CMD_SETPAOUTPUT:
			ret = sx127x_setpaoutput(data, arg);
			break;
		case SX127X_IOCTL_CMD_GETPAOUTPUT:
			ret = 0;
			break;
		default:
			ret = -EINVAL;
			break;
	}
	mutex_unlock(&data->mutex);
	return ret;
}

static struct file_operations fops = {
		.open = sx127x_dev_open,
		.read = sx127x_dev_read,
		.write = sx127x_dev_write,
		.release = sx127x_dev_release,
		.unlocked_ioctl = sx127x_dev_ioctl
};

static irqreturn_t sx127x_irq(int irq, void *dev_id)
{
	struct sx127x *data = dev_id;
	schedule_work(&data->irq_work);
	return IRQ_HANDLED;
}

static void sx127x_irq_work_handler(struct work_struct *work){
	struct sx127x *data = container_of(work, struct sx127x, irq_work);
	u8 irqflags, buf[128], len, snr, rssi;
	u32 fei;
	struct sx127x_pkt pkt;
	mutex_lock(&data->mutex);
	sx127x_reg_read(data->spidevice, SX127X_REG_LORA_IRQFLAGS, &irqflags);
	if(irqflags & SX127X_REG_LORA_IRQFLAGS_RXDONE){
		dev_warn(data->chardevice, "reading packet\n");
		memset(&pkt, 0, sizeof(pkt));

		sx127x_fifo_readpkt(data->spidevice, buf, &len);
		sx127x_reg_read(data->spidevice, SX127X_REG_LORA_PKTSNRVALUE, &snr);
		sx127x_reg_read(data->spidevice, SX127X_REG_LORA_PKTRSSIVALUE, &rssi);
		sx127x_reg_read24(data->spidevice, SX127X_REG_LORA_FEIMSB, &fei);

		pkt.hdrlen = sizeof(pkt);
		pkt.payloadlen = len;
		pkt.len = pkt.hdrlen + pkt.payloadlen;
		pkt.snr = (__s16)(snr << 2) / 4 ;
		pkt.rssi = -157 + rssi; //TODO fix this for the LF port

		if(irqflags & SX127X_REG_LORA_IRQFLAGS_PAYLOADCRCERROR){
			dev_warn(data->chardevice, "CRC Error for received payload\n");
			pkt.crcfail = 1;
		}

		kfifo_in(&data->out, &pkt, sizeof(pkt));
		kfifo_in(&data->out, buf, len);
		wake_up(&data->readwq);
	}
	else if(irqflags & SX127X_REG_LORA_IRQFLAGS_TXDONE){
		if(data->gpio_txen){
			gpiod_set_value(data->gpio_txen, 0);
		}
		dev_warn(data->chardevice, "transmitted packet\n");
		/* after tx the chip goes back to standby so restore the user selected mode if it wasn't standby */
		if(data->opmode != SX127X_OPMODE_STANDBY){
			dev_info(data->chardevice, "restoring opmode\n");
			sx127x_setopmode(data, data->opmode, false);
		}
		data->transmitted = 1;
		wake_up(&data->writewq);
	}
	else if(irqflags & SX127X_REG_LORA_IRQFLAGS_CADDONE){
		if(irqflags & SX127X_REG_LORA_IRQFLAGS_CADDETECTED){
			dev_info(data->chardevice, "CAD done, detected activity\n");
		}
		else {
			dev_info(data->chardevice, "CAD done, nothing detected\n");
		}
	}
	else {
		dev_err(&data->spidevice->dev, "unhandled interrupt state %02x\n", (unsigned) irqflags);
	}
	sx127x_reg_write(data->spidevice, SX127X_REG_LORA_IRQFLAGS, 0xff);
	mutex_unlock(&data->mutex);
}

static int sx127x_probe(struct spi_device *spi){
	int ret = 0;
	struct sx127x *data;
	u8 version;
	int irq;
	unsigned minor;

	// allocate all of the crap we need
	data = kmalloc(sizeof(*data), GFP_KERNEL);
	if(!data){
		printk("failed to allocate driver data\n");
		ret = -ENOMEM;
		goto err_allocdevdata;
	}

	data->open = 0;
	INIT_WORK(&data->irq_work, sx127x_irq_work_handler);
	INIT_LIST_HEAD(&data->device_entry);
	init_waitqueue_head(&data->readwq);
	init_waitqueue_head(&data->writewq);
	mutex_init(&data->mutex);
	data->fosc = 32000000; //TODO use value from DT
	data->pa = SX127X_PA_PABOOST; // TODO use value from DT
	data->spidevice = spi;
	data->opmode = SX127X_OPMODE_STANDBY;

	ret = kfifo_alloc(&data->out, PAGE_SIZE, GFP_KERNEL);
	if(ret){
		printk("failed to allocate out fifo\n");
		goto err_allocoutfifo;
	}

	// get the reset gpio and reset the chip
	data->gpio_reset = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_LOW);
	if(IS_ERR(data->gpio_reset)){
		dev_err(&spi->dev, "reset gpio is required");
		ret = -ENOMEM;
		goto err_resetgpio;
	}

	gpiod_set_value(data->gpio_reset, 1);
	mdelay(100);
	gpiod_set_value(data->gpio_reset, 0);
	mdelay(100);

	// get the rev from the chip and check it's what we expect
	sx127x_reg_read(spi, SX127X_REG_VERSION, &version);
	if(version != 0x12){
		dev_err(&spi->dev, "unknown chip version %x\n", version);
		ret = -EINVAL;
		goto err_chipid;
	}
	dev_warn(&spi->dev, "chip version %x\n",(unsigned) version);

	// get the other optional gpios
	data->gpio_txen = devm_gpiod_get_index(&spi->dev, "txrxswitch", 0, GPIOD_OUT_LOW);
	if(IS_ERR(data->gpio_txen)){
		dev_warn(&spi->dev, "no tx enable\n");
		data->gpio_txen = NULL;
	}

	data->gpio_rxen = devm_gpiod_get_index(&spi->dev, "txrxswitch", 1, GPIOD_OUT_LOW);
	if(IS_ERR(data->gpio_rxen)){
		dev_warn(&spi->dev, "no rx enable\n");
		data->gpio_rxen = NULL;
	}

	// get the irq
	irq = irq_of_parse_and_map(spi->dev.of_node, 0);
	if (!irq) {
		dev_err(&spi->dev, "No irq in platform data\n");
		ret = -EINVAL;
		goto err_irq;
	}
	devm_request_irq(&spi->dev, irq, sx127x_irq, 0, SX127X_DRIVERNAME, data);

	// create the frontend device and stash it in the spi device
	mutex_lock(&device_list_lock);
	minor = 0;
	data->devt = MKDEV(devmajor, minor);
	data->chardevice = device_create(devclass, &spi->dev, data->devt, data, SX127X_DEVICENAME, minor);
	if(IS_ERR(data->chardevice)){
		printk("failed to create char device\n");
		ret = -ENOMEM;
		goto err_createdevice;
	}
	list_add(&data->device_entry, &device_list);
	mutex_unlock(&device_list_lock);
	spi_set_drvdata(spi, data);

	// setup sysfs nodes
	ret = device_create_file(data->chardevice, &dev_attr_modulation);
	ret = device_create_file(data->chardevice, &dev_attr_opmode);
	ret = device_create_file(data->chardevice, &dev_attr_carrierfrequency);
	ret = device_create_file(data->chardevice, &dev_attr_rssi);
	ret = device_create_file(data->chardevice, &dev_attr_paoutput);
	ret = device_create_file(data->chardevice, &dev_attr_outputpower);
	// these are LoRa specifc
	ret = device_create_file(data->chardevice, &dev_attr_sf);
	ret = device_create_file(data->chardevice, &dev_attr_bw);
	ret = device_create_file(data->chardevice, &dev_attr_codingrate);
	ret = device_create_file(data->chardevice, &dev_attr_implicitheadermodeon);

	return 0;

	err_sysfs:
		device_destroy(devclass, data->devt);
	err_createdevice:
		mutex_unlock(&device_list_lock);
	err_irq:
	err_chipid:
	err_resetgpio:
		kfifo_free(&data->out);
	err_allocoutfifo:
		kfree(data);
	err_allocdevdata:
	return ret;
}

static int sx127x_remove(struct spi_device *spi){
	struct sx127x *data = spi_get_drvdata(spi);

	device_remove_file(data->chardevice, &dev_attr_modulation);
	device_remove_file(data->chardevice, &dev_attr_opmode);
	device_remove_file(data->chardevice, &dev_attr_carrierfrequency);
	device_remove_file(data->chardevice, &dev_attr_rssi);
	device_remove_file(data->chardevice, &dev_attr_paoutput);
	device_remove_file(data->chardevice, &dev_attr_outputpower);

	device_remove_file(data->chardevice, &dev_attr_sf);
	device_remove_file(data->chardevice, &dev_attr_bw);
	device_remove_file(data->chardevice, &dev_attr_codingrate);
	device_remove_file(data->chardevice, &dev_attr_implicitheadermodeon);

	device_destroy(devclass, data->devt);

	kfifo_free(&data->out);
	kfree(data);

	return 0;
}

static const struct of_device_id sx127x_of_match[] = {
	{
		.compatible = "semtech, sx127x",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, sx127x_of_match);


static struct spi_driver sx127x_driver = {
	.probe		= sx127x_probe,
	.remove		= sx127x_remove,
	.driver = {
		.name	= SX127X_DRIVERNAME,
		.of_match_table = of_match_ptr(sx127x_of_match),
		.owner = THIS_MODULE,
	},
};

static int __init sx127x_init(void)
{
	int ret;
	ret = register_chrdev(0, SX127X_DRIVERNAME, &fops);
	if(ret < 0){
		printk("failed to register char device\n");
		goto out;
	}

	devmajor = ret;

	devclass = class_create(THIS_MODULE, SX127X_CLASSNAME);
	if(!devclass){
		printk("failed to register class\n");
		ret = -ENOMEM;
		goto out1;
	}

	ret = spi_register_driver(&sx127x_driver);
	if(ret){
		printk("failed to register spi driver\n");
		goto out2;
	}

	goto out;

	out2:
	out1:
		class_destroy(devclass);
		devclass = NULL;
	out:
	return ret;
}
module_init(sx127x_init);

static void __exit sx127x_exit(void)
{
	spi_unregister_driver(&sx127x_driver);
	unregister_chrdev(devmajor, SX127X_DRIVERNAME);
	class_destroy(devclass);
	devclass = NULL;
}
module_exit(sx127x_exit);

MODULE_LICENSE("GPL");
