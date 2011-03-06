/*
 * zephyr.c - Zephyr Touchscreen driver, used in iPhone.
 *
 * Authors: Yidou Wang, Patrick Wildt, Ricky Taylor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <mach/iphone-spi.h>
#include <mach/gpio.h>

#ifdef CONFIG_IPHONE_2G
#define MT_GPIO_POWER 0x804
#define MT_ATN_INTERRUPT 0xa3
#else
#define MT_GPIO_POWER 0x701
#define MT_ATN_INTERRUPT 0x9b
#endif

#ifdef CONFIG_IPHONE_3G
#define MT_SPI 1
#define MT_SPI_CS GPIO_SPI1_CS0
#else
#define MT_SPI 2
#define MT_SPI_CS GPIO_SPI2_CS0
#endif

#define MT_INFO_FAMILYID 0xD1
#define MT_INFO_SENSORINFO 0xD3
#define MT_INFO_SENSORREGIONDESC 0xD0
#define MT_INFO_SENSORREGIONPARAM 0xA1
#define MT_INFO_SENSORDIM 0xD9

typedef struct MTFrameHeader
{
	u8 type;
	u8 frameNum;
	u8 headerLen;
	u8 unk_3;
	u32 timestamp;
	u8 unk_8;
	u8 unk_9;
	u8 unk_A;
	u8 unk_B;
	u16 unk_C;
	u16 isImage;

	u8 numFingers;
	u8 fingerDataLen;
	u16 unk_12;
	u16 unk_14;
	u16 unk_16;
} MTFrameHeader;

typedef struct FingerData
{
	u8 id;
	u8 event;
	u8 unk_2;
	u8 unk_3;
	s16 x;
	s16 y;
	s16 rel_x;
	s16 rel_y;
	u16 size_major;
	u16 size_minor;
	u16 orientation;
	u16 force_major;
	u16 force_minor;
	u16 unk_16;
	u16 unk_18;
	u16 unk_1A;
} FingerData;

#define MAX_FINGER_ORIENTATION  16384

static irqreturn_t zephyr_irq(int irq, void* pToken);

struct zephyr_data
{
	u8* OutputPacket;
	u8* InputPacket;
	u8* GetInfoPacket;
	u8* GetResultPacket;

	int InterfaceVersion;
	int MaxPacketSize;
	int FamilyID;
	int SensorWidth;
	int SensorHeight;
	int SensorColumns;
	int SensorRows;
	int BCDVersion;
	int Endianness;
	u8* SensorRegionDescriptor;
	int SensorRegionDescriptorLen;
	u8* SensorRegionParam;
	int SensorRegionParamLen;

	u8 min_pressure;

	int CurNOP;

	bool firmware_loaded;
	struct input_dev* input_dev;

	int irq_count;
	spinlock_t irq_lock;
	struct work_struct irq_work;

	struct spi_device *spi_dev;
};


// This is flipped between 0x64 and 0x65 for every transaction

typedef struct MTSPISetting
{
	int speed;
	int txDelay;
	int rxDelay;
} MTSPISetting;

const MTSPISetting MTNormalSpeed = {83000, 5, 10};
const MTSPISetting MTFastSpeed = {4500000, 0, 10};

#define NORMAL_SPEED (&MTNormalSpeed)
#define FAST_SPEED (&MTFastSpeed)

static int zephyr_txrx(struct zephyr_data *_z, const MTSPISetting* setting, const u8* outBuffer, int outLen, u8* inBuffer, int inLen);
static int zephyr_tx(struct zephyr_data *_z, const MTSPISetting* setting, const u8* outBuffer, int outLen);

static int makeBootloaderDataPacket(u8* output, u32 destAddress, const u8* data, int dataLen, int* cksumOut);
static bool verifyUpload(struct zephyr_data *_z, int checksum);
static void sendExecutePacket(struct zephyr_data *_z);
static void sendBlankDataPacket(struct zephyr_data *_z);

static bool loadASpeedFirmware(struct zephyr_data *_z, const u8* firmware, int len);
static bool loadMainFirmware(struct zephyr_data *_z, const u8* firmware, int len);
static bool determineInterfaceVersion(struct zephyr_data *_z);

static bool getReportInfo(struct zephyr_data *_z, int id, u8* err, u16* len);
static bool getReport(struct zephyr_data *_z, int id, u8* buffer, int* outLen);

static bool readFrameLength(struct zephyr_data *_z, int* len);
static int readFrame(struct zephyr_data *_z);
static bool readResultData(struct zephyr_data *_z, int len);

static void newPacket(struct zephyr_data *_z, const u8* data, int len);
static void zephyr_irq_work(struct work_struct* work);

u8* aspeed_fw;
size_t aspeed_fw_size;
u8* main_fw;
size_t main_fw_size;

DECLARE_WORK(zephyr_workqueue, &zephyr_irq_work);

int zephyr_setup(struct zephyr_data *_z, const u8* ASpeedFirmware, int ASpeedFirmwareLen, const u8* mainFirmware, int mainFirmwareLen)
{
	int i;
	int ret;
	u8* reportBuffer;
	int reportLen;

	printk("zephyr: A-Speed firmware at 0x%08x - 0x%08x, Main firmware at 0x%08x - 0x%08x\n",
			(u32) ASpeedFirmware, (u32)(ASpeedFirmware + ASpeedFirmwareLen),
			(u32) mainFirmware, (u32)(mainFirmware + mainFirmwareLen));

	_z->OutputPacket = (u8*) kmalloc(0x400, GFP_KERNEL);
	_z->InputPacket = (u8*) kmalloc(0x400, GFP_KERNEL);
	_z->GetInfoPacket = (u8*) kmalloc(0x400, GFP_KERNEL);
	_z->GetResultPacket = (u8*) kmalloc(0x400, GFP_KERNEL);

	memset(_z->GetInfoPacket, 0x82, 0x400);
	memset(_z->GetResultPacket, 0x68, 0x400);

	if(request_irq(MT_ATN_INTERRUPT + IPHONE_GPIO_IRQS, zephyr_irq, IRQF_TRIGGER_FALLING, "zephyr", (void*) 0))
	{
		printk("zephyr: Failed to request z interrupt.\n");
	}

	// Power up the device (turn it off then on again. ;])
	printk("zephyr: powering on\n");
	iphone_gpio_pin_output(MT_GPIO_POWER, 0);
	msleep(200);

	iphone_gpio_pin_output(MT_GPIO_POWER, 1);
	msleep(15);

	printk("zephyr: Sending A-Speed firmware...\n");
	if(!loadASpeedFirmware(_z, ASpeedFirmware, ASpeedFirmwareLen))
	{
		kfree(_z->InputPacket);
		kfree(_z->OutputPacket);
		kfree(_z->GetInfoPacket);
		kfree(_z->GetResultPacket);
		return -1;
	}

	msleep(1);

	printk("zephyr: Sending main firmware...\n");
	if(!loadMainFirmware(_z, mainFirmware, mainFirmwareLen))
	{
		kfree(_z->InputPacket);
		kfree(_z->OutputPacket);
		kfree(_z->GetInfoPacket);
		kfree(_z->GetResultPacket);
		return -1;
	}

	msleep(1);

	printk("zephyr: Determining interface version...\n");
	if(!determineInterfaceVersion(_z))
	{
		kfree(_z->InputPacket);
		kfree(_z->OutputPacket);
		kfree(_z->GetInfoPacket);
		kfree(_z->GetResultPacket);
		return -1;
	}

	reportBuffer = (u8*) kmalloc(_z->MaxPacketSize, GFP_KERNEL);

	if(!getReport(_z, MT_INFO_FAMILYID, reportBuffer, &reportLen))
	{
		printk("zephyr: failed getting family id!\n");
		kfree(_z->InputPacket);
		kfree(_z->OutputPacket);
		kfree(_z->GetInfoPacket);
		kfree(_z->GetResultPacket);
		return -1;
	}

	_z->FamilyID = reportBuffer[0];

	if(!getReport(_z, MT_INFO_SENSORINFO, reportBuffer, &reportLen))
	{
		printk("zephyr: failed getting sensor info!\n");
		kfree(_z->InputPacket);
		kfree(_z->OutputPacket);
		kfree(_z->GetInfoPacket);
		kfree(_z->GetResultPacket);
		return -1;
	}

	_z->SensorColumns = reportBuffer[2];
	_z->SensorRows = reportBuffer[1];
	_z->BCDVersion = ((reportBuffer[3] & 0xFF) << 8) | (reportBuffer[4] & 0xFF);
	_z->Endianness = reportBuffer[0];

	if(!getReport(_z, MT_INFO_SENSORREGIONDESC, reportBuffer, &reportLen))
	{
		printk("zephyr: failed getting sensor region descriptor!\n");
		kfree(_z->InputPacket);
		kfree(_z->OutputPacket);
		kfree(_z->GetInfoPacket);
		kfree(_z->GetResultPacket);
		return -1;
	}

	_z->SensorRegionDescriptorLen = reportLen;
	_z->SensorRegionDescriptor = (u8*) kmalloc(reportLen, GFP_KERNEL);
	memcpy(_z->SensorRegionDescriptor, reportBuffer, reportLen);

	if(!getReport(_z, MT_INFO_SENSORREGIONPARAM, reportBuffer, &reportLen))
	{
		printk("zephyr: failed getting sensor region param!\n");
		kfree(_z->InputPacket);
		kfree(_z->OutputPacket);
		kfree(_z->GetInfoPacket);
		kfree(_z->GetResultPacket);
		kfree(_z->SensorRegionDescriptor);
		return -1;
	}

	_z->SensorRegionParamLen = reportLen;
	_z->SensorRegionParam = (u8*) kmalloc(reportLen, GFP_KERNEL);
	memcpy(_z->SensorRegionParam, reportBuffer, reportLen);

	if(!getReport(_z, MT_INFO_SENSORDIM, reportBuffer, &reportLen))
	{
		printk("zephyr: failed getting sensor surface dimensions!\n");
		kfree(_z->InputPacket);
		kfree(_z->OutputPacket);
		kfree(_z->GetInfoPacket);
		kfree(_z->GetResultPacket);
		kfree(_z->SensorRegionDescriptor);
		kfree(_z->SensorRegionParam);
		return -1;
	}

	_z->SensorWidth = (9000 - *((u32*)&reportBuffer[0])) * 84 / 73;
	_z->SensorHeight = (13850 - *((u32*)&reportBuffer[4])) * 84 / 73;

	printk("Family ID                : 0x%x\n", _z->FamilyID);
	printk("Sensor rows              : 0x%x\n", _z->SensorRows);
	printk("Sensor columns           : 0x%x\n", _z->SensorColumns);
	printk("Sensor width             : 0x%x\n", _z->SensorWidth);
	printk("Sensor height            : 0x%x\n", _z->SensorHeight);
	printk("BCD Version              : 0x%x\n", _z->BCDVersion);
	printk("Endianness               : 0x%x\n", _z->Endianness);
	printk("Sensor region descriptor :");
	for(i = 0; i < _z->SensorRegionDescriptorLen; ++i)
		printk(" %02x", _z->SensorRegionDescriptor[i]);
	printk("\n");

	printk("Sensor region param      :");
	for(i = 0; i < _z->SensorRegionParamLen; ++i)
		printk(" %02x", _z->SensorRegionParam[i]);
	printk("\n");

	kfree(reportBuffer);

	_z->input_dev = input_allocate_device();
	if(!_z->input_dev)
	{
		kfree(_z->InputPacket);
		kfree(_z->OutputPacket);
		kfree(_z->GetInfoPacket);
		kfree(_z->GetResultPacket);
		kfree(_z->SensorRegionDescriptor);
		kfree(_z->SensorRegionParam);
		return -1;
	}


	_z->input_dev->name = "iPhone Zephyr Multitouch Screen";
	_z->input_dev->phys = "multitouch0";
	_z->input_dev->id.vendor = 0x05AC;
	_z->input_dev->id.product = 0;
	_z->input_dev->id.version = 0x0000;
	_z->input_dev->dev.parent = &_z->spi_dev->dev;
	_z->input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	_z->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(_z->input_dev, ABS_X, 0, _z->SensorWidth, 0, 0);
	input_set_abs_params(_z->input_dev, ABS_Y, 0, _z->SensorHeight, 0, 0);
	input_set_abs_params(_z->input_dev, ABS_MT_TOUCH_MAJOR, 0, max(_z->SensorHeight, _z->SensorWidth), 0, 0);
	input_set_abs_params(_z->input_dev, ABS_MT_TOUCH_MINOR, 0, max(_z->SensorHeight, _z->SensorWidth), 0, 0);
	input_set_abs_params(_z->input_dev, ABS_MT_WIDTH_MAJOR, 0, max(_z->SensorHeight, _z->SensorWidth), 0, 0);
	input_set_abs_params(_z->input_dev, ABS_MT_WIDTH_MINOR, 0, max(_z->SensorHeight, _z->SensorWidth), 0, 0);
	input_set_abs_params(_z->input_dev, ABS_MT_ORIENTATION, -MAX_FINGER_ORIENTATION, MAX_FINGER_ORIENTATION, 0, 0);
	input_set_abs_params(_z->input_dev, ABS_MT_POSITION_X, 0, _z->SensorWidth, 0, 0);
	input_set_abs_params(_z->input_dev, ABS_MT_POSITION_Y, 0, _z->SensorHeight, 0, 0);

	/* not sure what the actual max is */
	input_set_abs_params(_z->input_dev, ABS_MT_TRACKING_ID, 0, 32, 0, 0);

	ret = input_register_device(_z->input_dev);
	if(ret != 0)
	{
		kfree(_z->InputPacket);
		kfree(_z->OutputPacket);
		kfree(_z->GetInfoPacket);
		kfree(_z->GetResultPacket);
		kfree(_z->SensorRegionDescriptor);
		kfree(_z->SensorRegionParam);
		return -1;
	}

	_z->CurNOP = 0x64;
	_z->irq_count = 0;

	_z->firmware_loaded = true;

	readFrame(_z);

	return 0;
}

static void zephyr_irq_work(struct work_struct* work)
{
	struct zephyr_data *z = container_of(work, struct zephyr_data, irq_work);
	unsigned long flags;

	dev_dbg(&z->spi_dev->dev, "irq entered (%d).\n", z->irq_count);

	spin_lock_irqsave(&z->irq_lock, flags);

	z->irq_count++;

	if(z->irq_count > 1)
	{
		spin_unlock_irqrestore(&z->irq_lock, flags);
		return;
	}

	while(z->irq_count > 0)
	{
		spin_unlock_irqrestore(&z->irq_lock, flags);

		readFrame(z);

		spin_lock_irqsave(&z->irq_lock, flags);
		z->irq_count--;
	}
	spin_unlock_irqrestore(&z->irq_lock, flags);

	dev_dbg(&z->spi_dev->dev, "irq exited (%d).\n", z->irq_count);
}

static void newPacket(struct zephyr_data *_z, const u8* data, int len)
{
	int i;
	FingerData* finger;
	MTFrameHeader* header = (MTFrameHeader*) data;
	if(header->type != 0x44 && header->type != 0x43)
		printk("zephyr: unknown frame type 0x%x\n", header->type);

	finger = (FingerData*)(data + (header->headerLen));

	if(header->headerLen < 12)
		printk("zephyr: no finger data in frame\n");

	for(i = 0; i < header->numFingers; ++i)
	{
		if(finger->force_major > _z->min_pressure)
		{
			finger->force_major -= _z->min_pressure;
		}
		else
			finger->force_major = 0;

		if(finger->force_minor > _z->min_pressure)
		{
			finger->force_minor -= _z->min_pressure;
		}
		else 
			finger->force_minor = 0;

		if(finger->force_major > 0 || 
				finger->force_minor > 0)
		{
			input_report_abs(_z->input_dev, ABS_MT_TOUCH_MAJOR, finger->force_major);
			input_report_abs(_z->input_dev, ABS_MT_TOUCH_MINOR, finger->force_minor);
			input_report_abs(_z->input_dev, ABS_MT_WIDTH_MAJOR, finger->size_major);
			input_report_abs(_z->input_dev, ABS_MT_WIDTH_MINOR, finger->size_minor);
			input_report_abs(_z->input_dev, ABS_MT_ORIENTATION, MAX_FINGER_ORIENTATION - finger->orientation);
			input_report_abs(_z->input_dev, ABS_MT_TRACKING_ID, finger->id);
			input_report_abs(_z->input_dev, ABS_MT_POSITION_X, finger->x);
			input_report_abs(_z->input_dev, ABS_MT_POSITION_Y, _z->SensorHeight - finger->y);
		}

		input_mt_sync(_z->input_dev);

		finger = (FingerData*) (((u8*) finger) + header->fingerDataLen);
	}

	if(header->numFingers > 0)
	{
		finger = (FingerData*)(data + (header->headerLen));

		if (finger->force_minor > 0)
		{
			input_report_abs(_z->input_dev, ABS_X, finger->x);
			input_report_abs(_z->input_dev, ABS_Y, _z->SensorHeight - finger->y);
			input_report_key(_z->input_dev, BTN_TOUCH, finger->size_minor > 0);
		}
		else input_report_key(_z->input_dev, BTN_TOUCH, 0);
	}

	input_sync(_z->input_dev);
}

static int readFrame(struct zephyr_data *_z)
{
	int try = 0;

	for(try = 0; try < 4; ++try)
	{
		int len = 0;
		if(!readFrameLength(_z, &len))
		{
			printk("zephyr: error getting frame length\n");
			msleep(1);
			continue;
		}

		if(len)
		{
			if(!readResultData(_z, len + 1))
			{
				printk("zephyr: error getting frame data\n");
				msleep(1);
				continue;
			}

			if(_z->CurNOP == 0x64)
				_z->CurNOP = 0x65;
			else
				_z->CurNOP = 0x64;

			return 1;
		}

		return 0;
	}

	return -1;
}

static bool readResultData(struct zephyr_data *_z, int len)
{
	int try = 0;
	for(try = 0; try < 4; ++try)
	{
		int checksum;
		int myChecksum;
		int i;

		zephyr_txrx(_z, NORMAL_SPEED, _z->GetResultPacket, len, _z->InputPacket, len);

		if(_z->InputPacket[0] != 0xAA)
		{
			msleep(1);
			continue;
		}

		checksum = ((_z->InputPacket[len - 2] & 0xFF) << 8) | (_z->InputPacket[len - 1] & 0xFF);
		myChecksum = 0;

		for(i = 1; i < (len - 2); ++i)
			myChecksum += _z->InputPacket[i];

		myChecksum &= 0xFFFF;

		if(myChecksum != checksum)
		{
			msleep(1);
			continue;
		}

		newPacket(_z, _z->InputPacket + 1, len - 3);
		return true;
	}

	return false;

}

static bool readFrameLength(struct zephyr_data *_z, int* len)
{
	int try;
	u8 tx[8];
	u8 rx[8];
	memset(tx, _z->CurNOP, sizeof(tx));

	try = 0;
	for(try = 0; try < 4; ++try)
	{
		int tLen;
		int tLenCkSum;
		int checksum;

		zephyr_txrx(_z, NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

		if(rx[0] != 0xAA)
		{
			msleep(1);
			continue;
		}

		tLen = ((rx[4] & 0xFF) << 8) | (rx[5] & 0xFF);
		tLenCkSum = (rx[4] + rx[5]) & 0xFFFF;
		checksum = ((rx[6] & 0xFF) << 8) | (rx[7] & 0xFF);
		if(tLenCkSum != checksum)
		{
			msleep(1);
			continue;
		}

		if(tLen > _z->MaxPacketSize)
		{
			printk("zephyr: device unexpectedly requested to transfer a %d byte packet. Max size = %d\n", tLen, _z->MaxPacketSize);
			msleep(1);
			continue;
		}

		*len = tLen;

		return true;
	}

	return false;
}

static bool getReport(struct zephyr_data *_z, int id, u8* buffer, int* outLen)
{
	u8 err;
	u16 len;
	int try;
	if(!getReportInfo(_z, id, &err, &len))
		return false;

	if(err)
		return false;

	try = 0;
	for(try = 0; try < 4; ++try)
	{
		int checksum;
		int myChecksum;
		int i;

		_z->GetInfoPacket[1] = id;
		zephyr_txrx(_z, NORMAL_SPEED, _z->GetInfoPacket, len + 6, _z->InputPacket, len + 6);

		if(_z->InputPacket[0] != 0xAA)
		{
			msleep(1);
			continue;
		}

		checksum = ((_z->InputPacket[len + 4] & 0xFF) << 8) | (_z->InputPacket[len + 5] & 0xFF);
		myChecksum = id;

		for(i = 0; i < len; ++i)
			myChecksum += _z->InputPacket[i + 4];

		myChecksum &= 0xFFFF;

		if(myChecksum != checksum)
		{
			msleep(1);
			continue;
		}

		*outLen = len;
		memcpy(buffer, &_z->InputPacket[4], len);

		return true;
	}

	return false;
}

static bool getReportInfo(struct zephyr_data *_z, int id, u8* err, u16* len)
{
	u8 tx[8];
	u8 rx[8];

	int try;
	for(try = 0; try < 4; ++try)
	{
		int checksum;
		int myChecksum;

		memset(tx, 0x8F, 8);
		tx[1] = id;

		zephyr_txrx(_z, NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

		if(rx[0] != 0xAA)
		{
			msleep(1);
			continue;
		}

		checksum = ((rx[6] & 0xFF) << 8) | (rx[7] & 0xFF);
		myChecksum = (id + rx[4] + rx[5]) & 0xFFFF;

		if(checksum != myChecksum)
		{
			msleep(1);
			continue;
		}

		*err = (rx[4] >> 4) & 0xF;
		*len = ((rx[4] & 0xF) << 8) | (rx[5] & 0xFF);

		return true;
	}

	return false;
}

static bool determineInterfaceVersion(struct zephyr_data *_z)
{
	u8 tx[4];
	u8 rx[4];

	int try;
	for(try = 0; try < 4; ++try)
	{
		memset(tx, 0xD0, 4);

		zephyr_txrx(_z, NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

		if(rx[0] == 0xAA)
		{
			_z->InterfaceVersion = rx[1];
			_z->MaxPacketSize = (rx[2] << 8) | rx[3];

			printk("zephyr: interface version %d, max packet size: %d\n", _z->InterfaceVersion, _z->MaxPacketSize);
			return true;
		}

		_z->InterfaceVersion = 0;
		_z->MaxPacketSize = 1000;
		msleep(3);
	}

	printk("zephyr: failed getting interface version!\n");

	return false;
}

static bool loadASpeedFirmware(struct zephyr_data *_z, const u8* firmware, int len)
{
	u32 address = 0x40000000;
	const u8* data = firmware;
	int left = len;

	while(left > 0)
	{
		int checksum;
		int try;
		int toUpload = left;
		if(toUpload > 0x3F8)
			toUpload = 0x3F8;

		makeBootloaderDataPacket(_z->OutputPacket, address, data, toUpload, &checksum);

		for(try = 0; try < 5; ++try)
		{
			printk("zephyr: uploading data packet\n");
			zephyr_tx(_z, NORMAL_SPEED, _z->OutputPacket, 0x400);

			udelay(300);

			if(verifyUpload(_z, checksum))
				break;
		}

		if(try == 5)
			return false;

		address += toUpload;
		data += toUpload;
		left -= toUpload;
	}

	sendExecutePacket(_z);

	return true;
}

static bool loadMainFirmware(struct zephyr_data *_z, const u8* firmware, int len)
{
	int checksum = 0;

	int i;
	for(i = 0; i < len; ++i)
		checksum += firmware[i];

	for(i = 0; i < 5; ++i)
	{
		sendBlankDataPacket(_z);

		printk("zephyr: uploading main firmware\n");
		zephyr_tx(_z, FAST_SPEED, firmware, len);

		if(verifyUpload(_z, checksum))
			break;
	}

	if(i == 5)
		return false;

	sendExecutePacket(_z);

	return true;
}

static bool verifyUpload(struct zephyr_data *_z, int checksum)
{
	u8 tx[4];
	u8 rx[4];

	tx[0] = 5;
	tx[1] = 0;
	tx[2] = 0;
	tx[3] = 6;

	zephyr_txrx(_z, NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

	if(rx[0] != 0xD0 || rx[1] != 0x0)
	{
		printk("zephyr: data verification failed type bytes, got %02x %02x %02x %02x -- %x\n", rx[0], rx[1], rx[2], rx[3], checksum);
		return false;
	}

	if(rx[2] != ((checksum >> 8) & 0xFF))
	{
		printk("zephyr: data verification failed upper checksum, %02x != %02x\n", rx[2], (checksum >> 8) & 0xFF);
		return false;
	}

	if(rx[3] != (checksum & 0xFF))
	{
		printk("zephyr: data verification failed lower checksum, %02x != %02x\n", rx[3], checksum & 0xFF);
		return false;
	}

	printk("zephyr: data verification successful\n");
	return true;
}


static void sendExecutePacket(struct zephyr_data *_z)
{
	u8 tx[4];
	u8 rx[4];

	tx[0] = 0xC4;
	tx[1] = 0;
	tx[2] = 0;
	tx[3] = 0xC4;

	zephyr_txrx(_z, NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

	printk("zephyr: execute packet sent\n");
}

static void sendBlankDataPacket(struct zephyr_data *_z)
{
	u8 tx[4];
	u8 rx[4];

	tx[0] = 0xC2;
	tx[1] = 0;
	tx[2] = 0;
	tx[3] = 0;

	zephyr_txrx(_z, NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

	printk("zephyr: blank data packet sent\n");
}

static int makeBootloaderDataPacket(u8* output, u32 destAddress, const u8* data, int dataLen, int* cksumOut)
{
	int checksum;
	int i;

	if(dataLen > 0x3F8)
		dataLen = 0x3F8;

	output[0] = 0xC2;
	output[1] = (destAddress >> 24) & 0xFF;
	output[2] = (destAddress >> 16) & 0xFF;
	output[3] = (destAddress >> 8) & 0xFF;
	output[4] = destAddress & 0xFF;
	output[5] = 0;

	checksum = 0;

	for(i = 0; i < dataLen; ++i)
	{
		u8 byte = data[i];
		checksum += byte;
		output[6 + i] = byte;
	}

	for(i = 0; i < 6; ++i)
	{
		checksum += output[i];
	}

	memset(output + dataLen + 6, 0, 0x3F8 - dataLen);
	output[0x3FE] = (checksum >> 8) & 0xFF;
	output[0x3FF] = checksum & 0xFF;

	*cksumOut = checksum;

	return dataLen;
}

static irqreturn_t zephyr_irq(int irq, void* pToken)
{
	struct zephyr_data *z = (struct zephyr_data *)pToken;

	if(!z->firmware_loaded)
		return IRQ_HANDLED;

	dev_dbg(&z->spi_dev->dev, "irq.\n");

	schedule_work(&zephyr_workqueue);

	return IRQ_HANDLED;
}


int zephyr_tx(struct zephyr_data *_z, const MTSPISetting* setting, const u8* outBuffer, int outLen)
{
	int ret;
	struct spi_transfer tx = {
		.tx_buf = outBuffer,
		.len = outLen,
		.speed_hz = setting->speed,
		.delay_usecs = setting->txDelay * 1000,
	};
	struct spi_message msg;

	spi_message_init(&msg);
	spi_message_add_tail(&tx, &msg);

	ret = spi_sync(_z->spi_dev, &msg);
	if(ret != 0)
		dev_err(&_z->spi_dev->dev, "tx failed (%d).\n", ret);
	return ret;

}

int zephyr_txrx(struct zephyr_data *_z, const MTSPISetting* setting, const u8* outBuffer, int outLen, u8* inBuffer, int inLen)
{
	int ret;
	int sz = (outLen > inLen) ? outLen : inLen;
	struct spi_transfer tx = {
		.tx_buf = outBuffer,
		.rx_buf = inBuffer,
		.len = sz,
		.speed_hz = setting->speed,
		.delay_usecs = setting->rxDelay * 1000,
	};

	struct spi_message msg;

	spi_message_init(&msg);
	spi_message_add_tail(&tx, &msg);

	ret = spi_sync(_z->spi_dev, &msg);
	if(ret != 0)
		dev_err(&_z->spi_dev->dev, "tx failed (%d).\n", ret);
	return ret;
}

static void got_main(const struct firmware* fw, void *context)
{
	struct zephyr_data *z = (struct zephyr_data *)context;
	if(!fw)
	{
		printk("zephyr: couldn't get main firmware, trying again...\n");
		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "zephyr_main.bin", &z->spi_dev->dev, NULL, got_main);
		return;
	}

	main_fw = kmalloc(fw->size, GFP_KERNEL);
	main_fw_size = fw->size;
	memcpy(main_fw, fw->data, fw->size);

	printk("zephyr: initializing multitouch\n");
	zephyr_setup(z, aspeed_fw, aspeed_fw_size, main_fw, main_fw_size);

	/* caller will call release_firmware */
}

static void got_aspeed(const struct firmware* fw, void *context)
{
	struct zephyr_data *z = (struct zephyr_data *)context;
	if(!fw)
	{
		printk("zephyr: couldn't get a-speed firmware, trying again...\n");
		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "zephyr_aspeed.bin", &z->spi_dev->dev, NULL, got_aspeed);
		return;
	}

	aspeed_fw = kmalloc(fw->size, GFP_KERNEL);
	aspeed_fw_size = fw->size;
	memcpy(aspeed_fw, fw->data, fw->size);

	printk("zephyr: requesting main firmware\n");
	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "zephyr_main.bin", &z->spi_dev->dev, NULL, got_main);

	/* caller will call release_firmware */
}

// Device Attributes
static ssize_t zephyr_min_pressure_show(struct device *_dev, struct device_attribute *_attr, char *_buf)
{
	struct spi_device *spi_dev = container_of(_dev, struct spi_device, dev);
	struct zephyr_data *z = (struct zephyr_data*)spi_get_drvdata(spi_dev);

	return sprintf(_buf, "%u\n", (unsigned int)z->min_pressure);
}

static ssize_t zephyr_min_pressure_store(struct device *_dev, struct device_attribute *_attr, const char* _buf, size_t _count)
{
	struct spi_device *spi_dev = container_of(_dev, struct spi_device, dev);
	struct zephyr_data *z = (struct zephyr_data*)spi_get_drvdata(spi_dev);

	unsigned int new_val;
	ssize_t ret = sscanf(_buf, "%u\n", &new_val);

	if(ret <= 0)
		return ret;

	if(new_val >= 255)
		return 0;

	z->min_pressure = (u8)new_val;

	return ret;
}

static DEVICE_ATTR(min_pressure, 0666, &zephyr_min_pressure_show, &zephyr_min_pressure_store);


static int zephyr_probe(struct spi_device *_dev)
{
	int ret;
	struct zephyr_data *z = (struct zephyr_data *)kmalloc(sizeof(struct zephyr_data), GFP_KERNEL);
	memset(z, sizeof(struct zephyr_data), 0);

	spi_set_drvdata(_dev, z);
	_dev->bits_per_word = 8;
	ret = spi_setup(_dev);
	if(ret)
	{
		dev_err(&_dev->dev, "failed to setup SPI device.\n");
		return ret;
	}

	INIT_WORK(&z->irq_work, &zephyr_irq_work);
	spin_lock_init(&z->irq_lock);
	z->firmware_loaded = false;
	z->spi_dev = _dev;
	z->min_pressure = 100;
	z->irq_count = 0;

	ret = device_create_file(&_dev->dev, &dev_attr_min_pressure);
	if(ret)
		dev_err(&_dev->dev, "failed to create min_pressure attribute.\n");

	printk("zephyr: requesting A-Speed firmware\n");
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "zephyr_aspeed.bin", &z->spi_dev->dev, NULL, got_aspeed);
}

static int zephyr_remove(struct spi_device *_dev)
{
	device_remove_file(&_dev->dev, &dev_attr_min_pressure);

	return 0;
}

// TODO: power management
#define zephyr_shutdown NULL
#define zephyr_suspend NULL
#define zephyr_resume NULL


static struct spi_driver zephyr_driver = {
	.driver = {
		.name = "zephyr",
	},

	.probe = zephyr_probe,
	.remove = zephyr_remove,
	.shutdown = zephyr_shutdown,
	.suspend = zephyr_suspend,
	.resume = zephyr_resume,
};

static int __init zephyr_init(void)
{
	int ret;
	ret = spi_register_driver(&zephyr_driver);
	if(ret)
		printk("zephyr: failed to register driver.\n");
	return ret;
}
module_init(zephyr_init);

static void __exit zephyr_exit(void)
{
	spi_unregister_driver(&zephyr_driver);
}

module_exit(zephyr_exit);

MODULE_DESCRIPTION("iPhone Zephyr multitouch driver");
MODULE_AUTHOR("Yiduo Wang");
MODULE_LICENSE("GPL");
