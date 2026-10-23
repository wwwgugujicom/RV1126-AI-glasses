// SPDX-License-Identifier: GPL-2.0
/*
 * ICM-42670 / ICM-42688-P 6-axis IMU I2C driver for RV1126B Main Board
 *
 * Auto-detects the chip by WHO_AM_I (register 0x75):
 *   0x67 -> ICM-42670    PWR_MGMT_0=0x1F, GYRO_CONFIG0=0x20, ACCEL_CONFIG0=0x21,
 *                        accel data 0x0B, gyro data 0x11
 *   0x47 -> ICM-42688-P  PWR_MGMT0=0x4E,  GYRO_CONFIG0=0x4F, ACCEL_CONFIG0=0x50,
 *                        accel data 0x1F, gyro data 0x25, BANK_SEL=0x76
 *
 * Sensor data registers are big-endian in both parts.
 * Polling mode (no INT required), IIO direct access.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/types.h>

#define ICM_REG_WHO_AM_I	0x75

struct icm_chip_info {
	const char *name;
	u8 whoami;			  // 芯片名
	u8 bank_sel;		  /* 0 = no bank switching required */
	u8 pwr_mgmt0;
	u8 gyro_config0;
	//u8 gyro_config1;
	u8 accel_config0;
	//u8 accel_config1;
	u8 accel_data_x;
	u8 gyro_data_x;
	u8 pwr_val;
	u8 gyro_cfg_val;
	u8 accel_cfg_val;
};

static const struct icm_chip_info icm42670_chip = {
	.name		= "ICM-42670",
	.whoami		= 0x67,			
	.bank_sel	= 0,
	.pwr_mgmt0	= 0x1F,
	.gyro_config0	= 0x20,
	.accel_config0	= 0x21,
	.accel_data_x	= 0x0B,
	.gyro_data_x	= 0x11,
	.pwr_val	= 0x0F,		/* gyro LN + accel LN */
	.gyro_cfg_val	= 0x09,		/* ODR 100Hz, +-2000dps */
	.accel_cfg_val	= 0x09,		/* ODR 100Hz, +-16g */
};

static const struct icm_chip_info icm42688_chip = {
	.name			= "ICM-42688-P",
	.whoami			= 0x47,
	.bank_sel		= 0x76,
	.pwr_mgmt0		= 0x4E,
	.gyro_config0	= 0x4F,
	.accel_config0	= 0x50,
	.accel_data_x	= 0x1F,
	.gyro_data_x	= 0x25,
	.pwr_val		= 0x0F,		/* gyro LN + accel LN */
	.gyro_cfg_val	= 0x06,		/* ODR 1kHz, +-2000dps */
	.accel_cfg_val	= 0x06,		/* ODR 1kHz, +-16g */
};


// static const struct icm_chip_info icm20602_chip = {
// 	.name			= "ICM-20602",
// 	.whoami			= 0x12,
// 	.bank_sel		= 0x76,
// 	.pwr_mgmt0		= 0x6B,
//	.gyro_config    = 0x1A,
//	.SMPLRT_DIV     = 0x19,
// 	.gyro_config0	= 0x1B,
// 	.accel_config0	= 0x1C,
// 	.accel_data_x	= 0x3B,
// 	.gyro_data_x	= 0x43,
// 	.pwr_val		= 0x01,		/* gyro LN + accel LN */
// 	.gyro_cfg_val	= 0x1B,		/* ODR 1kHz, +-2000dps */
// 	.accel_cfg_val	= 0x18,		/* ODR 1kHz, +-16g */
//	.odr_data       =     ,
// };

struct icm_state {
	struct i2c_client *client;
	struct regmap *regmap;
	struct mutex lock;
	const struct icm_chip_info *chip;
};

static const struct regmap_config icm_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7F,
};

static int icm_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask)
{
	struct icm_state *st = iio_priv(indio_dev);
	const struct icm_chip_info *chip = st->chip;
	int ret;
	u8 addr;
	__be16 raw;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_ACCEL)
			addr = chip->accel_data_x + chan->address * 2;
		else if (chan->type == IIO_ANGL_VEL)
			addr = chip->gyro_data_x + chan->address * 2;
		else
			return -EINVAL;

		mutex_lock(&st->lock);
		ret = regmap_bulk_read(st->regmap, addr, &raw, sizeof(raw));
		mutex_unlock(&st->lock);
		if (ret)
			return ret;

		*val = (s16)be16_to_cpu(raw);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ACCEL:
			/* 16g full scale: 16g / 2^15 = 0.00048828125 */
			*val = 0;
			*val2 = 488;
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_ANGL_VEL:
			/* 2000dps: 2000 / 2^15 = 0.061035 */
			*val = 0;
			*val2 = 61035;
			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

/* _idx is the axis index (0/1/2), the actual register is computed at read time */
#define ICM_CHAN(_type, _idx, _axis, _scan) {	\
	.type = _type,				\
	.modified = 1,				\
	.channel2 = IIO_MOD_##_axis,		\
	.address = _idx,			\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.scan_index = _scan,			\
	.scan_type = {				\
		.sign = 's',			\
		.realbits = 16,			\
		.storagebits = 16,		\
		.endianness = IIO_BE,		\
	},					\
}

static const struct iio_chan_spec icm_channels[] = {
	ICM_CHAN(IIO_ACCEL, 0, X, 0),
	ICM_CHAN(IIO_ACCEL, 1, Y, 1),
	ICM_CHAN(IIO_ACCEL, 2, Z, 2),
	ICM_CHAN(IIO_ANGL_VEL, 0, X, 3),
	ICM_CHAN(IIO_ANGL_VEL, 1, Y, 4),
	ICM_CHAN(IIO_ANGL_VEL, 2, Z, 5),
};

static const struct iio_info icm_info = {
	.read_raw = icm_read_raw,
};

static int icm_chip_init(struct icm_state *st)
{
	const struct icm_chip_info *chip = st->chip;
	int ret;

	/* ICM-42688-P needs bank 0 selected for its config/data registers */
	if (chip->bank_sel) {
		ret = regmap_write(st->regmap, chip->bank_sel, 0x00);
		if (ret)
			return ret;
	}

	/* Enable accel + gyro in low-noise mode */
	ret = regmap_write(st->regmap, chip->pwr_mgmt0, chip->pwr_val);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	ret = regmap_write(st->regmap, chip->gyro_config0, chip->gyro_cfg_val);
	if (ret)
		return ret;

	return regmap_write(st->regmap, chip->accel_config0,
			    chip->accel_cfg_val);
}

static int icm_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	static const struct icm_chip_info * const chips[] = {
		&icm42670_chip, &icm42688_chip,
	};
	struct iio_dev *indio_dev;
	struct icm_state *st;
	unsigned int whoami;
	int ret, i;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->client = client;
	mutex_init(&st->lock);

	st->regmap = devm_regmap_init_i2c(client, &icm_regmap_config);
	if (IS_ERR(st->regmap)) {
		dev_err(&client->dev, "regmap init failed\n");
		return PTR_ERR(st->regmap);
	}

	ret = regmap_read(st->regmap, ICM_REG_WHO_AM_I, &whoami);
	if (ret) {
		dev_err(&client->dev, "read WHO_AM_I failed: %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(chips); i++) {
		if (chips[i]->whoami == whoami) {
			st->chip = chips[i];
			break;
		}
	}
	if (!st->chip) {
		dev_err(&client->dev,
			"unsupported WHO_AM_I: 0x%02x (expected 0x67 ICM-42670 or 0x47 ICM-42688-P)\n",
			whoami);
		return -ENODEV;
	}

	dev_info(&client->dev, "%s detected (WHO_AM_I=0x%02X)\n",
		 st->chip->name, whoami);

	ret = icm_chip_init(st);
	if (ret)
		return ret;

	indio_dev->name = st->chip->name;
	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &icm_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = icm_channels;
	indio_dev->num_channels = ARRAY_SIZE(icm_channels);

	i2c_set_clientdata(client, indio_dev);

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret)
		return ret;

	dev_info(&client->dev, "%s registered as '%s'\n",
		 st->chip->name, indio_dev->name);
	return 0;
}

static const struct of_device_id icm_of_match[] = {
	{ .compatible = "invensense,icm42670" },
	{ .compatible = "invensense,icm42688" },
	{ .compatible = "invensense,icm20602" },
	{ }
};
MODULE_DEVICE_TABLE(of, icm_of_match);

static const struct i2c_device_id icm_i2c_id[] = {
	{ "icm42670", 0 },
	{ "icm42688", 0 },
	{ "icm20602", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, icm_i2c_id);

static struct i2c_driver icm_driver = {
	.driver = {
		.name = "icm42670",
		.of_match_table = of_match_ptr(icm_of_match),
	},
	.probe = icm_probe,
	.id_table = icm_i2c_id,
};
module_i2c_driver(icm_driver);

MODULE_DESCRIPTION("ICM-42670 / ICM-42688-P 6-axis IMU I2C driver (RV1126B main board)");
MODULE_LICENSE("GPL");
