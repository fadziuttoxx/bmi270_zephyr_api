/*
 * Copyright (c) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <../../../../drivers/sensor/bmi270/bmi270.h>

struct bmi270_feature_reg bmi270_feature_reg_anymo_1 = {
	.page = 1,
	.addr = 0x3C,
};

struct bmi270_feature_reg bmi270_feature_reg_anymo_2 = {
	.page = 1,
	.addr = 0x3E,
};

int bmi270_feature_reg_write(const struct device *dev,
							 const struct bmi270_feature_reg *reg,
							 uint16_t value)
{
	int ret;
	uint8_t feat_page = reg->page;

	ret = bmi270_reg_write(dev, BMI270_REG_FEAT_PAGE, &feat_page, 1);
	if (ret < 0)
	{
		printf("bmi270_reg_write (0x%02x) failed: %d", BMI270_REG_FEAT_PAGE, ret);
		return ret;
	}

	printf("feature reg[0x%02x]@%d = 0x%04x \n", reg->addr, reg->page, value);

	ret = bmi270_reg_write(dev, reg->addr, (uint8_t *)&value, 2);
	if (ret < 0)
	{
		printf("bmi270_reg_write (0x%02x) failed: %d", reg->addr, ret);
		return ret;
	}

	return 0;
}

int bmi270_feature_reg_read(const struct device *dev,
							const struct bmi270_feature_reg *reg,
							uint16_t *value)
{
	int ret;
	uint8_t feat_page = reg->page;

	ret = bmi270_reg_write(dev, BMI270_REG_FEAT_PAGE, &feat_page, 1);
	if (ret < 0)
	{
		printf("bmi270_reg_write (0x%02x) failed: %d", BMI270_REG_FEAT_PAGE, ret);
		return ret;
	}

	printf("feature reg[0x%02x]@%d = 0x%04x \n", reg->addr, reg->page, value);

	ret = bmi270_reg_read(dev, reg->addr, (uint8_t *)value, 2);
	if (ret < 0)
	{
		printf("bmi270_reg_read (0x%02x) failed: %d", reg->addr, ret);
		return ret;
	}

	return 0;
}

void main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ONE(bosch_bmi270);
	struct sensor_value acc[3], gyr[3];
	struct sensor_value full_scale, sampling_freq, oversampling;
	int ret;

	if (!device_is_ready(dev))
	{
		printf("Device %s is not ready\n", dev->name);
		return;
	}

	printf("Device %p name is %s\n", dev, dev->name);
	
	k_sleep(K_MSEC(1));
	uint16_t reg_value;
	uint16_t no_motion_reg_value = 0xE005;
	uint8_t pwr_conf = 0x0;
	uint8_t feat_page = 0x2;
	uint8_t int1_map_feat = 0b01000000;
	uint8_t internal_status;
	uint8_t internal_error;
	uint8_t err;

	ret = bmi270_reg_read(dev, BMI270_REG_PWR_CONF, &reg_value, 1);
	printf("ret pwr_conf_read: %d\n", ret);
	printf("reg_value pwr_conf: %x\n\n", reg_value);

	ret = bmi270_reg_write(dev, BMI270_REG_PWR_CONF, &pwr_conf, 1);
	printf("ret pwr_conf_write: %d\n\n", ret);

	ret = bmi270_reg_read(dev, BMI270_REG_PWR_CONF, &reg_value, 1);
	printf("ret pwr_conf_read: %d\n", ret);
	printf("reg_value pwr_conf: %d\n\n", reg_value);

	ret = bmi270_reg_write(dev, BMI270_REG_FEAT_PAGE, &feat_page, 1);
	printf("ret feat_page_write: %d\n", ret);
	ret = bmi270_reg_read(dev, BMI270_REG_FEAT_PAGE, &reg_value, 1);
	printf("ret feat_page_read: %d\n", ret);
	printf("reg_value feat_page: %d\n\n", reg_value);

	ret = bmi270_reg_read(dev, 0x34, &reg_value, 2);
	printf("ret 0x34_read: %d\n", ret);
	printf("reg_value 0x34: %d\n\n", reg_value);

	ret = bmi270_reg_read(dev, BMI270_REG_FEAT_PAGE, &reg_value, 1);
	printf("ret feat_page_read: %d\n", ret);
	printf("reg_value feat_page: %d\n\n", reg_value);

	ret = bmi270_reg_read(dev, BMI270_REG_INTERNAL_STATUS, &internal_status, 1);
	printf("ret internal_status_read: %d\n", ret);
	printf("internal_status: %x\n\n", internal_status);

	ret = bmi270_reg_read(dev, BMI270_REG_INTERNAL_ERROR, &internal_error, 1);
	printf("ret internal_error_read: %d\n", ret);
	printf("internal_error: %x\n\n", internal_error);

	ret = bmi270_reg_read(dev, BMI270_REG_ERROR, &err, 1);
	printf("ret err_read: %d\n", ret);
	printf("err: %x\n\n", err);

	ret = bmi270_reg_write(dev, 0x30, (uint8_t *)&no_motion_reg_value, 2);
	printf("ret no_motion_reg_write: %d\n", ret);
	ret = bmi270_reg_read(dev, 0x30, &reg_value, 2);
	printf("ret no_motion_reg_read: %d\n", ret);
	printf("reg_value no_motion_reg: %d\n\n", reg_value);

	ret = bmi270_reg_read(dev, BMI270_REG_FEAT_PAGE, &reg_value, 1);
	printf("ret feat_page_read: %d\n", ret);
	printf("reg_value feat_page: %d\n\n", reg_value);



	ret = bmi270_reg_read(dev, BMI270_REG_INT1_MAP_FEAT, &reg_value, 1);
	printf("ret int1_map_feat_read: %d\n", ret);
	printf("reg_value int1_map_feat: %x\n\n", reg_value);

	ret = bmi270_reg_write(dev, BMI270_REG_INT1_MAP_FEAT, &int1_map_feat, 1);
	printf("ret int1_map_feat_write: %d\n\n", ret);

	ret = bmi270_reg_read(dev, BMI270_REG_INT1_MAP_FEAT, &reg_value, 1);
	printf("ret int1_map_feat_read: %d\n", ret);
	printf("reg_value int1_map_feat: %x\n\n", reg_value);


	ret = bmi270_feature_reg_read(dev, &bmi270_feature_reg_anymo_1, &reg_value);
	printf("reg_value: %d\n", reg_value);
	ret = bmi270_feature_reg_write(dev, &bmi270_feature_reg_anymo_1, 0xE005);
	ret = bmi270_feature_reg_read(dev, &bmi270_feature_reg_anymo_1, &reg_value);
	printf("reg_value: %d\n", reg_value);

	/* Setting scale in G, due to loss of precision if the SI unit m/s^2
	 * is used
	 */
	full_scale.val1 = 2; /* G */
	full_scale.val2 = 0;
	sampling_freq.val1 = 800; /* Hz. Performance mode */
	sampling_freq.val2 = 0;
	oversampling.val1 = 1; /* Normal mode */
	oversampling.val2 = 0;

	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE,
					&full_scale);
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING,
					&oversampling);
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change to 0.0Hz before changing
	 * other attributes
	 */
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ,
					SENSOR_ATTR_SAMPLING_FREQUENCY,
					&sampling_freq);

	/* Setting scale in degrees/s to match the sensor scale */
	full_scale.val1 = 500; /* dps */
	full_scale.val2 = 0;
	sampling_freq.val1 = 100; /* Hz. Performance mode */
	sampling_freq.val2 = 0;
	oversampling.val1 = 1; /* Normal mode */
	oversampling.val2 = 0;

	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE,
					&full_scale);
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING,
					&oversampling);
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change sampling frequency to
	 * 0.0Hz before changing other attributes
	 */
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ,
					SENSOR_ATTR_SAMPLING_FREQUENCY,
					&sampling_freq);

	while (1)
	{
		/* 10ms period, 100Hz Sampling frequency */
		k_sleep(K_MSEC(1000));

		sensor_sample_fetch(dev);

		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
		sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyr);

		printf("AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
			   "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d;\n",
			   acc[0].val1, acc[0].val2,
			   acc[1].val1, acc[1].val2,
			   acc[2].val1, acc[2].val2,
			   gyr[0].val1, gyr[0].val2,
			   gyr[1].val1, gyr[1].val2,
			   gyr[2].val1, gyr[2].val2);
	}
}
