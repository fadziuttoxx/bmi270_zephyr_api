/*
 * Copyright (c) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include "bmi270_api.h"
#include <../../../../drivers/sensor/bmi270/bmi270.h>

double sensor_get_double(const struct sensor_value *val);


sensor_trigger_handler_t trigger_handler(const struct device *dev,
			    const struct sensor_trigger *trig)
{
	struct sensor_value acc[3];
	static size_t cnt;
	int rc;

	++cnt;
	rc = sensor_sample_fetch(dev);
	if (rc != 0) {
		printf("sensor_sample_fetch error: %d\n", rc);
		return;
	}
	rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
	if (rc != 0) {
		printf("sensor_channel_get error: %d\n", rc);
		return;
	}

	printf("trigger fired %u, AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d;\n", cnt,
	       acc[0].val1, acc[0].val2, acc[1].val1, acc[1].val2, acc[2].val1, acc[2].val2);
}

void main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ONE(bosch_bmi270);
	struct sensor_value acc[3], gyr[3];
	int ret;
	uint8_t reg_value;
	uint8_t feat_page = 1;
	uint8_t pwr_mode = 0x00;
	uint16_t anymo_1_read;

	bmi270_init(dev);
	bmi270_config_anymotion_interrupt(dev,0x5,0xAA);
	bmi270_set_anymotion_trigger(dev, trigger_handler);
	bmi270_basic_config_acc(dev, 2, 800, 1);
	bmi270_basic_config_gyr(dev, 500, 800, 1);
	
	ret = bmi270_reg_read(dev, BMI270_REG_INT1_MAP_FEAT, &reg_value, 1);
	printf("ret int1_map_feat_read: %d\n", ret);
	printf("reg_value int1_map_feat: %x\n\n", reg_value);

	ret = bmi270_reg_write(dev, BMI270_REG_FEAT_PAGE, &feat_page, 1);
	printf("ret feat_page_write: %d\n", ret);
	ret = bmi270_reg_read(dev, BMI270_REG_FEAT_PAGE, &reg_value, 1);
	printf("ret feat_page_read: %d\n", ret);
	printf("reg_value feat_page: %d\n\n", reg_value);

	ret = bmi270_reg_read(dev, BMI270_REG_PWR_CONF, &reg_value, 1);
	printf("ret pwr_conf_read: %d\n", ret);
	printf("reg_value pwr_conf: %x\n\n", reg_value);

	ret = bmi270_reg_write(dev, BMI270_REG_PWR_CTRL, &pwr_mode, 1);
	printf("ret pwr_mode_write: %d\n", ret);

	ret = bmi270_reg_read(dev, BMI270_REG_PWR_CTRL, &reg_value, 1);
	printf("ret pwr_mode_read: %d\n", ret);
	printf("reg_value pwr_conf: %x\n\n", reg_value);

	ret = bmi270_reg_read(dev, 0x3C,&anymo_1_read, 2);
	printf("ret anymo_1_read: %d\n", ret);
	printf("anymo_1_read: 0x%x\n\n", anymo_1_read);
	// while (1) {
	// 	/* 10ms period, 100Hz Sampling frequency */
	// 	k_sleep(K_MSEC(500));

	// 	sensor_sample_fetch(dev);

	// 	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
	// 	sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyr);
	// 	printf("AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d;\n",
	// 	       acc[0].val1, acc[0].val2,
	// 	       acc[1].val1, acc[1].val2,
	// 	       acc[2].val1, acc[2].val2,
	// 	       gyr[0].val1, gyr[0].val2,
	// 	       gyr[1].val1, gyr[1].val2,
	// 	       gyr[2].val1, gyr[2].val2);
	// }
	while(1){
		k_sleep(K_MSEC(100));
	}
}
double sensor_get_double(const struct sensor_value *val)
{
	if(val->val1>=0) {
		return (double)val->val1 + (double)val->val2 / 1000000.0;
	}
	return (double)val->val1 - (double)val->val2 / 1000000.0;
}
