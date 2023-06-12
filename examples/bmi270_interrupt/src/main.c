/*
 * Kazimierz Roman
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include "bmi270_api.h"
#include <../../../../drivers/sensor/bmi270/bmi270.h>

struct bmi270_feature_reg bmi270_feature_reg_anymo_1 = {
	.page = 1,
	.addr = 0x3C,
};

struct bmi270_feature_reg bmi270_feature_reg_anymo_2 = {
	.page = 1,
	.addr = 0x3E,
};
struct bmi270_feature_reg bmi270_feature_reg_nomo_1 = {
	.page = 2,
	.addr = 0x30,
};
struct bmi270_feature_reg bmi270_feature_reg_nomo_2 = {
	.page = 2,
	.addr = 0x32,
};

void anymo_trigger_handler(const struct device *dev,
						   const struct sensor_trigger *trig)
{
	struct sensor_value acc[3];
	static size_t cnt;
	int rc;
	++cnt;
	rc = sensor_sample_fetch(dev);
	if (rc != 0)
	{
		printf("sensor_sample_fetch error: %d\n", rc);
	}
	rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
	if (rc != 0)
	{
		printf("sensor_channel_get error: %d\n", rc);
	}
	printf("anymo trigger fired %u, AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d;\n", cnt,
		   acc[0].val1, acc[0].val2, acc[1].val1, acc[1].val2, acc[2].val1, acc[2].val2);
}

void nomo_trigger_handler(const struct device *dev,
						  const struct sensor_trigger *trig)
{
	struct sensor_value acc[3];
	static size_t cnt;
	int rc;
	++cnt;
	rc = sensor_sample_fetch(dev);
	if (rc != 0)
	{
		printf("sensor_sample_fetch error: %d\n", rc);
	}
	rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
	if (rc != 0)
	{
		printf("sensor_channel_get error: %d\n", rc);
	}
	printf("nomo trigger fired %u, AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d;\n", cnt,
		   acc[0].val1, acc[0].val2, acc[1].val1, acc[1].val2, acc[2].val1, acc[2].val2);
}

void main(void)
{
	struct sensor_trigger anymo_trig;
	// struct sensor_trigger nomo_trig;

	const struct device *const dev = DEVICE_DT_GET_ONE(bosch_bmi270);
	// struct sensor_value acc[3], gyr[3];

	int ret;
	uint16_t reg_feature_read;
	uint8_t reg_read;

	bmi270_init(dev);
	
	/* for testing/exploring purposes only one interrupt triger is recommended*/
	bmi270_config_anymotion_interrupt(dev, 0x5, 0xAA);
	// bmi270_config_nomotion_interrupt(dev, 0x5, 0x90);
	bmi270_set_anymotion_trigger(dev, anymo_trigger_handler, &anymo_trig);
	// bmi270_set_nomotion_trigger(dev, nomo_trigger_handler, &nomo_trig);

	bmi270_basic_config_acc(dev, 2, 100, 1);
	bmi270_basic_config_gyr(dev, 500, 100, 1);
	bmi270_config_adv_power_save_mode(dev, 0);

	// feature config check
	ret = bmi270_feature_reg_read(dev, &bmi270_feature_reg_anymo_1, &reg_feature_read);
	printf("ret anymo_1_read: %d\n", ret);
	printf("anymo_1_read: 0x%x\n\n", reg_feature_read);

	ret = bmi270_feature_reg_read(dev, &bmi270_feature_reg_anymo_2, &reg_feature_read);
	printf("ret anymo_2_read: %d\n", ret);
	printf("anymo_2_read: 0x%x\n\n", reg_feature_read);

	ret = bmi270_feature_reg_read(dev, &bmi270_feature_reg_nomo_1, &reg_feature_read);
	printf("ret nomo_1_read: %d\n", ret);
	printf("nomo_1_read: 0x%x\n\n", reg_feature_read);

	ret = bmi270_feature_reg_read(dev, &bmi270_feature_reg_nomo_2, &reg_feature_read);
	printf("ret nomo_2_read: %d\n", ret);
	printf("nomo_2_read: 0x%x\n\n", reg_feature_read);

	// int1_map_feat check
	ret = bmi270_reg_read(dev, BMI270_REG_INT1_MAP_FEAT, &reg_read, sizeof(reg_read));
	printf("ret int1_map_feat: %d\n", ret);
	printf("int1_map_feat: 0x%x\n\n", reg_read);

	// acc conf check
	ret = bmi270_reg_read(dev, BMI270_REG_ACC_CONF, &reg_read, sizeof(reg_read));
	printf("ret acc_conf: %d\n", ret);
	printf("acc_conf: 0x%x\n\n", reg_read);

	//int_io_ctrl check
	bmi270_reg_read(dev, BMI270_REG_INT1_IO_CTRL, &reg_read, 1);
	printf("INT1_IO_CTRL: %d\n\n", reg_read);

	bmi270_reg_read(dev, BMI270_REG_INT2_IO_CTRL, &reg_read, 1);
	printf("INT2_IO_CTRL: %d\n", reg_read);

	// fifo_config check
	bmi270_reg_read(dev, BMI270_REG_FIFO_CONFIG_0, &reg_read, 1);
	printf("FIFO_CONFIG_0: 0x%x\n", reg_read);

	bmi270_reg_read(dev, BMI270_REG_FIFO_CONFIG_1, &reg_read, 1);
	printf("FIFO_CONFIG_1: 0x%x\n", reg_read);

	while (1)
	{
		k_sleep(K_MSEC(500));
		// sensor_sample_fetch(dev);
		// sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
		// sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyr);
		// printf("AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d;\n",
		// 	   acc[0].val1, acc[0].val2,
		// 	   acc[1].val1, acc[1].val2,
		// 	   acc[2].val1, acc[2].val2,
		// 	   gyr[0].val1, gyr[0].val2,
		// 	   gyr[1].val1, gyr[1].val2,
		// 	   gyr[2].val1, gyr[2].val2);
	}
}