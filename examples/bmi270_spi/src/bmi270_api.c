
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include "bmi270_api.h"
#include <../../../../drivers/sensor/bmi270/bmi270.h>
struct sensor_trigger trig;

void bmi270_init(const struct device *const bmi270_dev){
    if (!device_is_ready(bmi270_dev))
	{
		printf("Device %s is not ready\n", bmi270_dev->name);
		return;
	}

	printf("Device %p name is %s\n", bmi270_dev, bmi270_dev->name);
}

void bmi270_basic_config_acc(const struct device *const bmi270_dev, uint16_t full_scale, uint16_t sampling_freq, uint16_t oversampling){
    int rc;
	struct sensor_value Full_scale, Sampling_freq, Oversampling;
    Full_scale.val1 = full_scale;
    Full_scale.val2 = 0;
    Sampling_freq.val1 = sampling_freq;
    Sampling_freq.val2 = 0;
    Oversampling.val1 = oversampling;
    Oversampling.val2 = 0;
    rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE,
					&Full_scale);
	if (rc != 0) {
		printf("ACCEL full scale attr set failed: %d\n", rc);
		return;
	}
	rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING,
					&Oversampling);
	if (rc != 0) {
		printf("ACCEL oversampling attr set failed: %d\n", rc);
		return;
	}
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change to 0.0Hz before changing
	 * other attributes
	 */
	rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ,SENSOR_ATTR_SAMPLING_FREQUENCY,
					&Sampling_freq);
	if (rc != 0) {
		printf("ACCEL sampling frequency attr set failed: %d\n", rc);
		return;
	}
}
void bmi270_basic_config_gyr(const struct device *const bmi270_dev, uint16_t full_scale, uint16_t sampling_freq, uint16_t oversampling){
    int rc;
	struct sensor_value Full_scale, Sampling_freq, Oversampling;
    Full_scale.val1 = full_scale;
    Full_scale.val2 = 0;
    Sampling_freq.val1 = sampling_freq;
    Sampling_freq.val2 = 0;
    Oversampling.val1 = oversampling;
    Oversampling.val2 = 0;
    rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE,
					&Full_scale);
	if (rc != 0) {
		printf("GYRO full scale attr set failed: %d\n", rc);
		return;
	}
	rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING,
					&Oversampling);
	if (rc != 0) {
		printf("GYRO oversampling attr set failed: %d\n", rc);
		return;
	}
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change to 0.0Hz before changing
	 * other attributes
	 */
	rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_GYRO_XYZ,
					SENSOR_ATTR_SAMPLING_FREQUENCY,
					&Sampling_freq);
	if (rc != 0) {
		printf("GYRO sampling frequency attr set failed: %d\n", rc);
		return;
	}
}

void bmi270_config_anymotion_interrupt(const struct device *const bmi270_dev, uint16_t threshold, uint16_t duration){
	int rc;
	struct sensor_value Threshold, Duration;
	Threshold.val1 = threshold;
	Threshold.val2 = 0;
	Duration.val1 = duration;
	Duration.val2 = 0;
	rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ,
					SENSOR_ATTR_SLOPE_TH,
					&Threshold);
	if (rc != 0) {
		printf("ACCEL slope threshold attr set failed: %d\n", rc);
		return;
	}
	rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ,
					SENSOR_ATTR_SLOPE_DUR,
					&Duration);
	if (rc != 0) {
		printf("ACCEL slope duration attr set failed: %d\n", rc);
		return;
	}
}

void bmi270_set_anymotion_trigger(const struct device *const bmi270_dev, sensor_trigger_handler_t trigger_handler){
	int rc;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	trig.type = SENSOR_TRIG_MOTION;
	rc = sensor_trigger_set(bmi270_dev, &trig, trigger_handler);
	if (rc != 0) {
		printf("Trigger set failed: %d\n", rc);
		return;
	}
	printk("Trigger set got %d\n", rc);
}