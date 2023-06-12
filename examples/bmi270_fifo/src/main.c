/*
 * Copyright © Kazimierz Roman
 *
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <../../../../drivers/sensor/bmi270/bmi270.h>
#include "bmi2_defs.h"
#include "bmi270_api.h"

/******************************************************************************/
/*!                  Macros                                                   */

#define BMI270_SPI_DUMMY_BYTE 1
/*! Buffer size allocated to store raw FIFO data. */
#define BMI2_FIFO_RAW_DATA_BUFFER_SIZE UINT16_C(2048)

/*! Length of data to be read from FIFO. */
#define BMI2_FIFO_RAW_DATA_USER_LENGTH UINT16_C(2048)

/*! Number of accel frames to be extracted from FIFO. */

/*! Calculation for frame count: Total frame count = Fifo buffer size(2048)/ Total frames(6 Accel, 6 Gyro and 1 header,
 * totaling to 13) which equals to 157.
 *
 * Extra frames to parse sensortime data
 */
#define BMI2_FIFO_ACCEL_FRAME_COUNT UINT8_C(185)

/*! Number of gyro frames to be extracted from FIFO. */
#define BMI2_FIFO_GYRO_FRAME_COUNT UINT8_C(185)

/*! Macro to read sensortime byte in FIFO. */
#define SENSORTIME_OVERHEAD_BYTE UINT8_C(220)

/******************************************************************************/
/*!                        Global Variables                                   */

volatile uint8_t interrupt_status = 0;

/* To read sensortime, extra 3 bytes are added to fifo buffer. */
uint16_t fifo_buffer_size = BMI2_FIFO_RAW_DATA_BUFFER_SIZE + SENSORTIME_OVERHEAD_BYTE;

/* Number of bytes of FIFO data
 * NOTE : Dummy byte (for SPI Interface) required for FIFO data read must be given as part of array size
 * Array size same as fifo_buffer_size
 */
uint8_t fifo_data[BMI2_FIFO_RAW_DATA_BUFFER_SIZE + SENSORTIME_OVERHEAD_BYTE];

/* Array of accelerometer frames -> Total bytes =
 * 157 * (6 axes + 1 header bytes) = 1099 bytes */
struct bmi2_sens_axes_data fifo_accel_data[BMI2_FIFO_ACCEL_FRAME_COUNT] = {{0}};

/* Array of gyro frames -> Total bytes =
 * 157 * (6 axes + 1 header bytes) = 1099 bytes */
struct bmi2_sens_axes_data fifo_gyro_data[BMI2_FIFO_GYRO_FRAME_COUNT] = {{0}};

/******************************************************************************/
/*!                GPIO setup												 */
#define SW0_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
															   {0});
static struct gpio_callback button0_cb_data;

/******************************************************************************/
/*!                Callbacks                                     */

void button0_pressed(const struct device *dev, struct gpio_callback *cb,
					 uint32_t pins)
{
	interrupt_status = 1;
	printf("Button 0 pressed at %" PRIu32 "\n", k_cycle_get_32());
}

void main(void)
{
	uint64_t sens_en_stat = 0;
	// anymotion
	//  sens_en_stat |= BMI2_ANY_MOT_SEL;
	// nomotion
	// sens_en_stat |= BMI2_NO_MOT_SEL;

	gpio_init_manual(&button0);
	gpio_callback_config_manual(&button0_cb_data, button0_pressed, &button0);

	const struct device *const dev = DEVICE_DT_GET_ONE(bosch_bmi270);
	struct sensor_value acc[3], gyr[3];

	/* Status of api are returned to this variable. */
	int8_t rslt;

	uint16_t index = 0;
	uint16_t fifo_length = 0;

	/* Variable to get fifo full interrupt status. */

	uint16_t accel_frame_length = BMI2_FIFO_ACCEL_FRAME_COUNT;

	uint16_t gyro_frame_length = BMI2_FIFO_GYRO_FRAME_COUNT;

	int8_t try = 1;

	/* Initialize FIFO frame structure. */
	struct bmi2_fifo_frame fifoframe = {0};

	/* Update FIFO structure. */
	/* Mapping the buffer to store the fifo data. */
	fifoframe.data = fifo_data;

	/* Length of FIFO frame. */
	/* To read sensortime, extra 3 bytes are added to fifo user length. */
	fifoframe.length = BMI2_FIFO_RAW_DATA_USER_LENGTH + SENSORTIME_OVERHEAD_BYTE;

	uint8_t reg_read;

	bmi270_init(dev);
	
	// testing purpose only
	bmi270_reg_read(dev, BMI270_REG_INT1_IO_CTRL, &reg_read, 1);
	printf("INT2_IO_CTRL: %d\n", reg_read);

	bmi270_config_ffull_interrupt(dev, 1);
	bmi270_config_adv_power_save_mode(dev, 0);
	bmi270_config_fifo(dev, 0, 0, 1, 0);
	bmi270_basic_config_acc(dev, 2, 100, 1);
	bmi270_basic_config_gyr(dev, 500, 100, 1);

	struct bmi270_data *data = dev->data;
	while (try <= 10)
	{
		if (interrupt_status)
		{
			printf("\nIteration : %d\n", try);

			accel_frame_length = BMI2_FIFO_ACCEL_FRAME_COUNT;

			gyro_frame_length = BMI2_FIFO_GYRO_FRAME_COUNT;

			rslt = bmi270_get_fifo_length(&fifo_length, dev);
			printf("rslt bmi270_get_fifo_length: %d\n", rslt);

			/* Updating FIFO length to be read based on available length and dummy byte updation */
			fifoframe.length = fifo_length + SENSORTIME_OVERHEAD_BYTE + BMI270_SPI_DUMMY_BYTE;

			printf("\nFIFO data bytes available : %d \n", fifo_length);
			printf("\nFIFO data bytes requested : %d \n", fifoframe.length);

			rslt = bmi270_read_fifo_data(&fifoframe, dev, sens_en_stat, 1, 0);
			printf("rslt bmi270_get_fifo_data: %d\n", rslt);
			if (rslt == BMI2_OK)
			{
				printf("\nFIFO accel frames requested : %d \n", accel_frame_length);

				/* Parse the FIFO data to extract accelerometer data from the FIFO buffer. */
				(void)bmi270_extract_accel(fifo_accel_data, &accel_frame_length, &fifoframe, sens_en_stat);
				printf("\nFIFO accel frames extracted : %d \n", accel_frame_length);

				printf("\nExtracted accel frames\n");
				printf("ACCEL_DATA, X, Y, Z\n");

				/* Print the parsed accelerometer data from the FIFO buffer. */
				for (index = 0; index < accel_frame_length; index++)
				{
					channel_accel_convert_custom(&acc[0], fifo_accel_data[index].x, data->acc_range);
					channel_accel_convert_custom(&acc[1], fifo_accel_data[index].y, data->acc_range);
					channel_accel_convert_custom(&acc[2], fifo_accel_data[index].z, data->acc_range);
					printf("AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; \n",
						   acc[0].val1, acc[0].val2,
						   acc[1].val1, acc[1].val2,
						   acc[2].val1, acc[2].val2);
				}
				/* Print control frames like sensor time and skipped frame count. */
				printf("\nSkipped frame count = %d\n", fifoframe.skipped_frame_count);

				printf("Sensor time(in seconds) = %.4lf  s\r\n", (fifoframe.sensor_time * BMI2_SENSORTIME_RESOLUTION));

				try++;
				interrupt_status = 0;
			}
		}
	}
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
