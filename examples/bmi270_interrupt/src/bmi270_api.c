/*
 * Copyright © Kazimierz Roman
 *
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include "bmi270_api.h"
#include <../../../../drivers/sensor/bmi270/bmi270.h>

static int8_t get_aux_interface_config_custom(struct bmi2_aux_config *config, const struct device *dev);
static int8_t reset_fifo_frame_structure_custom(struct bmi2_fifo_frame *fifo, const struct device *dev, uint64_t sens_en_stat);
static int8_t extract_accel_header_mode_custom(struct bmi2_sens_axes_data *acc, uint16_t *accel_length, struct bmi2_fifo_frame *fifo, uint64_t sens_en_stat);
static int8_t extract_accel_headerless_mode_custom(struct bmi2_sens_axes_data *acc, uint16_t *accel_length, struct bmi2_fifo_frame *fifo, uint64_t sens_en_stat);
static void parse_if_virtual_header_custom(uint8_t *frame_header, uint16_t *data_index, const struct bmi2_fifo_frame *fifo);
static int8_t move_next_frame_custom(uint16_t *data_index, uint8_t current_frame_length, const struct bmi2_fifo_frame *fifo);
static int8_t unpack_accel_header_frame_custom(struct bmi2_sens_axes_data *acc, uint16_t *idx,uint16_t *acc_idx,uint8_t frame,const struct bmi2_fifo_frame *fifo,uint64_t sens_en_stat);
static int8_t unpack_accel_headerless_frame_custom(struct bmi2_sens_axes_data *acc,uint16_t *idx,uint16_t *acc_idx,uint8_t frame,const struct bmi2_fifo_frame *fifo,uint64_t sens_en_stat);
static void unpack_accel_data_custom(struct bmi2_sens_axes_data *acc,uint16_t data_start_index,const struct bmi2_fifo_frame *fifo);
static void unpack_virt_sensor_time_custom(struct bmi2_sens_axes_data *sens, uint16_t *idx, const struct bmi2_fifo_frame *fifo);
static int8_t parse_fifo_accel_len_custom(uint16_t *start_idx,uint16_t *len,uint8_t *skip_length,const uint16_t *acc_count,const struct bmi2_fifo_frame *fifo);
static int8_t check_dummy_frame_custom(uint8_t dummy_frame_header,uint16_t *data_index,uint8_t skip_length, const struct bmi2_fifo_frame *fifo);
static int8_t check_empty_fifo_custom(uint16_t *data_index, const struct bmi2_fifo_frame *fifo);
static int8_t unpack_sensortime_frame_custom(uint16_t *data_index, struct bmi2_fifo_frame *fifo);
static int8_t unpack_skipped_frame_custom(uint16_t *data_index, struct bmi2_fifo_frame *fifo);


void bmi270_init(const struct device *const bmi270_dev)
{
	if (!device_is_ready(bmi270_dev))
	{
		printf("Device %s is not ready\n", bmi270_dev->name);
		return;
	}
	printf("Device %p name is %s\n", bmi270_dev, bmi270_dev->name);
}

void bmi270_basic_config_acc(const struct device *const bmi270_dev, uint16_t full_scale, uint16_t sampling_freq, uint16_t oversampling)
{
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
	if (rc != 0)
	{
		printf("ACCEL full scale attr set failed: %d\n", rc);
		return;
	}
	rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING,
						 &Oversampling);
	if (rc != 0)
	{
		printf("ACCEL oversampling attr set failed: %d\n", rc);
		return;
	}
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change to 0.0Hz before changing
	 * other attributes
	 */
	rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,
						 &Sampling_freq);
	if (rc != 0)
	{
		printf("ACCEL sampling frequency attr set failed: %d\n", rc);
		return;
	}
}
void bmi270_basic_config_gyr(const struct device *const bmi270_dev, uint16_t full_scale, uint16_t sampling_freq, uint16_t oversampling)
{
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
	if (rc != 0)
	{
		printf("GYRO full scale attr set failed: %d\n", rc);
		return;
	}
	rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING,
						 &Oversampling);
	if (rc != 0)
	{
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
	if (rc != 0)
	{
		printf("GYRO sampling frequency attr set failed: %d\n", rc);
		return;
	}
}

void bmi270_config_anymotion_interrupt(const struct device *const bmi270_dev, uint16_t threshold, uint16_t duration)
{
	int rc;
	struct sensor_value Threshold, Duration;
	Threshold.val1 = threshold;
	Threshold.val2 = 0;
	Duration.val1 = duration;
	Duration.val2 = 0;
	rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ,
						 SENSOR_ATTR_SLOPE_TH,
						 &Threshold);
	if (rc != 0)
	{
		printf("ACCEL slope threshold attr set failed: %d\n", rc);
		return;
	}
	rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ,
						 SENSOR_ATTR_SLOPE_DUR,
						 &Duration);
	if (rc != 0)
	{
		printf("ACCEL slope duration attr set failed: %d\n", rc);
		return;
	}
}

void bmi270_config_nomotion_interrupt(const struct device *const bmi270_dev, uint16_t threshold, uint16_t duration)
{
	int rc;
	struct sensor_value Threshold, Duration;
	Threshold.val1 = threshold;
	Threshold.val2 = 0;
	Duration.val1 = duration;
	Duration.val2 = 0;
	rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ,
						 SENSOR_ATTR_SLOPE_TH_nomo,
						 &Threshold);
	if (rc != 0)
	{
		printf("ACCEL slope threshold attr set failed: %d\n", rc);
		return;
	}
	rc = sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ,
						 SENSOR_ATTR_SLOPE_DUR_nomo,
						 &Duration);
	if (rc != 0)
	{
		printf("ACCEL slope duration attr set failed: %d\n", rc);
		return;
	}
}

void bmi270_set_anymotion_trigger(const struct device *const bmi270_dev, sensor_trigger_handler_t trigger_handler, struct sensor_trigger *trig)
{
	int rc;
	trig->chan = SENSOR_CHAN_ACCEL_XYZ;
	trig->type = SENSOR_TRIG_MOTION;
	rc = sensor_trigger_set(bmi270_dev, trig, trigger_handler);
	if (rc != 0)
	{
		printf("Trigger set failed: %d\n", rc);
		return;
	}
	printk("Trigger set got %d\n", rc);
}

void bmi270_set_nomotion_trigger(const struct device *const bmi270_dev, sensor_trigger_handler_t trigger_handler, struct sensor_trigger *trig)
{
	int rc;
	trig->chan = SENSOR_CHAN_ACCEL_XYZ;
	trig->type = SENSOR_TRIG_STATIONARY;
	rc = sensor_trigger_set(bmi270_dev, trig, trigger_handler);
	if (rc != 0)
	{
		printf("Trigger set failed: %d\n", rc);
		return;
	}
	printk("Trigger set got %d\n", rc);
}

void gpio_init_manual(const struct gpio_dt_spec *spec)
{
	int ret;
	if (!device_is_ready(spec->port))
	{
		printf("Error: gpio device %s is not ready\n",
			   spec->port->name);
		return;
	}

	ret = gpio_pin_configure_dt(spec, GPIO_INPUT);
	if (ret != 0)
	{
		printf("Error %d: failed to configure %s pin %d\n",
			   ret, spec->port->name, spec->pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(spec,
										  GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0)
	{
		printf("Error %d: failed to configure interrupt on %s pin %d\n",
			   ret, spec->port->name, spec->pin);
		return;
	}
}

void gpio_callback_config_manual(struct gpio_callback *callback_data, gpio_callback_handler_t handler, const struct gpio_dt_spec *spec)
{
	gpio_init_callback(callback_data, handler, BIT(spec->pin));
	gpio_add_callback(spec->port, callback_data);
	printf("Set up gpio_cb at %s pin %d\n", spec->port->name, spec->pin);
}

int8_t bmi270_get_fifo_length(uint16_t *fifo_length, const struct device *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Variable to define byte index */
	uint8_t index = 0;

	/* Array to store FIFO data length */
	uint8_t data[BMI2_FIFO_DATA_LENGTH] = {0};

	if ((fifo_length != NULL))
	{
		/* Read FIFO length */
		rslt = bmi270_reg_read(dev, BMI2_FIFO_LENGTH_0_ADDR, data, BMI2_FIFO_DATA_LENGTH);
		if (rslt == BMI2_OK)
		{
			/* Get the MSB byte index */
			index = BMI2_FIFO_LENGTH_MSB_BYTE;

			/* Get the MSB byte of FIFO length */
			data[index] = BMI2_GET_BIT_POS0(data[index], BMI2_FIFO_BYTE_COUNTER_MSB);

			/* Get total FIFO length */
			(*fifo_length) = ((data[index] << 8) | data[index - 1]);
		}
	}
	else
	{
		rslt = BMI2_E_NULL_PTR;
	}
	return rslt;
}

int8_t bmi270_read_fifo_data(struct bmi2_fifo_frame *fifo, const struct device *dev, uint64_t sens_en_stat, bool spi, bool advanced_power_save)
{
	/* Variable to define error */
	int8_t rslt;

	/* Array to store FIFO configuration data */
	uint8_t config_data[2] = {0};

	/* Variable to define FIFO address */
	uint8_t addr = BMI2_FIFO_DATA_ADDR;

	if (fifo != NULL)
	{
		/* Clear the FIFO data structure */
		rslt = reset_fifo_frame_structure_custom(fifo, dev, sens_en_stat);

		if (rslt == BMI2_OK)
		{
			/* Configuring reg_addr for SPI Interface */
			if (spi)
			{
				addr = (addr | BMI2_SPI_RD_MASK);
			}

			/* Read FIFO data */
			rslt = bmi270_reg_read(dev, addr, fifo->data, (uint32_t)fifo->length);
			/* Provide delay based on advanced power saving mode status */
			if (advanced_power_save)
			{
				k_usleep(450);
			}
			else
			{
				k_usleep(2);
			}

			/* If interface read fails, update rslt variable with communication failure */
			if (rslt != BMI2_INTF_RET_SUCCESS)
			{
				rslt = BMI2_E_COM_FAIL;
			}

			if (rslt == BMI2_OK)
			{
				/* Get the set FIFO frame configurations */
				rslt = bmi270_reg_read(dev, BMI2_FIFO_CONFIG_0_ADDR, config_data, 2);
				if (rslt == BMI2_OK)
				{
					/* Get FIFO header status */
					fifo->header_enable = (uint8_t)((config_data[1]) & (BMI2_FIFO_HEADER_EN >> 8));

					/* Get sensor enable status, of which the data is to be read */
					fifo->data_enable =
						(uint16_t)(((config_data[0]) | ((uint16_t)config_data[1] << 8)) & BMI2_FIFO_ALL_EN);
				}
			}
			else
			{
				rslt = BMI2_E_COM_FAIL;
			}
		}
	}
	else
	{
		rslt = BMI2_E_NULL_PTR;
	}

	return rslt;
}

static int8_t get_aux_interface_config_custom(struct bmi2_aux_config *config, const struct device *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Variable to store data */
	uint8_t reg_data[2] = {0};

	rslt = bmi270_reg_read(dev, BMI2_AUX_DEV_ID_ADDR, reg_data, 2);
	if (rslt == BMI2_OK)
	{
		/* Get I2C address for auxiliary sensor */
		config->i2c_device_addr = BMI2_GET_BITS(reg_data[0], BMI2_AUX_SET_I2C_ADDR);

		/* Get the AUX IF to either manual or auto mode */
		config->manual_en = BMI2_GET_BITS(reg_data[1], BMI2_AUX_MAN_MODE_EN);

		/* Enables FCU write command on AUX IF for auxiliary sensors that need a trigger */
		config->fcu_write_en = BMI2_GET_BITS(reg_data[1], BMI2_AUX_FCU_WR_EN);

		/* Get the burst read length for manual mode */
		config->man_rd_burst = BMI2_GET_BITS(reg_data[1], BMI2_AUX_MAN_READ_BURST);

		/* Get the burst read length for data mode */
		config->aux_rd_burst = BMI2_GET_BIT_POS0(reg_data[1], BMI2_AUX_READ_BURST);

		/* If data mode, get the read address of the auxiliary sensor from where data is to be read */
		if (!config->manual_en)
		{
			rslt = bmi270_reg_read(dev, BMI2_AUX_RD_ADDR, &config->read_addr, 1);
		}
	}
	return rslt;
}

static int8_t reset_fifo_frame_structure_custom(struct bmi2_fifo_frame *fifo, const struct device *dev, uint64_t sens_en_stat)
{
	int8_t rslt;

	struct bmi2_aux_config config;

	/* Reset FIFO data structure */
	fifo->acc_byte_start_idx = 0;
	fifo->aux_byte_start_idx = 0;
	fifo->gyr_byte_start_idx = 0;
	fifo->sensor_time = 0;
	fifo->skipped_frame_count = 0;
	fifo->act_recog_byte_start_idx = 0;

	/* Get default configurations for the type of feature selected. */
	rslt = get_aux_interface_config_custom(&config, dev);

	if (rslt == BMI2_OK)
	{
		/* If S4S is enabled */
		if ((sens_en_stat & BMI2_EXT_SENS_SEL) == BMI2_EXT_SENS_SEL)
		{
			fifo->acc_frm_len = BMI2_FIFO_VIRT_ACC_LENGTH;
			fifo->gyr_frm_len = BMI2_FIFO_VIRT_GYR_LENGTH;
			fifo->aux_frm_len = BMI2_FIFO_VIRT_AUX_LENGTH;
			fifo->acc_gyr_frm_len = BMI2_FIFO_VIRT_ACC_GYR_LENGTH;
			fifo->acc_aux_frm_len = BMI2_FIFO_VIRT_ACC_AUX_LENGTH;
			fifo->aux_gyr_frm_len = BMI2_FIFO_VIRT_GYR_AUX_LENGTH;
			fifo->all_frm_len = BMI2_FIFO_VIRT_ALL_LENGTH;

			/* If S4S is not enabled */
		}
		else
		{
			if (config.aux_rd_burst == BMI2_AUX_READ_LEN_0)
			{
				fifo->aux_frm_len = BMI2_AUX_RD_BURST_FRM_LEN_1;
			}

			if (config.aux_rd_burst == BMI2_AUX_READ_LEN_1)
			{
				fifo->aux_frm_len = BMI2_AUX_RD_BURST_FRM_LEN_2;
			}

			if (config.aux_rd_burst == BMI2_AUX_READ_LEN_2)
			{
				fifo->aux_frm_len = BMI2_AUX_RD_BURST_FRM_LEN_6;
			}

			if (config.aux_rd_burst == BMI2_AUX_READ_LEN_3)
			{
				fifo->aux_frm_len = BMI2_AUX_RD_BURST_FRM_LEN_8;
			}

			fifo->acc_frm_len = BMI2_FIFO_ACC_LENGTH;
			fifo->gyr_frm_len = BMI2_FIFO_GYR_LENGTH;
			fifo->acc_gyr_frm_len = BMI2_FIFO_ACC_GYR_LENGTH;
			fifo->acc_aux_frm_len = (BMI2_FIFO_ACC_LENGTH + fifo->aux_frm_len);
			fifo->aux_gyr_frm_len = (BMI2_FIFO_GYR_LENGTH + fifo->aux_frm_len);
			fifo->all_frm_len = (BMI2_FIFO_ACC_LENGTH + BMI2_FIFO_GYR_LENGTH + fifo->aux_frm_len);
		}
	}

	return rslt;
}

int8_t bmi270_extract_accel(struct bmi2_sens_axes_data *accel_data, uint16_t *accel_length, struct bmi2_fifo_frame *fifo, uint64_t sens_en_stat)
{
	/* Variable to define error */
	int8_t rslt;

	if ((accel_data != NULL) && (accel_length != NULL) && (fifo != NULL))
	{
		/* Check if this is the first iteration of data unpacking
		 * if yes, then consider dummy byte on SPI
		 */
		if (fifo->acc_byte_start_idx == 0)
		{
			/* Dummy byte included */
			fifo->acc_byte_start_idx = 1;
		}

		/* Parsing the FIFO data in header-less mode */
		if (fifo->header_enable == 0)
		{
			/* Parsing the FIFO data in headerless mode */
			rslt = extract_accel_headerless_mode_custom(accel_data, accel_length, fifo, sens_en_stat);
		}
		else
		{
			/* Parsing the FIFO data in header mode */
			rslt = extract_accel_header_mode_custom(accel_data, accel_length, fifo, sens_en_stat);
		}
	}
	else
	{
		rslt = BMI2_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API parses and extracts the gyroscope frames from FIFO data
 * read by the "bmi2_read_fifo_data" API and stores it in the "gyro_data"
 * structure instance.
 */
int8_t bmi270_extract_gyro(struct bmi2_sens_axes_data *gyro_data, uint16_t *gyro_length, struct bmi2_fifo_frame *fifo, const struct bmi2_dev *dev)
{
	/* Variable to define error */
	uint8_t rslt;
	if ((gyro_data != NULL) && (gyro_length != NULL) && (fifo != NULL))
	{
		/* Check if this is the first iteration of data unpacking
		 * if yes, then consider dummy byte on SPI
		 */
		if (fifo->gyr_byte_start_idx == 0)
		{
			/* Dummy byte included */
			fifo->gyr_byte_start_idx = dev->dummy_byte;
		}

		/* Parsing the FIFO data in header-less mode */
		if (fifo->header_enable == 0)
		{
			/* Parsing the FIFO data in headerless mode */
			// rslt = extract_gyro_headerless_mode(gyro_data, gyro_length, fifo, dev);
		}
		else
		{
			/* Parsing the FIFO data in header mode */
			// rslt = extract_gyro_header_mode(gyro_data, gyro_length, fifo, dev);
		}
	}
	else
	{
		rslt = BMI2_E_NULL_PTR;
	}

	return rslt;
}

static int8_t extract_accel_header_mode_custom(struct bmi2_sens_axes_data *acc,
											   uint16_t *accel_length,
											   struct bmi2_fifo_frame *fifo,
											   uint64_t sens_en_stat)
{
	/* Variable to define error */
	int8_t rslt = BMI2_OK;

	/* Variable to define header frame */
	uint8_t frame_header = 0;

	/* Variable to index the data bytes */
	uint16_t data_index;

	/* Variable to index accelerometer frames */
	uint16_t accel_index = 0;

	/* Variable to indicate accelerometer frames read */
	uint16_t frame_to_read = *accel_length;

	for (data_index = fifo->acc_byte_start_idx; data_index < fifo->length;)
	{
		/* Get frame header byte */
		printf("fifo->data[data_index] = 0x%x\n", fifo->data[data_index] & BMI2_FIFO_TAG_INTR_MASK);

		frame_header = fifo->data[data_index] & BMI2_FIFO_TAG_INTR_MASK;

		/* Parse virtual header if S4S is enabled */
		parse_if_virtual_header_custom(&frame_header, &data_index, fifo);

		/* Index shifted to next byte where data starts */
		data_index++;
		switch (frame_header)
		{
		/* If header defines accelerometer frame */
		case BMI2_FIFO_HEADER_ACC_FRM:
		case BMI2_FIFO_HEADER_AUX_ACC_FRM:
		case BMI2_FIFO_HEADER_GYR_ACC_FRM:
		case BMI2_FIFO_HEADER_ALL_FRM:

			/* Unpack from normal frames */
			rslt = unpack_accel_header_frame_custom(acc, &data_index, &accel_index, frame_header, fifo, sens_en_stat);
			break;

		/* If header defines only gyroscope frame */
		case BMI2_FIFO_HEADER_GYR_FRM:
			rslt = move_next_frame_custom(&data_index, fifo->gyr_frm_len, fifo);
			break;

		/* If header defines only auxiliary frame */
		case BMI2_FIFO_HEADER_AUX_FRM:
			rslt = move_next_frame_custom(&data_index, fifo->aux_frm_len, fifo);
			break;

		/* If header defines only auxiliary and gyroscope frame */
		case BMI2_FIFO_HEADER_AUX_GYR_FRM:
			rslt = move_next_frame_custom(&data_index, fifo->aux_gyr_frm_len, fifo);
			break;

		/* If header defines sensor time frame */
		case BMI2_FIFO_HEADER_SENS_TIME_FRM:
			rslt = unpack_sensortime_frame_custom(&data_index, fifo);
			break;

		/* If header defines skip frame */
		case BMI2_FIFO_HEADER_SKIP_FRM:
			rslt = unpack_skipped_frame_custom(&data_index, fifo);
			break;

		/* If header defines Input configuration frame */
		case BMI2_FIFO_HEADER_INPUT_CFG_FRM:
			rslt = move_next_frame_custom(&data_index, BMI2_FIFO_INPUT_CFG_LENGTH, fifo);
			break;

		/* If header defines invalid frame or end of valid data */
		case BMI2_FIFO_HEAD_OVER_READ_MSB:

			/* Move the data index to the last byte to mark completion */
			data_index = fifo->length;

			/* FIFO is empty */
			rslt = BMI2_W_FIFO_EMPTY;
			break;
		case BMI2_FIFO_VIRT_ACT_RECOG_FRM:
			rslt = move_next_frame_custom(&data_index, BMI2_FIFO_VIRT_ACT_DATA_LENGTH, fifo);
			break;
		default:

			/* Move the data index to the last byte in case of invalid values */
			data_index = fifo->length;

			/* FIFO is empty */
			rslt = BMI2_W_FIFO_EMPTY;
			break;
		}

		/* Break if Number of frames to be read is complete or FIFO is mpty */
		if ((frame_to_read == accel_index) || (rslt == BMI2_W_FIFO_EMPTY))
		{
			break;
		}
	}

	/* Update the accelerometer frame index */
	(*accel_length) = accel_index;

	/* Update the accelerometer byte index */
	fifo->acc_byte_start_idx = data_index;

	return rslt;
}

static int8_t extract_accel_headerless_mode_custom(struct bmi2_sens_axes_data *acc, uint16_t *accel_length, struct bmi2_fifo_frame *fifo, uint64_t sens_en_stat)
{
	int8_t rslt;

	/* Variable to index the bytes */
	uint16_t data_index = 0;

	/* Variable to define the data enable byte */
	uint8_t data_enable;

	/* Variable to index accelerometer frames */
	uint16_t accel_index = 0;

	/* Variable to store the number of bytes to be read */
	uint16_t data_read_length = 0;

	/* Number of bytes to skip in case dummy frame is obtained */
	uint8_t skip_length = 0;

	/* Get the number of accelerometer bytes to be read */
	rslt = parse_fifo_accel_len_custom(&data_index, &data_read_length, &skip_length, accel_length, fifo);

	/* Convert word to byte since all sensor enables are in a byte */
	data_enable = (uint8_t)(fifo->data_enable >> 8);

	for (; (data_index < data_read_length) && (rslt != BMI2_W_FIFO_EMPTY);)
	{
		rslt = check_dummy_frame_custom(BMI2_FIFO_HEADERLESS_DUMMY_ACC, &data_index, skip_length, fifo);

		/* Unpack only if Valid frame is present */
		if (rslt == BMI2_OK)
		{
			/* Unpack frame to get the accelerometer data */
			rslt = unpack_accel_headerless_frame_custom(acc, &data_index, &accel_index, data_enable, fifo, sens_en_stat);

			if (rslt != BMI2_W_FIFO_EMPTY)
			{
				/* Check for the availability of next two bytes of FIFO data */
				rslt = check_empty_fifo_custom(&data_index, fifo);
			}
		}
	}

	/* Update number of accelerometer frames to be read */
	(*accel_length) = accel_index;

	/* Update the accelerometer byte index */
	fifo->acc_byte_start_idx = data_index;

	return rslt;
}

static void parse_if_virtual_header_custom(uint8_t *frame_header, uint16_t *data_index, const struct bmi2_fifo_frame *fifo)
{
	/* Variable to extract virtual header byte */
	uint8_t virtual_header_mode;

	/* Extract virtual header mode from the frame header */
	virtual_header_mode = BMI2_GET_BITS(*frame_header, BMI2_FIFO_VIRT_FRM_MODE);

	/* If the extracted header byte is a virtual header */
	if (virtual_header_mode == BMI2_FIFO_VIRT_FRM_MODE)
	{
		/* If frame header is not activity recognition header */
		if (*frame_header != 0xC8)
		{
			/* Index shifted to next byte where sensor frame is present */
			(*data_index) = (*data_index) + 1;

			/* Get the sensor frame header */
			*frame_header = fifo->data[*data_index] & BMI2_FIFO_TAG_INTR_MASK;
		}
	}
}

static int8_t move_next_frame_custom(uint16_t *data_index, uint8_t current_frame_length, const struct bmi2_fifo_frame *fifo)
{
	/* Variables to define error */
	int8_t rslt = BMI2_OK;

	/* Validate data index */
	if (((*data_index) + current_frame_length) > fifo->length)
	{
		/* Move the data index to the last byte */
		(*data_index) = fifo->length;

		rslt = BMI2_OK;
	}
	else
	{
		/* Move the data index to next frame */
		(*data_index) = (*data_index) + current_frame_length;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
	}

	return rslt;
}

static int8_t unpack_accel_header_frame_custom(struct bmi2_sens_axes_data *acc,
											   uint16_t *idx,
											   uint16_t *acc_idx,
											   uint8_t frame,
											   const struct bmi2_fifo_frame *fifo,
											   uint64_t sens_en_stat)
{
	/* Variable to define error */
	int8_t rslt = BMI2_OK;

	switch (frame)
	{
	/* If frame contains only accelerometer data */
	case BMI2_FIFO_HEADER_ACC_FRM:

		/* Partially read, then skip the data */
		if (((*idx) + fifo->acc_frm_len) > fifo->length)
		{
			/* Update the data index as complete*/
			(*idx) = fifo->length;

			rslt = BMI2_OK;
			break;
		}

		/* Get the accelerometer data */
		unpack_accel_data_custom(&acc[(*acc_idx)], *idx, fifo);

		/* Update data index */
		(*idx) = (*idx) + BMI2_FIFO_ACC_LENGTH;

		/* Get virtual sensor time if S4S is enabled */
		if (sens_en_stat & BMI2_EXT_SENS_SEL)
		{
			unpack_virt_sensor_time_custom(&acc[(*acc_idx)], idx, fifo);
		}

		/* Update accelerometer frame index */
		(*acc_idx)++;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
		break;

	/* If frame contains accelerometer and gyroscope data */
	case BMI2_FIFO_HEADER_GYR_ACC_FRM:

		/* Partially read, then skip the data */
		if (((*idx) + fifo->acc_gyr_frm_len) > fifo->length)
		{
			/* Move the data index to the last byte */
			(*idx) = fifo->length;

			rslt = BMI2_OK;
			break;
		}

		/* Get the accelerometer data */
		unpack_accel_data_custom(&acc[(*acc_idx)], ((*idx) + BMI2_FIFO_GYR_LENGTH), fifo);

		/* Update data index */
		(*idx) = (*idx) + BMI2_FIFO_ACC_GYR_LENGTH;

		/* Get virtual sensor time if S4S is enabled */
		if (sens_en_stat & BMI2_EXT_SENS_SEL)
		{
			unpack_virt_sensor_time_custom(&acc[(*acc_idx)], idx, fifo);
		}

		/* Update accelerometer frame index */
		(*acc_idx)++;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
		break;

	/* If frame contains accelerometer and auxiliary data */
	case BMI2_FIFO_HEADER_AUX_ACC_FRM:

		/* Partially read, then skip the data */
		if (((*idx) + fifo->acc_aux_frm_len) > fifo->length)
		{
			/* Move the data index to the last byte */
			(*idx) = fifo->length;

			rslt = BMI2_OK;
			break;
		}

		/* Get the accelerometer data */
		unpack_accel_data_custom(&acc[(*acc_idx)], ((*idx) + fifo->aux_frm_len), fifo);

		/* Update data index */
		(*idx) = (*idx) + (BMI2_FIFO_ACC_LENGTH + fifo->aux_frm_len);

		/* Get virtual sensor time if S4S is enabled */
		if (sens_en_stat & BMI2_EXT_SENS_SEL)
		{
			unpack_virt_sensor_time_custom(&acc[(*acc_idx)], idx, fifo);
		}

		/* Update accelerometer frame index */
		(*acc_idx)++;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
		break;

	/* If frame contains accelerometer, gyroscope and auxiliary data */
	case BMI2_FIFO_HEADER_ALL_FRM:

		/* Partially read, then skip the data*/
		if ((*idx + fifo->all_frm_len) > fifo->length)
		{
			/* Move the data index to the last byte */
			(*idx) = fifo->length;

			rslt = BMI2_OK;
			break;
		}

		/* Get the accelerometer data */
		unpack_accel_data_custom(&acc[(*acc_idx)], ((*idx) + (BMI2_FIFO_GYR_LENGTH + fifo->aux_frm_len)), fifo);

		/* Update data index */
		(*idx) = (*idx) + (BMI2_FIFO_ACC_LENGTH + BMI2_FIFO_GYR_LENGTH + fifo->aux_frm_len);

		/* Get virtual sensor time if S4S is enabled */
		if (sens_en_stat & BMI2_EXT_SENS_SEL)
		{
			unpack_virt_sensor_time_custom(&acc[(*acc_idx)], idx, fifo);
		}

		/* Update accelerometer frame index */
		(*acc_idx)++;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
		break;

	/* If frame contains gyroscope and auxiliary data */
	case BMI2_FIFO_HEADER_AUX_GYR_FRM:

		/* Update data index */
		(*idx) = (*idx) + fifo->aux_gyr_frm_len;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
		break;

	/* If frame contains only auxiliary data */
	case BMI2_FIFO_HEADER_AUX_FRM:

		/* Update data index */
		(*idx) = (*idx) + fifo->aux_frm_len;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
		break;

	/* If frame contains only gyroscope data */
	case BMI2_FIFO_HEADER_GYR_FRM:

		/* Update data index */
		(*idx) = (*idx) + fifo->gyr_frm_len;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
		break;
	default:

		/* Move the data index to the last byte in case of invalid values */
		(*idx) = fifo->length;

		/* FIFO is empty */
		rslt = BMI2_W_FIFO_EMPTY;
		break;
	}

	return rslt;
}

static int8_t unpack_accel_headerless_frame_custom(struct bmi2_sens_axes_data *acc,
												   uint16_t *idx,
												   uint16_t *acc_idx,
												   uint8_t frame,
												   const struct bmi2_fifo_frame *fifo,
												   uint64_t sens_en_stat)
{
	/* Variable to define error */
	int8_t rslt = BMI2_OK;

	switch (frame)
	{
	/* If frame contains only accelerometer data */
	case BMI2_FIFO_HEAD_LESS_ACC_FRM:

		/* Partially read, then skip the data */
		if (((*idx) + fifo->acc_frm_len) > fifo->length)
		{
			/* Update the data index as complete*/
			(*idx) = fifo->length;

			rslt = BMI2_OK;
			break;
		}

		/* Get the accelerometer data */
		unpack_accel_data_custom(&acc[(*acc_idx)], *idx, fifo);

		/* Update data index */
		(*idx) = (*idx) + BMI2_FIFO_ACC_LENGTH;

		/* Get virtual sensor time if S4S is enabled */
		if (sens_en_stat & BMI2_EXT_SENS_SEL)
		{
			unpack_virt_sensor_time_custom(&acc[(*acc_idx)], idx, fifo);
		}

		/* Update accelerometer frame index */
		(*acc_idx)++;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
		break;

	/* If frame contains accelerometer and gyroscope data */
	case BMI2_FIFO_HEAD_LESS_GYR_ACC_FRM:

		/* Partially read, then skip the data */
		if (((*idx) + fifo->acc_frm_len) > fifo->length)
		{
			/* Move the data index to the last byte */
			(*idx) = fifo->length;

			rslt = BMI2_OK;
			break;
		}

		/* Get the accelerometer data */
		unpack_accel_data_custom(&acc[(*acc_idx)], *idx, fifo);

		/* Update data index */
		(*idx) = (*idx) + BMI2_FIFO_ACC_GYR_LENGTH;

		/* Get virtual sensor time if S4S is enabled */
		if (sens_en_stat & BMI2_EXT_SENS_SEL)
		{
			unpack_virt_sensor_time_custom(&acc[(*acc_idx)], idx, fifo);
		}

		/* Update accelerometer frame index */
		(*acc_idx)++;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
		break;

	/* If frame contains accelerometer and auxiliary data */
	case BMI2_FIFO_HEAD_LESS_AUX_ACC_FRM:

		/* Partially read, then skip the data */
		if (((*idx) + fifo->acc_frm_len) > fifo->length)
		{
			/* Move the data index to the last byte */
			(*idx) = fifo->length;

			rslt = BMI2_OK;
			break;
		}

		/* Get the accelerometer data */
		unpack_accel_data_custom(&acc[(*acc_idx)], *idx, fifo);

		/* Update data index */
		(*idx) = (*idx) + (BMI2_FIFO_ACC_LENGTH + fifo->aux_frm_len);

		/* Get virtual sensor time if S4S is enabled */
		if (sens_en_stat & BMI2_EXT_SENS_SEL)
		{
			unpack_virt_sensor_time_custom(&acc[(*acc_idx)], idx, fifo);
		}

		/* Update accelerometer frame index */
		(*acc_idx)++;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
		break;

	/* If frame contains accelerometer, gyroscope and auxiliary data */
	case BMI2_FIFO_HEAD_LESS_ALL_FRM:

		/* Partially read, then skip the data*/
		if ((*idx + fifo->acc_frm_len) > fifo->length)
		{
			/* Move the data index to the last byte */
			(*idx) = fifo->length;

			rslt = BMI2_OK;
			break;
		}

		/* Get the accelerometer data */
		unpack_accel_data_custom(&acc[(*acc_idx)], *idx, fifo);

		/* Update data index */
		(*idx) = (*idx) + (BMI2_FIFO_ACC_LENGTH + BMI2_FIFO_GYR_LENGTH + fifo->aux_frm_len);

		/* Get virtual sensor time if S4S is enabled */
		if (sens_en_stat & BMI2_EXT_SENS_SEL)
		{
			unpack_virt_sensor_time_custom(&acc[(*acc_idx)], idx, fifo);
		}

		/* Update accelerometer frame index */
		(*acc_idx)++;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
		break;

	/* If frame contains gyroscope and auxiliary data */
	case BMI2_FIFO_HEAD_LESS_GYR_AUX_FRM:

		/* Update data index */
		(*idx) = (*idx) + fifo->aux_gyr_frm_len;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
		break;

	/* If frame contains only auxiliary data */
	case BMI2_FIFO_HEAD_LESS_AUX_FRM:

		/* Update data index */
		(*idx) = (*idx) + fifo->aux_frm_len;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
		break;

	/* If frame contains only gyroscope data */
	case BMI2_FIFO_HEAD_LESS_GYR_FRM:

		/* Update data index */
		(*idx) = (*idx) + fifo->gyr_frm_len;

		/* More frames could be read */
		rslt = BMI2_W_PARTIAL_READ;
		break;
	default:

		/* Move the data index to the last byte in case of invalid values */
		(*idx) = fifo->length;

		/* FIFO is empty */
		rslt = BMI2_W_FIFO_EMPTY;
		break;
	}

	return rslt;
}

static void unpack_accel_data_custom(struct bmi2_sens_axes_data *acc,
									 uint16_t data_start_index,
									 const struct bmi2_fifo_frame *fifo)
{
	/* Variables to store LSB value */
	uint16_t data_lsb;

	/* Variables to store MSB value */
	uint16_t data_msb;

	/* Accelerometer raw x data */
	data_lsb = fifo->data[data_start_index++];
	data_msb = fifo->data[data_start_index++];
	acc->x = (int16_t)((data_msb << 8) | data_lsb);

	/* Accelerometer raw y data */
	data_lsb = fifo->data[data_start_index++];
	data_msb = fifo->data[data_start_index++];
	acc->y = (int16_t)((data_msb << 8) | data_lsb);

	/* Accelerometer raw z data */
	data_lsb = fifo->data[data_start_index++];
	data_msb = fifo->data[data_start_index++];
	acc->z = (int16_t)((data_msb << 8) | data_lsb);
}
static void unpack_virt_sensor_time_custom(struct bmi2_sens_axes_data *sens, uint16_t *idx, const struct bmi2_fifo_frame *fifo)
{
	/* Variables to define 3 bytes of sensor time */
	uint32_t sensor_time_byte3;
	uint16_t sensor_time_byte2;
	uint8_t sensor_time_byte1;

	/* Get sensor time from the FIFO data */
	sensor_time_byte3 = (uint32_t)(fifo->data[(*idx) + BMI2_SENSOR_TIME_MSB_BYTE] << 16);
	sensor_time_byte2 = (uint16_t)fifo->data[(*idx) + BMI2_SENSOR_TIME_XLSB_BYTE] << 8;
	sensor_time_byte1 = fifo->data[(*idx)];

	/* Store sensor time in the sensor data structure */
	sens->virt_sens_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);

	/* Move the data index by 3 bytes */
	(*idx) = (*idx) + BMI2_SENSOR_TIME_LENGTH;
}

static int8_t parse_fifo_accel_len_custom(uint16_t *start_idx,
										  uint16_t *len,
										  uint8_t *skip_length,
										  const uint16_t *acc_count,
										  const struct bmi2_fifo_frame *fifo)
{
	/* Variable to define error */
	int8_t rslt = BMI2_OK;

	/* Data start index */
	(*start_idx) = fifo->acc_byte_start_idx;

	/* If only accelerometer is enabled */
	if (fifo->data_enable == BMI2_FIFO_ACC_EN)
	{
		/* Number of bytes to be read */
		(*len) = (uint16_t)((*acc_count) * BMI2_FIFO_ACC_LENGTH);

		/* Number of bytes to skip in case dummy frame is obtained */
		(*skip_length) = BMI2_FIFO_ACC_LENGTH;
	}
	/* If only accelerometer and auxiliary are enabled */
	else if (fifo->data_enable == (BMI2_FIFO_ACC_EN | BMI2_FIFO_AUX_EN))
	{
		/* Number of bytes to be read */
		(*len) = (uint16_t)((*acc_count) * (BMI2_FIFO_ACC_LENGTH + fifo->aux_frm_len));

		/* Number of bytes to skip in case dummy frame is obtained */
		(*skip_length) = (BMI2_FIFO_ACC_LENGTH + fifo->aux_frm_len);

		/* Data start index */
		(*start_idx) = fifo->acc_byte_start_idx + fifo->aux_frm_len;
	}
	/* If only accelerometer and gyroscope are enabled */
	else if (fifo->data_enable == (BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN))
	{
		/* Number of bytes to be read */
		(*len) = (uint16_t)((*acc_count) * BMI2_FIFO_ACC_GYR_LENGTH);

		/* Number of bytes to skip in case dummy frame is obtained */
		(*skip_length) = BMI2_FIFO_ACC_GYR_LENGTH;

		/* Data start index */
		(*start_idx) = fifo->acc_byte_start_idx + BMI2_FIFO_GYR_LENGTH;
	}
	/* If only accelerometer, gyroscope and auxiliary are enabled */
	else if (fifo->data_enable == (BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN | BMI2_FIFO_AUX_EN))
	{
		/* Number of bytes to be read */
		(*len) = (uint16_t)((*acc_count) * (BMI2_FIFO_ACC_LENGTH + BMI2_FIFO_GYR_LENGTH + fifo->aux_frm_len));

		/* Number of bytes to skip in case dummy frame is obtained */
		(*skip_length) = (BMI2_FIFO_ACC_LENGTH + BMI2_FIFO_GYR_LENGTH + fifo->aux_frm_len);

		/* Data start index */
		(*start_idx) = fifo->acc_byte_start_idx + (BMI2_FIFO_GYR_LENGTH + fifo->aux_frm_len);
	}
	else
	{
		/* Move the data index to the last byte to mark completion when
		 * no sensors or sensors apart from accelerometer are enabled
		 */
		(*start_idx) = fifo->length;

		/* FIFO is empty */
		rslt = BMI2_W_FIFO_EMPTY;
	}

	/* If more data is requested than available */
	if ((*len) > fifo->length)
	{
		(*len) = fifo->length;
	}

	return rslt;
}

static int8_t check_dummy_frame_custom(uint8_t dummy_frame_header,
									   uint16_t *data_index,
									   uint8_t skip_length,
									   const struct bmi2_fifo_frame *fifo)
{
	int8_t rslt;

	/* Validate data index */
	if (((*data_index) + 6) <= fifo->length)
	{
		/* Check if FIFO contains dummy frame */
		if (((fifo->data[(*data_index)] == dummy_frame_header) &&
			 (fifo->data[(*data_index) + 1] == BMI2_FIFO_HEADERLESS_DUMMY_BYTE_1) &&
			 (fifo->data[(*data_index) + 2] == BMI2_FIFO_HEADERLESS_DUMMY_BYTE_2)) &&
			((fifo->data[(*data_index) + 3] == BMI2_FIFO_HEADERLESS_DUMMY_BYTE_3)))
		{
			/* Move the data index to next frame */
			(*data_index) = (*data_index) + skip_length;

			/* Dummy byte parsed */
			rslt = BMI2_W_DUMMY_BYTE;
		}
		else
		{
			/* Valid frame */
			rslt = BMI2_OK;
		}
	}
	else
	{
		/* Move the data index to the last byte to mark completion */
		(*data_index) = fifo->length;

		/* FIFO is empty */
		rslt = BMI2_W_FIFO_EMPTY;
	}

	return rslt;
}

static int8_t check_empty_fifo_custom(uint16_t *data_index, const struct bmi2_fifo_frame *fifo)
{
	/* Variables to define error */
	int8_t rslt = BMI2_OK;

	/* Validate data index */
	if (((*data_index) + 6) < fifo->length)
	{
		/* Check if FIFO is empty */
		if (((fifo->data[(*data_index)] == BMI2_FIFO_LSB_CONFIG_CHECK) &&
			 (fifo->data[(*data_index) + 1] == BMI2_FIFO_MSB_CONFIG_CHECK) &&
			 (fifo->data[(*data_index) + 2] == BMI2_FIFO_LSB_CONFIG_CHECK)) &&
			((fifo->data[(*data_index) + 3] == BMI2_FIFO_MSB_CONFIG_CHECK) &&
			 (fifo->data[(*data_index) + 4] == BMI2_FIFO_LSB_CONFIG_CHECK)) &&
			((fifo->data[(*data_index) + 5] == BMI2_FIFO_MSB_CONFIG_CHECK)))
		{
			/* Move the data index to the last byte to mark completion */
			(*data_index) = fifo->length;

			/* FIFO is empty */
			rslt = BMI2_W_FIFO_EMPTY;
		}
		else
		{
			/* More frames could be read */
			rslt = BMI2_W_PARTIAL_READ;
		}
	}

	return rslt;
}

static int8_t unpack_sensortime_frame_custom(uint16_t *data_index, struct bmi2_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = BMI2_OK;

    /* Variables to define 3 bytes of sensor time */
    uint32_t sensor_time_byte3 = 0;
    uint16_t sensor_time_byte2 = 0;
    uint8_t sensor_time_byte1 = 0;

    /* Validate data index */
    if (((*data_index) + BMI2_SENSOR_TIME_LENGTH) > fifo->length)
    {
        /* Move the data index to the last byte */
        (*data_index) = fifo->length;

        rslt = BMI2_OK;
    }
    else
    {
        /* Get sensor time from the FIFO data */
        sensor_time_byte3 = fifo->data[(*data_index) + BMI2_SENSOR_TIME_MSB_BYTE] << 16;
        sensor_time_byte2 = fifo->data[(*data_index) + BMI2_SENSOR_TIME_XLSB_BYTE] << 8;
        sensor_time_byte1 = fifo->data[(*data_index)];

        /* Update sensor time in the FIFO structure */
        fifo->sensor_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);

        /* Move the data index by 3 bytes */
        (*data_index) = (*data_index) + BMI2_SENSOR_TIME_LENGTH;

        /* More frames could be read */
        rslt = BMI2_W_PARTIAL_READ;
    }

    return rslt;
}

static int8_t unpack_skipped_frame_custom(uint16_t *data_index, struct bmi2_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = BMI2_OK;

    /* Validate data index */
    if ((*data_index) >= fifo->length)
    {
        /* Update the data index to the last byte */
        (*data_index) = fifo->length;

        rslt = BMI2_OK;
    }
    else
    {
        /* Update skipped frame count in the FIFO structure */
        fifo->skipped_frame_count = fifo->data[(*data_index)];

        /* Move the data index by 1 byte */
        (*data_index) = (*data_index) + 1;

        /* More frames could be read */
        rslt = BMI2_W_PARTIAL_READ;
    }

    return rslt;
}

void channel_accel_convert_custom(struct sensor_value *val, int64_t raw_val, uint8_t range)
{
	/* 16 bit accelerometer. 2^15 bits represent the range in G */
	/* Converting from G to m/s^2 */
	raw_val = (raw_val * SENSOR_G * (int64_t) range) / INT16_MAX;

	val->val1 = raw_val / 1000000LL;
	val->val2 = raw_val % 1000000LL;
}
void bmi270_config_adv_power_save_mode(const struct device *const bmi270_dev, uint8_t enable){
	uint8_t reg_write = 0b011;
	uint8_t ret, reg_read;
	if(enable == 0){
		reg_write &= 0b010;
	}
	ret = bmi270_reg_write(bmi270_dev, BMI270_REG_PWR_CONF, &reg_write, sizeof(reg_write));
	printf("ret PWR_CONF write: %d\n", ret);
	ret = bmi270_reg_read(bmi270_dev, BMI270_REG_PWR_CONF, &reg_read, sizeof(reg_read));
	printf("ret PWR_CONF read: %d\n", ret);
	printf("PWR_CONF: 0x%x\n\n", reg_write);
}

void bmi270_config_fifo(const struct device *const bmi270_dev, uint8_t fifo_header_en, uint8_t fifo_aux_en, uint8_t fifo_acc_en, uint8_t fifo_gyr_en){
	uint8_t reg_write = 0;
	uint8_t ret, reg_read;
	if(fifo_header_en == 1){
		reg_write |= 0b00010000;
	}
	if(fifo_aux_en == 1){
		reg_write |= 0b00100000;
	}
	if(fifo_acc_en == 1){
		reg_write |= 0b01000000;
	}
	if(fifo_gyr_en == 1){
		reg_write |= 0b10000000;
	}
	ret = bmi270_reg_write(bmi270_dev, BMI270_REG_FIFO_CONFIG_1, &reg_write, sizeof(reg_write));
	printf("ret FIFO_CONFIG_1 write: %d\n", ret);
	ret = bmi270_reg_read(bmi270_dev, BMI270_REG_FIFO_CONFIG_1, &reg_read, sizeof(reg_read));
	printf("ret FIFO_CONFIG_1 read: %d\n", ret);
	printf("FIFO_CONFIG_1: 0x%x\n\n", reg_write);
}

void bmi270_config_ffull_interrupt(const struct device *const bmi270_dev, uint16_t enable){
	uint8_t reg_write = 0;
	uint8_t ret, reg_read;
	if(enable == 1){
		reg_write |= 0b00010000;
	}
	ret = bmi270_reg_write(bmi270_dev, BMI270_REG_INT_MAP_DATA, &reg_write, sizeof(reg_write));
	printf("ret INT_MAP_DATA write: %d\n", ret);
	ret = bmi270_reg_read(bmi270_dev, BMI270_REG_INT_MAP_DATA, &reg_read, sizeof(reg_read));
	printf("ret INT_MAP_DATA read: %d\n", ret);
	printf("INT_MAP_DATA: 0x%x\n\n", reg_write);
}
/*
fix for bmi270 over i2c - nrf5340
static int bmi270_reg_write_i2c(const union bmi270_bus *bus, uint8_t start,
				const uint8_t *data, uint16_t len)
{
	//return i2c_burst_write_dt(&bus->i2c, start, data, len);
	uint8_t buffer[1+len];
		__ASSERT((1U + len) <= sizeof(buffer),
						  "burst buffer too small");
		buffer[0]=start;
		memmove(buffer+1,data,len);
	return i2c_write_dt(&bus->i2c,buffer,1+len);

}

*/