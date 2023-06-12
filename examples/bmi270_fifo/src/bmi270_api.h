/*
 * Copyright © Kazimierz Roman
 *
 *
 */
#include <zephyr/drivers/gpio.h>
#include "bmi2_defs.h"

void bmi270_init(const struct device *const bmi270_dev);
void bmi270_basic_config_acc(const struct device *const bmi270_dev, uint16_t full_scale, uint16_t sampling_freq, uint16_t oversampling);
void bmi270_basic_config_gyr(const struct device *const bmi270_dev, uint16_t full_scale, uint16_t sampling_freq, uint16_t oversampling);
void bmi270_config_anymotion_interrupt(const struct device *const bmi270_dev, uint16_t threshold, uint16_t duration);
void bmi270_config_nomotion_interrupt(const struct device *const bmi270_dev, uint16_t threshold, uint16_t duration);
void bmi270_set_anymotion_trigger(const struct device *const bmi270_dev, sensor_trigger_handler_t trigger_handler, struct sensor_trigger *trig);
void bmi270_set_nomotion_trigger(const struct device *const bmi270_dev, sensor_trigger_handler_t trigger_handler, struct sensor_trigger *trig);
void bmi270_config_ffull_interrupt(const struct device *const bmi270_dev, uint16_t ffifo_enable);
void bmi270_config_adv_power_save_mode(const struct device *const bmi270_dev, uint8_t enable);
void bmi270_config_fifo(const struct device *const bmi270_dev, uint8_t fifo_header_en, uint8_t fifo_aux_en, uint8_t fifo_acc_en, uint8_t fifo_gyr_en);

void gpio_init_manual(const struct gpio_dt_spec *spec);
void gpio_callback_config_manual(struct gpio_callback *callback_data, gpio_callback_handler_t handler, const struct gpio_dt_spec *spec);
int8_t bmi270_get_fifo_length(uint16_t *fifo_length, const struct device *dev);
int8_t bmi270_read_fifo_data(struct bmi2_fifo_frame *fifo, const struct device *dev, uint64_t sens_en_stat, bool spi, bool advanced_power_save);
int8_t bmi270_extract_accel(struct bmi2_sens_axes_data *accel_data, uint16_t *accel_length, struct bmi2_fifo_frame *fifo, uint64_t sens_en_stat);
int8_t bmi270_extract_gyro(struct bmi2_sens_axes_data *gyro_data, uint16_t *gyro_length, struct bmi2_fifo_frame *fifo, const struct bmi2_dev *dev);
void channel_accel_convert_custom(struct sensor_value *val, int64_t raw_val, uint8_t range);
