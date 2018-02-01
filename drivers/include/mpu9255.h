/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_mpu9255 MPU-9255
 * @ingroup     drivers_sensors
 * @brief       Device driver interface for the MPU-9255
 * @{
 *
 * @file
 * @brief       Device driver interface for the MPU-9255
 *
 */

#ifndef MPU9255_H_
#define MPU9255_H_

#include "periph/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU9255_WHOAMI_ANSWER  (0x73)

/**
 * @name Sample rate macro definitions
 * @{
 */
#define MPU9255_MIN_SAMPLE_RATE     (4)
#define MPU9255_MAX_SAMPLE_RATE     (1000)
#define MPU9255_DEFAULT_SAMPLE_RATE (50)
#define MPU9255_MIN_COMP_SMPL_RATE  (1)
#define MPU9255_MAX_COMP_SMPL_RATE  (100)
/** @} */


#define G_DLPF_SAMPLE_RATE_8KHZ_BW_250		(0x0)
#define G_DLPF_SAMPLE_RATE_1KHZ_BW_184		(0x1)
#define G_DLPF_SAMPLE_RATE_1KHZ_BW_92		(0x2)
#define G_DLPF_SAMPLE_RATE_1KHZ_BW_41		(0x3)
#define G_DLPF_SAMPLE_RATE_1KHZ_BW_20		(0x4)
#define G_DLPF_SAMPLE_RATE_1KHZ_BW_10		(0x5)
#define G_DLPF_SAMPLE_RATE_1KHZ_BW_5		(0x6)
#define G_DLPF_SAMPLE_RATE_8KHZ_BW_3600		(0x7)

#define A_DLPF_SAMPLE_RATE_1KHZ_BW_460		(0x0)
#define A_DLPF_SAMPLE_RATE_1KHZ_BW_184		(0x1)
#define A_DLPF_SAMPLE_RATE_1KHZ_BW_92		(0x2)
#define A_DLPF_SAMPLE_RATE_1KHZ_BW_41		(0x3)
#define A_DLPF_SAMPLE_RATE_1KHZ_BW_20		(0x4)
#define A_DLPF_SAMPLE_RATE_1KHZ_BW_10		(0x5)
#define A_DLPF_SAMPLE_RATE_1KHZ_BW_5		(0x6)

/**
 * @name Power Management 1 register macros
 * @{
 */
#define MPU9255_PWR_WAKEUP          (0x00)
#define MPU9255_PWR_PLL             (0x01)
#define MPU9255_PWR_RESET           (0x80)
/** @} */

/**
 * @name Power Management 2 register macros
 * @{
 */
#define MPU9255_PWR_GYRO            (0x07)
#define MPU9255_PWR_ACCEL           (0x56)
/** @} */

/**
 * @name Sleep times in microseconds
 * @{
 */
#define MPU9255_COMP_MODE_SLEEP_US  (1000)
//#define MPU9255_BYPASS_SLEEP_US     (3000)
#define MPU9255_BYPASS_SLEEP_US     (1500)
#define MPU9255_PWR_CHANGE_SLEEP_US (50000)
#define MPU9255_RESET_SLEEP_US      (100000)
/** @} */

/**
 * @name MPU-9255 compass operating modes and reg values
 * @{
 */
#define MPU9255_COMP_POWER_DOWN_MODE            (0x00)
#define MPU9255_COMP_SINGLE_MEASURE_MODE        (0x01)
#define MPU9255_COMP_CONTINUOUS_MEASURE_1_MODE  (0x02)
#define MPU9255_COMP_CONTINUOUS_MEASURE_2_MODE  (0x06)
#define MPU9255_COMP_SELF_TEST_MODE             (0x08)
#define MPU9255_COMP_FUSE_ROM_MODE              (0x0F)
#define MPU9255_COMP_WHOAMI_ANSWER              (0x48)
/** @} */

/**
 * @brief Power enum values
 */
typedef enum {
    MPU9255_SENSOR_PWR_OFF = 0x00,
    MPU9255_SENSOR_PWR_ON = 0x01,
} mpu9255_pwr_t;

/**
 * @brief Possible MPU-9255 hardware addresses (wiring specific)
 */
typedef enum {
    MPU9255_HW_ADDR_HEX_68 = 0x68,
    MPU9255_HW_ADDR_HEX_69 = 0x69,
} mpu9255_hw_addr_t;

/**
 * @brief Possible compass addresses (wiring specific)
 */
typedef enum {
    MPU9255_COMP_ADDR_HEX_0C = 0x0C,
    MPU9255_COMP_ADDR_HEX_0D = 0x0D,
    MPU9255_COMP_ADDR_HEX_0E = 0x0E,
    MPU9255_COMP_ADDR_HEX_0F = 0x0F,
} mpu9255_comp_addr_t;

/**
 * @brief Possible full scale ranges for the gyroscope
 */
typedef enum {
    MPU9255_GYRO_FSR_250DPS = 0x00,
    MPU9255_GYRO_FSR_500DPS = 0x01,
    MPU9255_GYRO_FSR_1000DPS = 0x02,
    MPU9255_GYRO_FSR_2000DPS = 0x03,
} mpu9255_gyro_ranges_t;

/**
 * @brief Possible full scale ranges for the accelerometer
 */
typedef enum {
    MPU9255_ACCEL_FSR_2G = 0x00,
    MPU9255_ACCEL_FSR_4G = 0x01,
    MPU9255_ACCEL_FSR_8G = 0x02,
    MPU9255_ACCEL_FSR_16G = 0x03,
} mpu9255_accel_ranges_t;

typedef enum {
	 MPU9255_ACCEL_LPF_1130	=	0x08,                    // 1130Hz, 0.75mS delay
	 MPU9255_ACCEL_LPF_460	=	0x00,                    // 460Hz, 1.94mS delay
	 MPU9255_ACCEL_LPF_184	=	0x01,                    // 184Hz, 5.80mS delay
	 MPU9255_ACCEL_LPF_92	=	0x02,                    // 92Hz, 7.80mS delay
	 MPU9255_ACCEL_LPF_41	=	0x03,                    // 41Hz, 11.80mS delay
	 MPU9255_ACCEL_LPF_20	=	0x04,                    // 20Hz, 19.80mS delay
	 MPU9255_ACCEL_LPF_10	=	0x05,                    // 10Hz, 35.70mS delay
	 MPU9255_ACCEL_LPF_5	=	0x06,                    // 5Hz, 66.96mS delay
} mpu9255_accel_lpf_t;


//  Gyro LPF options

typedef enum {
	MPU9255_GYRO_LPF_8800	=	0x11,                    // 8800Hz, 0.64mS delay
	MPU9255_GYRO_LPF_3600	=	0x10,                    // 3600Hz, 0.11mS delay
	MPU9255_GYRO_LPF_250	=	0x00,                    // 250Hz, 0.97mS delay
	MPU9255_GYRO_LPF_184	=	0x01,                    // 184Hz, 2.9mS delay
	MPU9255_GYRO_LPF_92	=	0x02,                    // 92Hz, 3.9mS delay
	MPU9255_GYRO_LPF_41	=	0x03,                    // 41Hz, 5.9mS delay
	MPU9255_GYRO_LPF_20	=	0x04,                    // 20Hz, 9.9mS delay
	MPU9255_GYRO_LPF_10	=	0x05,                    // 10Hz, 17.85mS delay
	MPU9255_GYRO_LPF_5	=	0x06,                    // 5Hz, 33.48mS delay
} mpu9255_gyro_lpf_t;



/**
 * @brief MPU-9255 result vector struct
 */
typedef struct {
    int16_t x_axis;             /**< X-Axis measurement result */
    int16_t y_axis;             /**< Y-Axis measurement result */
    int16_t z_axis;             /**< Z-Axis measurement result */
} mpu9255_results_t;

/**
 * @brief Configuration struct for the MPU-9255 sensor
 */
typedef struct {
    mpu9255_pwr_t accel_pwr;            /**< Accel power status (on/off) */
    mpu9255_pwr_t gyro_pwr;             /**< Gyro power status (on/off) */
    mpu9255_pwr_t compass_pwr;          /**< Compass power status (on/off) */
    mpu9255_gyro_ranges_t gyro_fsr;     /**< Configured gyro full-scale range */
    mpu9255_accel_ranges_t accel_fsr;   /**< Configured accel full-scale range */
    mpu9255_gyro_lpf_t gyro_lpf;   	/**< Gyro LPF freq */
    mpu9255_accel_lpf_t accel_lpf;   	/**< Accel LPF freq */
    uint16_t sample_rate;               /**< Configured sample rate for accel and gyro */
    uint8_t compass_sample_rate;        /**< Configured compass sample rate */
    uint8_t compass_x_adj;              /**< Compass X-Axis sensitivity adjustment value */
    uint8_t compass_y_adj;              /**< Compass Y-Axis sensitivity adjustment value */
    uint8_t compass_z_adj;              /**< Compass Z-Axis sensitivity adjustment value */
} mpu9255_status_t;

/**
 * @brief Device descriptor for the MPU-9255 sensor
 */
typedef struct {
    i2c_t i2c_dev;              /**< I2C device which is used */
    uint8_t hw_addr;            /**< Hardware address of the MPU-9255 */
    uint8_t comp_addr;          /**< Address of the MPU-9255s compass */
    mpu9255_status_t conf;      /**< Device configuration */
} mpu9255_t;

/**
 * @brief Initialize the given MPU9255 device
 *
 * @param[out] dev          Initialized device descriptor of MPU9255 device
 * @param[in]  i2c          I2C bus the sensor is connected to
 * @param[in]  hw_addr      The device's address on the I2C bus
 * @param[in]  comp_addr    The compass address on the I2C bus
 *
 * @return                  0 on success
 * @return                  -1 if given I2C is not enabled in board config
 */
int mpu9255_init(mpu9255_t *dev, i2c_t i2c, mpu9255_hw_addr_t hw_addr,
        mpu9255_comp_addr_t comp_addr);

/**
 * @brief Enable or disable accelerometer power
 *
 * @param[in] dev           Device descriptor of MPU9255 device
 * @param[in] pwr_conf      Target power setting: PWR_ON or PWR_OFF
 *
 * @return                  0 on success
 * @return                  -1 if given I2C is not enabled in board config
 */
int mpu9255_set_accel_power(mpu9255_t *dev, mpu9255_pwr_t pwr_conf);

/**
 * @brief Enable or disable gyroscope power
 *
 * @param[in] dev           Device descriptor of MPU9255 device
 * @param[in] pwr_conf      Target power setting: PWR_ON or PWR_OFF
 *
 * @return                  0 on success
 * @return                  -1 if given I2C is not enabled in board config
 */
int mpu9255_set_gyro_power(mpu9255_t *dev, mpu9255_pwr_t pwr_conf);

/**
 * @brief Enable or disable compass power
 *
 * @param[in] dev           Device descriptor of MPU9255 device
 * @param[in] pwr_conf      Target power setting: PWR_ON or PWR_OFF
 *
 * @return                  0 on success
 * @return                  -1 if given I2C is not enabled in board config
 */
int mpu9255_set_compass_power(mpu9255_t *dev, mpu9255_pwr_t pwr_conf);

/**
 * @brief Read angular speed values from the given MPU9255 device, returned in dps
 *
 * The raw gyroscope data is read from the sensor and normalized with respect to
 * the configured gyroscope full-scale range.
 *
 * @param[in]  dev          Device descriptor of MPU9255 device to read from
 * @param[out] output       Result vector in dps per axis
 *
 * @return                  0 on success
 * @return                  -1 if device's I2C is not enabled in board config
 * @return                  -2 if gyro full-scale range is configured wrong
 */
int mpu9255_read_gyro(mpu9255_t *dev, mpu9255_results_t *output);

/**
 * @brief Read acceleration values from the given MPU9255 device, returned in mG
 *
 * The raw acceleration data is read from the sensor and normalized with respect to
 * the configured accelerometer full-scale range.
 *
 * @param[in]  dev          Device descriptor of MPU9255 device to read from
 * @param[out] output       Result vector in mG per axis
 *
 * @return                  0 on success
 * @return                  -1 if device's I2C is not enabled in board config
 * @return                  -2 if accel full-scale range is configured wrong
 */
int mpu9255_read_accel(mpu9255_t *dev, mpu9255_results_t *output);

/**
 * @brief Read magnetic field values from the given MPU9255 device, returned in mikroT
 *
 * The raw compass data is read from the sensor and normalized with respect to
 * the compass full-scale range (which can not be configured).
 *
 * @param[in]  dev          Device descriptor of MPU9255 device to read from
 * @param[out] output       Result vector in mikroT per axis
 *
 * @return                  0 on success
 * @return                  -1 if device's I2C is not enabled in board config
 */
int mpu9255_read_compass(mpu9255_t *dev, mpu9255_results_t *output);

/**
 * @brief Read temperature value from the given MPU9255 device, returned in m°C
 *
 * @note
 * The measured temperature is slightly higher than the real room temperature.
 * Tests showed that the offset varied around 2-3 °C (but no warranties here).
 *
 * @param[in] dev           Device descriptor of MPU9255 device to read from
 * @param[out] output       Temperature in m°C
 *
 * @return                  0 on success
 * @return                  -1 if device's I2C is not enabled in board config
 */
int mpu9255_read_temperature(mpu9255_t *dev, int32_t *output);

/**
 * @brief Set the full-scale range for raw gyroscope data
 *
 * @param[in] dev           Device descriptor of MPU9255 device
 * @param[in] fsr           Target full-scale range
 *
 * @return                  0 on success
 * @return                  -1 if device's I2C is not enabled in board config
 * @return                  -2 if given full-scale target value is not valid
 */
int mpu9255_set_gyro_fsr(mpu9255_t *dev, mpu9255_gyro_ranges_t fsr);

/**
 * @brief Set the full-scale range for raw accelerometer data
 *
 * @param[in] dev           Device descriptor of MPU9255 device
 * @param[in] fsr           Target full-scale range
 *
 * @return                  0 on success
 * @return                  -1 if device's I2C is not enabled in board config
 * @return                  -2 if given full-scale target value is not valid
 */
int mpu9255_set_accel_fsr(mpu9255_t *dev, mpu9255_accel_ranges_t fsr);

/**
 * @brief Set the rate at which the gyroscope and accelerometer data is sampled
 *
 * Sample rate can be chosen between 4 Hz and 1kHz. The actual set value might
 * slightly differ. If necessary, check the actual set value in the device's
 * config member afterwards.
 *
 * @param[in] dev           Device descriptor of MPU9255 device
 * @param[in] rate          Target sample rate in Hz
 *
 * @return                  0 on success
 * @return                  -1 if device's I2C is not enabled in board config
 * @return                  -2 if given target sample rate is not valid
 */
int mpu9255_set_sample_rate(mpu9255_t *dev, uint16_t rate);

/**
 * @brief Set the rate at which the compass data is sampled
 *
 * Sample rate can be chosen between 1 Hz and 100 Hz but has to be a fraction
 * of the configured accel/gyro sample rate. The actual set value might
 * slightly differ. If necessary, check the actual set value in the device's
 * config member afterwards.
 *
 * @param[in] dev           Device descriptor of MPU9255 device
 * @param[in] rate          Target sample rate in Hz
 *
 * @return                  0 on success
 * @return                  -1 if device's I2C is not enabled in board config
 * @return                  -2 if given target sample rate is not valid
 */
int mpu9255_set_compass_sample_rate(mpu9255_t *dev, uint8_t rate);

#ifdef __cplusplus
}
#endif

#endif /* MPU9255_H_ */
/** @} */
