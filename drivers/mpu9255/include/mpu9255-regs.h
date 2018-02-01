#define MPU9255_YG_OFFS_TC_REG          (0x01)

#define BIT_SLV0_DELAY_EN               (0x01)
#define BIT_SLV1_DELAY_EN               (0x02)
#define BIT_I2C_BYPASS_EN               (0x02)
#define BIT_I2C_MST_EN                  (0x20)
#define BIT_PWR_MGMT1_SLEEP             (0x40)
#define BIT_WAIT_FOR_ES                 (0x40)
#define BIT_I2C_MST_VDDIO               (0x80)
#define BIT_SLAVE_RW                    (0x80)
#define BIT_SLAVE_EN                    (0x80)
#define BIT_DMP_EN                      (0x80)

#define MPU9255_SELF_TEST_X_GYRO	(0x00)
#define MPU9255_SELF_TEST_Y_GYRO	(0x01)
#define MPU9255_SELF_TEST_Z_GYRO	(0x02)
#define MPU9255_SELF_TEST_X_ACCEL	(0x0D)
#define MPU9255_SELF_TEST_Y_ACCEL	(0x0E)
#define MPU9255_SELF_TEST_Z_ACCEL	(0x0F)
#define MPU9255_XG_OFFSET_H		(0x13)
#define MPU9255_XG_OFFSET_L		(0x14)
#define MPU9255_YG_OFFSET_H		(0x15)
#define MPU9255_YG_OFFSET_L		(0x16)
#define MPU9255_ZG_OFFSET_H		(0x17)
#define MPU9255_ZG_OFFSET_L		(0x18)
#define MPU9255_RATE_DIV_REG		(0x19)
#define MPU9255_CONFIG_REG		(0x1A)
#define MPU9255_GYRO_CFG_REG		(0x1B)
#define MPU9255_ACCEL_CFG_REG		(0x1C)
#define MPU9255_ACCEL_CONFIG2_REG	(0x1D)

#define MPU9255_LP_ACCEL_ODR		(0x1E)
#define MPU9255_WOM_THR			(0x1F)
#define MPU9255_FIFO_EN_REG		(0x23)
#define MPU9255_I2C_MST_REG		(0x24)
#define MPU9255_SLAVE0_ADDR_REG		(0x25)
#define MPU9255_SLAVE0_REG_REG		(0x26)
#define MPU9255_SLAVE0_CTRL_REG		(0x27)
#define MPU9255_SLAVE1_ADDR_REG		(0x28)
#define MPU9255_SLAVE1_REG_REG		(0x29)
#define MPU9255_SLAVE1_CTRL_REG		(0x2A)
#define MPU9255_SLAVE2_ADDR		(0x2B)
#define MPU9255_SLAVE2_REG		(0x2C)
#define MPU9255_SLAVE2_CTRL		(0x2D)
#define MPU9255_SLAVE3_ADDR		(0x2E)
#define MPU9255_SLAVE3_REG		(0x2F)
#define MPU9255_SLAVE3_CTRL		(0x30)
#define MPU9255_SLAVE4_ADDR		(0x31)
#define MPU9255_SLAVE4_REG		(0x32)
#define MPU9255_SLAVE4_DATA_OUT_REG	(0x33)
#define MPU9255_SLAVE4_CTRL_REG		(0x34)
#define MPU9255_SLAVE4_DI		(0x35)
#define MPU9255_I2C_MST_STATUS		(0x36)
#define MPU9255_INT_PIN_CFG_REG		(0x37)
#define MPU9255_INT_ENABLE_REG		(0x38)
#define MPU9255_INT_STATUS		(0x3A)
#define MPU9255_ACCEL_START_REG		(0x3B)
#define MPU9255_ACCEL_XOUT_L		(0x3C)
#define MPU9255_ACCEL_YOUT_H		(0x3D)
#define MPU9255_ACCEL_YOUT_L		(0x3E)
#define MPU9255_ACCEL_ZOUT_H		(0x3F)
#define MPU9255_ACCEL_ZOUT_L		(0x40)
#define MPU9255_TEMP_START_REG		(0x41)
#define MPU9255_TEMP_OUT_L		(0x42)
#define MPU9255_GYRO_START_REG		(0x43)
#define MPU9255_GYRO_XOUT_L		(0x44)
#define MPU9255_GYRO_YOUT_H		(0x45)
#define MPU9255_GYRO_YOUT_L		(0x46)
#define MPU9255_GYRO_ZOUT_H		(0x47)
#define MPU9255_GYRO_ZOUT_L		(0x48)
#define MPU9255_EXT_SENS_DATA_START_REG	(0x49)
#define MPU9255_EXT_SENS_DATA_01	(0x4A)
#define MPU9255_EXT_SENS_DATA_02	(0x4B)
#define MPU9255_EXT_SENS_DATA_03	(0x4C)
#define MPU9255_EXT_SENS_DATA_04	(0x4D)
#define MPU9255_EXT_SENS_DATA_05	(0x4E)
#define MPU9255_EXT_SENS_DATA_06	(0x4F)
#define MPU9255_EXT_SENS_DATA_07	(0x50)
#define MPU9255_EXT_SENS_DATA_08	(0x51)
#define MPU9255_EXT_SENS_DATA_09	(0x52)
#define MPU9255_EXT_SENS_DATA_10	(0x53)
#define MPU9255_EXT_SENS_DATA_11	(0x54)
#define MPU9255_EXT_SENS_DATA_12	(0x55)
#define MPU9255_EXT_SENS_DATA_13	(0x56)
#define MPU9255_EXT_SENS_DATA_14	(0x57)
#define MPU9255_EXT_SENS_DATA_15	(0x58)
#define MPU9255_EXT_SENS_DATA_16	(0x59)
#define MPU9255_EXT_SENS_DATA_17	(0x5A)
#define MPU9255_EXT_SENS_DATA_18	(0x5B)
#define MPU9255_EXT_SENS_DATA_19	(0x5C)
#define MPU9255_EXT_SENS_DATA_20	(0x5D)
#define MPU9255_EXT_SENS_DATA_21	(0x5E)
#define MPU9255_EXT_SENS_DATA_22	(0x5F)
#define MPU9255_EXT_SENS_DATA_23	(0x60)
#define MPU9255_SLAVE0_DATA_OUT_REG		(0x63)
#define MPU9255_SLAVE1_DATA_OUT_REG		(0x64)
#define MPU9255_SLAVE2_DATA_OUT_REG		(0x65)
#define MPU9255_SLAVE3_DATA_OUT_REG		(0x66)
#define MPU9255_I2C_DELAY_CTRL_REG	(0x67)
#define MPU9255_SIGNAL_PATH_RESET	(0x68)
#define MPU9255_MOT_DETECT_CTRL		(0x69)
#define MPU9255_USER_CTRL_REG		(0x6A)
#define MPU9255_PWR_MGMT_1_REG		(0x6B)
#define MPU9255_PWR_MGMT_2_REG		(0x6C)
#define MPU9255_FIFO_COUNTH		(0x72)
#define MPU9255_FIFO_COUNTL		(0x73)
#define MPU9255_FIFO_R_W		(0x74)
#define MPU9255_WHO_AM_I_REG		(0x75)
#define MPU9255_XA_OFFSET_H		(0x77)
#define MPU9255_XA_OFFSET_L		(0x78)
#define MPU9255_YA_OFFSET_H		(0x7A)
#define MPU9255_YA_OFFSET_L		(0x7B)
#define MPU9255_ZA_OFFSET_H		(0x7D)
#define MPU9255_ZA_OFFSET_L		(0x7E)

// AK8963 magnetometer
#define COMPASS_WHOAMI_REG              (0x00)
#define COMPASS_ST1_REG                 (0x02)
#define COMPASS_DATA_START_REG          (0x03)
#define COMPASS_ST2_REG                 (0x09)
#define COMPASS_CNTL_REG                (0x0A)
#define COMPASS_ASTC_REG                (0x0C)
#define COMPASS_ASAX_REG                (0x10)
#define COMPASS_ASAY_REG                (0x11)
#define COMPASS_ASAZ_REG                (0x12)
