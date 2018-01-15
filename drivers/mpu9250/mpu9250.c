#include <math.h>
#include "mpu9250.h"
#include "include/mpu9250-regs.h"
#include "periph/i2c.h"
#include "xtimer.h"

#define ENABLE_DEBUG        (0)
#include "debug.h"

#define MPU9250_NO_INTERRUPTS           (0x00)
#define MPU9250_DISABLE_FIFO            (0x00)
#define MPU9250_FIFO_GYRO_ACCEL         (0x78)

#define REG_RESET           (0x00)
#define MAX_VALUE           (0x7FFF)

/* Default config settings */
static const mpu9250_status_t DEFAULT_STATUS = {
    .accel_pwr = MPU9250_SENSOR_PWR_ON,
    .gyro_pwr = MPU9250_SENSOR_PWR_ON,
    .compass_pwr = MPU9250_SENSOR_PWR_ON,
    .gyro_fsr = MPU9250_GYRO_FSR_2000DPS,
    .accel_fsr = MPU9250_ACCEL_FSR_16G,
    .gyro_lpf = MPU9250_GYRO_LPF_41,
    .accel_lpf = MPU9250_ACCEL_LPF_41,
    .sample_rate = 80,
    .compass_sample_rate = 0,
    .compass_x_adj = 0,
    .compass_y_adj = 0,
    .compass_z_adj = 0,
};

float gyroBias[3] = { 0, 0, 0 };
uint32_t accelBias[3] = { 0, 0, 0 };

/* Internal function prototypes */
static int compass_init(mpu9250_t *dev);
static void conf_bypass(mpu9250_t *dev, uint8_t bypass_enable);
bool mpu9250_selfTest(mpu9250_t *dev);

/*---------------------------------------------------------------------------*
 *                          MPU9250 Core API                                 *
 *---------------------------------------------------------------------------*/

bool mpu9250_calibrate(mpu9250_t *dev, float *gyroBias, uint32_t *accelBias)
{
    i2c_acquire(dev->i2c_dev);

    /* code here shamelessy based on SparkFun MPU-9250 Arduino library */
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_1_REG, MPU9250_PWR_RESET);
    xtimer_usleep(MPU9250_RESET_SLEEP_US);

    // get stable time source; Auto select clock source to be PLL gyroscope
    // reference if ready else use the internal oscillator, bits 2:0 = 001
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_1_REG, MPU9250_PWR_PLL);  
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_2_REG, MPU9250_PWR_WAKEUP);
    xtimer_usleep(2 * MPU9250_RESET_SLEEP_US);

    // Configure device for bias calculation
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_INT_ENABLE_REG, MPU9250_NO_INTERRUPTS);   // Disable all interrupts
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_FIFO_EN_REG, MPU9250_DISABLE_FIFO);      // Disable FIFO
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_1_REG, 0x00);   // Turn on internal clock source
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_I2C_MST_REG, 0x00); // Disable I2C master
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_USER_CTRL_REG, 0x00);    // Disable FIFO and I2C master modes
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_USER_CTRL_REG, 0x0C);    // Reset FIFO and DMP
    xtimer_usleep(15000);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_CONFIG_REG, G_DLPF_SAMPLE_RATE_1KHZ_BW_184);    // Set low-pass filter to 184 Hz
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_RATE_DIV_REG, 0x00);  // Set sample rate to 1 kHz
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_GYRO_CFG_REG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_ACCEL_CFG_REG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity


    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_USER_CTRL_REG, 0x40);   // Enable FIFO  
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_FIFO_EN_REG, MPU9250_FIFO_GYRO_ACCEL);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    xtimer_usleep(40000); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_FIFO_EN_REG, MPU9250_DISABLE_FIFO);        // Disable gyro and accelerometer sensors for FIFO

    char data[12];
    i2c_read_reg(dev->i2c_dev, dev->hw_addr, MPU9250_FIFO_COUNTH, data); // read FIFO sample count
    uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
    uint16_t packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    int32_t gyro_bias[3]  = {0, 0, 0};
    int32_t accel_bias[3] = {0, 0, 0};
    for (int i = 0; i < packet_count; i++)
    {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        i2c_read_reg(dev->i2c_dev, dev->hw_addr, MPU9250_FIFO_R_W,  data); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );

        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];

        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }

    // get avg bias (as measured)
    accel_bias[0] /= (int32_t) packet_count;
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;

    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    // this assumes device is oriented with Z down...
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g for 2G FSR
    if(accel_bias[2] > 0L)
    {
        accel_bias[2] -= (int32_t) accelsensitivity;  // Remove gravity from the z-axis accelerometer bias calculation
    }
    else
    {
        accel_bias[2] += (int32_t) accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    // // weird issue with gyro bias - need to resolve before enabling this...
/*    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_XG_OFFSET_H, data[0]);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_XG_OFFSET_L, data[1]);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_YG_OFFSET_H, data[2]);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_YG_OFFSET_L, data[3]);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_ZG_OFFSET_H, data[4]);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_ZG_OFFSET_L, data[5]);
*/

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    // Output scaled gyro biases for display in the main program
    gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
    gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases

    i2c_read_reg(dev->i2c_dev, dev->hw_addr, MPU9250_XA_OFFSET_H, data); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1] >> 1);

    i2c_read_reg(dev->i2c_dev, dev->hw_addr, MPU9250_YA_OFFSET_H, data);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1] >> 1);

    i2c_read_reg(dev->i2c_dev, dev->hw_addr, MPU9250_ZA_OFFSET_H, data);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1] >> 1);


    /*uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
    for(int i = 0; i < 3; i++)
    {
        if((accel_bias_reg[i] & mask))
            mask_bit[i] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }*/
printf(" ab: %"SCNd32 ", %"SCNd32 ", %"SCNd32 "\n", accel_bias_reg[0], accel_bias_reg[1], accel_bias_reg[2]);    
printf("mab: %"SCNd32 ", %"SCNd32 ", %"SCNd32 "\n", accel_bias[0], accel_bias[1], accel_bias[2]);    

    // Construct total accelerometer bias, (including calculated average
    // accelerometer bias from above).  NB Scale measured bias to 2048 LSB/g
    // (from 16384 by dividing by 8 (we measured the accels in 2G=16384 LSB/g)
    // because the offset regs use 16 g full scale
    accel_bias_reg[0] -= (accel_bias[0]/8);
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);
printf("nab: %"SCNd32 ", %"SCNd32 ", %"SCNd32 "\n\n", accel_bias_reg[0], accel_bias_reg[1], accel_bias_reg[2]);    

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;

    // preserve temperature compensation bit when writing back to accelerometer bias registers (according to
    // register map doc the offsets don't hold temp compensate?? JK) 
    //data[1] = data[1] | mask_bit[0];
    //data[3] = data[3] | mask_bit[1];
    //data[5] = data[5] | mask_bit[2];

    // Apparently this is not working for the acceleration biases in the MPU-9250
    // Are we handling the temperature correction bit properly? (according to
    // register map doc the offsets don't hold temp compensate?? JK) Push
    // accelerometer biases to hardware registers
    // weird issue with accel bias - need to resolve before enabling this...
    /*i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_XA_OFFSET_H, data[0]);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_XA_OFFSET_L, data[1] << 1);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_YA_OFFSET_H, data[2]);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_YA_OFFSET_L, data[3 << 1]);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_ZA_OFFSET_H, data[4]);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_ZA_OFFSET_L, data[5 << 1]);*/

    i2c_release(dev->i2c_dev);

    accelBias[0] = accel_bias[0]; 
    accelBias[1] = accel_bias[1];
    accelBias[2] = accel_bias[2];

    // Output scaled accelerometer biases for display in the main program
    /*accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity; 
    accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
    accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;*/
    return true;
}

int mpu9250_init(mpu9250_t *dev, i2c_t i2c, mpu9250_hw_addr_t hw_addr,
        mpu9250_comp_addr_t comp_addr)
{
    dev->i2c_dev = i2c;
    dev->hw_addr = hw_addr;
    dev->comp_addr = comp_addr;
    dev->conf = DEFAULT_STATUS;

    /* Initialize I2C interface */
    if (i2c_init_master(dev->i2c_dev, I2C_SPEED_FAST)) {
        DEBUG("[Error] I2C device not enabled\n");
        return -1;
    }

    /* Acquire exclusive access */
    i2c_acquire(dev->i2c_dev);

    /* Reset MPU9250 registers and afterwards wake up the chip */
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_1_REG, MPU9250_PWR_RESET);
    xtimer_usleep(MPU9250_RESET_SLEEP_US);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_1_REG, MPU9250_PWR_WAKEUP);


    /* Check whether MPU answers correctly */
    char data[1];
    i2c_read_reg(dev->i2c_dev, dev->hw_addr, MPU9250_WHO_AM_I_REG, data);
    if (data[0] != MPU9250_WHOAMI_ANSWER) {
        DEBUG("[Error] Wrong answer from MPU\n");
        printf("got 0x%x instead of 0x%x\n", data[0], MPU9250_WHOAMI_ANSWER);
        i2c_release(dev->i2c_dev);
        return -1;
    }

    /* Release the bus, it is acquired again inside each function */
    i2c_release(dev->i2c_dev);

    if (! mpu9250_selfTest(dev))
    {
        puts("MPU9250 failed self test");
        return -1;
    }

    mpu9250_calibrate(dev, gyroBias, accelBias);
    printf("gyro bias: %f %f %f\naccel bias: %"SCNd32 " %"SCNd32 " %"SCNd32 "\n", gyroBias[0], gyroBias[1], gyroBias[2], accelBias[0], accelBias[1], accelBias[2]);

    /* Initialize magnetometer */
    if (compass_init(dev)) {
        return -2;
    }
    mpu9250_set_compass_sample_rate(dev, 40);

    /* Enable all sensors */
    i2c_acquire(dev->i2c_dev);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_1_REG, MPU9250_PWR_PLL);
    char temp;
    i2c_read_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_2_REG, &temp);
    temp &= ~(MPU9250_PWR_ACCEL | MPU9250_PWR_GYRO);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_2_REG, temp);
    i2c_release(dev->i2c_dev);
    xtimer_usleep(MPU9250_PWR_CHANGE_SLEEP_US);

    return 0;
}

int16_t convertRawToS16BE(uint8_t *data)
{
    int16_t v = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    return v;
}

int16_t convertRawToS16LE(char *data)
{
    int16_t v = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    return v;
}


bool mpu9250_selfTest(mpu9250_t *dev)
{
    /* Acquire exclusive access */
    if (i2c_acquire(dev->i2c_dev)) {
        return false;
    }

    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_RATE_DIV_REG, 0x00);    // Set gyro sample rate to 1 kHz
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_CONFIG_REG, G_DLPF_SAMPLE_RATE_1KHZ_BW_92);    // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_GYRO_CFG_REG, 1 << MPU9250_GYRO_FSR_250DPS);    // Set full scale range for the gyro to 250 dps
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_ACCEL_CONFIG2_REG, A_DLPF_SAMPLE_RATE_1KHZ_BW_92);   // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_ACCEL_CFG_REG, 1 << MPU9250_ACCEL_FSR_2G);      // Set full scale range for the accelerometer to 2 g

    // accumulate a sample of gyro and acclerometer readings
    uint8_t data[6];
    int16_t gAvg[3] = { 0, 0, 0 };
    int16_t aAvg[3] = { 0, 0, 0 };
    uint8_t maxCount = 200;
    for(uint8_t i = 0; i < maxCount; i++)
    {  
        // accelerometer
        i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU9250_ACCEL_START_REG, data, 6);
        aAvg[0] += convertRawToS16BE(data);
        aAvg[1] += convertRawToS16BE(data+2);
        aAvg[2] += convertRawToS16BE(data+4);

        // gyro
        i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU9250_GYRO_START_REG, data, 6);
        gAvg[0] += convertRawToS16BE(data);
        gAvg[1] += convertRawToS16BE(data+2);
        gAvg[2] += convertRawToS16BE(data+4);
    }

    for (uint8_t i =0; i < 3; i++)  // calculate average of values
    {
        aAvg[i] /= maxCount;
        gAvg[i] /= maxCount;
    }

    int16_t aSTAvg[3] = { 0, 0, 0 };
    int16_t gSTAvg[3] = { 0, 0, 0 };
    // Configure the accelerometer for self-test
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_ACCEL_CFG_REG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_GYRO_CFG_REG, 0xE0);  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    i2c_release(dev->i2c_dev);
    xtimer_usleep(25000);
    if (i2c_acquire(dev->i2c_dev)) {
        return false;
    }

    for( int i = 0; i < maxCount; i++) {  // get average self-test values of gyro and acclerometer
        // accelerometer
        i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU9250_ACCEL_START_REG, data, 6);
        aSTAvg[0] += convertRawToS16BE(data);
        aSTAvg[1] += convertRawToS16BE(data+2);
        aSTAvg[2] += convertRawToS16BE(data+4);

        // gyro
        i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU9250_GYRO_START_REG, data, 6);
        gSTAvg[0] += convertRawToS16BE(data);
        gSTAvg[1] += convertRawToS16BE(data+2);
        gSTAvg[2] += convertRawToS16BE(data+4);
    }

    for (int i = 0; i < 3; i++) {
        aSTAvg[i] /= maxCount;
        gSTAvg[i] /= maxCount;
    }   

    // Configure the gyro and accelerometer for normal operation
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_GYRO_CFG_REG, 0x00);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_ACCEL_CFG_REG, 0x00);
    i2c_release(dev->i2c_dev);
    xtimer_usleep(25000);
    if (i2c_acquire(dev->i2c_dev)) {
        return false;
    }

    char selfTest[6];
    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU9250_SELF_TEST_X_ACCEL, selfTest, 1);
    i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU9250_SELF_TEST_Y_ACCEL, selfTest+1, 1);
    i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU9250_SELF_TEST_Z_ACCEL, selfTest+2, 1);

    i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU9250_SELF_TEST_X_GYRO, selfTest+3, 1);
    i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU9250_SELF_TEST_Y_GYRO, selfTest+4, 1);
    i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU9250_SELF_TEST_Z_GYRO, selfTest+5, 1);

    i2c_release(dev->i2c_dev);

    // Retrieve factory self-test value from self-test code reads
    float aFactor = 2620/(1 << MPU9250_ACCEL_FSR_2G);
    float gFactor = 2620/(1 << MPU9250_GYRO_FSR_250DPS);
    float factoryTrim[6];
    factoryTrim[0] = aFactor * pow(1.01 , ((float)selfTest[0] - 1.0)); // Xa factory trim calculation
    factoryTrim[1] = aFactor * pow(1.01 , ((float)selfTest[1] - 1.0)); // Ya factory trim calculation
    factoryTrim[2] = aFactor * pow(1.01 , ((float)selfTest[2] - 1.0)); // Za factory trim calculation
    factoryTrim[3] = gFactor * pow(1.01 , ((float)selfTest[3] - 1.0)); // Xg factory trim calculation
    factoryTrim[4] = gFactor * pow(1.01 , ((float)selfTest[4] - 1.0)); // Yg factory trim calculation
    factoryTrim[5] = gFactor * pow(1.01 , ((float)selfTest[5] - 1.0)); // Zg factory trim calculation

    // calculate ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    float destination[6];
    for (int i = 0; i < 3; i++)
    {
        destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // calculate percent differences
        destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // calculate percent differences
    }
    printf("x-axis self test: acceleration trim within: %4.2f%% of factory value\n", destination[0]);
    printf("y-axis self test: acceleration trim within: %4.2f%% of factory value\n", destination[1]);
    printf("z-axis self test: acceleration trim within: %4.2f%% of factory value\n\n", destination[2]);
    printf("x-axis self test: gyro trim within: %4.2f%% of factory value\n", destination[3]);
    printf("y-axis self test: gyro trim within: %4.2f%% of factory value\n", destination[4]);
    printf("z-axis self test: gyro trim within: %4.2f%% of factory value\n", destination[5]);
    return true;
}

int mpu9250_set_accel_power(mpu9250_t *dev, mpu9250_pwr_t pwr_conf)
{
    uint8_t pwr_1_setting, pwr_2_setting;

    if (dev->conf.accel_pwr == pwr_conf) {
        return 0;
    }

    /* Acquire exclusive access */
    if (i2c_acquire(dev->i2c_dev)) {
        return -1;
    }

    /* Read current power management 2 configuration */
    i2c_read_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_2_REG, &pwr_2_setting);
    /* Prepare power register settings */
    if (pwr_conf == MPU9250_SENSOR_PWR_ON) {
        pwr_1_setting = MPU9250_PWR_WAKEUP;
        pwr_2_setting &= ~(MPU9250_PWR_ACCEL);
    }
    else {
        pwr_1_setting = BIT_PWR_MGMT1_SLEEP;
        pwr_2_setting |= MPU9250_PWR_ACCEL;
    }
    /* Configure power management 1 register if needed */
    if ((dev->conf.gyro_pwr == MPU9250_SENSOR_PWR_OFF)
            && (dev->conf.compass_pwr == MPU9250_SENSOR_PWR_OFF)) {
        i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_1_REG, pwr_1_setting);
    }
    /* Enable/disable accelerometer standby in power management 2 register */
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_2_REG, pwr_2_setting);

    /* Release the bus */
    i2c_release(dev->i2c_dev);

    dev->conf.accel_pwr = pwr_conf;
    xtimer_usleep(MPU9250_PWR_CHANGE_SLEEP_US);

    return 0;
}

int mpu9250_set_gyro_power(mpu9250_t *dev, mpu9250_pwr_t pwr_conf)
{
    uint8_t pwr_2_setting;

    if (dev->conf.gyro_pwr == pwr_conf) {
        return 0;
    }

    /* Acquire exclusive access */
    if (i2c_acquire(dev->i2c_dev)) {
        return -1;
    }

    /* Read current power management 2 configuration */
    i2c_read_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_2_REG, &pwr_2_setting);
    /* Prepare power register settings */
    if (pwr_conf == MPU9250_SENSOR_PWR_ON) {
        /* Set clock to pll */
        i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_1_REG, MPU9250_PWR_PLL);
        pwr_2_setting &= ~(MPU9250_PWR_GYRO);
    }
    else {
        /* Configure power management 1 register */
        if ((dev->conf.accel_pwr == MPU9250_SENSOR_PWR_OFF)
                && (dev->conf.compass_pwr == MPU9250_SENSOR_PWR_OFF)) {
            /* All sensors turned off, put the MPU-9250 to sleep */
            i2c_write_reg(dev->i2c_dev, dev->hw_addr,
                    MPU9250_PWR_MGMT_1_REG, BIT_PWR_MGMT1_SLEEP);
        }
        else {
            /* Reset clock to internal oscillator */
            i2c_write_reg(dev->i2c_dev, dev->hw_addr,
                    MPU9250_PWR_MGMT_1_REG, MPU9250_PWR_WAKEUP);
        }
        pwr_2_setting |= MPU9250_PWR_GYRO;
    }
    /* Enable/disable gyroscope standby in power management 2 register */
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_2_REG, pwr_2_setting);

    /* Release the bus */
    i2c_release(dev->i2c_dev);

    dev->conf.gyro_pwr = pwr_conf;
    xtimer_usleep(MPU9250_PWR_CHANGE_SLEEP_US);

    return 0;
}

int mpu9250_set_compass_power(mpu9250_t *dev, mpu9250_pwr_t pwr_conf)
{
puts("compass power not available");
return 0;
    char pwr_1_setting, usr_ctrl_setting, s1_do_setting;

    if (dev->conf.compass_pwr == pwr_conf) {
        return 0;
    }

    /* Acquire exclusive access */
    if (i2c_acquire(dev->i2c_dev)) {
        return -1;
    }

    /* Read current user control configuration */
    i2c_read_reg(dev->i2c_dev, dev->hw_addr, MPU9250_USER_CTRL_REG, &usr_ctrl_setting);
    /* Prepare power register settings */
    if (pwr_conf == MPU9250_SENSOR_PWR_ON) {
        pwr_1_setting = MPU9250_PWR_WAKEUP;
        s1_do_setting = MPU9250_COMP_SINGLE_MEASURE_MODE;
        usr_ctrl_setting |= BIT_I2C_MST_EN;
    }
    else {
        pwr_1_setting = BIT_PWR_MGMT1_SLEEP;
        s1_do_setting = MPU9250_COMP_POWER_DOWN_MODE;
        usr_ctrl_setting &= ~(BIT_I2C_MST_EN);
    }
    /* Configure power management 1 register if needed */
    if ((dev->conf.gyro_pwr == MPU9250_SENSOR_PWR_OFF)
            && (dev->conf.accel_pwr == MPU9250_SENSOR_PWR_OFF)) {
        i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_PWR_MGMT_1_REG, pwr_1_setting);
    }
    /* Configure mode writing by slave line 1 */
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_SLAVE1_DATA_OUT_REG, s1_do_setting);
    /* Enable/disable I2C master mode */
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_USER_CTRL_REG, usr_ctrl_setting);

    /* Release the bus */
    i2c_release(dev->i2c_dev);

    dev->conf.compass_pwr = pwr_conf;
    xtimer_usleep(MPU9250_PWR_CHANGE_SLEEP_US);

    return 0;
}

int mpu9250_read_gyro(mpu9250_t *dev, mpu9250_results_t *output)
{
    float fsr;

    switch (dev->conf.gyro_fsr) {
        case MPU9250_GYRO_FSR_250DPS:
            fsr = 250.0;
            break;
        case MPU9250_GYRO_FSR_500DPS:
            fsr = 500.0;
            break;
        case MPU9250_GYRO_FSR_1000DPS:
            fsr = 1000.0;
            break;
        case MPU9250_GYRO_FSR_2000DPS:
            fsr = 2000.0;
            break;
        default:
            return -2;
    }

    /* Acquire exclusive access */
    if (i2c_acquire(dev->i2c_dev)) {
        return -1;
    }
    /* Read raw data */
    uint8_t data[6];
    i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU9250_GYRO_START_REG, data, 6);
    /* Release the bus */
    i2c_release(dev->i2c_dev);

    float scale = fsr / MAX_VALUE;

    /* Normalize data according to configured full scale range */
    int16_t temp;
    temp = convertRawToS16BE(data);
    output->x_axis = temp * scale;
    temp = convertRawToS16BE(data+2);
    output->y_axis = temp * scale;
    temp = convertRawToS16BE(data+4);
    output->z_axis = temp * scale;

    return 0;
}

int mpu9250_read_accel(mpu9250_t *dev, mpu9250_results_t *output)
{
    float fsr;

    switch (dev->conf.accel_fsr) {
        case MPU9250_ACCEL_FSR_2G:
            fsr = 2000.0;
            break;
        case MPU9250_ACCEL_FSR_4G:
            fsr = 4000.0;
            break;
        case MPU9250_ACCEL_FSR_8G:
            fsr = 8000.0;
            break;
        case MPU9250_ACCEL_FSR_16G:
            fsr = 16000.0;
            break;
        default:
            return -2;
    }

    /* Acquire exclusive access */
    if (i2c_acquire(dev->i2c_dev)) {
        return -1;
    }
    /* Read raw data */
    uint8_t data[6];
    i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU9250_ACCEL_START_REG, data, 6);
    /* Release the bus */
    i2c_release(dev->i2c_dev);

    float scale = fsr / MAX_VALUE;

    /* Normalize data according to configured full scale range */
    int16_t temp;
    temp = convertRawToS16BE(data);
    output->x_axis = temp * scale;
    temp = convertRawToS16BE(data+2);
    output->y_axis = temp * scale;
    temp = convertRawToS16BE(data+4);
    output->z_axis = temp * scale;

    return 0;
}


int16_t magAdjust(int16_t magRaw, int16_t asa)
{
    return magRaw * (((asa-128.0) / 256.0) + 1);
}

int mpu9250_read_compass(mpu9250_t *dev, mpu9250_results_t *output)
{
    /* Acquire exclusive access */
    if (i2c_acquire(dev->i2c_dev)) {
        puts("could not aq dev");
        return -1;
    }
    conf_bypass(dev, 1);
    uint8_t mRes = 1;
    uint8_t mMode = MPU9250_COMP_SINGLE_MEASURE_MODE;
    uint8_t mode = mRes << 4 | mMode;
    i2c_write_reg(dev->i2c_dev, dev->comp_addr, COMPASS_CNTL_REG, mode);


    char data[8]; // data[0] = ST1 reg, data[1..6] = mag data and data[7] = ST2 reg
    i2c_read_regs(dev->i2c_dev, dev->comp_addr, COMPASS_ST1_REG, data, 8);

    conf_bypass(dev, 0);
    i2c_release(dev->i2c_dev);

    // ready flag (st1 & 0x01 0) not relevant in continuous mode
    bool ready = data[0] & 0x01;
    bool overrun = data[0] & 0x02;
    bool overflow = data[7] & 0x08;

    if (overrun)
    {
        return mpu9250_read_compass(dev, output);
    }

    if (overflow)
    {
        return mpu9250_read_compass(dev, output);
    }

    if (ready)
    {

        output->x_axis = convertRawToS16LE(data+1);
        output->y_axis = convertRawToS16LE(data+3);
        output->z_axis = convertRawToS16LE(data+5);

        /* Compute sensitivity adjustment */
        output->x_axis = magAdjust(output->x_axis, dev->conf.compass_x_adj);
        output->y_axis = magAdjust(output->y_axis, dev->conf.compass_y_adj);
        output->z_axis = magAdjust(output->z_axis, dev->conf.compass_z_adj);
    }
    else
    {
        puts("compass not rdy");
        return -1;
    }

    return 0;
}

int mpu9250_read_temperature(mpu9250_t *dev, int32_t *output)
{
    uint8_t data[2];
    int16_t temp;

    /* Acquire exclusive access */
    if (i2c_acquire(dev->i2c_dev)) {
        return -1;
    }
    /* Read raw temperature value */
    i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU9250_TEMP_START_REG, data, 2);
    /* Release the bus */
    i2c_release(dev->i2c_dev);

    temp = convertRawToS16BE(data);
    *output = ((((int32_t)temp) * 1000) / 340) + (35*1000);

    return 0;
}

int mpu9250_set_gyro_fsr(mpu9250_t *dev, mpu9250_gyro_ranges_t fsr)
{
    if (dev->conf.gyro_fsr == fsr) {
        return 0;
    }

    switch (fsr) {
        case MPU9250_GYRO_FSR_250DPS:
        case MPU9250_GYRO_FSR_500DPS:
        case MPU9250_GYRO_FSR_1000DPS:
        case MPU9250_GYRO_FSR_2000DPS:
            if (i2c_acquire(dev->i2c_dev)) {
                return -1;
            }
            i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_GYRO_CFG_REG, (char)(fsr << 3));
            i2c_release(dev->i2c_dev);
            dev->conf.gyro_fsr = fsr;
            break;
        default:
            return -2;
    }

    return 0;
}

int mpu9250_set_accel_fsr(mpu9250_t *dev, mpu9250_accel_ranges_t fsr)
{
    if (dev->conf.accel_fsr == fsr) {
        return 0;
    }

    switch (fsr) {
        case MPU9250_ACCEL_FSR_2G:
        case MPU9250_ACCEL_FSR_4G:
        case MPU9250_ACCEL_FSR_8G:
        case MPU9250_ACCEL_FSR_16G:
            if (i2c_acquire(dev->i2c_dev)) {
                return -1;
            }
            i2c_write_reg(dev->i2c_dev, dev->hw_addr,
                    MPU9250_ACCEL_CFG_REG, (char)(fsr << 3));
            i2c_release(dev->i2c_dev);
            dev->conf.accel_fsr = fsr;
            break;
        default:
            return -2;
    }

    return 0;
}

bool setAccelLPF(mpu9250_t *dev, mpu9250_accel_ranges_t fsr, mpu9250_accel_lpf_t lpf)
{
    unsigned char accelConfig = fsr + ((lpf >> 3) & 3);
    unsigned char accelLpf = lpf & 7;

    if (i2c_acquire(dev->i2c_dev)) 
    {
	return false;
    }
    if (! i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_ACCEL_CFG_REG, (char)(accelConfig << 3)))
    {
	return false;
    }
    if (! i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_ACCEL_CONFIG2_REG, (char)(accelLpf << 3)))
    {
	return false;
    }
    i2c_release(dev->i2c_dev);
    dev->conf.accel_fsr = fsr;
    dev->conf.accel_lpf = lpf;
    return true;

}


bool setGyroLPF(mpu9250_t *dev, mpu9250_gyro_ranges_t fsr, mpu9250_gyro_lpf_t lpf)
{
    unsigned char gyroConfig = fsr + ((lpf >> 3) & 3);
    unsigned char gyroLpf = lpf & 7;

    if (i2c_acquire(dev->i2c_dev)) 
    {
	return false;
    }
    if (! i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_GYRO_CFG_REG, (char)(gyroConfig << 3)))
    {
	return false;
    }
    if (! i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_CONFIG_REG, (char)(gyroLpf << 3)))
    {
	return false;
    }
    i2c_release(dev->i2c_dev);
    dev->conf.gyro_fsr = fsr;
    dev->conf.gyro_lpf = lpf;
    return true;

}

int mpu9250_set_sample_rate(mpu9250_t *dev, uint16_t rate)
{
    uint8_t divider;

    if ((rate < MPU9250_MIN_SAMPLE_RATE) || (rate > MPU9250_MAX_SAMPLE_RATE)) {
        return -2;
    }
    else if (dev->conf.sample_rate == rate) {
        return 0;
    }

    /* Compute divider to achieve desired sample rate and write to rate div register */
    divider = (1000 / rate - 1);

    if (i2c_acquire(dev->i2c_dev)) {
        return -1;
    }
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_RATE_DIV_REG, (char) divider);
    i2c_release(dev->i2c_dev);

    /* Store configured sample rate */
    dev->conf.sample_rate = 1000 / (((uint16_t) divider) + 1);

    setGyroLPF(dev, dev->conf.gyro_fsr, dev->conf.gyro_lpf);
    setAccelLPF(dev, dev->conf.accel_fsr, dev->conf.accel_lpf);


    return 0;
}

int mpu9250_set_compass_sample_rate(mpu9250_t *dev, uint8_t rate)
{
    uint8_t divider;

    if ((rate < MPU9250_MIN_COMP_SMPL_RATE) || (rate > MPU9250_MAX_COMP_SMPL_RATE)
            || (rate > dev->conf.sample_rate)) {
        return -2;
    }
    else if (dev->conf.compass_sample_rate == rate) {
        return 0;
    }

    /* Compute divider to achieve desired sample rate and write to slave ctrl register */
    divider = (dev->conf.sample_rate / rate - 1);

    if (i2c_acquire(dev->i2c_dev)) {
        return -1;
    }
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_SLAVE4_CTRL_REG, (char) divider);
    i2c_release(dev->i2c_dev);

    /* Store configured sample rate */
    dev->conf.compass_sample_rate = dev->conf.sample_rate / (((uint16_t) divider) + 1);

    return 0;
}

/*------------------------------------------------------------------------------------*/
/*                                Internal functions                                  */
/*------------------------------------------------------------------------------------*/

/**
 * Initialize compass
 */
static int compass_init(mpu9250_t *dev)
{
    i2c_acquire(dev->i2c_dev);

    /* Enable Bypass Mode to speak to compass directly */
    conf_bypass(dev, 1);

    char data[3];
    /* Check whether compass answers correctly */
    i2c_read_reg(dev->i2c_dev, dev->comp_addr, COMPASS_WHOAMI_REG, data);
    if (data[0] != MPU9250_COMP_WHOAMI_ANSWER) {
        puts("[Error] Wrong answer from compass");
        printf("got 0x%x instead of 0x%x\n", data[0], MPU9250_COMP_WHOAMI_ANSWER);
        i2c_release(dev->i2c_dev);
        return -1;
    }

    /* Configure Power Down mode */
    i2c_write_reg(dev->i2c_dev, dev->comp_addr, COMPASS_CNTL_REG, MPU9250_COMP_POWER_DOWN_MODE);
    xtimer_usleep(MPU9250_COMP_MODE_SLEEP_US);

    /* Configure Fuse ROM access */
    i2c_write_reg(dev->i2c_dev, dev->comp_addr, COMPASS_CNTL_REG, MPU9250_COMP_FUSE_ROM_MODE);
    xtimer_usleep(MPU9250_COMP_MODE_SLEEP_US);
    /* Read sensitivity adjustment values from Fuse ROM */
    i2c_read_regs(dev->i2c_dev, dev->comp_addr, COMPASS_ASAX_REG, data, 3);
    dev->conf.compass_x_adj = data[0];
    dev->conf.compass_y_adj = data[1];
    dev->conf.compass_z_adj = data[2];
    /* Configure Power Down mode again */
    i2c_write_reg(dev->i2c_dev, dev->comp_addr, COMPASS_CNTL_REG, MPU9250_COMP_POWER_DOWN_MODE);
    xtimer_usleep(MPU9250_COMP_MODE_SLEEP_US);

    // Choose either 14-bit (0) or 16-bit (1) magnetometer resolution
    uint8_t mRes = 1;
    // set read mode (eg continuous)
    // continuous measurement mode 1 (MODE[3:0]=“0010”) = 8Hz or 2 (MODE[3:0]=“0110”) = 100Hz rate
    //uint8_t mMode = MPU9250_COMP_CONTINUOUS_MEASURE_1_MODE;
    uint8_t mMode = MPU9250_COMP_CONTINUOUS_MEASURE_1_MODE;
    uint8_t mode = mRes << 4 | mMode;
    i2c_write_reg(dev->i2c_dev, dev->comp_addr, COMPASS_CNTL_REG, mode); // Set magnetometer data resolution and sample ODR
    xtimer_usleep(MPU9250_COMP_MODE_SLEEP_US);

    /* Disable Bypass Mode to configure MPU as master to the compass */
    conf_bypass(dev, 0);
    i2c_release(dev->i2c_dev);
    return 0;
}

/**
 * Configure bypass mode
 * Caution: This internal function does not acquire exclusive access to the I2C bus.
 *          Acquisation and release is supposed to be handled by the calling function.
 */
static void conf_bypass(mpu9250_t *dev, uint8_t bypass_enable)
{
   uint8_t data;
   i2c_read_reg(dev->i2c_dev, dev->hw_addr, MPU9250_USER_CTRL_REG, &data);

   if (bypass_enable) {
       data &= ~(BIT_I2C_MST_EN);
       i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_USER_CTRL_REG, data);
       xtimer_usleep(MPU9250_BYPASS_SLEEP_US);
       i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_INT_PIN_CFG_REG, BIT_I2C_BYPASS_EN);
   }
   else {
       data |= BIT_I2C_MST_EN;
       i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_USER_CTRL_REG, data);
       xtimer_usleep(MPU9250_BYPASS_SLEEP_US);
       i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU9250_INT_PIN_CFG_REG, REG_RESET);
   }
}

