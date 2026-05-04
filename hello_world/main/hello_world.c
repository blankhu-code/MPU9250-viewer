/*
 * ESP32-S3 IMU数据采集系统
 * 硬件: ESP32-S3-N16R8 + MPU9250 (集成AK8963磁力计)
 * 功能: 加速度计/陀螺仪/磁力计数据采集、校准、滤波、串口输出
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/rmt_tx.h"
#include "driver/uart.h"
#include "led_strip_encoder.h"

/* ==================== WS2812 LED 配置 ==================== */
#define WS2812_GPIO_NUM           48        // WS2812 LED数据引脚
#define WS2812_LED_NUM            1         // LED数量
#define RMT_RESOLUTION_HZ         10000000  // RMT时钟分辨率 10MHz

/* ==================== I2C总线配置 ==================== */
#define I2C_NUM                   I2C_NUM_0 // I2C端口号
#define I2C_SCL_IO                8         // I2C时钟引脚
#define I2C_SDA_IO                9         // I2C数据引脚
#define I2C_FREQ_HZ               400000    // I2C通信频率 400kHz

/* ==================== MPU9250 寄存器地址定义 ==================== */
#define MPU9250_ADDR              0x68      // MPU9250 I2C设备地址(AD0=0)
#define MPU9250_WHO_AM_I          0x75      // 器件ID寄存器(应返回0x71)
#define MPU9250_PWR_MGMT_1        0x6B      // 电源管理寄存器1
#define MPU9250_ACCEL_XOUT_H      0x3B      // 加速度计X轴数据高字节
#define MPU9250_GYRO_XOUT_H       0x43      // 陀螺仪X轴数据高字节
#define MPU9250_TEMP_OUT_H        0x41      // 温度数据高字节
#define MPU9250_ACCEL_CONFIG      0x1C      // 加速度计量程配置
#define MPU9250_GYRO_CONFIG       0x1B      // 陀螺仪量程配置
#define MPU9250_CONFIG            0x1A      // 陀螺仪DLPF配置
#define MPU9250_ACCEL_CONFIG2     0x1D      // 加速度计DLPF配置
#define MPU9250_DLPF_CFG_10HZ     0x05      // DLPF配置值: 截止频率10Hz
#define MPU9250_INT_PIN_CFG       0x37      // 中断/旁路配置寄存器
#define MPU9250_EXT_SENS_DATA_00  0x49      // 外部传感器数据寄存器

/* ==================== AK8963磁力计寄存器地址定义 ==================== */
#define AK8963_ADDR               0x0C      // AK8963 I2C设备地址
#define AK8963_WHO_AM_I           0x00      // 器件ID寄存器(应返回0x48)
#define AK8963_CNTL1              0x0A      // 控制寄存器1(模式设置)
#define AK8963_HXL                0x03      // 磁力计X轴数据低字节
#define AK8963_CNTL2              0x0B      // 控制寄存器2(软复位)
#define AK8963_ST1                0x02      // 状态寄存器1(数据就绪标志)
#define AK8963_HXH                0x04      // 磁力计X轴数据高字节
#define AK8963_ST2                0x09      // 状态寄存器2(溢出标志)

/* ==================== UART串口配置 ==================== */
#define UART_NUM                  UART_NUM_0 // UART端口号
#define UART_BUF_SIZE             256        // UART缓冲区大小

/* ==================== 全局变量 ==================== */
static uint8_t led_pixels[WS2812_LED_NUM * 3];  // LED像素缓冲区(GRB格式)
static i2c_master_bus_handle_t i2c_bus;          // I2C总线句柄
static bool mpu9250_initialized = false;         // MPU9250初始化标志

/* ==================== IMU校准数据结构 ==================== */
typedef struct {
    float accel_bias[3];           // 加速度计零偏
    float gyro_bias[3];            // 陀螺仪零偏
    float accel_scale[3];          // 加速度计比例因子
    float accel_ortho[3][3];       // 加速度计正交校正矩阵
    bool calibrated;               // 校准完成标志
} imu_calib_t;

/* 校准参数默认值(未校准时使用) */
static imu_calib_t calib = {
    .accel_bias = {0, 0, 0},
    .gyro_bias = {0, 0, 0},
    .accel_scale = {1, 1, 1},
    .accel_ortho = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
    .calibrated = false
};

/* ==================== 校准状态变量 ==================== */
static bool calib_mode = false;                    // 校准模式开关
static int calib_face_count = 0;                   // 已采集的面数
static int calib_face_counts[6] = {0, 0, 0, 0, 0, 0};  // 每个面的采样计数
static float calib_face_data[6][3];                // 每个面的加速度累加值
static float calib_gyro_sum[3] = {0, 0, 0};       // 陀螺仪累加值
static int calib_gyro_count = 0;                   // 陀螺仪采样计数
static bool calib_gyro_done = false;               // 陀螺仪校准完成标志
static int calib_gyro_stationary_count = 0;        // 静止采样计数
static float calib_gyro_stationary_sum[3] = {0, 0, 0}; // 静止陀螺仪累加值

/*
 * mpu9250_register_read - 读取MPU9250/AK8963寄存器
 * @dev_addr: 设备I2C地址
 * @reg: 寄存器地址
 * @data: 读取数据缓冲区
 * @len: 读取字节数
 * 返回值: ESP_OK表示成功, 其他表示失败
 * 说明: 临时创建设备句柄, 读取后删除, 适用于不频繁访问的场景
 */
static esp_err_t mpu9250_register_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) return ret;
    ret = i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, -1);
    i2c_master_bus_rm_device(dev_handle);
    return ret;
}

/*
 * mpu9250_register_write - 写入MPU9250/AK8963寄存器
 * @dev_addr: 设备I2C地址
 * @reg: 寄存器地址
 * @data: 要写入的数据
 * 返回值: ESP_OK表示成功, 其他表示失败
 */
static esp_err_t mpu9250_register_write(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) return ret;
    uint8_t buf[2] = {reg, data};
    ret = i2c_master_transmit(dev_handle, buf, 2, -1);
    i2c_master_bus_rm_device(dev_handle);
    return ret;
}

/*
 * mpu9250_init - 初始化MPU9250和AK8963
 * 返回值: ESP_OK表示成功, 其他表示失败
 * 初始化流程:
 *   1. 验证MPU9250器件ID
 *   2. 唤醒传感器(退出睡眠模式)
 *   3. 配置加速度计和陀螺仪量程(±2g, ±250°/s)
 *   4. 配置DLPF低通滤波器(10Hz截止频率)
 *   5. 启用I2C旁路模式(允许直接访问AK8963)
 *   6. 初始化AK8963磁力计(连续测量模式, 16位精度)
 */
static esp_err_t mpu9250_init(void)
{
    // 验证MPU9250器件ID
    uint8_t who_am_i = 0;
    esp_err_t ret = mpu9250_register_read(MPU9250_ADDR, MPU9250_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) return ret;
    if (who_am_i != 0x71) return ESP_FAIL;

    // 唤醒传感器: 选择内部时钟, 退出睡眠模式
    ret = mpu9250_register_write(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    // 配置加速度计量程为±2g (0x00 = ±2g, 灵敏度16384 LSB/g)
    ret = mpu9250_register_write(MPU9250_ADDR, MPU9250_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    // 配置陀螺仪量程为±250°/s (0x00 = ±250°/s, 灵敏度131 LSB/°/s)
    ret = mpu9250_register_write(MPU9250_ADDR, MPU9250_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    // 配置陀螺仪DLPF, 截止频率10Hz, 滤除高频噪声
    ret = mpu9250_register_write(MPU9250_ADDR, MPU9250_CONFIG, MPU9250_DLPF_CFG_10HZ);
    if (ret != ESP_OK) return ret;

    // 配置加速度计DLPF, 截止频率10Hz
    ret = mpu9250_register_write(MPU9250_ADDR, MPU9250_ACCEL_CONFIG2, MPU9250_DLPF_CFG_10HZ);
    if (ret != ESP_OK) return ret;

    // 启用I2C旁路模式: 允许ESP32直接通过I2C总线访问AK8963磁力计
    // 0x02 = BYPASS_EN, 将AK8963连接到主I2C总线
    ret = mpu9250_register_write(MPU9250_ADDR, MPU9250_INT_PIN_CFG, 0x02);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 验证AK8963器件ID并初始化
    uint8_t ak8963_id = 0;
    ret = mpu9250_register_read(AK8963_ADDR, AK8963_WHO_AM_I, &ak8963_id, 1);
    if (ret == ESP_OK && ak8963_id == 0x48) {
        // 软复位AK8963
        mpu9250_register_write(AK8963_ADDR, AK8963_CNTL2, 0x01);
        vTaskDelay(pdMS_TO_TICKS(10));
        // 设置连续测量模式2, 16位输出精度 (0x06 = 0b00000110)
        mpu9250_register_write(AK8963_ADDR, AK8963_CNTL1, 0x06);
    }

    return ESP_OK;
}

/*
 * mpu9250_read_accel - 读取加速度计数据
 * @ax, @ay, @az: 输出参数, 单位为g
 * 返回值: true表示成功, false表示失败
 * 说明: 量程±2g, 灵敏度16384 LSB/g
 */
static bool mpu9250_read_accel(float *ax, float *ay, float *az)
{
    uint8_t data[6];
    esp_err_t ret = mpu9250_register_read(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, data, 6);
    if (ret != ESP_OK) return false;
    int16_t raw_ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_az = (int16_t)((data[4] << 8) | data[5]);
    *ax = (float)raw_ax / 16384.0f;
    *ay = (float)raw_ay / 16384.0f;
    *az = (float)raw_az / 16384.0f;
    return true;
}

/*
 * mpu9250_read_gyro - 读取陀螺仪数据
 * @gx, @gy, @gz: 输出参数, 单位为°/s
 * 返回值: true表示成功, false表示失败
 * 说明: 量程±250°/s, 灵敏度131 LSB/°/s
 */
static bool mpu9250_read_gyro(float *gx, float *gy, float *gz)
{
    uint8_t data[6];
    esp_err_t ret = mpu9250_register_read(MPU9250_ADDR, MPU9250_GYRO_XOUT_H, data, 6);
    if (ret != ESP_OK) return false;
    int16_t raw_gx = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_gy = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_gz = (int16_t)((data[4] << 8) | data[5]);
    *gx = (float)raw_gx / 131.0f;
    *gy = (float)raw_gy / 131.0f;
    *gz = (float)raw_gz / 131.0f;
    return true;
}

/*
 * mpu9250_read_temperature - 读取温度数据
 * @temp: 输出参数, 单位为摄氏度
 * 返回值: true表示成功, false表示失败
 * 说明: 温度公式: Temp = RAW/333.87 + 21.0
 */
static bool mpu9250_read_temperature(float *temp)
{
    uint8_t data[2];
    esp_err_t ret = mpu9250_register_read(MPU9250_ADDR, MPU9250_TEMP_OUT_H, data, 2);
    if (ret != ESP_OK) return false;
    int16_t raw_temp = (int16_t)((data[0] << 8) | data[1]);
    *temp = ((float)raw_temp / 333.87f) + 21.0f;
    return true;
}

/*
 * mpu9250_read_mag - 读取AK8963磁力计数据
 * @mx, @my, @mz: 输出参数, 单位为微特斯拉(uT)
 * 返回值: true表示成功, false表示失败
 * 说明: 
 *   1. 先检查ST1状态寄存器, 确认数据已就绪(bit0=1)
 *   2. 读取8字节数据(X/Y/Z各2字节 + ST2状态字节)
 *   3. 检查ST2溢出标志(bit3=1表示溢出, 数据无效)
 *   4. 灵敏度0.15 uT/LSB (16位模式)
 */
static bool mpu9250_read_mag(float *mx, float *my, float *mz)
{
    // 检查数据就绪标志
    uint8_t st1 = 0;
    esp_err_t ret = mpu9250_register_read(AK8963_ADDR, AK8963_ST1, &st1, 1);
    if (ret != ESP_OK || !(st1 & 0x01)) return false;

    // 读取磁力计数据和状态寄存器
    uint8_t data[8];
    ret = mpu9250_register_read(AK8963_ADDR, AK8963_HXL, data, 8);
    if (ret != ESP_OK) return false;

    // 组合16位原始数据(小端格式)
    int16_t raw_mx = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_my = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_mz = (int16_t)((data[5] << 8) | data[4]);

    // 检查数据溢出标志(ST2的bit3), 溢出时数据无效
    if (data[7] & 0x08) return false;

    // 转换为微特斯拉单位(灵敏度0.15 uT/LSB)
    *mx = (float)raw_mx * 0.15f;
    *my = (float)raw_my * 0.15f;
    *mz = (float)raw_mz * 0.15f;
    return true;
}

/*
 * apply_calibration - 应用校准参数到传感器数据
 * @ax, @ay, @az: 加速度计数据(输入输出)
 * @gx, @gy, @gz: 陀螺仪数据(输入输出)
 * 校准步骤:
 *   1. 加速度计零偏校正: (raw - bias) * scale
 *   2. 加速度计正交校正: 消除轴间非正交误差
 *   3. 陀螺仪零偏校正: raw - bias
 */
static void apply_calibration(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    if (!calib.calibrated) return;

    // 加速度计零偏和比例因子校正
    *ax = (*ax - calib.accel_bias[0]) * calib.accel_scale[0];
    *ay = (*ay - calib.accel_bias[1]) * calib.accel_scale[1];
    *az = (*az - calib.accel_bias[2]) * calib.accel_scale[2];

    // 加速度计正交校正矩阵乘法
    float ox = calib.accel_ortho[0][0] * *ax + calib.accel_ortho[0][1] * *ay + calib.accel_ortho[0][2] * *az;
    float oy = calib.accel_ortho[1][0] * *ax + calib.accel_ortho[1][1] * *ay + calib.accel_ortho[1][2] * *az;
    float oz = calib.accel_ortho[2][0] * *ax + calib.accel_ortho[2][1] * *ay + calib.accel_ortho[2][2] * *az;
    *ax = ox; *ay = oy; *az = oz;

    // 陀螺仪零偏校正
    *gx -= calib.gyro_bias[0];
    *gy -= calib.gyro_bias[1];
    *gz -= calib.gyro_bias[2];
}

/*
 * detect_face - 检测IMU当前朝向
 * @ax, @ay, @az: 加速度计数据
 * 返回值: 0~5 对应 +X, -X, +Y, -Y, +Z, -Z 六个面
 * 原理: 静止时加速度计主要测量重力, 最大分量方向即为朝下面
 */
static int detect_face(float ax, float ay, float az)
{
    float abs_x = fabsf(ax);
    float abs_y = fabsf(ay);
    float abs_z = fabsf(az);
    
    if (abs_x > abs_y && abs_x > abs_z) {
        return (ax > 0) ? 0 : 1;
    } else if (abs_y > abs_x && abs_y > abs_z) {
        return (ay > 0) ? 2 : 3;
    } else {
        return (az > 0) ? 4 : 5;
    }
}

static const char* face_names[] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};

/*
 * handle_calibration_command - 处理上位机发送的校准命令
 * @cmd: JSON格式的校准命令字符串
 * 支持命令:
 *   calib_start: 开始校准
 *   calib_stop: 停止校准
 *   apply_calib: 应用上位机计算好的校准参数
 */
static void handle_calibration_command(const char *cmd)
{
    if (strncmp(cmd, "{\"type\":\"calib_start\"}", 23) == 0) {
        // 开始校准: 重置所有校准状态
        calib_mode = true;
        calib_gyro_done = false;
        calib_gyro_stationary_count = 0;
        calib_gyro_stationary_sum[0] = 0;
        calib_gyro_stationary_sum[1] = 0;
        calib_gyro_stationary_sum[2] = 0;
        calib_face_count = 0;
        calib_gyro_count = 0;
        calib_gyro_sum[0] = 0;
        calib_gyro_sum[1] = 0;
        calib_gyro_sum[2] = 0;
        for (int i = 0; i < 6; i++) {
            calib_face_counts[i] = 0;
            calib_face_data[i][0] = 0;
            calib_face_data[i][1] = 0;
            calib_face_data[i][2] = 0;
        }
        printf("{\"type\":\"status\",\"msg\":\"calib_started\",\"faces_needed\":6}\n");
        fflush(stdout);
    } else if (strncmp(cmd, "{\"type\":\"calib_stop\"}", 22) == 0) {
        // 停止校准
        calib_mode = false;
        printf("{\"type\":\"status\",\"msg\":\"calib_stopped\"}\n");
        fflush(stdout);
    } else if (strncmp(cmd, "{\"type\":\"apply_calib\"", 21) == 0) {
        // 应用校准参数(由上位机计算后下发)
        float sx = 0, sy = 0, sz = 0, ox = 0, oy = 0, oz = 0;
        float gb_x = 0, gb_y = 0, gb_z = 0;
        float ab_x = 0, ab_y = 0, ab_z = 0;
        sscanf(cmd, "{\"type\":\"apply_calib\",\"sx\":%f,\"sy\":%f,\"sz\":%f,\"ox\":%f,\"oy\":%f,\"oz\":%f,\"gb_x\":%f,\"gb_y\":%f,\"gb_z\":%f,\"ab_x\":%f,\"ab_y\":%f,\"ab_z\":%f}",
               &sx, &sy, &sz, &ox, &oy, &oz, &gb_x, &gb_y, &gb_z, &ab_x, &ab_y, &ab_z);

        calib.accel_scale[0] = sx;
        calib.accel_scale[1] = sy;
        calib.accel_scale[2] = sz;

        calib.accel_ortho[0][0] = 1; calib.accel_ortho[0][1] = ox; calib.accel_ortho[0][2] = oy;
        calib.accel_ortho[1][0] = ox; calib.accel_ortho[1][1] = 1; calib.accel_ortho[1][2] = oz;
        calib.accel_ortho[2][0] = oy; calib.accel_ortho[2][1] = oz; calib.accel_ortho[2][2] = 1;

        calib.gyro_bias[0] = gb_x;
        calib.gyro_bias[1] = gb_y;
        calib.gyro_bias[2] = gb_z;

        calib.accel_bias[0] = ab_x;
        calib.accel_bias[1] = ab_y;
        calib.accel_bias[2] = ab_z;

        calib.calibrated = true;

        printf("{\"type\":\"status\",\"msg\":\"calib_applied\"}\n");
        fflush(stdout);
    }
}

/*
 * app_main - ESP32主函数(入口点)
 * 功能:
 *   1. 初始化UART串口(用于与上位机通信)
 *   2. 初始化I2C总线(用于与MPU9250通信)
 *   3. 初始化MPU9250传感器
 *   4. 配置RMT外设(用于驱动WS2812 LED)
 *   5. 主循环: 读取传感器数据 -> 校准 -> 串口输出 -> LED控制
 */
void app_main(void)
{
    printf("{\"type\":\"status\",\"msg\":\"app_start\"}\n");
    fflush(stdout);

    // 配置UART串口参数: 115200波特率, 8N1, 无流控
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0);

    // 配置I2C主总线: 400kHz, 启用内部上拉
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM,
        .scl_io_num = I2C_SCL_IO,
        .sda_io_num = I2C_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_bus);
    if (ret != ESP_OK) {
        printf("{\"type\":\"error\",\"msg\":\"i2c_init_failed\",\"code\":%d}\n", ret);
        fflush(stdout);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("{\"type\":\"status\",\"msg\":\"i2c_init_ok\"}\n");
    fflush(stdout);

    // 初始化MPU9250传感器
    esp_err_t init_ret = mpu9250_init();
    if (init_ret == ESP_OK) {
        mpu9250_initialized = true;
        printf("{\"type\":\"status\",\"msg\":\"mpu9250_init_ok\"}\n");
    } else {
        printf("{\"type\":\"error\",\"msg\":\"mpu9250_init_failed\",\"code\":%d}\n", init_ret);
    }
    fflush(stdout);

    // 配置RMT发送通道(用于WS2812 LED驱动)
    rmt_channel_handle_t led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = WS2812_GPIO_NUM,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    // 创建LED条纹编码器(将RGB数据转换为WS2812时序)
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    // RMT发送配置: 不循环
    rmt_transmit_config_t tx_config = {.loop_count = 0};

    // 呼吸灯参数配置
    uint32_t breath_start = xTaskGetTickCount();     // 呼吸灯起始时间
    const uint32_t breath_half_period_ms = 3000;     // 半周期3秒(最亮到最暗)
    const uint32_t breath_period_ms = 6000;          // 完整周期6秒
    const uint8_t breath_max = 4;                    // 最大亮度4/255(极柔和白色)

    // UART接收缓冲区
    char uart_buf[UART_BUF_SIZE];
    int uart_buf_idx = 0;

    // 主循环
    while (1) {
        // 读取UART接收到的数据(上位机命令)
        int len = uart_read_bytes(UART_NUM, (uint8_t*)uart_buf, UART_BUF_SIZE - 1, 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            uart_buf[len] = '\0';
            for (int i = 0; i < len; i++) {
                // 遇到换行符表示一条完整命令
                if (uart_buf[i] == '\n' || uart_buf[i] == '\r') {
                    uart_buf[uart_buf_idx] = '\0';
                    if (uart_buf_idx > 0) {
                        handle_calibration_command(uart_buf);
                    }
                    uart_buf_idx = 0;
                } else if (uart_buf_idx < UART_BUF_SIZE - 1) {
                    uart_buf[uart_buf_idx++] = uart_buf[i];
                }
            }
        }

        // 读取并发送传感器数据
        if (mpu9250_initialized) {
            float ax, ay, az, gx, gy, gz, temp, mx = 0, my = 0, mz = 0;
            if (mpu9250_read_accel(&ax, &ay, &az) && 
                mpu9250_read_gyro(&gx, &gy, &gz) && 
                mpu9250_read_temperature(&temp)) {
                
                // 读取磁力计数据(独立读取, 不影响主数据流)
                mpu9250_read_mag(&mx, &my, &mz);
                
                // 保存原始陀螺仪数据(用于校准)
                float raw_gx = gx, raw_gy = gy, raw_gz = gz;
                // 应用校准参数
                apply_calibration(&ax, &ay, &az, &gx, &gy, &gz);
                
                // 通过UART发送JSON格式的传感器数据
                printf("{\"type\":\"imu\",\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,\"mx\":%.2f,\"my\":%.2f,\"mz\":%.2f,\"temp\":%.2f}\n", 
                       ax, ay, az, gx, gy, gz, mx, my, mz, temp);
                fflush(stdout);

                // 校准模式处理
                if (calib_mode) {
                    // 第一阶段: 采集静止状态下的陀螺仪零偏
                    if (!calib_gyro_done) {
                        calib_gyro_stationary_sum[0] += raw_gx;
                        calib_gyro_stationary_sum[1] += raw_gy;
                        calib_gyro_stationary_sum[2] += raw_gz;
                        calib_gyro_stationary_count++;

                        if (calib_gyro_stationary_count == 1) {
                            printf("{\"type\":\"status\",\"msg\":\"keep_still\",\"info\":\"Collecting gyro bias, keep device still\"}\n");
                            fflush(stdout);
                        }

                        // 采集50个样本后计算陀螺仪零偏
                        if (calib_gyro_stationary_count >= 50) {
                            calib.gyro_bias[0] = calib_gyro_stationary_sum[0] / calib_gyro_stationary_count;
                            calib.gyro_bias[1] = calib_gyro_stationary_sum[1] / calib_gyro_stationary_count;
                            calib.gyro_bias[2] = calib_gyro_stationary_sum[2] / calib_gyro_stationary_count;
                            calib_gyro_done = true;
                            printf("{\"type\":\"status\",\"msg\":\"gyro_bias_collected\",\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,\"info\":\"Now rotate IMU to 6 faces\"}\n",
                                   calib.gyro_bias[0], calib.gyro_bias[1], calib.gyro_bias[2]);
                            fflush(stdout);
                        }
                    } else {
                        // 第二阶段: 采集6个朝向的加速度计数据
                        int face = detect_face(ax, ay, az);
                        calib_face_data[face][0] += ax;
                        calib_face_data[face][1] += ay;
                        calib_face_data[face][2] += az;
                        calib_face_counts[face]++;

                        // 统计已采集的面数(每个面至少10个样本)
                        int collected_count = 0;
                        for (int i = 0; i < 6; i++) {
                            if (calib_face_counts[i] >= 10) {
                                collected_count++;
                            }
                        }

                        // 上报进度
                        if (collected_count > 0 && collected_count != calib_face_count) {
                            calib_face_count = collected_count;
                            printf("{\"type\":\"status\",\"msg\":\"face_collected\",\"face\":\"%s\",\"progress\":%d,\"total\":6}\n",
                                   face_names[face], calib_face_count);
                            fflush(stdout);
                        }

                        // 6个面全部采集完成
                        if (collected_count >= 6) {
                            // 计算每个面的平均值
                            float face_avg[6][3];
                            for (int i = 0; i < 6; i++) {
                                face_avg[i][0] = calib_face_data[i][0] / calib_face_counts[i];
                                face_avg[i][1] = calib_face_data[i][1] / calib_face_counts[i];
                                face_avg[i][2] = calib_face_data[i][2] / calib_face_counts[i];
                            }

                            // 发送校准结果给上位机
                            printf("{\"type\":\"status\",\"msg\":\"calib_complete\",\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,\"x_pos_ax\":%.4f,\"x_pos_ay\":%.4f,\"x_pos_az\":%.4f,\"x_neg_ax\":%.4f,\"x_neg_ay\":%.4f,\"x_neg_az\":%.4f,\"y_pos_ax\":%.4f,\"y_pos_ay\":%.4f,\"y_pos_az\":%.4f,\"y_neg_ax\":%.4f,\"y_neg_ay\":%.4f,\"y_neg_az\":%.4f,\"z_pos_ax\":%.4f,\"z_pos_ay\":%.4f,\"z_pos_az\":%.4f,\"z_neg_ax\":%.4f,\"z_neg_ay\":%.4f,\"z_neg_az\":%.4f}\n",
                                   calib.gyro_bias[0], calib.gyro_bias[1], calib.gyro_bias[2],
                                   face_avg[0][0], face_avg[0][1], face_avg[0][2],
                                   face_avg[1][0], face_avg[1][1], face_avg[1][2],
                                   face_avg[2][0], face_avg[2][1], face_avg[2][2],
                                   face_avg[3][0], face_avg[3][1], face_avg[3][2],
                                   face_avg[4][0], face_avg[4][1], face_avg[4][2],
                                   face_avg[5][0], face_avg[5][1], face_avg[5][2]);
                            fflush(stdout);
                            calib_mode = false;
                            calib.calibrated = true;
                        }
                    }
                }
            }
        } else {
            // MPU9250未初始化, 尝试重新初始化
            esp_err_t ret = mpu9250_init();
            if (ret == ESP_OK) {
                mpu9250_initialized = true;
                printf("{\"type\":\"status\",\"msg\":\"mpu9250_init_ok\"}\n");
                fflush(stdout);
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        // WS2812 LED呼吸灯效果(极柔和白色)
        // 使用余弦函数生成平滑的呼吸曲线, 完整周期6秒(最亮到最暗3秒)
        uint32_t now = xTaskGetTickCount();
        uint32_t elapsed = now - breath_start;
        float breath_phase = (float)(elapsed % breath_period_ms) / breath_period_ms;
        float breath_value = (1.0f - cosf(breath_phase * 2.0f * 3.14159f)) / 2.0f;
        uint8_t brightness = (uint8_t)(breath_max * breath_value);
        led_pixels[0] = brightness;  // G通道
        led_pixels[1] = brightness;  // R通道
        led_pixels[2] = brightness;  // B通道
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_pixels, sizeof(led_pixels), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
