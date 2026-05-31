#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_mac.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "nvs_flash.h"
#include "ping/ping_sock.h"
#include "driver/gpio.h"

//sensor fusion
#include "sdkconfig.h"
#include "driver/i2c_master.h"
#include "sensorFusionEntryPoint.h"
#include "sensorFusionEntryPoint_initialize.h"
#include "sensorFusionEntryPoint_types.h"
#include <math.h>


//dshot stuff
#include "dshot.h"

rmt_channel_handle_t esc_chan[4];
rmt_encoder_handle_t copy_encoder = NULL;
const gpio_num_t esc_pins[4]={
    GPIO_NUM_26,/*pitch_up*/
    GPIO_NUM_33,/*pitch_down*/
    GPIO_NUM_25,/*yaw_right*/
    GPIO_NUM_32,/*yaw_left*/
};




//sensor fusion
static const char *TAGMPU   = "MPU6500";
static const char *cpTAG = "CMPS2";
static const char *sfTAG = "FUSION";
 
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       1000
 
#define MPU6500_SENSOR_ADDR         0x68
#define MPU6500_WHO_AM_I_REG        0x75
#define MPU6500_PWR_MGMT_1_REG      0x6B
#define CMPS2_ADDR                  0x30
 
// Sample rate 
#define SAMPLE_RATE_HZ              100
#define SAMPLE_PERIOD_S             (1.0f / SAMPLE_RATE_HZ)


#define DECLINATION_DEG             4.3f
 
// number of samples for gyro calibration
#define GYRO_CAL_SAMPLES            200
 
// time mag calibration runs
#define MAG_CAL_DURATION_MS         30000

// 2C helpers
static esp_err_t mpu_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}
 
static esp_err_t mpu_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(dev, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS);
}
 
// I2C init
static void i2c_master_init(i2c_master_bus_handle_t *bus,
                             i2c_master_dev_handle_t *mpu_dev,
                             i2c_master_dev_handle_t *cmps_dev)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port               = I2C_MASTER_NUM,
        .sda_io_num             = I2C_MASTER_SDA_IO,
        .scl_io_num             = I2C_MASTER_SCL_IO,
        .clk_source             = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt      = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus));
 
    i2c_device_config_t mpu_config = {
        .dev_addr_length  = I2C_ADDR_BIT_LEN_7,
        .device_address   = MPU6500_SENSOR_ADDR,
        .scl_speed_hz     = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus, &mpu_config, mpu_dev));
 
    i2c_device_config_t cmps_config = {
        .dev_addr_length  = I2C_ADDR_BIT_LEN_7,
        .device_address   = CMPS2_ADDR,
        .scl_speed_hz     = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus, &cmps_config, cmps_dev));
}
 
// CMPS2 blocking single read (used during calibration) 
static esp_err_t compass_read_blocking(i2c_master_dev_handle_t dev,
                                        float *mx, float *my, float *mz)
{
    // Trigger measurement
    uint8_t trig[2] = {0x07, 0x01};
    esp_err_t err = i2c_master_transmit(dev, trig, 2, I2C_MASTER_TIMEOUT_MS);
    if (err != ESP_OK) return err;
 
    // Wait for measurment to be done
    vTaskDelay(pdMS_TO_TICKS(10));
 
    // read resoults
    uint8_t reg = 0x00;
    uint8_t buf[6];
    err = i2c_master_transmit_receive(dev, &reg, 1, buf, 6, I2C_MASTER_TIMEOUT_MS);
    if (err != ESP_OK) return err;
 
    // read the data send the data
    uint16_t raw_xu = ((uint16_t)buf[1] << 8) | (uint16_t)buf[0];
    uint16_t raw_yu = ((uint16_t)buf[3] << 8) | (uint16_t)buf[2];
    uint16_t raw_zu = ((uint16_t)buf[5] << 8) | (uint16_t)buf[4];
    // convert to angesl
    *mx = ((float)raw_yu - 32768.0f) / 2048.0f;
    *my = ((float)raw_zu - 32768.0f) / 2048.0f;
    *mz = ((float)raw_xu - 32768.0f) / 2048.0f;
    return ESP_OK;
}
 
// CMPS2 async state mach
typedef enum { COMPASS_IDLE, COMPASS_WAITING } compass_state_t;
 
static compass_state_t  compass_state    = COMPASS_IDLE;
static int64_t          compass_ready_at = 0;
 
float mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;
float mag_mid_x   = 0.0f;
float mag_mid_y   = 0.0f;
float mag_scale_x = 1.0f;
float mag_scale_y = 1.0f;
 // compass trigger for async reading
static void compass_trigger(i2c_master_dev_handle_t dev)
{
    uint8_t buf[2] = {0x07, 0x01};
    esp_err_t err = i2c_master_transmit(dev, buf, 2, I2C_MASTER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(cpTAG, "Compass trigger failed: %s", esp_err_to_name(err));
        return;
    }
    compass_ready_at = esp_timer_get_time() + 9000;
    compass_state    = COMPASS_WAITING;
}
// read for aysin reading
static void compass_read_async(i2c_master_dev_handle_t dev)
{
    uint8_t reg = 0x00;
    uint8_t buf[6];
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, buf, 6, I2C_MASTER_TIMEOUT_MS);
    if (err == ESP_OK) {
        uint16_t raw_xu = ((uint16_t)buf[1] << 8) | (uint16_t)buf[0];
        uint16_t raw_yu = ((uint16_t)buf[3] << 8) | (uint16_t)buf[2];
        uint16_t raw_zu = ((uint16_t)buf[5] << 8) | (uint16_t)buf[4];
        mag_z = -((float)raw_xu - 32768.0f) / 2048.0f;
        float raw_mx = ((float)raw_yu - 32768.0f) / 2048.0f;
        float raw_my = ((float)raw_zu - 32768.0f) / 2048.0f;

        mag_x =raw_mx;
        mag_y =raw_my;
    } else {
        ESP_LOGE(cpTAG, "CMPS2 read failed: %s", esp_err_to_name(err));
    }
    compass_state = COMPASS_IDLE;
}
 // check wether we are reading or waiting for a reading
static void compass_update(i2c_master_dev_handle_t dev)
{
    if (compass_state == COMPASS_IDLE) {
        compass_trigger(dev);
    } else if (esp_timer_get_time() >= compass_ready_at) {
        compass_read_async(dev);
    }
}
 
//  Accelrometer read reads 
static esp_err_t read_accel(i2c_master_dev_handle_t dev,
                             float *ax, float *ay, float *az)
{
    uint8_t reg = 0x3B;
    uint8_t raw[6];
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, raw, 6, 1000);
    if (err != ESP_OK) {  
        ESP_LOGE(TAGMPU, "Accel read failed: %s", esp_err_to_name(err));
        return err;
    }
    *ax = (int16_t)((raw[0] << 8) | raw[1]) / 16384.0f;
    *ay = (int16_t)((raw[2] << 8) | raw[3]) / 16384.0f;
    *az = (int16_t)((raw[4] << 8) | raw[5]) / 16384.0f;
    // convert data
    return ESP_OK;
}
 // just like the stuff for  accel
static esp_err_t read_gyro(i2c_master_dev_handle_t dev,
                            float *gyrx, float *gyry, float *gyrz)
{
    uint8_t reg = 0x43;
    uint8_t raw[6];
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, raw, 6, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAGMPU, "Gyro read failed: %s", esp_err_to_name(err));
        return err;
    }
    *gyrx = (int16_t)((raw[0] << 8) | raw[1]) / 131.0f;
    *gyry = (int16_t)((raw[2] << 8) | raw[3]) / 131.0f;
    *gyrz = (int16_t)((raw[4] << 8) | raw[5]) / 131.0f;
    return ESP_OK;
}
 

static void calibrate_gyro(i2c_master_dev_handle_t dev,   // calibration of gyro
                            float *gx_bias, float *gy_bias, float *gz_bias)
{
    ESP_LOGI(TAGMPU, "Gyro cal — keep board STILL for ~2 seconds...");
    double sum_x = 0, sum_y = 0, sum_z = 0;
    float gx, gy, gz;
 
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        if (read_gyro(dev, &gx, &gy, &gz) == ESP_OK) {
            sum_x += gx;
            sum_y += gy;
            sum_z += gz;
        }  // calculates average ofset for each axis, 
        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE_HZ));
    }
 
    *gx_bias = (float)(sum_x / GYRO_CAL_SAMPLES);
    *gy_bias = (float)(sum_y / GYRO_CAL_SAMPLES);
    *gz_bias = (float)(sum_z / GYRO_CAL_SAMPLES);
 
    ESP_LOGI(TAGMPU, "Gyro bias: GX=%.4f  GY=%.4f  GZ=%.4f deg/s",
             *gx_bias, *gy_bias, *gz_bias);
}
 

static void calibrate_mag(i2c_master_dev_handle_t dev,
                           float *mid_x,
                           float *mid_y,
                           float *scale_x,
                           float *scale_y)
{  // callibation of the mag
    ESP_LOGI(cpTAG, "=== MAG CALIBRATION: rotate board in all directions for %d seconds ===",
             MAG_CAL_DURATION_MS / 1000);
 
    float mx, my, mz;
    float mag_max_x = -1e9f, mag_min_x =  1e9f;
    float mag_max_y = -1e9f, mag_min_y =  1e9f;
 
    int64_t start_us   = esp_timer_get_time();
    int64_t end_us     = start_us + (int64_t)MAG_CAL_DURATION_MS * 1000;
    int64_t next_log   = start_us + 1000000; /* log every 1 s */
 
    while (esp_timer_get_time() < end_us) {
 
        if (compass_read_blocking(dev, &mx, &my, &mz) == ESP_OK) {
             // Hard iron calibration
            if (mx > mag_max_x) mag_max_x = mx;
            if (mx < mag_min_x) mag_min_x = mx;
            if (my > mag_max_y) mag_max_y = my;
            if (my < mag_min_y) mag_min_y = my;
 // soft iron calibration
            float x_radius = (mag_max_x - mag_min_x) / 2.0f;
            float y_radius = (mag_max_y - mag_min_y) / 2.0f;

            float avg_radius = (x_radius + y_radius) / 2.0f;

            if (x_radius > 0.001f)
                *scale_x = avg_radius / x_radius;
            else
                *scale_x = 1.0f;

            if (y_radius > 0.001f){
                *scale_y = avg_radius / y_radius;}
            else{
                *scale_y = 1.0f;}
// Hard iron calibration
            *mid_x = (mag_max_x + mag_min_x) / 2.0f;
            *mid_y = (mag_max_y + mag_min_y) / 2.0f;

ESP_LOGI(cpTAG,
         "Mag cal done:"
         " MidX=%.4f MidY=%.4f"
         " ScaleX=%.4f ScaleY=%.4f",
         *mid_x, *mid_y,
         *scale_x, *scale_y);
        }
 
        // prints proges of calibrations
        if (esp_timer_get_time() >= next_log) {
            next_log += 1000000;
            float elapsed = (esp_timer_get_time() - start_us) / 1e6f;
            ESP_LOGI(cpTAG, "t=%4.1fs  MidX=%.4f  MidY=%.4f  [MaxX=%.4f MinX=%.4f  MaxY=%.4f MinY=%.4f]",
                     elapsed, *mid_x, *mid_y,
                     mag_max_x, mag_min_x, mag_max_y, mag_min_y);
        }
    }
 
    ESP_LOGI(cpTAG, "=== Mag cal done: MidX=%.4f  MidY=%.4f ===", *mid_x, *mid_y);
}
// mutex structure for motor controll
typedef struct {
    int pitch_up;
    int pitch_down;
    int yaw_right;
    int yaw_left;
} motor_commands_t;

// One shared struct + one mutex, visible to both tasks
static motor_commands_t g_motor_cmd = {0, 0, 0, 0};
static SemaphoreHandle_t g_motor_mutex = NULL;


// queue handel for using queue
QueueHandle_t data_queue;
QueueHandle_t pitch_queue,roll_queue,yaw_queue,yaw_w_dot_queue,pitch_w_dot_queue;  // data given to pc     
QueueHandle_t pitch_set_queue,yaw_set_queue;                                       // set heading
QueueHandle_t spin_queue;                                                          // arming
QueueHandle_t K_p_yaw_queue, K_d_yaw_queue, K_p_pitch_queue, K_d_pitch_queue;       // pd tuning
//TCP stuff
static int g_sock = -1;
#define PORT 6767
#define SSID "S24 Ultra von Stephan"
#define PASS "ty2gmvusqjr88c3"
static const char *TAG = "TCP SOCKET Client";
static bool start_con=0;
//static const char *payload = "Message from ESP32 TCP Socket Client";

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting WIFI_EVENT_STA_START ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected WIFI_EVENT_STA_CONNECTED ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection WIFI_EVENT_STA_DISCONNECTED ... \n");
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP ... \n\n");
        break;
    default:
        break;
    }
}

void wifi_connection()
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = SSID,
            .password = PASS}};
    esp_wifi_set_config(WIFI_IF_STA, &wifi_configuration);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_connect();
}


void control_task(void *pvParameters)
{   // the controll task
    esp_err_t err;
    uint8_t data = 0;
    // sets up i2c 
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t mpu_dev;
    i2c_master_dev_handle_t cmps_dev;
 // initilazies i2c
    i2c_master_init(&bus, &mpu_dev, &cmps_dev);
    ESP_LOGI(TAGMPU, "I2C initialized");
 
    // chesk CMP2
    uint8_t cmps_id = 0, id_reg = 0x20;
    err = i2c_master_transmit_receive(cmps_dev, &id_reg, 1, &cmps_id, 1, 1000);
    if (err != ESP_OK)
        ESP_LOGE(cpTAG, "CMPS2 not responding: %s", esp_err_to_name(err));
    else
        ESP_LOGI(cpTAG, "CMPS2 Product ID = 0x%02X (expect 0x06)", cmps_id);
 
    // wakes up mpu6050
    err = mpu_write(mpu_dev, MPU6500_PWR_MGMT_1_REG, 0x00); 
    if (err != ESP_OK) {
        ESP_LOGE(TAGMPU, "Failed to wake MPU6500: %s", esp_err_to_name(err));
        return;
    }
    // configure mpu to right setting
    vTaskDelay(pdMS_TO_TICKS(50));
    mpu_write(mpu_dev, MPU6500_PWR_MGMT_1_REG, 0x01); // clock = gyro X PLL
    vTaskDelay(pdMS_TO_TICKS(50));
    mpu_write(mpu_dev, 0x1A, 0x03);                   // DLPF 44 Hz — smooths noise 
    mpu_write(mpu_dev, 0x1B, 0x00);                   // GYRO_CONFIG:  ±250 deg/s 
    mpu_write(mpu_dev, 0x1C, 0x00);                   // ACCEL_CONFIG: +-2 g      
    vTaskDelay(pdMS_TO_TICKS(100));
 
    // 

 
    
    {  // intizlazes CMPs2
        uint8_t mag_cmd[2];
        mag_cmd[0] = 0x07; mag_cmd[1] = 0x80;
        i2c_master_transmit(cmps_dev, mag_cmd, 2, I2C_MASTER_TIMEOUT_MS);
        vTaskDelay(pdMS_TO_TICKS(700));
        mag_cmd[0] = 0x07; mag_cmd[1] = 0x20;
        i2c_master_transmit(cmps_dev, mag_cmd, 2, I2C_MASTER_TIMEOUT_MS);
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(cpTAG, "MMC3416 initialised (Refill Cap + SET)");
        {
                float tx, ty, tz;
            compass_read_blocking(cmps_dev, &tx, &ty, &tz);
            ESP_LOGI(cpTAG, "First mag read after init: X=%.4f Y=%.4f Z=%.4f", tx, ty, tz);
    }
    }

    // start gyro calibration
    float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;
    calibrate_gyro(mpu_dev, &gx_bias, &gy_bias, &gz_bias);
 // starts mag calibration

    calibrate_mag(cmps_dev,
              &mag_mid_x,
              &mag_mid_y,
              &mag_scale_x,
              &mag_scale_y);
 
    // initliazies sensor fusion
    sensorFusionEntryPoint_initialize();
 
    // give intail values to magenomter
    float init_mx = 0.0f, init_my = 0.0f, init_mz = 0.0f;
    compass_read_blocking(cmps_dev, &init_mx, &init_my, &init_mz);
    float corr_mx = (init_mx - mag_mid_x) * mag_scale_x;
        float corr_my = (init_my - mag_mid_y) * mag_scale_y;
    float init_yaw = fmodf(
        atan2f(-corr_mx, corr_my) * 57.2957802f
        + 4.33f + 360.0f, 360.0f);
    ESP_LOGI(sfTAG, "Initial yaw: %.1f deg", init_yaw);

    struct0_T sf_state = {
    .pitch     = 0.0f,
    .roll      = 0.0f,
    .yaw       = init_yaw,
    .magMidX   = mag_mid_x,
    .magMidY   = mag_mid_y,
    .magScaleX = mag_scale_x,
    .magScaleY = mag_scale_y,
    };
 // sets up varibales for control loop
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gyrx = 0.0f, gyry = 0.0f, gyrz = 0.0f;
    float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;

    float yaw_set=0.0f,pitch_set=0.0;
    float K_p_yaw=4.0;
    float K_d_yaw=480.0;
    float K_p_pitch=0.25;
    float K_d_pitch=0.1;
    float error_yaw,error_pitch;
    float old_error_yaw=0.0f,old_error_pitch=0.0f;
    int pitch_up, pitch_down;
    int yaw_right, yaw_left;
    bool spin=0;
    float d_pitch,d_yaw;
    short pd_yaw,pd_pitch;
    ESP_LOGI(sfTAG, "Starting sensor fusion at %d Hz", SAMPLE_RATE_HZ);
    start_con=1;
    
    while (1)
    {  // main control loop
        xQueueReceive(spin_queue, &spin,0);
        xQueueReceive(pitch_set_queue, &pitch_set,0);
        xQueueReceive(yaw_set_queue, &yaw_set, 0);

        xQueueReceive(K_d_pitch_queue, &K_d_pitch, 0);
        xQueueReceive(K_p_pitch_queue, &K_p_pitch, 0);
        xQueueReceive(K_d_yaw_queue, &K_d_yaw, 0);
        xQueueReceive(K_p_yaw_queue, &K_p_yaw, 0);
        // gets data for tuning and arming/disarming and wanted picth and yaw

        // accel data
        int64_t time=esp_timer_get_time();//in us
        read_accel(mpu_dev, &ax, &ay, &az);

        // read gyro data
        if (read_gyro(mpu_dev, &gyrx, &gyry, &gyrz) == ESP_OK) {
            gyrx -= gx_bias;
            gyry -= gy_bias;
            gyrz -= gz_bias;
        }
 
        // Non-blocking magnetometer update
        compass_update(cmps_dev);
 
        
        // gives data to sensorfusion
        sensorFusionEntryPoint(ax, ay, az,
                               gyrx, gyry, gyrz,
                               mag_x, mag_y, mag_z,
                               SAMPLE_PERIOD_S,
                               &sf_state,
                               &pitch, &roll, &yaw);
 
        // print data ever 500 ms
        static int log_counter = 0;
        if (++log_counter >= 10) {
            log_counter = 0;
            ESP_LOGI(sfTAG, "Pitch: %+6.2f  Roll: %+6.2f  Yaw: %5.1f  M_X=%+6.3f  M_Y=%+6.3f  M_Z=%+6.3f",
                     pitch, roll, yaw, mag_x, mag_y, mag_z);
            // sends data to queue will sent it to the part which sent data to pc
            xQueueSend(yaw_queue,&yaw,0);
            xQueueSend(roll_queue,&roll,0);
            xQueueSend(pitch_queue,&pitch,0);
        
            xQueueSend(pitch_w_dot_queue,&gyry,0); //pitch is arround the y-axis!!!
            xQueueSend(yaw_w_dot_queue,&gyrz,0);   //yaw is arround the z-axis!!!
            
            
        }
        // calulated error for yaw and pitch
        error_yaw=yaw_set-yaw;
        error_pitch=pitch_set-pitch;
        // pd calulateion
        pd_yaw=(K_p_yaw*error_yaw+K_d_yaw*(error_yaw-old_error_yaw)*2/((double)(esp_timer_get_time()-time)*1E-3));//SAMPLE_PERIOD_S
        d_pitch=(K_d_pitch*(error_pitch-old_error_pitch)*2/((double)(esp_timer_get_time()-time)*1E-3));//change to sample period in us if still not working!
        pd_pitch=K_p_pitch*error_pitch+d_pitch;
       
        pd_pitch=K_p_pitch*error_pitch+d_pitch; 
        // save current error as old error, for 
        old_error_yaw=error_yaw;


        old_error_pitch=error_pitch;

        // split pd in to values for the differntail thrust set up
        yaw_left=400-(pd_yaw/2);
        yaw_right=400+(pd_yaw/2);
        pitch_up=400+(pd_pitch/2);
        pitch_down=400-(pd_pitch/2);
        
        pitch_up=400-(pd_pitch/2);
        pitch_down=400+(pd_pitch/2);
        
        // set end cap for Dshot so motors don´t stall out
        if(pitch_up<70)
        pitch_up=70;
        if(pitch_down<70)
        pitch_down=70;
        if(yaw_left<70)
        yaw_left=70;
        if(yaw_right<70)
        yaw_right=70;
        // set high cap for dshot so motors don´t draw to much current
        if(pitch_up>1100)
        pitch_up=1100;
        if(pitch_down>1100)
        pitch_down=1100;
        if(yaw_left>1100)
        yaw_left=1100;
        if(yaw_right>1100)
        yaw_right=1100; 

        // Take the mutex with timeout 0 — don't wait, just skip if busy
        if(xSemaphoreTake(g_motor_mutex, 0) == pdTRUE) {
        if(spin){
            g_motor_cmd.yaw_right  = yaw_right;
            g_motor_cmd.yaw_left   = yaw_left;
            g_motor_cmd.pitch_down = pitch_down;
            g_motor_cmd.pitch_up   = pitch_up;
        } else { // if we are not armed motor power is 0
            g_motor_cmd.yaw_right  = 0;
            g_motor_cmd.yaw_left   = 0;
            g_motor_cmd.pitch_down = 0;
            g_motor_cmd.pitch_up   = 0;
        }
            xSemaphoreGive(g_motor_mutex); 
        }       
        // ensure proper lengh of control loop
        vTaskDelay(pdMS_TO_TICKS((1000 / SAMPLE_RATE_HZ)));

    }
 
   
    i2c_master_bus_rm_device(mpu_dev);
    i2c_master_bus_rm_device(cmps_dev);
    i2c_del_master_bus(bus);
}


void Dshot600_task(void *pvParameters)
{
    
           // Local snapshot — motors hold last known command if mutex is briefly busy
    motor_commands_t local_cmd = {0, 0, 0, 0};
 
    while (1)
    {
        // Try to read the latest motor commands written by control_task.
        // Timeout 2 ms: short enough not to delay motors, long enough to
        // wait out a brief mutex hold by the PD controller on Core 1.
        if (xSemaphoreTake(g_motor_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            local_cmd = g_motor_cmd;   // copy all 4 values atomically
            xSemaphoreGive(g_motor_mutex);
        }
        // else: keep local_cmd from previous cycle — motors hold last command
 
        dshot_send(esc_chan[0], local_cmd.pitch_up,   copy_encoder);
        dshot_send(esc_chan[1], local_cmd.pitch_down,  copy_encoder);
        dshot_send(esc_chan[2], local_cmd.yaw_right,   copy_encoder);
        dshot_send(esc_chan[3], local_cmd.yaw_left,    copy_encoder);


        
        // optional refresh rate
        vTaskDelay(pdMS_TO_TICKS(7));//and this
    }
    
}

void tcp_connection_task(void *pvParameters)
{
    char host_ip[] = "10.38.17.78";

    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = inet_addr(host_ip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    while (1)
    {
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        ESP_LOGI(TAG, "Connecting...");

        int err = connect(sock,
                          (struct sockaddr *)&dest_addr,
                          sizeof(dest_addr));

        if (err != 0)
        {
            ESP_LOGE(TAG, "Socket unable to connect");
            close(sock);

            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        ESP_LOGI(TAG, "Connected");

        g_sock = sock;

        // Stay alive while connected
        while (g_sock != -1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        ESP_LOGW(TAG, "Disconnected");

        shutdown(sock, 0);
        close(sock);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
void tcp_rx_task(void *pvParameters)
{
    char rx_buffer[128];

    float yaw = 0;
    float pitch = 0;
    bool spin=0;
    const int stop=0;
    float kp_yaw=4.0;
    float kd_yaw=480.0;
    float kp_pitch=0.25; 
    float kd_pitch=0.1;

    while (1)
    {
        if (g_sock < 0)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int len = recv(g_sock,
                       rx_buffer,
                       sizeof(rx_buffer) - 1,
                       0);

        if (len <= 0)
        {
            ESP_LOGE(TAG, "RX disconnected");

            g_sock = -1;
            continue;
        }

        rx_buffer[len] = 0;

        ESP_LOGI(TAG, "RX: %s", rx_buffer);

       
        // COMMAND PARSER
       

        if (strncmp(rx_buffer, "THROTTLE:", 9) == 0)
        {
            uint16_t throttle = atoi(rx_buffer + 9);
            uint16_t start=100;
            uint16_t throttle_up=350;
            uint16_t throttle_down;
            uint16_t end=0;
            if(throttle==1){spin=1;
            xQueueSend(spin_queue,&spin,0);
            }
        }
        else if (strncmp(rx_buffer, "set yaw:", 8) == 0)
        {
            yaw = atof(rx_buffer + 8);
            xQueueSend(yaw_set_queue,&yaw,0);

            ESP_LOGI(TAG, "yaw=%f", yaw);
        }
        else if (strncmp(rx_buffer, "change yaw:", 11) == 0)
        {
            yaw += atof(rx_buffer + 11);
            xQueueSend(yaw_set_queue,&yaw,0);

            ESP_LOGI(TAG, "yaw=%f", yaw);
        }
        else if (strncmp(rx_buffer, "set pitch:", 10) == 0)
        {
            pitch = atof(rx_buffer + 10);
            xQueueSend(pitch_set_queue,&pitch,0);
            ESP_LOGI(TAG, "pitch=%f", pitch);
        }
        else if (strncmp(rx_buffer, "change pitch:", 13) == 0)
        {
            pitch += atof(rx_buffer + 13);
            xQueueSend(pitch_set_queue,&pitch,0);
            ESP_LOGI(TAG, "pitch=%f", pitch);
        }
        else if (strncmp(rx_buffer, "STOP", 4) == 0)
        {
            spin=0;
            xQueueSend(spin_queue,&spin,0);
            ESP_LOGI(TAG, "Spin stop");
        }
        else if (strncmp(rx_buffer, "kd_yaw:", 7) == 0)
        {
            kd_yaw = atof(rx_buffer + 7);
            xQueueSend(K_d_yaw_queue,&kd_yaw,0);
            ESP_LOGI(TAG, "kd yaw=%f", kd_yaw);
        }
        else if (strncmp(rx_buffer, "kp_yaw:", 7) == 0)
        {
            kp_yaw = atof(rx_buffer + 7);
            xQueueSend(K_p_yaw_queue,&kp_yaw,0);
            ESP_LOGI(TAG, "kp yaw=%f", kp_yaw);
        }
        else if (strncmp(rx_buffer, "kd_pitch:", 9) == 0)
        {
            kd_pitch = atof(rx_buffer + 9);
            xQueueSend(K_d_pitch_queue,&kd_pitch,0);
            ESP_LOGI(TAG, "kd pitch=%f", kd_pitch);
        }
        else if (strncmp(rx_buffer, "kp_pitch:", 9) == 0)
        {
            kp_pitch = atof(rx_buffer + 9);
            xQueueSend(K_p_pitch_queue,&kp_pitch,0);
            ESP_LOGI(TAG, "kp pitch=%f", kp_pitch);
        }
    
       //go here
    }
}
void tcp_tx_task(void *pvParameters)
{
    float yaw_read, pitch_read, roll_read,pitch_omega_dot,yaw_omega_dot;

 
    while (1)
    {
        if (g_sock < 0)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        // gets data from the control loop
        xQueueReceive(roll_queue, &roll_read, pdMS_TO_TICKS(2));
        xQueueReceive(pitch_queue, &pitch_read, pdMS_TO_TICKS(2));
        xQueueReceive(yaw_queue, &yaw_read, pdMS_TO_TICKS(2));
        xQueueReceive(pitch_w_dot_queue, &pitch_omega_dot, pdMS_TO_TICKS(2));
        xQueueReceive(yaw_w_dot_queue, &yaw_omega_dot, pdMS_TO_TICKS(2));

        //esp_timer_get_time()
        char readings[256];
        long long current_time=esp_timer_get_time();
        sprintf(readings, "SF: Pitch:%+6.2f;Roll:%+6.2f;Yaw:%5.1f;Pitch_w_dot:%6.2f;Yaw_w_dot:%6.2f;Time:%lld\n",//could change in the future
        pitch_read, roll_read, yaw_read,pitch_omega_dot,yaw_omega_dot,current_time);
        int err = send(g_sock,readings,strlen(readings),0);
        

        if (err < 0)
        {
            ESP_LOGE(TAG, "TX failed");

            g_sock = -1;
        }

        vTaskDelay(pdMS_TO_TICKS(20));//change to how fast the readings are
    }
}

void app_main(void)
{
    
    srand(25);
    //data share
    data_queue = xQueueCreate(10, sizeof(int));
    // gets data from the mutex
    g_motor_mutex = xSemaphoreCreateMutex();


    yaw_queue = xQueueCreate(2, sizeof(float));
    roll_queue = xQueueCreate(2, sizeof(float));
    pitch_queue = xQueueCreate(2, sizeof(float));
    yaw_w_dot_queue = xQueueCreate(2, sizeof(float));
    pitch_w_dot_queue = xQueueCreate(2, sizeof(float));

    yaw_set_queue = xQueueCreate(2, sizeof(float));
    pitch_set_queue = xQueueCreate(2, sizeof(float));

    spin_queue = xQueueCreate(2, sizeof(bool));

    K_p_yaw_queue = xQueueCreate(2, sizeof(float));
    K_d_yaw_queue = xQueueCreate(2, sizeof(float));
    K_p_pitch_queue = xQueueCreate(2, sizeof(float));
    K_d_pitch_queue = xQueueCreate(2, sizeof(float));
    
    rmt_copy_encoder_config_t copy_config = {};
    ESP_ERROR_CHECK(
        rmt_new_copy_encoder(&copy_config, &copy_encoder)
    );

    for (int i = 0; i < 4; i++) {//channel creation

        rmt_tx_channel_config_t esc_chan_config = {
            .gpio_num = esc_pins[i],
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 40000000,
            .mem_block_symbols = 128,
            .trans_queue_depth = 1,
            .flags = {
                .invert_out = false,
            },
            .intr_priority=1,
            
        };

        ESP_ERROR_CHECK(
            rmt_new_tx_channel(
                &esc_chan_config,
                &esc_chan[i]
            )
        );

        ESP_ERROR_CHECK(
            rmt_enable(esc_chan[i])
        );
    }
    //arming

for (int t = 0; t < 1000; t++)
{
    for (int m = 0; m < 4; m++)
    {
        dshot_send(esc_chan[m], 0, copy_encoder);
    }

    // 1 kHz update rate
    vTaskDelay(pdMS_TO_TICKS(1));
}
for (int t = 0; t < 1000; t++)
{
    for (int m = 0; m < 4; m++)
    {
        dshot_send(esc_chan[m], 50, copy_encoder);
    }
    // 1 kHz update rate
    vTaskDelay(pdMS_TO_TICKS(7));
}
vTaskDelay(pdMS_TO_TICKS(1000));


    // Control loop on Core 1
    xTaskCreatePinnedToCore(
        control_task,
        "control_task",
        12288,
        NULL,
        5,
        NULL,
        1   // Core 1
    );


    

    while(!(start_con))vTaskDelay(pdMS_TO_TICKS(100));
    wifi_connection();
    vTaskDelay(pdMS_TO_TICKS(5000));
    xTaskCreatePinnedToCore(tcp_connection_task,
            "tcp_connection_task",
            4096,
            NULL,
            5,
            NULL,
            0
        );

    xTaskCreatePinnedToCore(tcp_rx_task,
            "tcp_rx_task",
            4096,
            NULL,
            5,
            NULL,
            0
        );

    xTaskCreatePinnedToCore(tcp_tx_task,
            "tcp_tx_task",
            4096,
            NULL,
            5,
            NULL,
            0
        );
    xTaskCreatePinnedToCore(
        Dshot600_task,
        "Dshot600_task",
        1024,
        NULL,
        5,
        NULL,
        0   // Core 0
    );

}
