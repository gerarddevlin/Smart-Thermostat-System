/* Simple HTTP Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   Modifed to handle:
   -- GET request to fetch mac address of the ESP32
   -- PUT request to toggle onboard LED on and off

   Emily Lam -- October 2018
*/

#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>

#include "freertos/queue.h"
#include "driver/gpio.h"

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include "driver/uart.h"

#include <esp_http_server.h>

#include <string.h>

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

//timer
#include "esp_types.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h" 
#include "driver/timer.h"

#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/apps/sntp.h"

#include "driver/i2c.h"
#include "driver/uart.h"
#include "sdkconfig.h"

/* A simple example that demonstrates how to create GET and POST
 * handlers for the web server.
 * The examples use simple WiFi configuration that you can set via
 * 'make menuconfig'.
 * If you'd rather not, just change the below entries to strings
 * with the config you want -
 * ie. #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID "Group_15"
#define EXAMPLE_WIFI_PASS "smart-systems"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

//Servo
#define SERVO_MIN_PULSEWIDTH 500 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 60//60 //Maximum angle in degree upto which servo can rotate

//timer
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
//#define TIMER_INTERVAL0_SEC   (20 )//3.4179) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (1)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

#define GPIO_INPUT_IO_0       4
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_IO_1     5
#define GPIO_INPUT_IO_2     36
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2))

//Alphanumeric
/* ***DONT NEED THIS STUFF*** */
 #define DATA_LENGTH                        512              /*!<Data buffer length for test buffer*/
#define I2C_EXAMPLE_MASTER_SCL_IO          22            /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          23           /*!< gpio number for I2C master data  */

// *** There are two i2c ports I2C_NUM_0 or I2C_NUM_1  -- don't need to change ***
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */

// *** This is not needed for Master write, disable -- don't need to change  ***
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */

// *** Communication frequency -- don't need to change
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */


// *** These need to be changed to the address of the matirx driver (in datasheet for HT16K33), Rename defines ***
#define MATRIX_DRIVER_ADDR                 0x70            /*!< slave address for BH1750 sensor */
// *** This applies to a different device, our device will have different CMDs
// #define BH1750_CMD_START                   0x23             /*!< Command to set measure mode */

// *** Not needed, we're setup as a master, not slave ***
#define ESP_SLAVE_ADDR                     0x00            /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */
#define BUF_SIZE (8192)
const int uart_num = UART_NUM_0;


#define LEDPIN 13
#define Heat_On 26
#define Button_Out 25
static const char *TAG="APP";

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

//static const char *TAG = "example";

/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */
//RTC_DATA_ATTR static int boot_count = 0;

static void obtain_time(void);
static void initialize_sntp(void);
//static void initialise_wifi(void);
static esp_err_t event_handler(void *ctx, system_event_t *event);



bool on = false;
int templevel = 90;

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}



static esp_err_t i2c_example_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr)
{
    // *** This creates a structure (class) called cmd
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // *** adds the i2c start bit into cmd
    i2c_master_start(cmd);
    // *** This adds the alpha i2c driver address to cmd
    i2c_master_write_byte(cmd, ( MATRIX_DRIVER_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // *** Add the commmand payload you want to send to device
    i2c_master_write_byte(cmd, *data_wr, ACK_CHECK_EN);
    // *** adds the i2c stop bit to cmd
    i2c_master_stop(cmd);
    // *** This command is what puts the cmd payload onto the i2c bus
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

 /////////////////////////////////////////////////// ///////////////////////////////////////////////////

static esp_err_t write_display(i2c_port_t i2c_num, uint8_t* data_wr){
  // *** This creates a structure (class) called cmd
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // *** adds the i2c start bit into cmd
    i2c_master_start(cmd);
    // *** This adds the alpha i2c driver address to cmd
    i2c_master_write_byte(cmd, ( MATRIX_DRIVER_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    //i2c_master_write_byte(cmd, 0x21, ACK_CHECK_EN);
   // i2c_master_write_byte(cmd, 0xEF, ACK_CHECK_EN);
   // i2c_master_write_byte(cmd, 0x81, ACK_CHECK_EN);
    // *** Add the commmand payload you want to send to device
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    for(int i = 0; i < 8; i++){
      i2c_master_write_byte(cmd, data_wr[i], ACK_CHECK_EN);
    }
    // *** adds the i2c stop bit to cmd
    i2c_master_stop(cmd);
    // *** This command is what puts the cmd payload onto the i2c bus
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static void i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}

uint8_t* calc_nums(char* val){
  uint8_t* data = (uint8_t*) malloc(DATA_LENGTH);
  /*printf("VAL: ");
  for(int i =0; i < 4; i++)
    printf("%c", val[i]);
  printf("\n");*/
  switch(val[0]) {
    
     
    case '0' :
    data[1] = 0b00001100;
    data[0] = 0b00111111;
    break;

    case '1' :
    data[1] = 0b00000000;
    data[0] = 0b00000110;
    break;

    case '2' :
    data[1] = 0b00000000;
    data[0] = 0b11011011;
    break;

    case '3' :
    data[1] = 0b00000000;
    data[0] = 0b10001111;
    break;

    case '4' :
    data[1] = 0b00000000;
    data[0] = 0b11100110;
    break;

    case '5' :
    data[1] = 0b00100000;
    data[0] = 0b01101001;
    break;

    case '6' :
    data[1] = 0b00000000;
    data[0] = 0b11111101;
    break;

    case '7' :
    data[1] = 0b00000000;
    data[0] = 0b00000111;
    break;

    case '8' :
    data[1] = 0b00000000;
    data[0] = 0b11111111;
    break;

    case '9' :
    data[1] = 0b00000000;
    data[0] = 0b11101111;
    break;
  }

  switch(val[1]) {
    
    case '0' :
    data[3] = 0b01001100;
    data[2] = 0b00111111;
    break;

    case '1' :
    data[3] = 0b01000000;
    data[2] = 0b00000110;
    break;

    case '2' :
    data[3] = 0b01000000;
    data[2] = 0b11011011;
    break;

    case '3' :
    data[3] = 0b01000000;
    data[2] = 0b10001111;
    break;

    case '4' :
    data[3] = 0b01000000;
    data[2] = 0b11100110;
    break;

    case '5' :
    data[3] = 0b01100000;
    data[2] = 0b01101001;
    break;

    case '6' :
    data[3] = 0b01000000;
    data[2] = 0b11111101;
    break;

    case '7' :
    data[3] = 0b01000000;
    data[2] = 0b00000111;
    break;

    case '8' :
    data[3] = 0b01000000;
    data[2] = 0b11111111;
    break;

    case '9' :
    data[3] = 0b01000000;
    data[2] = 0b11101111;
    break;
  }

  switch(val[3]) {
    
    case '0' :
    data[5] = 0b00001100;
    data[4] = 0b00111111;
    break;

    case '1' :
    data[5] = 0b00000000;
    data[4] = 0b00000110;
    break;

    case '2' :
    data[5] = 0b00000000;
    data[4] = 0b11011011;
    break;

    case '3' :
    data[5] = 0b00000000;
    data[4] = 0b10001111;
    break;

    case '4' :
    data[5] = 0b00000000;
    data[4] = 0b11100110;
    break;

    case '5' :
    data[5] = 0b00100000;
    data[4] = 0b01101001;
    break;

    case '6' :
    data[5] = 0b00000000;
    data[4] = 0b11111101;
    break;

    case '7' :
    data[5] = 0b00000000;
    data[4] = 0b00000111;
    break;

    case '8' :
    data[5] = 0b00000000;
    data[4] = 0b11111111;
    break;

    case '9' :
    data[5] = 0b00000000;
    data[4] = 0b11101111;
    break;
  }
    data[7] = 0b00000000;
    data[6] = 0b01110001;
  /*switch(val[3]) {
    
    case '0' :
    data[7] = 0b00001100;
    data[6] = 0b00111111;
    break;

    case '1' :
    data[7] = 0b00000000;
    data[6] = 0b00000110;
    break;

    case '2' :
    data[7] = 0b00000000;
    data[6] = 0b11011011;
    break;

    case '3' :
    data[7] = 0b00000000;
    data[6] = 0b10001111;
    break;

    case '4' :
    data[7] = 0b00000000;
    data[6] = 0b11100110;
    break;

    case '5' :
    data[7] = 0b00100000;
    data[6] = 0b01101001;
    break;

    case '6' :
    data[7] = 0b00000000;
    data[6] = 0b11111101;
    break;

    case '7' :
    data[7] = 0b00000000;
    data[6] = 0b00000111;
    break;

    case '8' :
    data[7] = 0b00000000;
    data[6] = 0b11111111;
    break;

    case '9' :
    data[7] = 0b00000000;
    data[6] = 0b11101111;
    break;
  }*/

  return data;
  free(data);

} 

static void setup(){//setup
  int ret;
 // uint8_t* data = (uint8_t*) malloc(DATA_LENGTH);
  
 // uint32_t hour = 13;
  //uint32_t min = 30;
  //data = calc_nums(hour, min, data);

 // ret = i2c_example_master_write_slave(I2C_EXAMPLE_MASTER_NUM, data);

    uint8_t osc = 0x21;
    uint8_t* oscp = &osc;
    ret = i2c_example_master_write_slave(I2C_EXAMPLE_MASTER_NUM, oscp);

    uint8_t bright = 0xEF;
    uint8_t* brightp = &bright;
    ret = i2c_example_master_write_slave(I2C_EXAMPLE_MASTER_NUM, brightp);

    uint8_t blink = 0x81;
    uint8_t* blinkp = &blink;
    ret = i2c_example_master_write_slave(I2C_EXAMPLE_MASTER_NUM, blinkp);

  //ret = i2c_example_master_write_nums(I2C_EXAMPLE_MASTER_NUM, data);

}

static void obtain_time(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    //initialise_wifi();
   // xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
     //                   false, true, portMAX_DELAY);
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while(timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    ESP_ERROR_CHECK( esp_wifi_stop() );
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}


uint8_t macAddr[6];
int TIMER_INTERVAL0_SEC;

typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;
xQueueHandle timer_queue1;


void IRAM_ATTR timer_group0_isr(void *para)
{
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value = 
        ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
        | TIMERG0.hw_timer[timer_idx].cnt_low;

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
        evt.type = TEST_WITHOUT_RELOAD;
        TIMERG0.int_clr_timers.t0 = 1;
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
        TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
    } else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
        evt.type = TEST_WITH_RELOAD;
        TIMERG0.int_clr_timers.t1 = 1;
    } else {
        evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);
}
/*
void IRAM_ATTR timer_group1_isr(void *para)
{
    int timer_idx = (int) para;

     Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt 
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value = 
        ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
        | TIMERG0.hw_timer[timer_idx].cnt_low;

     Prepare basic event data
       that will be then sent back to the main program task 
    timer_event_t evt;
    evt.timer_group = 1;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

     Clear the interrupt
       and update the alarm time for the timer with without reload 
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
        evt.type = TEST_WITHOUT_RELOAD;
        TIMERG0.int_clr_timers.t0 = 1;
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
        TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
    } else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
        evt.type = TEST_WITH_RELOAD;
        TIMERG0.int_clr_timers.t1 = 1;
    } else {
        evt.type = -1; // not supported even type
    }

    After the alarm has been triggered
      we need enable it again, so it is triggered the next time 
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
     Now just send the event data back to the main program task 
    xQueueSendFromISR(timer_queue1, &evt, NULL);
}*/
//Timer
static void example_tg0_timer_init(int timer_idx, 
    bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, 
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}


static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 26);    //Set GPIO 18 as PWM0A, to which servo is connected
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

void servo_on()
{
    uint32_t angle, count;
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();
     //timer_event_t evt;

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    /*
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
        example_tg0_timer_init(TIMER_1, TEST_WITHOUT_RELOAD, TIMER_INTERVAL1_SEC);
        count = 0;
    while(1){

            timer_event_t evt;
            xQueueReceive(timer_queue, &evt, portMAX_DELAY);

            printf("Angle of rotation: %d\n", count);
            angle = servo_per_degree_init(count);
            printf("pulse width: %dus\n", angle);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
            if(count >= SERVO_MAX_DEGREE){
                count = 0;
            }
            else{
                count++;
            }
        
    }*/
    for(int i = 0; i < 1; i++) {
        for (count = 0; count < SERVO_MAX_DEGREE; count++) {
            printf("Angle of rotation: %d\n", count);
            angle = servo_per_degree_init(count);
            printf("pulse width: %dus\n", angle);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
            vTaskDelay(10);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        }
    }
}
float currtemp = 0;
static void checktemp(){
    while(1){
    //Continuously sample ADC1
        if(on){
    //char* distp = (char *) malloc(6);
            //gpio set level
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t mV = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        float Vout = mV/ 1000.0;
        float R1 = ((3.3*10000) / Vout)  - 10000; 

        //trendline
        //y=30212e-0.037x

        //float temp = -22 .832 * log(R1) + 269.29;
        float temp = -30.79 * log(R1) + 308.96;

        currtemp = (temp * 1.8) + 32;



        /*float voltage = mV/1000.0;
        float power = powf(voltage,-1.18);
        float meters = (60.3*power); 
        meters = meters/100.0;*/
        /*
        sprintf(distp,"%02.2f\r\n", meters);
        uart_write_bytes(uart_num_c, distp, 6);*/
        printf("temp: %3.2f\n",currtemp);
    }
    else{
       currtemp = 0.0;  
    } 
       // printf("%3.2f\n",temp1);
        //vTaskDelay(pdMS_TO_TICKS(1000))
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

// An HTTP GET handler for hello world
esp_err_t hello_get_handler(httpd_req_t *req)
{
    // Send hello world response
    const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, resp_str, strlen(resp_str));

    return ESP_OK;
}

httpd_uri_t hello = {
    .uri       = "/hello",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = "Hello World!\n"
};

//  An HTTP GET handler to return mac address of the ESP32
esp_err_t mac_get_handler(httpd_req_t *req)
{
    // Convert mac address to string
    char macChar[18] = {0};
    sprintf(macChar, "%02X:%02X:%02X:%02X:%02X:%02X\n", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);

    // Send response
    const char* resp_buf = (const char*) macChar;
    httpd_resp_send(req, resp_buf, strlen(resp_buf));

    return ESP_OK;
}

httpd_uri_t mac = {
    .uri       = "/mac",
    .method    = HTTP_GET,
    .handler   = mac_get_handler,
    .user_ctx  = NULL
};

// This demonstrates turning on an LED with real-time commands
esp_err_t power_put_handler(httpd_req_t *req)
{
    char buf;
    int ret;

    // Received
    if ((ret = httpd_req_recv(req, &buf, 1)) < 0) {
        return ESP_FAIL;
    }

    // LED off
    if (buf == '0') {
        ESP_LOGI(TAG, "Device Off");
        gpio_set_level(LEDPIN, 0);
        on = false;
    }
    // LED on
    else {
        ESP_LOGI(TAG, "Device On");
        gpio_set_level(LEDPIN, 1);
        on = true;
    }

    /* Respond with empty body */
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

httpd_uri_t power = {
    .uri       = "/power",
    .method    = HTTP_PUT,
    .handler   = power_put_handler,
    .user_ctx  = NULL
};

esp_err_t button_put_handler(httpd_req_t *req)
{
    char buf;
    int ret;

    // Received
    if ((ret = httpd_req_recv(req, &buf, 1)) < 0) {
        return ESP_FAIL;
    }

    // LED off
    if (buf == '0') {
        ESP_LOGI(TAG, "Button Off");
        gpio_set_level(Button_Out, 0);
    }
    // LED on
    else {
        ESP_LOGI(TAG, "Button On");
        gpio_set_level(Button_Out, 1);
    }

    /* Respond with empty body */
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

httpd_uri_t button = {
    .uri       = "/button",
    .method    = HTTP_PUT,
    .handler   = button_put_handler,
    .user_ctx  = NULL
};


esp_err_t temp_get_handler(httpd_req_t *req)
{
    // Convert mac address to string
    char *temperature = (char *) malloc(7);
    //float temp1;
    //temp1 = gettemp();
    sprintf(temperature, "%3.2f\n",currtemp);

    // Send response
    const char* resp_buf = (const char*) temperature;
    httpd_resp_send(req, resp_buf, strlen(resp_buf));
    free(temperature);
    return ESP_OK;
}


httpd_uri_t gettemp = {
    .uri       = "/gettemp",
    .method    = HTTP_GET,
    .handler   = temp_get_handler,
    .user_ctx  = NULL
};

esp_err_t settemp_get_handler(httpd_req_t *req)
{
    // Convert mac address to string
    char *temperature = (char *) malloc(7);
    //float temp1;
    //temp1 = gettemp();
    sprintf(temperature, "%d\n",templevel);

    // Send response
    const char* resp_buf = (const char*) temperature;
    httpd_resp_send(req, resp_buf, strlen(resp_buf));
    free(temperature);
    return ESP_OK;
}

httpd_uri_t getsettemp = {
    .uri       = "/getsettemp",
    .method    = HTTP_GET,
    .handler   = settemp_get_handler,
    .user_ctx  = NULL
};

esp_err_t temp_put_handler(httpd_req_t *req){
  ESP_LOGI(TAG, "Temp set");

    char buf[10];
    int ret;

    // Received
    if ((ret = httpd_req_recv(req, buf, sizeof(buf))) < 0) {
        return ESP_FAIL;
    }
    char t[2];
   

    t[0] = buf[0];
    t[1] = buf[1];
    templevel = atoi(t);
    printf("New set temp = %d\n", templevel);

   

    /* Respond with empty body */
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
    // Send response
}

httpd_uri_t settemp = {
    .uri       = "/settemp",
    .method    = HTTP_PUT,
    .handler   = temp_put_handler,
    .user_ctx  = NULL
};

esp_err_t servo_put_handler(httpd_req_t *req)
{
     ESP_LOGI(TAG, "servo On");

    //NTP TIMEEEEE
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }
    char strftime_buf[64];

    // Set timezone to Eastern Standard Time and print local time
    setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in New York is: %s", strftime_buf);


    char time[8];
    int hour1,min1,sec1;
    strncpy(time,strftime_buf+11,8); //only look at time of day
    printf("time = %s",time);
    char h1[2];
    h1[0] = time[0];
    h1[1] = time[1];
    hour1 = atoi(h1);
    printf("HOURS: %d\n",hour1);
    char m1[2];
    m1[0] = time[3];
    m1[1] = time[4];
    min1 = atoi(m1);
    printf("MIN: %d\n",min1);
    char s1[2];
    s1[0] = time[6];
    s1[1] = time[7];
    sec1 = atoi(s1);
    printf("SEC: %d\n",sec1);

    printf("hour = %d, min = %d, sec = %d\n",hour1,min1,sec1);

    char buf[10];
    int ret;

    // Received
    if ((ret = httpd_req_recv(req, buf, sizeof(buf))) < 0) {
        return ESP_FAIL;
    }
    
    char hour[2];
   

    hour[0] = buf[0];
    hour[1] = buf[1];
    int h = atoi(hour);

    
    char min[2];
    min[0] = buf[2];
    min[1] = buf[3];
   
    int m = atoi(min);
    printf("Hours:%d\n",h);
    printf("Minutes:%d\n",m);
     //LED off
    int currtime = (hour1*3600) + (min1*60) + sec1;
    int settime  = (h*3600) + (m*60);

    int timer = settime - currtime; 
   
    //TIMER_INTERVAL0_SEC = 1;
    // LED on

       // ESP_LOGI(TAG, "servo On");
        //gpio_set_level(LEDPIN, 1);
        timer_event_t evt;
        timer_queue = xQueueCreate(10, sizeof(timer_event_t));
       TIMER_INTERVAL0_SEC = timer;
        //printf("%d",buf);
        example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
        //for(;;) {
            if(xQueueReceive(timer_queue, &evt, portMAX_DELAY)) {
                servo_on();
            }
        //}
        
    

    /* Respond with empty body */
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

httpd_uri_t servo = {
    .uri       = "/servo",
    .method    = HTTP_PUT,
    .handler   = servo_put_handler,
    .user_ctx  = NULL
};

esp_err_t now_put_handler(httpd_req_t *req)
{
    char buf;
    int ret;

    // Received
    if ((ret = httpd_req_recv(req, &buf, sizeof(buf))) < 0) {
        return ESP_FAIL;
    }

    
   
    //TIMER_INTERVAL0_SEC = 1;
    // LED on
        ESP_LOGI(TAG, "servo On");
        //gpio_set_level(LEDPIN, 1);
        /*timer_event_t evt;
        timer_queue = xQueueCreate(10, sizeof(timer_event_t));
       TIMER_INTERVAL0_SEC = 1;
        printf("%d",buf);
        example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
        for(;;) {
            if(xQueueReceive(timer_queue, &evt, portMAX_DELAY)) {
                servo_on();
            }
        }*/
        servo_on();
        
    

    /* Respond with empty body */
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

httpd_uri_t now = {
    .uri       = "/now",
    .method    = HTTP_PUT,
    .handler   = now_put_handler,
    .user_ctx  = NULL
};

// Code for the httpd server
httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
       // httpd_register_uri_handler(server, &hello);
        //httpd_register_uri_handler(server, &mac);
        httpd_register_uri_handler(server, &power);
       // httpd_register_uri_handler(server, &button);
        httpd_register_uri_handler(server, &gettemp);
        httpd_register_uri_handler(server, &settemp);
        httpd_register_uri_handler(server, &getsettemp);
       // httpd_register_uri_handler(server, &servo);
       // httpd_register_uri_handler(server, &now);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    httpd_handle_t *server = (httpd_handle_t *) ctx;

    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        ESP_LOGI(TAG, "Got IP: '%s'",
                ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);

        /* Start the web server */
        if (*server == NULL) {
            *server = start_webserver();
        }
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
        ESP_ERROR_CHECK(esp_wifi_connect());
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);

        /* Stop the web server */
        if (*server) {
            stop_webserver(*server);
            *server = NULL;
        }
        break;
    default:
        break;
    }
    return ESP_OK;
}

// wifi init code
static void initialise_wifi(void *arg)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, arg));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void check_efuse()
{  
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void temp_init(){
    check_efuse();

if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}

/*static void check_temp(){
    check_efuse();

if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}*/


static void heatcontrol(){
    while(1){
        if(on){
            if(currtemp < templevel ){
                gpio_set_level(Heat_On,1);
            }
            else{
                gpio_set_level(Heat_On,0);
            }
        }
        else{
            gpio_set_level(Heat_On,0);
        }


        vTaskDelay(100/portTICK_PERIOD_MS);
    }

}

void button_task(){
  uint32_t io_num;
  while(1) {

    if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
       // printf("io_Num = %d\n",io_num);
        if(io_num == 4){
            if(on){
                on = false;
                ESP_LOGI(TAG, "Device Off");
                gpio_set_level(LEDPIN,0);
            }
            else{
                on = true;
                ESP_LOGI(TAG, "Device On");
                gpio_set_level(LEDPIN,1);

            }
        }
        else if(io_num == 5){
            templevel++;
            printf("New set temp = %d\n", templevel);
        }
        else if(io_num == 36){
            templevel--;
            printf("New set temp = %d\n", templevel);
        }

    
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

}


void alphanumeric(){

        //Continuously sample ADC1
    char* distp = (char *) malloc(6);
    //uint8_t* distp1 = (uint8_t *) malloc(6);
    uint8_t* data= (uint8_t *) malloc(DATA_LENGTH);

    while(1){
    if(on){
        sprintf(distp,"%2.1f", currtemp);
    }
    else{
        sprintf(distp,"0000");
    }
        //distp1 = distp; 
        /*for(int i = 0; i < 4; i++){
          printf("%c",distp[i]);
        }
        printf("\n");*/
        data = calc_nums(distp);

        write_display(I2C_EXAMPLE_MASTER_NUM, data);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

}




void app_main()
{
    // Initilize GPIO for debug
    gpio_pad_select_gpio(LEDPIN);
    gpio_set_direction(LEDPIN, GPIO_MODE_OUTPUT);
     gpio_pad_select_gpio(Heat_On);
    gpio_set_direction(Heat_On, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(Button_Out);
    gpio_set_direction(Button_Out, GPIO_MODE_OUTPUT);


    gpio_config_t io_conf;

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
//create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);
    
    temp_init();

    // Get Mac address
    esp_read_mac(macAddr, ESP_MAC_WIFI_STA);;

i2c_example_master_init();
    setup();
    xTaskCreate(alphanumeric, "alphanumeric", 1024*4, NULL, configMAX_PRIORITIES, NULL);
    // Httpd Sever and WiFi
    static httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(nvs_flash_init());
    initialise_wifi(&server);
     xTaskCreate(heatcontrol, "heatcontrol", 1024*4, NULL, configMAX_PRIORITIES, NULL);
     xTaskCreate(checktemp, "checktemp", 1024*4, NULL, configMAX_PRIORITIES, NULL);
     //xTaskCreate(button_task, "button_task", 1024*4, NULL, configMAX_PRIORITIES, NULL);

}
