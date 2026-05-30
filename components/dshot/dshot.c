#include "dshot.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// DSHOT timing constants for 40 MHz RMT
#define DSHOT_1_HIGH 50
#define DSHOT_1_LOW  17
#define DSHOT_0_HIGH 25
#define DSHOT_0_LOW  42

//static rmt_encoder_handle_t copy_encoder = NULL;

static uint16_t dshot_create_packet(uint16_t throttle, bool telemetry)
{
    throttle &= 0x7FF;
    uint16_t packet = (throttle << 1) | telemetry;
    uint16_t csum = 0, csum_data = packet;

    for (int i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0xF;
    return (packet << 4) | csum;
}
static void dshot_send_cmd(rmt_channel_handle_t channel, uint16_t throttle,rmt_encoder_handle_t encoder)
{
 if(throttle<=47)
 {
   
    
    uint16_t packet = dshot_create_packet(throttle, false);

    rmt_symbol_word_t symbols[16];

    for (int i = 0; i < 16; i++) {

        bool bit = packet & (1 << (15 - i));

        if (bit) {
            symbols[i] = (rmt_symbol_word_t){
                .level0 = 1,
                .duration0 = DSHOT_1_HIGH,
                .level1 = 0,
                .duration1 = DSHOT_1_LOW,
            };
        } else {
            symbols[i] = (rmt_symbol_word_t){
                .level0 = 1,
                .duration0 = DSHOT_0_HIGH,
                .level1 = 0,
                .duration1 = DSHOT_0_LOW,
            };
        }
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    ESP_ERROR_CHECK(
        rmt_transmit(
            channel,
            encoder,
            symbols,
            sizeof(symbols),
            &tx_config
        )
    );

    ESP_ERROR_CHECK(rmt_tx_wait_all_done(channel, portMAX_DELAY));
 }
}
void dshot_send(rmt_channel_handle_t channel, uint16_t throttle,rmt_encoder_handle_t encoder)
{
    if((throttle>=48 && throttle<=2047) ||throttle==0)
 {
   
    
    uint16_t packet = dshot_create_packet(throttle, false);

    rmt_symbol_word_t symbols[16];

    for (int i = 0; i < 16; i++) {

        bool bit = packet & (1 << (15 - i));

        if (bit) {
            symbols[i] = (rmt_symbol_word_t){
                .level0 = 1,
                .duration0 = DSHOT_1_HIGH,
                .level1 = 0,
                .duration1 = DSHOT_1_LOW,
            };
        } else {
            symbols[i] = (rmt_symbol_word_t){
                .level0 = 1,
                .duration0 = DSHOT_0_HIGH,
                .level1 = 0,
                .duration1 = DSHOT_0_LOW,
            };
        }
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    ESP_ERROR_CHECK(
        rmt_transmit(
            channel,
            encoder,
            symbols,
            sizeof(symbols),
            &tx_config
        )
    );

    ESP_ERROR_CHECK(rmt_tx_wait_all_done(channel, portMAX_DELAY));
    //ESP_LOGI("DSHOT", "encoder = %p", encoder);
    
    /*
    uint16_t packet = dshot_create_packet(throttle, false);

    rmt_symbol_word_t symbols[17];

    for (int i = 0; i < 16; i++)
    {
        bool bit = packet & (1 << (15 - i));

        if (bit)
        {
            symbols[i] = (rmt_symbol_word_t){
                .level0 = 1,
                .duration0 = DSHOT_1_HIGH,
                .level1 = 0,
                .duration1 = DSHOT_1_LOW,
            };
        }
        else
        {
            symbols[i] = (rmt_symbol_word_t){
                .level0 = 1,
                .duration0 = DSHOT_0_HIGH,
                .level1 = 0,
                .duration1 = DSHOT_0_LOW,
            };
        }
    }

    // reset gap
    symbols[16] = (rmt_symbol_word_t){
        .level0 = 0,
        .duration0 = 80,
        .level1 = 0,
        .duration1 = 0,
    };
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    ESP_ERROR_CHECK(rmt_transmit(channel, encoder, symbols, sizeof(symbols), &tx_config));

    rmt_tx_wait_all_done(channel, portMAX_DELAY);
    */
 }
}
void dshot_motor_arm(rmt_channel_handle_t channel, int delay_ms, int count, rmt_encoder_handle_t encoder)//dshot_motor_t *motor
{
    for (int i = 0; i < count; i++) {
        dshot_send(channel,0,encoder);
        //dshot_motor_set_throttle(motor, 0, false);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}
void dshot_send_command(rmt_channel_handle_t channel, uint16_t command, rmt_encoder_handle_t encoder)
{
    for (int i = 0; i < 6; i++)
    {
        dshot_send_cmd(channel, command, encoder);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}