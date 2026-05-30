#pragma once
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    rmt_channel_handle_t channel;
    gpio_num_t pin;
} dshot_motor_t;


void dshot_send(rmt_channel_handle_t channel, uint16_t throttle, rmt_encoder_handle_t encoder);
void dshot_send_command(rmt_channel_handle_t channel,
                        uint16_t command,
                        rmt_encoder_handle_t encoder);
/**
 * @brief Convenience: arm a motor (send zero throttle repeatedly)
 */
void dshot_motor_arm(rmt_channel_handle_t channel, int delay_ms, int count, rmt_encoder_handle_t encoder);
//dshot_motor_t *motor
#ifdef __cplusplus
}
#endif
