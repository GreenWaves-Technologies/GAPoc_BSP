/*
 * Copyright (C) 2019 GreenWaves Technologies
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "stddef.h"
#include "string.h"
#include "pmsis.h"
#include "pmsis_l2_malloc.h"
#include "pmsis_task.h"
#include "uart.h"
#include "udma_uart.h"
//#include "bsp/bsp.h"
#include "nina_b112.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if 0
typedef struct
{
    struct pi_device *uart_device;
    uint8_t *rx_char;
    uint8_t *rx_buffer;//[AT_RESP_ARRAY_LENGTH];
} nina_t;
struct pi_device uart_device;
#else
#endif

/* Callback. */
typedef void (*ble_callback_f)(void *arg);

typedef enum
{
    WR_RES_OK = 0,
    WR_RES_ERR = -1,
    WR_RES_UNSOL = -2,
    WR_RES_NA = -3
} write_res_t;

static volatile unsigned char rx_char;
static volatile char rx_buffer[AT_RESP_ARRAY_LENGTH];
static volatile at_resp_state_t at_resp_state;
static volatile unsigned char new_rx_byte;

#if 0
static ble_api_t nina_b112_api = {
    .open = &nina_b112_open,
    .close = NULL,
    .ioctl = NULL,
    .send_AT_cmd = &nina_b112_AT_cmd_send,
    .send_AT_cmd_async = NULL,
    .query_AT_cmd = &nina_b112_AT_query,
    .wait_for_event = &nina_b112_AT_wait_for_event,
    .send_data = NULL,
    .send_data_async = NULL,
    .get_data = &nina_b112_get_data_blocking,
    .get_data_async = &nina_b112_get_data
};
#endif

/*******************************************************************************
 * API implementation
 ******************************************************************************/

void nina_b112_init_conf(nina_t *ble)
{
}

void nina_b112_open(nina_t *ble)
{
    struct pi_uart_conf *uart_conf = pmsis_l2_malloc(sizeof(struct pi_uart_conf));
    pi_uart_conf_init(uart_conf);
    uart_conf->baudrate_bps = (uint32_t) NINA_UART_AT_BAUDRATE_bps;
    /* Format 8N-0-1. */
    //uart_conf.bit_length = UART_8_BITS;
    //uart_conf.parity_mode = UART_PARITY_DISABLED;
    //uart_conf.stop_bit_count = UART_ONE_STOP_BIT;
    uart_conf->enable_rx = 1;
    uart_conf->enable_tx = 1;
    uart_conf->src_clock_Hz = SystemCoreClock;

    ble->rx_char = (uint8_t *) pmsis_l2_malloc(sizeof(uint8_t));
    if (ble->rx_char == NULL)
    {
        printf("Char null\n");
        return;
    }
    ble->rx_buffer = (uint8_t *) pmsis_l2_malloc(sizeof(uint8_t) * (uint32_t) AT_RESP_ARRAY_LENGTH);
    if (ble->rx_buffer == NULL)
    {
        printf("Buffer null\n");
        return;
    }
    ble->uart_device.config = (void *) uart_conf;
    pi_uart_open(&(ble->uart_device));

    new_rx_byte = 0;
}

void nina_b112_AT_cmd_send(nina_t *ble, const char* cmd)
{
    /* Optimize : malloc(sizeof(AT) + strlen(cmd) + sizeof(S3str)). */
    static uint8_t cmd_string[AT_CMD_ARRAY_LENGTH];
    strcpy((char*) cmd_string, (char*) "AT");
    strcat((char*) cmd_string, (char*) cmd);
    strcat((char*) cmd_string, (char*) S3str);
    //printf("Sending %s\n", cmd_string);
    //printf("Sending %s Sizeof : %d %d\n", cmd_string, length, strlen(cmd));
    pi_uart_write(&(ble->uart_device), cmd_string, strlen((char *) cmd_string));
}

/* Uart rx callback. */
void __nina_b112_data_received(void *arg)
{
    //nina_t *ble = (nina_t *) arg;
    static uint32_t index = 0;
    static unsigned char prev_byte = S3char;
    new_rx_byte = 1;
    //printf("Received data : %c %d\n", *(ble->rx_char), new_rx_byte);
    //printf("Received data : %c %d\n", rx_char, new_rx_byte);
    //printf("at_resp_state %d index %d\n", at_resp_state, index);
    if ((at_resp_state == AT_RESP_NOT_STARTED) &&
        (prev_byte == S3char) && (rx_char == S4char))
    {
        //printf("Here?\n");
        index = 0;
        at_resp_state = AT_RESP_IN_PROGRESS;
    }
    else if (at_resp_state == AT_RESP_IN_PROGRESS)
    {
        if ((prev_byte == S3char) && (rx_char == S4char))
        {
            rx_buffer[--index] = '\0';
            at_resp_state = AT_RESP_DONE;
        }
        else
        {
            rx_buffer[index++] = rx_char;
        }
    }
    prev_byte = rx_char;
    //printf("Prev %c Current %c Buffer %s\n", prev_byte, rx_char, rx_buffer);
}

uint32_t nina_b112_AT_send(nina_t *ble, const char* cmd)
{
    at_resp_state = AT_RESP_NOT_STARTED;
    new_rx_byte = 0;
    pi_task_t rx_cb = {0};
    pi_task_callback_no_mutex(&rx_cb, __nina_b112_data_received, ble);
    pi_uart_read_async(&(ble->uart_device), (void *) &rx_char, sizeof(uint8_t), &rx_cb);
    nina_b112_AT_cmd_send(ble, cmd);
    //printf("Cmd sent\n");

    write_res_t write_result = WR_RES_NA;
    while ((write_result == WR_RES_NA) || (write_result == WR_RES_UNSOL))
    {
        while (at_resp_state != AT_RESP_DONE)
        {
            if (new_rx_byte)
            {
                new_rx_byte = 0;
                //printf("Blocked here? %d %d %c\n", at_resp_state, new_rx_byte, rx_char);
                pi_task_callback_no_mutex(&rx_cb, __nina_b112_data_received, ble);
                pi_uart_read_async(&(ble->uart_device), (void *) &rx_char, sizeof(uint8_t), &rx_cb);
            }
        }
        DEBUG_PRINTF("Got write resp : %s\n", rx_buffer);

        uint32_t last_char_pos = strlen((const char *) rx_buffer) - 1;
        if ((rx_buffer[last_char_pos - 1] == 'O') &&
            (rx_buffer[last_char_pos - 0] == 'K'))
        {
            DEBUG_PRINTF("OK response received !\n");
            write_result = WR_RES_OK;
        }
        else if ((rx_buffer[last_char_pos - 4] == 'E') &&
                 (rx_buffer[last_char_pos - 3] == 'R') &&
                 (rx_buffer[last_char_pos - 2] == 'R') &&
                 (rx_buffer[last_char_pos - 1] == 'O') &&
                 (rx_buffer[last_char_pos - 0] == 'R'))
        {
            DEBUG_PRINTF("Error response received !\n");
            write_result = WR_RES_ERR;
        }
        else
        {
            DEBUG_PRINTF("Unsollicited/unrecognised response received : %s !\n",
                         rx_buffer);
            write_result = WR_RES_UNSOL;
            //break;
        }
    }
    return write_result;
}

void nina_b112_AT_query(nina_t *ble, const char* cmd, char* resp)
{
    at_resp_state = AT_RESP_NOT_STARTED;
    new_rx_byte = 0;
    pi_task_t rx_cb = {0};
    pi_task_callback_no_mutex(&rx_cb, __nina_b112_data_received, ble);
    pi_uart_read_async(&(ble->uart_device), ble->rx_char, sizeof(uint8_t), &rx_cb);
    nina_b112_AT_cmd_send(ble, cmd);

    while(at_resp_state != AT_RESP_DONE)
    {
        if (new_rx_byte)
        {
            new_rx_byte = 0;
            pi_uart_read_async(&(ble->uart_device), ble->rx_char, sizeof(uint8_t), &rx_cb);
        }
    }
    strcpy((char *) resp, (char *) ble->rx_buffer);
    DEBUG_PRINTF("Got query resp : %s\n", resp);
}

void nina_b112_wait_for_event(nina_t *ble, char* resp)
{
    at_resp_state = AT_RESP_NOT_STARTED;
    new_rx_byte = 0;
    pi_task_t rx_cb = {0};
    pi_task_callback_no_mutex(&rx_cb, __nina_b112_data_received, ble);
    pi_uart_read_async(&(ble->uart_device), ble->rx_char, sizeof(uint8_t), &rx_cb);

    while(at_resp_state != AT_RESP_DONE)
    {
        if (new_rx_byte)
        {
            new_rx_byte = 0;
            pi_uart_read_async(&(ble->uart_device), ble->rx_char, sizeof(uint8_t), &rx_cb);
        }
    }
    strcpy((char *) resp, (char *) ble->rx_buffer);
    DEBUG_PRINTF("Got unsollicited resp : %s\n", resp);
}

void nina_b112_get_data_blocking(nina_t *ble, uint8_t* buffer, uint32_t size)
{
    pi_uart_read(&(ble->uart_device), buffer, size);
}

void nina_b112_get_data(nina_t *ble, uint8_t* buffer, uint32_t size, struct pi_task *task)
{
    pi_uart_read_async(&(ble->uart_device), buffer, size, task);
}

#if 0
void nina_b112_send_data_blocking(nina_t *ble, const uint8_t* buffer, uint32_t size)
{
}

void nina_b112_send_data(nina_t *ble, const uint8_t* buffer, uint32_t size)
{
}


void nina_b112_AT_cmd_send(nina_t *ble, const char* cmd)
{
    /* Optimize : malloc(sizeof(AT) + strlen(cmd) + sizeof(S3str)). */
    uint32_t cmd_length = strlen(cmd);
    uint32_t length = cmd_length + 2 + 1; /* cmd length + "AT" + '\r'. */
    char *cmd_string = (char *) pmsis_l2_malloc(sizeof(char) * length);
    if (cmd_string == NULL)
    {
        return;
    }
    strcpy((char*) cmd_string, (char*) "AT");
    strcat((char*) cmd_string, (char*) cmd);
    strcat((char*) cmd_string, (char*) S3str);
    //printf("Sending %s\n", cmd_string);
    //printf("Sending %s Sizeof : %d %d\n", cmd_string, length, strlen(cmd));
    pi_uart_write(&(ble->uart_device), cmd_string, strlen(cmd_string));
}
#endif


