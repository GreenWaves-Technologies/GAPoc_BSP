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

#ifndef __BSP_NINA_B112_H__
#define __BSP_NINA_B112_H__

//#include "ble.h"
#include "pmsis.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#ifdef DEBUG
#define DEBUG_PRINTF printf
#else
#define DEBUG_PRINTF(...) ((void) 0)
#endif  /* DEBUG */


#define S3char  '\r'
/* ASCII 0x0D, factory default S3 response formatting character for NINA Module =CR (see AT commands spec). */
#define S4char  '\n' /* ASCII 0x0A, factory default S4 response formatting character for NINA Module =LF (see AT commands spec). */
/* Same, but to use in string functions (strcmp etc.) : */
#define S3str  "\r"
#define S4str  "\n"

#define AT_CMD_ARRAY_LENGTH   32 /* arbitrary limit */
#define AT_RESP_ARRAY_LENGTH  64 /* arbitrary limit */

#define NINA_UART_AT_BAUDRATE_bps   115200 /* Baudrate. */

#if 0
struct nina_b112_conf
{
    struct ble_conf ble;
    uint32_t uart_itf;          /* uart ID. */
};
#endif

typedef struct
{
    struct pi_device uart_device;
    uint8_t *rx_char;
    uint8_t *rx_buffer;//[AT_RESP_ARRAY_LENGTH];
} nina_t;

typedef enum _at_resp_state
{
    AT_RESP_NOT_STARTED,
    AT_RESP_IN_PROGRESS,
    AT_RESP_DONE
} at_resp_state_t;

/*******************************************************************************
 * API
 ******************************************************************************/

void nina_b112_conf_init(nina_t *ble);

void nina_b112_open(nina_t *ble);

void nina_b112_AT_cmd_send(nina_t *ble, const char* cmd);

/* Not the same as above(for now). */
uint32_t nina_b112_AT_send(nina_t *ble, const char* cmd);

void nina_b112_AT_query(nina_t *ble, const char* cmd, char* resp);

void nina_b112_wait_for_event(nina_t *ble, char* resp);

void nina_b112_get_data_blocking(nina_t *ble, uint8_t* buffer, uint32_t size);

void nina_b112_get_data(nina_t *ble, uint8_t* buffer, uint32_t size, struct pi_task *task);

#if 0
void nina_b112_send_data_blocking(nina_t *ble, const uint8_t* buffer, uint32_t size);

void nina_b112_send_data(nina_t *ble, const uint8_t* buffer, uint32_t size);
#endif
#endif  /* __BSP_NINA_B112_H__ */
