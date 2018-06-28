/**
 * @file    uart.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "string.h"

#include "gd32f10x.h"
#include "uart.h"
#include "gpio.h"
#include "util.h"
#include "circ_buf.h"
#include "IO_Config.h"

// For usart
#define CDC_UART                     USART2
#define CDC_UART_ENABLE()            RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_USART2, ENABLE)
#define CDC_UART_DISABLE()           RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_USART2, DISABLE)
#define CDC_UART_IRQn                USART2_IRQn
#define CDC_UART_IRQn_Handler        USART2_IRQHandler

#define UART_PINS_PORT_ENABLE()      RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_GPIOA, ENABLE)
#define UART_PINS_PORT_DISABLE()     RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_GPIOA, DISABLE)

#define UART_TX_PORT                 GPIOA
#define UART_TX_PIN                  GPIO_PIN_2

#define UART_RX_PORT                 GPIOA
#define UART_RX_PIN                  GPIO_PIN_3

#define UART_CTS_PORT                GPIOA
#define UART_CTS_PIN                 GPIO_PIN_0

#define UART_RTS_PORT                GPIOA
#define UART_RTS_PIN                 GPIO_PIN_1


#define RX_OVRF_MSG         "<DAPLink:Overflow>\n"
#define RX_OVRF_MSG_SIZE    (sizeof(RX_OVRF_MSG) - 1)
#define BUFFER_SIZE         (1024)

circ_buf_t write_buffer;
uint8_t write_buffer_data[BUFFER_SIZE];
circ_buf_t read_buffer;
uint8_t read_buffer_data[BUFFER_SIZE];

static UART_Configuration configuration = {
    .Baudrate = 9600,
    .DataBits = UART_DATA_BITS_8,
    .Parity = UART_PARITY_NONE,
    .StopBits = UART_STOP_BITS_1,
    .FlowControl = UART_FLOW_CONTROL_NONE,
};

extern uint32_t SystemCoreClock;


static void clear_buffers(void)
{
    circ_buf_init(&write_buffer, write_buffer_data, sizeof(write_buffer_data));
    circ_buf_init(&read_buffer, read_buffer_data, sizeof(read_buffer_data));
}

int32_t uart_initialize(void)
{
    GPIO_InitPara    GPIO_InitStructure;
    NVIC_InitPara    NVIC_InitStructure;

    USART_INT_Set(CDC_UART, USART_INT_RBNE | USART_INT_TBE, DISABLE);
    clear_buffers();

    CDC_UART_ENABLE();
    UART_PINS_PORT_ENABLE();

    // TX pin
    GPIO_InitStructure.GPIO_Pin = UART_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_Init(UART_TX_PORT, &GPIO_InitStructure);
    // RX pin
    GPIO_InitStructure.GPIO_Pin = UART_RX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_IN_FLOATING;
    GPIO_Init(UART_RX_PORT, &GPIO_InitStructure);
    // CTS pin, input
    GPIO_InitStructure.GPIO_Pin = UART_CTS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_IPU;
    GPIO_Init(UART_CTS_PORT, &GPIO_InitStructure);
    // RTS pin, output low
    GPIO_ResetBits(UART_RTS_PORT, UART_RTS_PIN);
    GPIO_InitStructure.GPIO_Pin = UART_RTS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT_PP;
    GPIO_Init(UART_RTS_PORT, &GPIO_InitStructure);

    // Config NVIC
    NVIC_InitStructure.NVIC_IRQ = CDC_UART_IRQn;
    NVIC_InitStructure.NVIC_IRQPreemptPriority = 0;
    NVIC_InitStructure.NVIC_IRQSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQEnable = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_ClearPendingIRQ(CDC_UART_IRQn);

    return 1;
}

int32_t uart_uninitialize(void)
{
    USART_Enable(CDC_UART, DISABLE);
    USART_INT_Set(CDC_UART, USART_INT_TBE | USART_INT_RBNE, DISABLE);
    clear_buffers();
    return 1;
}

int32_t uart_reset(void)
{
    USART_INT_Set(CDC_UART, USART_INT_TBE | USART_INT_RBNE, DISABLE);
    clear_buffers();
    USART_INT_Set(CDC_UART, USART_INT_RBNE, ENABLE);
    return 1;
}

int32_t uart_set_configuration(UART_Configuration *config)
{
    USART_InitPara uart_handle;

    memset(&uart_handle, 0, sizeof(uart_handle));

    // Parity
    configuration.Parity = config->Parity;
    if(config->Parity == UART_PARITY_ODD) {
        uart_handle.USART_Parity = USART_PARITY_SETODD;
    } else if(config->Parity == UART_PARITY_EVEN) {
        uart_handle.USART_Parity = USART_PARITY_SETEVEN;
    } else if(config->Parity == UART_PARITY_NONE) {
        uart_handle.USART_Parity = USART_PARITY_RESET;
    } else {
        uart_handle.USART_Parity = USART_PARITY_RESET;
        configuration.Parity = UART_PARITY_NONE;
    }

    // Stop bits
    configuration.StopBits = config->StopBits;
    if(config->StopBits == UART_STOP_BITS_2) {
        uart_handle.USART_STBits = USART_STBITS_2;
    } else if(config->StopBits == UART_STOP_BITS_1_5) {
        uart_handle.USART_STBits = USART_STBITS_1_5;
    } else if(config->StopBits == UART_STOP_BITS_1) {
        uart_handle.USART_STBits = USART_STBITS_1;
    } else {
        uart_handle.USART_STBits = USART_STBITS_1;
        configuration.StopBits = UART_STOP_BITS_1;
    }

    // Only 8 bit support
    configuration.DataBits = UART_DATA_BITS_8;
    uart_handle.USART_WL = USART_WL_8B;

    // Specified baudrate
    configuration.Baudrate = config->Baudrate;
    uart_handle.USART_BRR = config->Baudrate;

    // Flow control by default if baud is over 921600
    if (config->Baudrate >= 921600) {
        configuration.FlowControl = UART_FLOW_CONTROL_RTS_CTS;
        uart_handle.USART_HardwareFlowControl = USART_HARDWAREFLOWCONTROL_RTS_CTS;
    } else {
        configuration.FlowControl = UART_FLOW_CONTROL_NONE;
        uart_handle.USART_HardwareFlowControl = USART_HARDWAREFLOWCONTROL_NONE;
    }

    // TX and RX
    uart_handle.USART_RxorTx = USART_RXORTX_TX | USART_RXORTX_RX;

    // Disable uart and tx/rx interrupt
    USART_Enable(CDC_UART, DISABLE);
    USART_INT_Set(CDC_UART, USART_INT_TBE | USART_INT_RBNE, DISABLE);

    clear_buffers();

    USART_DeInit(CDC_UART);
    USART_Init(CDC_UART, &uart_handle);

    USART_INT_Set(CDC_UART, USART_INT_RBNE, ENABLE);
    USART_Enable(CDC_UART, ENABLE);

    return 1;
}

int32_t uart_get_configuration(UART_Configuration *config)
{
    config->Baudrate = configuration.Baudrate;
    config->DataBits = configuration.DataBits;
    config->Parity   = configuration.Parity;
    config->StopBits = configuration.StopBits;
    // Flow control by default if baud is over 921600
    if (config->Baudrate >= 921600) {
        config->FlowControl = UART_FLOW_CONTROL_RTS_CTS;
    } else {
        config->FlowControl = UART_FLOW_CONTROL_NONE;
    }

    return 1;
}

int32_t uart_write_free(void)
{
    return circ_buf_count_free(&write_buffer);
}

int32_t uart_write_data(uint8_t *data, uint16_t size)
{
    uint32_t cnt = circ_buf_write(&write_buffer, data, size);

    if (cnt != 0) {
        USART_INT_Set(CDC_UART, USART_INT_TBE, ENABLE);
    } else {
        util_assert(0);
    }

    return cnt;
}

int32_t uart_read_data(uint8_t *data, uint16_t size)
{
    return circ_buf_read(&read_buffer, data, size);
}

void CDC_UART_IRQn_Handler(void)
{
    if (USART_GetIntBitState(CDC_UART, USART_INT_RBNE) != RESET) {
        uint8_t dat = USART_DataReceive(CDC_UART);
        uint32_t free = circ_buf_count_free(&read_buffer);
        if (free > RX_OVRF_MSG_SIZE) {
            circ_buf_push(&read_buffer, dat);
        } else if (RX_OVRF_MSG_SIZE == free) {
            util_assert(0);
            circ_buf_write(&read_buffer, (uint8_t*)RX_OVRF_MSG, RX_OVRF_MSG_SIZE);
        } else {
            util_assert(0);
        }
    }

    if (USART_GetIntBitState(CDC_UART, USART_INT_TBE) != RESET) {
        if (USART_GetIntBitState(CDC_UART, USART_INT_CTSF) != RESET) {
            util_assert(0);
        }
        if (circ_buf_count_used(&write_buffer) > 0) {
            USART_DataSend(CDC_UART, circ_buf_pop(&write_buffer));
        } else {
            USART_INT_Set(CDC_UART, USART_INT_TBE, DISABLE);
        }
    }
}
