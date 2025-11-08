/*

  serial.c - serial port implementation for STM32F3xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include <string.h>

#include "serial.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"

#include "main.h"

#define USART USART3
#define USART_IRQHandler USART3_IRQHandler

static stream_rx_buffer_t rxbuf = {0};
static stream_tx_buffer_t txbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command;

static bool uart_release (uint8_t instance);
static const io_stream_status_t *get_uart_status (uint8_t instance);

static io_stream_status_t stream_status[] = {
    {
        .baud_rate = 115200,
        .format = {
            .width = Serial_8bit,
            .stopbits = Serial_StopBits1,
            .parity = Serial_ParityNone,
        }
    }
};

static io_stream_properties_t serial[] = {
    {
      .type = StreamType_Serial,
      .instance = 0,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = Off,
      .claim = serialInit,
      .release = uart_release,
      .get_status = get_uart_status
    }
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

    static const periph_pin_t tx0 = {
        .function = Output_TX,
        .group = PinGroup_UART1,
        .port  = GPIOB,
        .pin   = 10,
        .mode  = { .mask = PINMODE_OUTPUT }
    };

    static const periph_pin_t rx0 = {
        .function = Input_RX,
        .group = PinGroup_UART1,
        .port  = GPIOB,
        .pin   = 11,
        .mode  = { .mask = PINMODE_NONE }
    };

    hal.periph_port.register_pin(&rx0);
    hal.periph_port.register_pin(&tx0);

    stream_register_streams(&streams);
}

static const io_stream_status_t *get_uart_status (uint8_t instance)
{
    stream_status[instance].flags = serial[instance].flags;

    return &stream_status[instance];
}


static bool uart_release (uint8_t instance)
{
    bool ok;

    if((ok = serial[instance].flags.claimed))
        serial[instance].flags.claimed = Off;

    return ok;
}

#ifdef RS485_DIR_PORT

static void rs485SetDirection (bool tx)
{
    DIGITAL_OUT(RS485_DIR_PORT, RS485_DIR_PIN, tx);
}

#endif // RS485_DIR_PORT

//
// Returns number of free characters in serial input buffer
//
static uint16_t serialRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;

    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serialRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serialRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Writes a character to the serial output stream
//
static bool serialPutC (const uint8_t c)
{
    uint16_t next_head = BUFNEXT(txbuf.head, txbuf);    // Get pointer to next free slot in buffer

    while(txbuf.tail == next_head) {                    // While TX buffer full
        if(!hal.stream_blocking_callback())             // check if blocking for space,
            return false;                               // exit if not (leaves TX buffer in an inconsistent state)
    }

    txbuf.data[txbuf.head] = c;                         // Add data to buffer,
    txbuf.head = next_head;                             // update head pointer and
    USART->CR1 |= USART_CR1_TXEIE;                      // enable TX interrupts

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serialWriteS (const char *s)
{
    char c, *ptr = (char *)s;

    while((c = *ptr++) != '\0')
        serialPutC((uint8_t)c);
}

//
// Writes a number of characters from string to the serial output stream followed by EOL, blocks if buffer full
//
/*
static void serialWrite(const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serialPutC(*ptr++);
}
*/
//
// serialGetC - returns -1 if no data available
//
static int32_t serialGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;            // Get buffer pointer

    if(tail == rxbuf.head)
        return -1; // no data available

    int32_t data = (int32_t)rxbuf.data[tail];   // Get next character
    rxbuf.tail = BUFNEXT(tail, rxbuf);          // and update pointer

    return data;
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool serialEnqueueRtCommand (uint8_t c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *serialInit (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .is_connected = stream_connected,
        .read = serialGetC,
        .write = serialWriteS,
        .write_char = serialPutC,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRxFree,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
#if MODBUS_RTU_STREAM == 0 && defined(RS485_DIR_PORT)
        .set_direction = rs485SetDirection,
#endif
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(serial[0].flags.claimed || baud_rate != 115200)
        return NULL;

    serial[0].flags.claimed = On;

    if(!serial[0].flags.init_ok) {

        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_USART3_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Pin = GPIO_PIN_10|GPIO_PIN_11, // tx = 10 (yl), rx = 11 (bl)
            .Alternate = GPIO_AF7_USART3
        };
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        USART->CR1 = USART_CR1_RE|USART_CR1_TE;
        USART->BRR = UART_DIV_SAMPLING16(HAL_RCC_GetPCLK1Freq(), 115200);
        USART->CR1 |= (USART_CR1_UE|USART_CR1_RXNEIE);

        HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);

        serial[0].flags.init_ok = On;
    }

    stream_set_defaults(&stream, baud_rate);

    return &stream;
}

void USART_IRQHandler (void)
{
    if(USART->ISR & USART_ISR_RXNE) {
        uint8_t data = USART->RDR;
        if(!enqueue_realtime_command(data)) {                   // Check and strip realtime commands...
            uint16_t next_head = BUFNEXT(rxbuf.head, rxbuf);    // Get and increment buffer pointer
            if(next_head == rxbuf.tail)                         // If buffer full
                rxbuf.overflow = 1;                             // flag overflow
            else {
                rxbuf.data[rxbuf.head] = data;                  // if not add data to buffer
                rxbuf.head = next_head;                         // and update pointer
            }
        }
    }

    if((USART->ISR & USART_ISR_TXE) && (USART->CR1 & USART_CR1_TXEIE)) {
        uint_fast16_t tail = txbuf.tail;            // Get buffer pointer
        USART->TDR = txbuf.data[tail];              // Send next character
        txbuf.tail = tail = BUFNEXT(tail, txbuf);   // and increment pointer
        if(tail == txbuf.head)                      // If buffer empty then
            USART->CR1 &= ~USART_CR1_TXEIE;         // disable UART TX interrupt
   }
}
