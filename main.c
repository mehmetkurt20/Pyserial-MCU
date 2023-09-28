/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/
/**
 * @file    main.c
 * @brief   UART - Pyserial Interrupt Driven 
 * @details 
 */
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "led.h"
#include "board.h"
#include <string.h>
#include "pb.h"
#include "mxc_delay.h"
#include "uart.h"
#include "dma.h"
#include "nvic_table.h"
#include "uart_regs.h"
#include "rtc.h"

#define UART_BAUD 115200
#define BUFF_SIZE 8
#define HEADERPAYLOAD_SIZE 7
#define HEADER_BYTE 0xAA
#define CRC 0xFF
#define IDLE 0
#define RECEIVED 1
#define PACKET_MSG_LEN_OFFSET 8
#define PACKET_HEADER_SIZE 1
#define PACKET_PAYLOAD_SIZE 6
#define MXC_UART_RECEIVER MXC_UART0



uint8_t COMMAND_TEST1[] = {0xAA, 0x01, 0x04, 0x05, 0x06, 0x07, 0x08};
uint8_t COMMAND_TEST2[] = {0xAA, 0x02, 0x04, 0x05, 0x06, 0x07, 0x08};
uint8_t COMMAND_TEST3[] = {0xAA, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
uint8_t crc_manual = 0xFF;


// Function to initialize the CRC-8 lookup table


int current_state = IDLE;
volatile int packetsent = 0;
volatile int no_data_flag = 1;

uint8_t Buffer[BUFF_SIZE];
uint8_t subBuffer[7];

volatile int rx_buff_write_idx = 0;

// Define a timeout variable and a counter
volatile int timeout = 0;
volatile int timeout_counter = 0;
const int TIMEOUT_THRESHOLD = 1000000;
volatile int trim_value;
 // Adjust this threshold as needed


#define CRC_TABLE_SIZE 256
#define CRC_POLYNOMIAL 0x07

// CRC-8 lookup table
uint8_t crc8_table[CRC_TABLE_SIZE];

// Function to initialize the CRC-8 lookup table
void init_crc8_table() {
    for (int i = 0; i < CRC_TABLE_SIZE; i++) {
        uint8_t crc = i;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ CRC_POLYNOMIAL : (crc << 1);
        }
        crc8_table[i] = crc;
    }
}

// Function to calculate the CRC-8 checksum for a given data array
uint8_t calculate_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = crc8_table[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}


void cancel_previous_actions() {
    // Add code here to cancel or reset any ongoing actions triggered by the previous packet.
    //
    LED_Off(LED1);
    LED_Off(LED2);
    MXC_RTC_SquareWaveStop();
    // Reset any other relevant variables or states.
}

void uart_handler() {
    volatile int uartRxFifoAvail;
    // Replace with your 8-byte array

    // Create a pointer to reference the first 7 bytes
    uint8_t* BufferPayload = Buffer;  // Point to the beginning of the array

    init_crc8_table();
    uint8_t crc_result = calculate_crc8(BufferPayload, 7);

    while ((uartRxFifoAvail = MXC_UART_GetRXFIFOAvailable(MXC_UART_RECEIVER)) > 0) {
        MXC_UART_ReadRXFIFO(MXC_UART_RECEIVER, &Buffer[rx_buff_write_idx], 1);

        if (rx_buff_write_idx == 0 && Buffer[0] != HEADER_BYTE) {
            // Skip until a valid packet header is found
            continue;

        }
        rx_buff_write_idx++;

        if (rx_buff_write_idx >= BUFF_SIZE) {

            cancel_previous_actions(); // Cancel previous actions when a new packet is received
            //init_crc8_table();

            //uint8_t crc = calculate_crc8(COMMAND_TEST1, sizeof(COMMAND_TEST1));
            if(Buffer[7] == crc_result){
            packetsent = 1;
            current_state = RECEIVED;
            rx_buff_write_idx = 0;
            timeout_counter = 0; // Reset the timeout counter
            break;  // Exit the loop when a complete packet is received
            trim_value = Buffer[6]; // for instance get the trim value from the packet
            }
            else{
                printf("CRC ERROR\n");
                rx_buff_write_idx = 0;
                timeout_counter = 0;
                break;
            }
        }
    }
}

void UART0_Handler(void) {
    if (MXC_UART_GetFlags(MXC_UART_RECEIVER)) {
        uart_handler();
        MXC_UART_ClearFlags(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
    }
}

int main(void) {
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_DisableIRQ(UART0_IRQn);
    MXC_NVIC_SetVector(UART0_IRQn, UART0_Handler);
    NVIC_EnableIRQ(UART0_IRQn);

    MXC_UART_Init(MXC_UART_RECEIVER, UART_BAUD, MAP_A);
    MXC_UART_SetRXThreshold(MXC_UART_RECEIVER, 1);
    MXC_UART_EnableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);

    
    // Copy the first 7 bytes from byteArray to subArray
    while (1) {
        for (int i = 0; i < 7; i++) {
        subBuffer[i] = Buffer[i];
    }
        //printf("%x\n", calculate_crc8(COMMAND_TEST1, sizeof(COMMAND_TEST1)));
        if (current_state == IDLE) {
            // Increment the timeout counter
            timeout_counter++;

            if (packetsent == 1) {
                MXC_UART_DisableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
                // Process the packet here if needed
                if (memcmp(subBuffer, COMMAND_TEST1, HEADERPAYLOAD_SIZE) == 0) {
                    printf("Test1\n");
                    LED_On(LED1);
                }
                else if (memcmp(subBuffer, COMMAND_TEST2, HEADERPAYLOAD_SIZE) == 0) {
                    printf("Test2\n");
                    LED_On(LED2);
                }
                else if (memcmp(subBuffer, COMMAND_TEST3, HEADERPAYLOAD_SIZE) == 0) {
                    printf("Test3\n");
                    MXC_RTC_Start();
                    MXC_RTC_SquareWaveStart(MXC_RTC_F_32KHZ);
                    MXC_RTC_Trim(trim_value);
                }
                packetsent = 0;  // Reset the flag
                MXC_UART_EnableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
                timeout_counter = 0; // Reset the timeout counter
            }

            if (timeout_counter >= TIMEOUT_THRESHOLD && packetsent == 0) {
                printf("NO DATA\n");
                // Reset the timeout variables
                cancel_previous_actions(); // Cancel previous actions when a new packet is received
                timeout_counter = 0;
                timeout = 0;
                
            }
        } else if (current_state == RECEIVED) {
            MXC_UART_DisableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
            current_state = IDLE;
            MXC_UART_EnableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
        }
    }
}
/*-----------------------------------------------------------------------------------------------*/



/* #include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "led.h"
#include "board.h"
#include <string.h>
#include "pb.h"
#include "mxc_delay.h"
#include "uart.h"
#include "dma.h"
#include "nvic_table.h"
#include "uart_regs.h"

#define UART_BAUD 115200
#define BUFF_SIZE 8
#define HEADER_BYTE 0xAA
#define CRC 0xFF
#define IDLE 0
#define RECEIVED 1
#define PACKET_MSG_LEN_OFFSET 8
#define PACKET_HEADER_SIZE 1
#define PACKET_PAYLOAD_SIZE 6
#define MXC_UART_RECEIVER MXC_UART0

uint8_t COMMAND_TEST1[] = {0xAA, 0x01, 0x04, 0x05, 0x06, 0x07, 0x08, 0xFF};
uint8_t COMMAND_TEST2[] = {0xAA, 0x02, 0x04, 0x05, 0x06, 0x07, 0x08, 0xFF};
uint8_t COMMAND_TEST3[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

int current_state = IDLE;
volatile int packetsent = 0;
volatile int no_data_flag = 1;
uint8_t Buffer[BUFF_SIZE];
volatile int rx_buff_write_idx = 0;

// Define a timeout variable and a counter
volatile int timeout = 0;
volatile int timeout_counter = 0;
const int TIMEOUT_THRESHOLD = 1000000; // Adjust this threshold as needed

void uart_handler() {
    volatile int uartRxFifoAvail;

    while ((uartRxFifoAvail = MXC_UART_GetRXFIFOAvailable(MXC_UART_RECEIVER)) > 0) {
        MXC_UART_ReadRXFIFO(MXC_UART_RECEIVER, &Buffer[rx_buff_write_idx], 1);

        if (rx_buff_write_idx == 0 && Buffer[0] != HEADER_BYTE) {
            // Skip until a valid packet header is found
            continue;
        }

        rx_buff_write_idx++;

        if (rx_buff_write_idx >= BUFF_SIZE) {
            packetsent = 1;
            current_state = RECEIVED;
            rx_buff_write_idx = 0;
            timeout_counter = 0; // Reset the timeout counter
            break;  // Exit the loop when a complete packet is received
        }
    }
}

void UART0_Handler(void) {
    if (MXC_UART_GetFlags(MXC_UART_RECEIVER)) {
        uart_handler();
        MXC_UART_ClearFlags(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
    }
}

int main(void) {
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_DisableIRQ(UART0_IRQn);
    MXC_NVIC_SetVector(UART0_IRQn, UART0_Handler);
    NVIC_EnableIRQ(UART0_IRQn);

    MXC_UART_Init(MXC_UART_RECEIVER, UART_BAUD, MAP_A);
    MXC_UART_SetRXThreshold(MXC_UART_RECEIVER, 1);
    MXC_UART_EnableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);

    while (1) {
        if (current_state == IDLE) {
            // Increment the timeout counter
            timeout_counter++;

            if (packetsent == 1) {
                MXC_UART_DisableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
                // Process the packet here if needed
                packetsent = 0;  // Reset the flag
                MXC_UART_EnableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
                timeout_counter = 0; // Reset the timeout counter
            }

            if (timeout_counter >= TIMEOUT_THRESHOLD) {
                printf("No data receiving.\n");
                // Reset the timeout variables
                timeout_counter = 0;
                timeout = 0;
            }
        } else if (current_state == RECEIVED) {
            MXC_UART_DisableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
            if (Buffer[0] == HEADER_BYTE && Buffer[7] == CRC) {
                if (memcmp(Buffer, COMMAND_TEST1, BUFF_SIZE) == 0) {
                    printf("Test1\n");
                    LED_On(LED1);
                } else if (memcmp(Buffer, COMMAND_TEST2, BUFF_SIZE) == 0) {
                    printf("Test2\n");
                    LED_On(LED2);
                }
            }
            current_state = IDLE;
            MXC_UART_EnableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
        }
    }
} */

/*#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "led.h"
#include "board.h"
#include <string.h>
#include "pb.h"
#include "mxc_delay.h"
#include "uart.h"
#include "dma.h"
#include "nvic_table.h"
#include "uart_regs.h"

#define UART_BAUD 115200
#define BUFF_SIZE 8
#define HEADER_BYTE 0xAA
#define CRC 0xFF
#define IDLE 0
#define RECEIVED 1
#define PACKET_MSG_LEN_OFFSET 8
#define PACKET_HEADER_SIZE 1
#define PACKET_PAYLOAD_SIZE 6
#define MXC_UART_RECEIVER MXC_UART0

uint8_t COMMAND_TEST1[] = {0xAA, 0x01, 0x04, 0x05, 0x06, 0x07, 0x08, 0xFF};
uint8_t COMMAND_TEST2[] = {0xAA, 0x02, 0x04, 0x05, 0x06, 0x07, 0x08, 0xFF};
uint8_t COMMAND_TEST3[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

int current_state = IDLE;
volatile int packetsent = 0;
volatile int no_data_flag = 1;
uint8_t Buffer[BUFF_SIZE];
volatile int rx_buff_write_idx = 0;

// Define a timeout variable and a counter
volatile int timeout = 0;
volatile int timeout_counter = 0;
const int TIMEOUT_THRESHOLD = 1000000; // Adjust this threshold as needed

void cancel_current_action() {
    // Add code here to cancel any ongoing actions triggered by the previous packet.
    // For example, you can stop a motor, reset a state machine, or perform other actions.
}

void process_received_packet(uint8_t *packet) {
    // Add code here to process the received packet.
    if (memcmp(packet, COMMAND_TEST1, BUFF_SIZE) == 0) {
        printf("Test1\n");
        // Perform actions for Test1 packet (e.g., turn on LED1).
    } else if (memcmp(packet, COMMAND_TEST2, BUFF_SIZE) == 0) {
        printf("Test2\n");
        // Perform actions for Test2 packet (e.g., turn on LED2).
    }
}

void uart_handler() {
    volatile int uartRxFifoAvail;

    while ((uartRxFifoAvail = MXC_UART_GetRXFIFOAvailable(MXC_UART_RECEIVER)) > 0) {
        MXC_UART_ReadRXFIFO(MXC_UART_RECEIVER, &Buffer[rx_buff_write_idx], 1);

        if (rx_buff_write_idx == 0 && Buffer[0] != HEADER_BYTE) {
            // Skip until a valid packet header is found
            continue;
        }

        rx_buff_write_idx++;

        if (rx_buff_write_idx >= BUFF_SIZE) {
            cancel_current_action(); // Cancel any ongoing actions
            packetsent = 1;
            current_state = RECEIVED;
            rx_buff_write_idx = 0;
            timeout_counter = 0; // Reset the timeout counter
            break;  // Exit the loop when a complete packet is received
        }
    }
}

void UART0_Handler(void) {
    if (MXC_UART_GetFlags(MXC_UART_RECEIVER)) {
        uart_handler();
        MXC_UART_ClearFlags(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
    }
}

int main(void) {
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_DisableIRQ(UART0_IRQn);
    MXC_NVIC_SetVector(UART0_IRQn, UART0_Handler);
    NVIC_EnableIRQ(UART0_IRQn);

    MXC_UART_Init(MXC_UART_RECEIVER, UART_BAUD, MAP_A);
    MXC_UART_SetRXThreshold(MXC_UART_RECEIVER, 1);
    MXC_UART_EnableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);

    while (1) {
        if (current_state == IDLE) {
            // Increment the timeout counter
            timeout_counter++;

            if (packetsent == 1) {
                MXC_UART_DisableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
                // Process the packet here if needed
                process_received_packet(Buffer);
                packetsent = 0;  // Reset the flag
                MXC_UART_EnableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
                timeout_counter = 0; // Reset the timeout counter
            }

            if (timeout_counter >= TIMEOUT_THRESHOLD) {
                printf("NO DATA\n");
                // Reset the timeout variables
                timeout_counter = 0;
                timeout = 0;
            }
        } else if (current_state == RECEIVED) {
            MXC_UART_DisableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
            current_state = IDLE;
            MXC_UART_EnableInt(MXC_UART_RECEIVER, MXC_F_UART_INT_FL_RX_FIFO_THRESH);
        }
    }
} */



//In the IDLE state, the UART is waiting for a packet to be received. And If no data comes from pyserial,
//the UART will stay in the IDLE state. If a packet is received, the UART will go to the RECEIVED state.



/* int main(void){

    uint8_t RxData[BUFF_SIZE];
    
    memset(RxData, 0x0, BUFF_SIZE);
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_DisableIRQ(UART0_IRQn);
    MXC_NVIC_SetVector(UART0_IRQn, UART0_Handler);
    NVIC_EnableIRQ(UART0_IRQn);

    MXC_UART_Init(MXC_UART_RECEIVER, UART_BAUD, MAP_A);
    MXC_UART_EnableInt(MXC_UART_RECEIVER, MXC_F_UART_STATUS_RX_FULL_POS);
    MXC_UART_SetRXThreshold(MXC_UART_RECEIVER, 1);

    volatile int flags = MXC_UART_GetFlags(MXC_UART_RECEIVER); 
    
    
    volatile int current_state = IDLE;

    while (1) {
   
        switch(current_state) {
            
            case IDLE:
                MXC_UART_ReadRXFIFO(MXC_UART0, RxData, BUFF_SIZE);

                printf("State: Initial\n");
                if(RxData[0] == HEADER_BYTE && RxData[7] == CRC) {
                    for(int bytecount = 0; bytecount < BUFF_SIZE; bytecount++)
                        Buffer[bytecount] = RxData[bytecount];
                    current_state = STATE_READING;
                }
                else {
                    memset(RxData, 0x0, BUFF_SIZE);
                    current_state = IDLE;

                    continue;
                }
                continue;
             case STATE_READING:
                printf("State: Reading\n");
                if (memcmp(Buffer, COMMAND_TEST1, BUFF_SIZE)==0) {
                    
                    LED_Toggle(LED1);
                    
                    current_state = IDLE;
                    continue;
                }
                if (memcmp(Buffer, COMMAND_TEST2, BUFF_SIZE)==0) {

                    LED_Toggle(LED2);
                    
                    current_state = IDLE;
                    continue;
                }
                else {
                    current_state = IDLE;
                    
                    continue;
                } 
    } 

      }
} */
