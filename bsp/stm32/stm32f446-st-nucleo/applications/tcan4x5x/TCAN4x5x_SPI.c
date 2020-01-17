/*
 * TCAN4x5x_SPI.c
 * Description: This file is responsible for abstracting the lower-level microcontroller SPI read and write functions
 *
 *
 *
 * Copyright (c) 2019 Texas Instruments Incorporated.  All rights reserved.
 * Software License Agreement
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "TCAN4x5x_SPI.h"

/*
 * @brief Single word write
 *
 * @param address A 16-bit address of the destination register
 * @param data A 32-bit word of data to write to the destination register
 */
void
AHB_WRITE_32(uint16_t address, uint32_t data)
{
#if 0
    AHB_WRITE_BURST_START(address, 1);
    AHB_WRITE_BURST_WRITE(data);
    AHB_WRITE_BURST_END();
#endif
}


/*
 * @brief Single word read
 *
 * @param address A 16-bit address of the source register
 *
 * @return Returns 32-bit word of data from source register
 */
uint32_t
AHB_READ_32(uint16_t address)
{
#if 0
    uint32_t returnData;

    AHB_READ_BURST_START(address, 1);
    returnData = AHB_READ_BURST_READ();
    AHB_READ_BURST_END();

    return returnData;
#endif
return 0;	
}


/*
 * @brief Burst write start
 *
 * The SPI transaction contains 3 parts: the header (start), the payload, and the end of data (end)
 * This function is the start, where the register address and number of words are transmitted
 *
 * @param address A 16-bit address of the destination register
 * @param words The number of 4-byte words that will be transferred. 0 = 256 words
 */
void
AHB_WRITE_BURST_START(uint16_t address, uint8_t words)
{
#if 0
    //set the CS low to start the transaction
    GPIO_setOutputLowOnPin(SPI_CS_GPIO_PORT, SPI_CS_GPIO_PIN);

    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, AHB_WRITE_OPCODE);

    // Send the 16-bit address
    WAIT_FOR_TRANSMIT;
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, HWREG8(&address + 1));
    WAIT_FOR_TRANSMIT;
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, HWREG8(&address));


    WAIT_FOR_TRANSMIT;
    // Send the number of words to read
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, words);
#endif
}


/*
 * @brief Burst write
 *
 * The SPI transaction contains 3 parts: the header (start), the payload, and the end of data (end)
 * This function writes a single word at a time
 *
 * @param data A 32-bit word of data to write to the destination register
 */
void
AHB_WRITE_BURST_WRITE(uint32_t data)
{
#if 0
    WAIT_FOR_TRANSMIT;
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, HWREG8(&data + 3));
    WAIT_FOR_TRANSMIT;
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, HWREG8(&data + 2));
    WAIT_FOR_TRANSMIT;
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, HWREG8(&data + 1));
    WAIT_FOR_TRANSMIT;
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, HWREG8(&data));
#endif
}


/*
 * @brief Burst write end
 *
 * The SPI transaction contains 3 parts: the header (start), the payload, and the end of data (end)
 * This function ends the burst transaction by pulling nCS high
 */
void
AHB_WRITE_BURST_END(void)
{
#if 0
    WAIT_FOR_IDLE;
    GPIO_setOutputHighOnPin(SPI_CS_GPIO_PORT, SPI_CS_GPIO_PIN);
#endif
}


/*
 * @brief Burst read start
 *
 * The SPI transaction contains 3 parts: the header (start), the payload, and the end of data (end)
 * This function is the start, where the register address and number of words are transmitted
 *
 * @param address A 16-bit start address to begin the burst read
 * @param words The number of 4-byte words that will be transferred. 0 = 256 words
 */
void
AHB_READ_BURST_START(uint16_t address, uint8_t words)
{
#if 0
    // Set the CS low to start the transaction
    GPIO_setOutputLowOnPin(SPI_CS_GPIO_PORT, SPI_CS_GPIO_PIN);
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, AHB_READ_OPCODE);

    // Send the 16-bit address
    WAIT_FOR_TRANSMIT;
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, HWREG8(&address + 1));
    WAIT_FOR_TRANSMIT;
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, HWREG8(&address));

    // Send the number of words to read
    WAIT_FOR_TRANSMIT;
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, words);
#endif
}


/*
 * @brief Burst read start
 *
 * The SPI transaction contains 3 parts: the header (start), the payload, and the end of data (end)
 * This function where each word of data is read from the TCAN4x5x
 *
 * @return A 32-bit single data word that is read at a time
 */
uint32_t
AHB_READ_BURST_READ(void)
{
#if 0
    uint8_t readData;
    uint8_t readData1;
    uint8_t readData2;
    uint8_t readData3;
    uint32_t returnData;

    WAIT_FOR_IDLE;
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, 0x00); // pause after this
    WAIT_FOR_IDLE;

    readData = HWREG8(SPI_HW_ADDR + OFS_UCBxRXBUF);
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, 0x00);


    WAIT_FOR_IDLE;
    readData1 = HWREG8(SPI_HW_ADDR + OFS_UCBxRXBUF);
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, 0x00);

    WAIT_FOR_IDLE;
    readData2 = HWREG8(SPI_HW_ADDR + OFS_UCBxRXBUF);
    EUSCI_B_SPI_transmitData(SPI_HW_ADDR, 0x00);

    WAIT_FOR_IDLE;
    readData3 = HWREG8(SPI_HW_ADDR + OFS_UCBxRXBUF);


    returnData = (((uint32_t)readData) << 24) | (((uint32_t)readData1 << 16)) | (((uint32_t)readData2) << 8) | readData3;
    return returnData;
#endif
return 0;
}


/*
 * @brief Burst write end
 *
 * The SPI transaction contains 3 parts: the header (start), the payload, and the end of data (end)
 * This function ends the burst transaction by pulling nCS high
 */
void
AHB_READ_BURST_END(void)
{
#if 0
    WAIT_FOR_IDLE;
    GPIO_setOutputHighOnPin(SPI_CS_GPIO_PORT, SPI_CS_GPIO_PIN);
#endif
}
