// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "string.h"
#include "driver.h"
#include "lora.h"
#include "stdlib.h"
bool LoRa_Init(long frequency)
{
    // setup pins
    LoRa_setPins();

    // perform reset
    LoRa_Chip_Reset(0);
    HAL_Delay(100);
    LoRa_Chip_Reset(1);
    HAL_Delay(100);

    // check version
    uint8_t version = LoRa_readRegister(REG_VERSION);

    if(version != 0x12)
    {
        return 0;
    }

    // put in sleep mode
    LoRa_sleep();

    // set frequency
    LoRa_setFrequency(frequency);

    // set base addresses
    LoRa_writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    LoRa_writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    // set LNA boost
    LoRa_writeRegister(REG_LNA, LoRa_readRegister(REG_LNA) | 0x03);

    // set auto AGC
    LoRa_writeRegister(REG_MODEM_CONFIG_3, 0x04);

    // set output power to 17 dBm
    LoRa_setTxPower(19, PA_OUTPUT_PA_BOOST_PIN);

    // put in standby mode
    LoRa_idle();

    return 1;
}

void LoRa_end(void)
{
    // put in sleep mode
    LoRa_sleep();
}

bool LoRa_beginPacket(bool implicitHeader)
{
    if(LoRa_isTransmitting())
    {
        return 0;
    }

    // put in standby mode
    LoRa_idle();

    if(implicitHeader)
    {
        LoRa_implicitHeaderMode();
    }
    else
    {
        LoRa_explicitHeaderMode();
    }

    // reset FIFO address and paload length
    LoRa_writeRegister(REG_FIFO_ADDR_PTR, 0);
    LoRa_writeRegister(REG_PAYLOAD_LENGTH, 0);

    return 1;
}

bool LoRa_endPacket(bool async)
{
    // put in TX mode
    if(async)
    {
        LoRa_writeRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE
    }

    LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    if(!async)
    {
        // wait for TX done
        while((LoRa_readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {}

        // clear IRQ's
        LoRa_writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return 1;
}

bool LoRa_isTransmitting(void)
{
    if((LoRa_readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX)
    {
        return 1;
    }

    if(LoRa_readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK)
    {
        // clear IRQ's
        LoRa_writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return 0;
}

int LoRa_parsePacket(int size)
{
    int packetLength = 0;
    int irqFlags = LoRa_readRegister(REG_IRQ_FLAGS);

    if(size > 0)
    {
        LoRa_implicitHeaderMode();
        LoRa_writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    }
    else
    {
        LoRa_explicitHeaderMode();
    }

    // clear IRQ's
    LoRa_writeRegister(REG_IRQ_FLAGS, irqFlags);

    if((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
    {
        // received a packet
        _packetIndex = 0;

        // read packet length
        if(_implicitHeaderMode)
            packetLength = LoRa_readRegister(REG_PAYLOAD_LENGTH);
        else
            packetLength = LoRa_readRegister(REG_RX_NB_BYTES);

        // set FIFO address to current RX address
        LoRa_writeRegister(REG_FIFO_ADDR_PTR, LoRa_readRegister(REG_FIFO_RX_CURRENT_ADDR));

        // put in standby mode
        LoRa_idle();
    }
    else if (LoRa_readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
    {
        // not currently in RX mode

        // reset FIFO address
        LoRa_writeRegister(REG_FIFO_ADDR_PTR, 0);

        // put in single RX mode
        LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }

    return packetLength;
}

int LoRa_packetRssi(void)
{
    return (LoRa_readRegister(REG_PKT_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float LoRa_packetSnr(void)
{
    return ((int8_t)LoRa_readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long LoRa_packetFrequencyError(void)
{
    int32_t freqError = 0;

    freqError = (int32_t)(LoRa_readRegister(REG_FREQ_ERROR_MSB) & 7);
    freqError <<= 8L;
    freqError += (int32_t)(LoRa_readRegister(REG_FREQ_ERROR_MID));
    freqError <<= 8L;
    freqError += (int32_t)(LoRa_readRegister(REG_FREQ_ERROR_LSB));

    if(LoRa_readRegister(REG_FREQ_ERROR_MSB) & 8) // Sign bit is on
        freqError -= 524288; // B1000'0000'0000'0000'0000

    const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
    const float fError = (((float)(freqError) * (1L << 24)) / fXtal) * (LoRa_getSignalBandwidth() / 500000.0f); // p. 37

    return (long)(fError);
}

int LoRa_rssi(void)
{
    return (LoRa_readRegister(REG_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

int LoRa_sendPacket(const char *buffer)
{
    LoRa_beginPacket(0);
    int size = LoRa_write(buffer, strlen(buffer));
    LoRa_endPacket(1);

    return size;
}

void LoRa_readMessage(char *buffer, uint8_t length)
{
    for(int i = 0; i < length; i++)
    {
        *buffer++ = LoRa_read();
    }
}
int LoRa_receivePacket(char *buffer, uint32_t timeout)
{
    while(!_dio0_rx_isr)
    {
        if(!--timeout)
        {
            return 0;
        }

        HAL_Delay(1);
    }

    _dio0_rx_isr = 0;
    int i, size = _packetSize;
    _packetSize = 0;

    if(size > 0)
    {
        for(i = 0; i < size; i++)
        {
            while(LoRa_available())
            {
                buffer[i++] = LoRa_read();
            }
        }
    }

    return size;
}
size_t LoRa_write(const char *buffer, size_t size)
{
    uint8_t currentLength = LoRa_readRegister(REG_PAYLOAD_LENGTH);

    // check size
    if((currentLength + size) > MAX_PKT_LENGTH)
    {
        size = MAX_PKT_LENGTH - currentLength;
    }

    // write data
    size_t i;

    for(i = 0; i < size; i++)
    {
        LoRa_writeRegister(REG_FIFO, buffer[i]);
    }

    // update length
    LoRa_writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

    return size;
}

int LoRa_available(void)
{
    return (LoRa_readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRa_read(void)
{
    if(!LoRa_available())
    {
        return -1;
    }

    _packetIndex++;

    return LoRa_readRegister(REG_FIFO);
}

int LoRa_peek(void)
{
    if(!LoRa_available())
    {
        return -1;
    }

    // store current FIFO address
    int currentAddress = LoRa_readRegister(REG_FIFO_ADDR_PTR);

    // read
    uint8_t b = LoRa_readRegister(REG_FIFO);

    // restore FIFO address
    LoRa_writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

    return b;
}

void LoRa_flush(void) {}

void LoRa_receive(int size)
{
    LoRa_writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

    if(size > 0)
    {
        LoRa_implicitHeaderMode();
        LoRa_writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    }
    else
    {
        LoRa_explicitHeaderMode();
    }

    LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoRa_idle(void)
{
    LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRa_sleep(void)
{
    LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRa_setTxPower(int level, bool outputPin)
{
    if(PA_OUTPUT_RFO_PIN == outputPin)
    {
        // RFO
        if(level < 0)
        {
            level = 0;
        }
        else if(level > 14)
        {
            level = 14;
        }

        LoRa_writeRegister(REG_PA_CONFIG, 0x70 | level);
    }
    else
    {
        // PA BOOST
        if(level > 17)
        {
            if(level > 20)
            {
                level = 20;
            }

            // subtract 3 from level, so 18 - 20 maps to 15 - 17
            level -= 3;

            // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
            LoRa_writeRegister(REG_PA_DAC, 0x87);
            LoRa_setOCP(140);
        }
        else
        {
            if(level < 2)
            {
                level = 2;
            }

            //Default value PA_HF/LF or +17dBm
            LoRa_writeRegister(REG_PA_DAC, 0x84);
            LoRa_setOCP(100);
        }

        LoRa_writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
    }
}

void LoRa_setFrequency(long frequency)
{
    _frequency = frequency;

    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

    LoRa_writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    LoRa_writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    LoRa_writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int LoRa_getSpreadingFactor(void)
{
    return LoRa_readRegister(REG_MODEM_CONFIG_2) >> 4;
}

void LoRa_setSpreadingFactor(int sf)
{
    if(sf < 6)
    {
        sf = 6;
    }
    else if(sf > 12)
    {
        sf = 12;
    }

    if(sf == 6)
    {
        LoRa_writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        LoRa_writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    }
    else
    {
        LoRa_writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        LoRa_writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    LoRa_writeRegister(REG_MODEM_CONFIG_2, (LoRa_readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
    LoRa_setLdoFlag();
}

long LoRa_getSignalBandwidth(void)
{
    uint8_t bw = (LoRa_readRegister(REG_MODEM_CONFIG_1) >> 4);

    switch (bw)
    {
    case 0: return 7.8E3;
    case 1: return 10.4E3;
    case 2: return 15.6E3;
    case 3: return 20.8E3;
    case 4: return 31.25E3;
    case 5: return 41.7E3;
    case 6: return 62.5E3;
    case 7: return 125E3;
    case 8: return 250E3;
    case 9: return 500E3;
    }

    return -1;
}

void LoRa_setSignalBandwidth(long sbw)
{
    int bw;

    if(sbw <= 7.8E3)
    {
        bw = 0;
    }
    else if(sbw <= 10.4E3)
    {
        bw = 1;
    }
    else if(sbw <= 15.6E3)
    {
        bw = 2;
    }
    else if(sbw <= 20.8E3)
    {
        bw = 3;
    }
    else if(sbw <= 31.25E3)
    {
        bw = 4;
    }
    else if(sbw <= 41.7E3)
    {
        bw = 5;
    }
    else if(sbw <= 62.5E3)
    {
        bw = 6;
    }
    else if(sbw <= 125E3)
    {
        bw = 7;
    }
    else if(sbw <= 250E3)
    {
        bw = 8;
    }
    else /*if(sbw <= 250E3)*/
    {
        bw = 9;
    }

    LoRa_writeRegister(REG_MODEM_CONFIG_1, (LoRa_readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
    LoRa_setLdoFlag();
}

void LoRa_setLdoFlag(void)
{
    uint8_t ldoOn;

    // Section 4.1.1.5
    long symbolDuration = 1000 / (LoRa_getSignalBandwidth() / (1L << LoRa_getSpreadingFactor())) ;

    // Section 4.1.1.6
    if(symbolDuration > 16)
        ldoOn = 0x08;
    else
        ldoOn = 0x00;

    uint8_t config3 = (LoRa_readRegister(REG_MODEM_CONFIG_3) | ldoOn);
    LoRa_writeRegister(REG_MODEM_CONFIG_3, config3);
}

void LoRa_setCodingRate4(int denominator)
{
    if(denominator < 5)
    {
        denominator = 5;
    }
    else if(denominator > 8)
    {
        denominator = 8;
    }

    int cr = denominator - 4;

    LoRa_writeRegister(REG_MODEM_CONFIG_1, (LoRa_readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LoRa_setPreambleLength(long length)
{
    LoRa_writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    LoRa_writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoRa_setSyncWord(int sw)
{
    LoRa_writeRegister(REG_SYNC_WORD, sw);
}

void LoRa_enableCrc(void)
{
    LoRa_writeRegister(REG_MODEM_CONFIG_2, LoRa_readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LoRa_disableCrc(void)
{
    LoRa_writeRegister(REG_MODEM_CONFIG_2, LoRa_readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void LoRa_enableInvertIQ(void)
{
    LoRa_writeRegister(REG_INVERTIQ, 0x66);
    LoRa_writeRegister(REG_INVERTIQ2, 0x19);
}

void LoRa_disableInvertIQ(void)
{
    LoRa_writeRegister(REG_INVERTIQ, 0x27);
    LoRa_writeRegister(REG_INVERTIQ2, 0x1d);
}

void LoRa_setOCP(uint8_t mA)
{
    uint8_t ocpTrim = 27;

    if(mA <= 120)
    {
        ocpTrim = (mA - 45) / 5;
    }
    else if(mA <= 240)
    {
        ocpTrim = (mA + 30) / 10;
    }

    LoRa_writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void LoRa_setGain(uint8_t gain)
{
    // check allowed range
    if(gain > 6)
    {
        gain = 6;
    }

    // set to standby
    LoRa_idle();

    // set gain
    if(gain == 0)
    {
        // if gain = 0, enable AGC
        LoRa_writeRegister(REG_MODEM_CONFIG_3, 0x04);
    }
    else
    {
        // disable AGC
        LoRa_writeRegister(REG_MODEM_CONFIG_3, 0x00);

        // clear Gain and set LNA boost
        LoRa_writeRegister(REG_LNA, 0x03);

        // set gain
        LoRa_writeRegister(REG_LNA, LoRa_readRegister(REG_LNA) | (gain << 5));
    }
}

uint8_t LoRa_random(void)
{
    return LoRa_readRegister(REG_RSSI_WIDEBAND);
}

void LoRa_setPins(void)
{
    // configure CS pin for LoRa inAir4
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    // configure Reset pin LoRa inAir4
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

//	__enable_irq();
}

void LoRa_explicitHeaderMode(void)
{
    _implicitHeaderMode = 0;
    LoRa_writeRegister(REG_MODEM_CONFIG_1, LoRa_readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRa_implicitHeaderMode(void)
{
    _implicitHeaderMode = 1;
    LoRa_writeRegister(REG_MODEM_CONFIG_1, LoRa_readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void LoRa_handleDio0Rise(void)
{
    int irqFlags = LoRa_readRegister(REG_IRQ_FLAGS);

    // clear IRQ's
    LoRa_writeRegister(REG_IRQ_FLAGS, irqFlags);

    if((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
    {
        if((irqFlags & IRQ_RX_DONE_MASK) != 0)
        {
            // received a packet
            _packetIndex = 0;

            // read packet length
            int packetLength = _implicitHeaderMode ? LoRa_readRegister(REG_PAYLOAD_LENGTH) : LoRa_readRegister(REG_RX_NB_BYTES);
            _packetSize = packetLength;

            // set FIFO address to current RX address
            LoRa_writeRegister(REG_FIFO_ADDR_PTR, LoRa_readRegister(REG_FIFO_RX_CURRENT_ADDR));
            _dio0_rx_isr = 1;
        }
        else if((irqFlags & IRQ_TX_DONE_MASK) != 0)
        {
        	 _dio0_tx_isr = 1;
        }
    }
}

uint8_t LoRa_readRegister(uint8_t address)
{
    return LoRa_singleTransfer(address & 0x7F, 0x00);
}

void LoRa_writeRegister(uint8_t address, uint8_t data)
{
    LoRa_singleTransfer(address | 0x80, data);
}

uint8_t SPI_Tranfer(uint8_t address)
{
	uint8_t data;
	HAL_SPI_Transmit(&hspi1, &address, 1, 1000);
	HAL_SPI_Receive(&hspi1, &data, 1, 1000);
	return data;
}

uint8_t LoRa_singleTransfer(uint8_t address, uint8_t data)
{
	uint8_t rxByte = 0x00;

    LoRa_Chip_Select(0);

	HAL_SPI_Transmit(&hspi1, &address, 1, 10000);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

	HAL_SPI_TransmitReceive(&hspi1, &data, &rxByte, 1, 10000);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

    LoRa_Chip_Select(1);

    return rxByte;
}

void LoRa_Chip_Select(uint8_t mode)
{
    if(mode == 0)
    {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    }
    else if(mode == 1)
    {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }
}

void LoRa_Chip_Reset(uint8_t mode)
{
    if(mode == 0)
    {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    else if(mode == 1)
    {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    }
}
