#include "camera.h"

uint8_t g_master_rx_buff[I2C_DATA_LENGTH];
uint8_t g_master_tx_buff[I2C_DATA_LENGTH];

void Pixy_Init(void) {
    i2c_master_config_t masterConfig;
    uint32_t sourceClock;

    /*
     * masterConfig->baudRate_Bps = 100000U;
     * masterConfig->enableStopHold = false;
     * masterConfig->glitchFilterWidth = 0U;
     * masterConfig->enableMaster = true;
     */

    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    sourceClock = I2C_MASTER_CLK_FREQ;

    I2C_MasterInit(EXAMPLE_I2C_MASTER_BASEADDR, &masterConfig, sourceClock);
}

ReturnType Pixy_GetVectors(VectorType *pVectors, uint8_t *pVectorCount) {
    ReturnType retValue = E_NOT_OK;
    i2c_master_transfer_t masterXfer;
    uint8_t deviceAddress = 0x01U;

    // I2C write command
    g_master_tx_buff[0] = 0xae;
    g_master_tx_buff[1] = 0xc1;
    g_master_tx_buff[2] = 0x30;
    g_master_tx_buff[3] = 0x2;
    g_master_tx_buff[4] = 0x1; // all features
    g_master_tx_buff[5] = 0x1; // vectors only

    memset(&masterXfer, 0, sizeof(masterXfer));

    // I2C transfer
    masterXfer.slaveAddress = I2C_MASTER_SLAVE_ADDR_7BIT;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = deviceAddress;
    masterXfer.subaddressSize = 0;
    masterXfer.data = g_master_tx_buff;
    masterXfer.dataSize = 6;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferBlocking(EXAMPLE_I2C_MASTER_BASEADDR, &masterXfer);

    delay(3000); // Wait for response

    // Read data from Pixy (max 32 bytes set in config.h)
    masterXfer.direction = kI2C_Read;
    masterXfer.data = g_master_rx_buff;
    masterXfer.dataSize = I2C_DATA_LENGTH;

    I2C_MasterTransferBlocking(EXAMPLE_I2C_MASTER_BASEADDR, &masterXfer);

    // Parse vectors
    uint8_t index = 6;
    uint8_t currentVector = 0;
    uint8_t packetLength = g_master_rx_buff[3] + 4;

    while (index < packetLength && currentVector < 8) {
        uint8_t featureType = g_master_rx_buff[index];
        uint8_t featureLen = g_master_rx_buff[index + 1];

        if (featureType == 1) {
            uint8_t numVec = featureLen / 6;
            uint8_t *ptr = &g_master_rx_buff[index + 2];

            for (uint8_t i = 0; i < numVec && currentVector < 8; i++) {
                uint8_t o = i * 6;
                pVectors[currentVector].m_x0 = ptr[o];
                pVectors[currentVector].m_y0 = ptr[o + 1];
                pVectors[currentVector].m_x1 = ptr[o + 2];
                pVectors[currentVector].m_y1 = ptr[o + 3];
                pVectors[currentVector].m_index = ptr[o + 4];
                pVectors[currentVector].m_flags = ptr[o + 5];
                currentVector++;
            }
        }

        index += 2 + featureLen;
    }

    *pVectorCount = currentVector;
    if (currentVector > 0) {
        retValue = E_OK;
    }

    return retValue;
}

ReturnType Pixy_GetVector(VectorType *pVector1, VectorType *pVector2) {
    ReturnType retValue = E_OK;

    i2c_master_transfer_t masterXfer;

    // Prepare the command for Pixy
    g_master_tx_buff[0] = 0xae;
    g_master_tx_buff[1] = 0xc1;
    g_master_tx_buff[2] = 0x30;
    g_master_tx_buff[3] = 0x2;
    g_master_tx_buff[4] = 0x1; // 0-main features, 1-all features
    g_master_tx_buff[5] = 0x1; // vectors only

    memset(&masterXfer, 0, sizeof(masterXfer));

    /* subAddress = 0x01, data = g_master_txBuff - write to slave.
      start + slaveaddress(w) + subAddress + length of data buffer + data buffer + stop*/
    uint8_t deviceAddress = 0x01U;
    masterXfer.slaveAddress = I2C_MASTER_SLAVE_ADDR_7BIT;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = (uint32_t)deviceAddress;
    masterXfer.subaddressSize = 0;
    masterXfer.data = g_master_tx_buff;
    masterXfer.dataSize = 6;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferBlocking(EXAMPLE_I2C_MASTER_BASEADDR, &masterXfer);

    delay(3000);

    /* subAddress = 0x01, data = g_master_rxBuff - read from slave.
    start + slaveaddress(w) + subAddress + repeated start + slaveaddress(r) + rx data buffer + stop
  */
    masterXfer.slaveAddress = I2C_MASTER_SLAVE_ADDR_7BIT;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = (uint32_t)deviceAddress;
    masterXfer.subaddressSize = 0;
    masterXfer.data = g_master_rx_buff;
    masterXfer.dataSize = 20U;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferBlocking(EXAMPLE_I2C_MASTER_BASEADDR, &masterXfer);

    *pVector1 = *(VectorType *)(&g_master_rx_buff[8]);
    *pVector2 = *(VectorType *)(&g_master_rx_buff[14]);

    // Check that the returned data from the camera is valid.
    if (254 == (*pVector1).m_x0) {
        retValue = E_NOT_OK;
    }

    return retValue;
}
