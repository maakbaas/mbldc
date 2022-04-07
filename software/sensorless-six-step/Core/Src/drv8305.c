#include "drv8305.h"
#include "cmsis_os.h"

uint8_t isBusy = 0;
uint8_t DRV8305_Warnings[2];
uint8_t DRV8305_FaultsOVVDS[2];
uint8_t DRV8305_FaultsIC[2];
uint8_t DRV8305_FaultsVGS[2];
uint8_t DRV8305_GateControlHS[2];
uint8_t DRV8305_GateControlLS[2];
uint8_t DRV8305_CurrentGains[2];

uint8_t DRV8305_WriteBuffer[2];

HAL_StatusTypeDef DRV8305_SPI(uint8_t *tx, uint8_t *rx, uint8_t isBlocking);

HAL_StatusTypeDef DRV8305_WriteGateControl(uint8_t drvBmISink, uint8_t drvBmISource, uint8_t drvBmTdrive)
{
    HAL_StatusTypeDef spiStatus;

    uint8_t txBuffer[2] = {0b00000000, 0b00101000};
    txBuffer[1] |= drvBmTdrive; 
    txBuffer[0] = drvBmISource + (drvBmISink << 4);
    spiStatus = DRV8305_SPI(txBuffer, DRV8305_WriteBuffer, 1);

    txBuffer[1] = 0b00110000; //change ID from 5 to 6
    txBuffer[1] |= drvBmTdrive;
    return spiStatus | DRV8305_SPI(txBuffer, DRV8305_WriteBuffer, 1);

    //increase dead time
    // txBuffer[1] = 0b00111010;
    // txBuffer[0] = 0b01110110;
    // return spiStatus | DRV8305_SPI(txBuffer, DRV8305_WriteBuffer, 1);
}

HAL_StatusTypeDef DRV8305_WriteCurrentGains(uint8_t gain)
{
    uint8_t txBuffer[2] = {0b00000000, 0b01010000};
    txBuffer[0] += gain + (gain << 2) + (gain << 4);
    return DRV8305_SPI(txBuffer, DRV8305_WriteBuffer, 1);
}

HAL_StatusTypeDef DRV8305_ReadGateControlHS()
{
    uint8_t txBuffer[2] = {0b00000000, 0b10101000};
    return DRV8305_SPI(txBuffer, DRV8305_GateControlHS, 1);
}

HAL_StatusTypeDef DRV8305_ReadGateControlLS()
{
    uint8_t txBuffer[2] = {0b00000000, 0b10110000};
    return DRV8305_SPI(txBuffer, DRV8305_GateControlLS, 1);
}

HAL_StatusTypeDef DRV8305_ReadCurentGains()
{
    uint8_t txBuffer[2] = {0b00000000, 0b11010000};
    return DRV8305_SPI(txBuffer, DRV8305_CurrentGains, 1);
}

HAL_StatusTypeDef DRV8305_ReadWarnings()
{
    uint8_t txBuffer[2] = {0b00000000, 0b10001000};
    return DRV8305_SPI(txBuffer, DRV8305_Warnings, 1);
}

HAL_StatusTypeDef DRV8305_ReadFaultsOVVDS()
{
    uint8_t txBuffer[2] = {0b00000000, 0b10010000};
    return DRV8305_SPI(txBuffer, DRV8305_FaultsOVVDS, 1);
}

HAL_StatusTypeDef DRV8305_ReadFaultsIC()
{
    uint8_t txBuffer[2] = {0b00000000, 0b10011000};
    return DRV8305_SPI(txBuffer, DRV8305_FaultsIC, 1);
}

HAL_StatusTypeDef DRV8305_ReadFaultsVGS()
{
    uint8_t txBuffer[2] = {0b00000000, 0b10100000};
    return DRV8305_SPI(txBuffer, DRV8305_FaultsVGS, 1);
}

HAL_StatusTypeDef DRV8305_SPI(uint8_t *tx, uint8_t *rx, uint8_t isBlocking)
{
    isBusy = 1;
    HAL_StatusTypeDef spiStatus;

    //set chip select pin low
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    //start SPI DMA transmit/receive
    if ((spiStatus = HAL_SPI_TransmitReceive_DMA(&hspi1, tx, rx, 1)) == HAL_OK)
    {
        if (isBlocking)
        {
            while (isBusy)
            {
                taskYIELD();
            }
        }
    }

    return spiStatus;
}

//release the CS pin on complete callback
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    //restore the chip select pin when the DMA SPI action is complete
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    isBusy = 0;
}