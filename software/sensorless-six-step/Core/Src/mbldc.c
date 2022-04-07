#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "drv8305.h"
#include "mbldc.h"
#include "math.h"
#include "SEGGER_RTT.h"

float busVoltage_V = 0;
float mosfetTemp_C = 0;
uint8_t drvFault;
uint8_t drvPowerOK;
uint8_t drvEnbl = 1;
uint8_t drvBmISink = 11; //max gate current
uint8_t drvBmISource = 11; //max gate current
uint8_t drvBmTdrive = 1; //gate time

uint8_t statusUH = 0;
uint8_t statusUL = 0;
uint8_t statusVH = 0;
uint8_t statusVL = 0;
uint8_t statusWH = 0;
uint8_t statusWL = 0;

float dutyRatioSetpoint = 0;
float dutyRatioOL = 15;
float dutyRatioCurrent = 0;
float dutyRatioOut = 0;

uint16_t timerARR = 0;
uint16_t switchFreq_Hz = 20000;

int8_t tableU[6] = {-1, 0, 1, 1, 0, -1};
int8_t tableV[6] = {1, 1, 0, -1, -1, 0};
int8_t tableW[6] = {0, -1, -1, 0, 1, 1};
float bemf = 0;
int32_t bemf_int = 0;
float prevBemf = 0;
int16_t bemfCounter = 0;
uint8_t bemfDelay = 0;
uint16_t bemfSpeed = 0;
uint8_t bemfActive = 0;
float bemfTimeoutThreshold = 0.1;
float bemfStartThreshold = 0.3;
float bemfIntegral = 0;
float bemfIntegralThreshold = 0.8;
uint16_t bemfTimeout = 0;
uint16_t crossingTimeout = 0;

uint8_t step = 0;
uint8_t stepOvrEnbl = 0;

#define COMM_IDLE 0
#define COMM_OPEN_LOOP 1
#define COMM_WAITING_FOR_ZEROCROSSING 2
#define COMM_ZEROCROSSING_DETECTED 3
uint8_t commState = 0;

#define CTRL_DR 0
#define CTRL_CURR 1
#define CTRL_SPEED 2
uint8_t ctrlState = CTRL_SPEED;

uint16_t counterHLC = 0;
uint8_t msHLC = 10;

#define CL_TIMEOUT 10

uint16_t speedCounterHLC = 65535;
#define OL_MAX_SPEED 2000
#define OL_SWITCH_SPEED 1000
#define OL_RAMP_RATE 5000
#define OL_SPEED_TARGET 25
#define OL_FF_FACTOR 0.4
float speedTargetOL = OL_SPEED_TARGET;
#define SPEED_MAX_STEPS 25
#define SPEED_MAX_TICKS 5000
#define SPEED_BUFFER_LEN 250
uint16_t speedBuffer[SPEED_BUFFER_LEN];
uint8_t speedBufferIndex = 0;
uint8_t speedBufferIndexPrev = 0;
float speedMeas = 0;
int8_t speedDir = 0;
float speedSetpoint = 0;
float speedIntegrator = 0;
float speedKp = 0.003;
float speedFilter = 1;
float speedKi = 0.00015;

float speedTicks = 0;
float speedSteps = 0;

uint32_t bufferU[3] = {0, 0, 0};
uint32_t bufferV[3] = {0, 0, 0};
uint32_t bufferW[3] = {0, 0, 0};
uint32_t *neutral[6] =  {&(bufferW[1]), &(bufferU[1]), &(bufferV[1]), &(bufferW[1]), &(bufferU[1]), &(bufferV[1])};
uint32_t *lowside[6] =  {&(bufferU[0]), &(bufferW[0]), &(bufferW[0]), &(bufferV[0]), &(bufferV[0]), &(bufferU[0])};
uint32_t *highside[6] = {&(bufferV[0]), &(bufferV[0]), &(bufferU[0]), &(bufferU[0]), &(bufferW[0]), &(bufferW[0])};

float currentMeas = 0;
float currentSetpoint = 0;
float currentSetpointSpeed = 0;
float currentSetpointOL = 4;
float currentOut = 0;
float currentIntegrator = 0;
float currentKp = 1;
float currentKi = 0.1;
float currentLim = 10;

int8_t OLsign = 0;

uint8_t segger_buffer[5];

void LED_Green()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
}

void LED_Red()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
}

void LED_Orange()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
}

void LED_Off()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
}

float ADC_VBus(uint32_t raw) 
{
    //voltage divider with 39.2 and 2.2 kohm
    //12 bit adc
    return (float)raw / 4095 * 3.3 * (39.2 + 2.2) / 2.2;
}

float ADC_Temp(uint32_t raw)
{
    //3J B is 3380 3422 3435 3453 for range to 50 75 85 100 C
    //pick 3435 for B in 1/T = 1/TO + 1/B ⋅ ln (R/RO)
    //T0 is 298.15
    //12 bit adc
    float resistance = 10000 * (float)raw / (4095 - (float)raw);
    return 1/(1 / 298.15 + 1 / (float)3435 * log(resistance / 10000)) - 273.15;
}

void STM32_DisableBootPin()
{
    FLASH_OBProgramInitTypeDef FLASH_OBInitStruct;

    //read stored option bytes
    HAL_FLASHEx_OBGetConfig(&FLASH_OBInitStruct);

    //set nSWBOOT0 to 0 and nBOOT0 to 1 to bypass the physical BOOT0 pin
    FLASH_OBInitStruct.OptionType = OPTIONBYTE_USER;
    FLASH_OBInitStruct.USERType = OB_USER_nSWBOOT0 | OB_USER_nBOOT0;
    FLASH_OBInitStruct.USERConfig = OB_BOOT0_FROM_OB | OB_nBOOT0_SET;

    //store the option bytes
    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();
    HAL_FLASHEx_OBProgram(&FLASH_OBInitStruct);
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
    HAL_FLASH_OB_Launch();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static uint8_t statusADC = 0; 
    static uint16_t counterADC = 0;

    //first read all three ADC's
    if (hadc->Instance == ADC1)    
        statusADC += 1;    
    else if (hadc->Instance == ADC2)
        statusADC += 2;
    else if (hadc->Instance == ADC4)
        statusADC += 4;
    
    if (statusADC == 7)
    {
        //all three ADC's have been read. Perform fast loop
        counterADC++;
        statusADC = 0;

        //calculate the back-emf of the neutral phase
        prevBemf = bemf;
        bemf = ((float)*(neutral[step]) - (float)(bufferU[1] + bufferV[1] + bufferW[1])/3) * 0.004595;// 4095 * 3.3 * (39.2 + 2.2) / 2.2;
        bemf_int = bemf*1000; //for J-SCOPE      
        
        //calculate the phase current
        //40V/V need to adjust for 5V version of DRV8305
        //Calibrate using value of dead phase
        if (dutyRatioOut>=0)
            currentMeas = -((float)*(lowside[step])-(float)*(neutral[step]-1))*0.010073;///4095*3.3)/40/0.002;    
        else
            currentMeas = ((float)*(highside[step])-(float)*(neutral[step]-1))*0.010073;///4095*3.3)/40/0.002;             
       
        ///////////////////////////////////////////////////////////
        // Commutation state machine
        ///////////////////////////////////////////////////////////
        switch (commState)
        {
            //do nothing
            case COMM_IDLE:
                if ((ctrlState == CTRL_DR && dutyRatioSetpoint != 0) || 
                    (ctrlState == CTRL_CURR && currentSetpoint != 0)  || 
                    (ctrlState == CTRL_SPEED && speedSetpoint != 0))
                {                   
                    speedTargetOL = OL_SPEED_TARGET; //reset to default settings
                    commState = COMM_OPEN_LOOP;
                    TIM3->CNT=0; 
                }
            break;

            //strategy for spinning up the motor
            case COMM_OPEN_LOOP:
                if (TIM3->CNT >= 1000000/speedTargetOL) //when the speed target is reached
                {
                    if (!stepOvrEnbl) { 
                        //as long as there still is a request
                        if ((ctrlState == CTRL_DR && dutyRatioOut >= 0) || 
                            (ctrlState != CTRL_DR && currentOut >= 0))
                        {
                            step = (step + 1) % 6; //do a CW commutation step
                            speedDir = 1;
                        } else {
                            (step == 0) ? step=5 : step--; //do a CCW commutation step
                            speedDir = -1;
                        }                            
                    }

                    //write to speed buffer                    
                    speedBuffer[speedBufferIndex]=TIM3->CNT;
                    TIM3->CNT=0;
                    if (speedBufferIndex<(SPEED_BUFFER_LEN-1))
                        speedBufferIndex++;
                    else
                        speedBufferIndex=0;
                    
                    //if the amplitude of the back-emf becomes high enough
                    //switch to closed loop
                    if (speedTargetOL > OL_SWITCH_SPEED || (ctrlState == CTRL_SPEED && speedTargetOL > fabs(speedSetpoint)))
                    {   
                        commState = COMM_WAITING_FOR_ZEROCROSSING;                        
                    }

                    if (speedTargetOL > OL_MAX_SPEED) //timeout
                    {
                        commState = COMM_IDLE;
                    }
                }

                speedTargetOL+=OL_RAMP_RATE/(float)switchFreq_Hz; //increase the speed

            break;

            case COMM_WAITING_FOR_ZEROCROSSING:  
                //count timeout if the speed becomes too low to detect back-emf                          
                if (fabs(bemf)>bemfTimeoutThreshold)                
                    bemfTimeout=0;
                else 
                    bemfTimeout++;

                crossingTimeout++;

                if (((prevBemf <= 0 && bemf > 0) && (step==1 || step==3 || step==5)) || 
                    ((prevBemf >= 0 && bemf < 0) && (step==0 || step==2 || step==4)))
                {
                    bemfIntegral = bemf;
                    crossingTimeout = 0;
                    commState = COMM_ZEROCROSSING_DETECTED;
                }
                //timeout if the speed timer saturates
                //if waiting too long for a crossing
                //or if the bemf magnitude remains low
                else if (bemfTimeout > switchFreq_Hz/CL_TIMEOUT ||
                         crossingTimeout > switchFreq_Hz/CL_TIMEOUT ||
                         TIM3->CNT > 60000) //timeout
                {
                    bemfTimeout = 0;
                    crossingTimeout = 0;
                    commState = COMM_IDLE; //disable drive
                }
            break;

            case COMM_ZEROCROSSING_DETECTED:
                //count timeout if the speed becomes too low to detect back-emf                          
                if (fabs(bemf)>bemfTimeoutThreshold)                
                    bemfTimeout=0;
                else 
                    bemfTimeout++;
                
                bemfIntegral += bemf;
                //if the integral is reached
                //or if it is expected to be reached in the first half of the next interval
                if (fabs(bemfIntegral + 0.5*bemf) > bemfIntegralThreshold)
                {
                    if (!stepOvrEnbl) { 
                        speedBuffer[speedBufferIndex]=TIM3->CNT;
                        TIM3->CNT=0;                            
                        if (speedBufferIndex<(SPEED_BUFFER_LEN-1))
                            speedBufferIndex++;
                        else
                            speedBufferIndex=0;                      

                        if (((bemfIntegral>0 && (step==1 || step==3 || step==5)) ||
                            (bemfIntegral<0 && (step==0 || step==2 || step==4))) &&
                            (((ctrlState == CTRL_DR && dutyRatioSetpoint >= 0) || 
                            (ctrlState == CTRL_CURR && currentSetpoint >= 0)  || 
                            (ctrlState == CTRL_SPEED && speedSetpoint >= 0))))
                        {
                        step = (step + 1) % 6; //do a CW commutation step
                        if (speedMeas>-500) //switch speed sign if not too far away from zero
                            speedDir=1;
                        }
                        else 
                        if (((bemfIntegral>0 && (step==1 || step==3 || step==5)) ||
                            (bemfIntegral<0 && (step==0 || step==2 || step==4))) &&
                            (((ctrlState == CTRL_DR && dutyRatioSetpoint <= 0) || 
                            (ctrlState == CTRL_CURR && currentSetpoint <= 0)  || 
                            (ctrlState == CTRL_SPEED && speedSetpoint <= 0))))
                        {
                        (step == 0) ? step=5 : step--; //do a CCW commutation step
                        if (speedMeas<500) //switch speed sign if not too far away from zero
                            speedDir=-1;
                        }                    
                    }
                    commState = COMM_WAITING_FOR_ZEROCROSSING;
                }

                if (bemfTimeout > switchFreq_Hz/CL_TIMEOUT ||
                    TIM3->CNT > 60000) //10ms timeout
                {
                    bemfTimeout = 0;
                    crossingTimeout = 0;
                    commState = COMM_IDLE; //disable drive
                }
                
            break;
        }

        ///////////////////////////////////////////////////////////
        // Control method
        ///////////////////////////////////////////////////////////
        if (drvEnbl) 
        { 
            if (ctrlState == CTRL_DR) //duty ratio control
            {
                if (commState == COMM_OPEN_LOOP) 
                {
                    if (dutyRatioSetpoint>=0)
                        dutyRatioOut = dutyRatioOL;
                    else 
                        dutyRatioOut = -dutyRatioOL;
                }
                else 
                {
                    dutyRatioOut = dutyRatioSetpoint;
                }
            }            
            else if (commState == COMM_IDLE) 
            {
                dutyRatioOut = 0;
            }            
            else //do current control loop
            {
                if (commState == COMM_OPEN_LOOP) 
                {
                    speedIntegrator = 0;

                    if ((ctrlState == CTRL_CURR && currentSetpoint >= 0)  || 
                        (ctrlState == CTRL_SPEED && speedSetpoint >= 0))
                    {
                        currentOut = currentSetpointOL;
                        OLsign = 1;
                    } 
                    else 
                    {
                        currentOut = -currentSetpointOL;
                        OLsign = -1;
                    }
                }
                else 
                {
                    if (ctrlState == CTRL_SPEED) //speed control loop
                    {
                        currentOut = currentSetpointSpeed;
                    }
                    else
                    {
                        currentOut = currentSetpoint; //manual current setpoint
                    }
                }

                currentIntegrator = currentIntegrator + (currentOut - currentMeas) * currentKi;

                //anti-windup
                if (currentIntegrator>100)                
                    currentIntegrator=100;
                if (currentIntegrator<-100)
                    currentIntegrator=-100;                    

                dutyRatioOut = (currentOut - currentMeas) * currentKp + currentIntegrator;

                //saturation
                if (dutyRatioOut>100)                
                    dutyRatioOut=100;
                if (dutyRatioOut<-100)
                    dutyRatioOut=-100; 
            }            
        } 
        else 
        {
            commState=COMM_IDLE; //move to idle when the gate driver is disabled
        }

        //write to J-SCOPE
        memcpy(segger_buffer,&bemf_int,4);
        segger_buffer[4] = step;
        SEGGER_RTT_Write(0, segger_buffer, 5);       

        //energize the switches
        if ((tableU[step] == 1 && dutyRatioOut>=0) || (tableU[step] == -1 && dutyRatioOut<0)) //highside
        {
            TIM1->CCR1 = (float)(1-((float)fabs((float)(dutyRatioOut)) / 100)) * timerARR;
            TIM1->CCER |= TIM_CCER_CC1E;
            TIM1->CCER |= TIM_CCER_CC1NE;
        }
        else if ((tableU[step] == 1 && dutyRatioOut<0) || (tableU[step] == -1 && dutyRatioOut>=0)) //lowside
        {            
            TIM1->CCER &= ~TIM_CCER_CC1E;
            TIM1->CCER |= TIM_CCER_CC1NE;              
            TIM1->CCR1 = 0;
        }
        else //off
        {
            TIM1->CCER &= ~TIM_CCER_CC1E;
            TIM1->CCER &= ~TIM_CCER_CC1NE;  
            TIM1->CCR1 = 0;
        }

        if ((tableV[step] == 1 && dutyRatioOut>=0) || (tableV[step] == -1 && dutyRatioOut<0)) //highside
        {            
            TIM1->CCR2 = (float)(1-((float)fabs((float)(dutyRatioOut)) / 100)) * timerARR;
            TIM1->CCER |= TIM_CCER_CC2E;
            TIM1->CCER |= TIM_CCER_CC2NE;
        }
        else if ((tableV[step] == 1 && dutyRatioOut<0) || (tableV[step] == -1 && dutyRatioOut>=0)) //lowside
        {
            TIM1->CCER &= ~TIM_CCER_CC2E;
            TIM1->CCER |= TIM_CCER_CC2NE;
            TIM1->CCR2 = 0;         
        }
        else //off
        {
            TIM1->CCER &= ~TIM_CCER_CC2E;
            TIM1->CCER &= ~TIM_CCER_CC2NE;  
            TIM1->CCR2 = 0;
        }

        if ((tableW[step] == 1 && dutyRatioOut>=0) || (tableW[step] == -1 && dutyRatioOut<0)) //highside
        {
            TIM1->CCR3 = (float)(1-((float)fabs((float)(dutyRatioOut)) / 100)) * timerARR;
            TIM1->CCER |= TIM_CCER_CC3E;
            TIM1->CCER |= TIM_CCER_CC3NE;
        }
        else if ((tableW[step] == 1 && dutyRatioOut<0) || (tableW[step] == -1 && dutyRatioOut>=0)) //lowside
        {
            TIM1->CCER &= ~TIM_CCER_CC3E;
            TIM1->CCER |= TIM_CCER_CC3NE;
            TIM1->CCR3 = 0;
        }
        else //off
        {
            TIM1->CCER &= ~TIM_CCER_CC3E;
            TIM1->CCER &= ~TIM_CCER_CC3NE;            
            TIM1->CCR3 = 0;
        }

    }
    
}

void task_hlc(void *pvParameters)
{
    TickType_t xLastWakeTime;

    // We do this only once
    xLastWakeTime = xTaskGetTickCount();

    SEGGER_RTT_SetNameUpBuffer(0, "JScope_i4u1");

    //hlc task init
    LED_Green();

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);

    HAL_ADC_Start_DMA(&hadc1, bufferU, 3); // start adc in DMA mode
    HAL_ADC_Start_DMA(&hadc2, bufferV, 3); // start adc in DMA mode
    HAL_ADC_Start_DMA(&hadc4, bufferW, 3); // start adc in DMA mode

    TIM1->CR2 |= TIM_CR2_CCPC; //enable preload for capture compare
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_5);
    HAL_TIM_Base_Start(&htim3);

    //set gate current settings over SPI
    DRV8305_WriteGateControl(drvBmISink, drvBmISource, drvBmTdrive);
    DRV8305_WriteCurrentGains(2); //40V/V

    //hlc loop
    for (;;)
    {
        counterHLC++;

        busVoltage_V = ADC_VBus(bufferU[2]);
        mosfetTemp_C = ADC_Temp(bufferW[2]);
        drvFault = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
        drvPowerOK = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

        //set gate driver
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, drvEnbl);

        //set switching frequency        
        timerARR = 80000000 / switchFreq_Hz;
        TIM1->ARR = timerARR;
        TIM1->CCR5 = timerARR-1;

        //read switching outputs
        statusUH = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
        statusUL = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
        statusVH = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
        statusVL = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
        statusWH = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
        statusWL = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

        //speed calculation  
        speedTicks = 0;
        speedSteps = 0;

        uint8_t speedBufferIndexTemp = speedBufferIndex;
        while (speedSteps<SPEED_MAX_STEPS && speedTicks<SPEED_MAX_TICKS && speedBufferIndexTemp != speedBufferIndexPrev)
        {
            if (speedBufferIndexTemp==0)
                speedBufferIndexTemp = SPEED_BUFFER_LEN-1;
            else
                speedBufferIndexTemp--;
                
            speedTicks += speedBuffer[speedBufferIndexTemp];      
            speedSteps++;                
        }   
        speedBufferIndexPrev = speedBufferIndex;

        if (speedSteps>0 && speedTicks>0) {
            speedCounterHLC = 1;
            speedTicks = speedTicks/speedSteps;
        } else {
            speedTicks = (float)speedCounterHLC*(float)msHLC*1000;
            if (speedCounterHLC<65535)
                speedCounterHLC++;
        }
            
        speedMeas = (float)(1000000/speedTicks) * speedDir;

        //speed loop
        if (ctrlState == CTRL_SPEED) //speed control loop
        {
            speedIntegrator = speedIntegrator + (speedSetpoint - speedMeas) * speedKi;

            //anti-windup
            if (speedIntegrator>currentLim)                
                speedIntegrator=currentLim;
            if (speedIntegrator<-currentLim)
                speedIntegrator=-currentLim;                    

            if (speedSetpoint == 0) //soft rampdown
                currentSetpointSpeed = 0;
            else
                currentSetpointSpeed = (speedSetpoint - speedMeas) * speedKp + speedIntegrator + OL_FF_FACTOR*(float)OLsign*currentSetpointOL;
        }

        // variable xLastWakeTime is updated internally
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(msHLC));
    }
}

void task_bgt(void *pvParameters)
{
    TickType_t xLastWakeTime;

    // We do this only once
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        DRV8305_ReadWarnings();
        DRV8305_ReadFaultsOVVDS();
        DRV8305_ReadFaultsIC();
        DRV8305_ReadFaultsVGS();
        DRV8305_ReadGateControlHS();
        DRV8305_ReadGateControlLS();
        DRV8305_ReadCurentGains();

        // variable xLastWakeTime is updated internally
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
    }
}