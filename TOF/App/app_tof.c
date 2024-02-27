/**
  ******************************************************************************
  * @file          : app_tof.c
  * @author        : IMG SW Application Team
  * @brief         : This file provides code for the configuration
  *                  of the STMicroelectronics.X-CUBE-TOF1.3.4.0 instances.
  ******************************************************************************
  *
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_tof.h"
#include "main.h"
#include <stdio.h>

#include "53l7a1_ranging_sensor.h"
#include "app_tof_pin_conf.h"
#include "stm32f4xx_nucleo.h"

#include "../../Drivers/ssd1306/ssd1306.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TIMING_BUDGET (30U) /* 5 ms < TimingBudget < 100 ms */
#define RANGING_FREQUENCY (10U) /* Ranging frequency Hz (shall be consistent with TimingBudget value) */
#define POLLING_PERIOD (1000U/RANGING_FREQUENCY) /* refresh rate for polling mode (milliseconds) */

/* Private variables ---------------------------------------------------------*/
static RANGING_SENSOR_ProfileConfig_t Profile;
static RANGING_SENSOR_Result_t Result;
static int32_t status = 0;
static uint8_t ToF_Present[RANGING_SENSOR_INSTANCES_NBR] = {0};
volatile uint8_t ToF_EventDetected = 0;
extern uint32_t targetTurningAnglePWM;
extern int8_t turningAngleOffset;

static const char *TofDevStr[] =
{
  [VL53L7A1_DEV_LEFT] = "LEFT",
//  [VL53L7A1_DEV_CENTER] = "CENTER",
  [VL53L7A1_DEV_RIGHT] = "RIGHT"
};

/* Private function prototypes -----------------------------------------------*/
static void MX_53L7A1_MultiSensorRanging_Init(void);
static void MX_53L7A1_MultiSensorRanging_Process(void);

static void print_result(RANGING_SENSOR_Result_t *Result);
static void obstacle_avoidance(uint8_t device, RANGING_SENSOR_Result_t *Result);
static void display_result(uint8_t device, RANGING_SENSOR_Result_t *Result);
static void display_result_cells(uint8_t device, RANGING_SENSOR_Result_t *Result);
static void display_cell(uint8_t x, uint8_t y, long distance);
static void write_lowpower_pin(uint8_t device, GPIO_PinState pin_state);
static void reset_all_sensors(void);
uint32_t degreesToPWM(float degrees);

void MX_TOF_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN TOF_Init_PreTreatment */
	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();

	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("Initializing", Font_7x10, White);
	ssd1306_SetCursor(0, 10);
	ssd1306_WriteString("TOF sensors", Font_7x10, White);
	ssd1306_UpdateScreen();
  /* USER CODE END TOF_Init_PreTreatment */

  /* Initialize the peripherals and the TOF components */

  MX_53L7A1_MultiSensorRanging_Init();

  /* USER CODE BEGIN TOF_Init_PostTreatment */
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
  /* USER CODE END TOF_Init_PostTreatment */
}

/*
 * LM background task
 */
void MX_TOF_Process(void)
{
  /* USER CODE BEGIN TOF_Process_PreTreatment */

  /* USER CODE END TOF_Process_PreTreatment */

  MX_53L7A1_MultiSensorRanging_Process();

  /* USER CODE BEGIN TOF_Process_PostTreatment */

  /* USER CODE END TOF_Process_PostTreatment */
}

static void MX_53L7A1_MultiSensorRanging_Init(void)
{
  uint8_t device;
  uint16_t i2c_addr;
  uint32_t id;

  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  printf("53L7A1 Multi Sensor Ranging demo application\n");

  reset_all_sensors();

  /* Turn off all the sensors */
  for (device = 0; device < RANGING_SENSOR_INSTANCES_NBR; device++)
  {
    write_lowpower_pin(device, GPIO_PIN_RESET);
  }

  /* initializes each device and put it in low power mode */
  for (device = 0; device < RANGING_SENSOR_INSTANCES_NBR; device++)
  {
    /* enable only one sensor */
    write_lowpower_pin(device, GPIO_PIN_SET);
    HAL_Delay(2);

    status = VL53L7A1_RANGING_SENSOR_Init(device);

    if (status != BSP_ERROR_NONE)
    {
      printf("VL53L7A1_RANGING_SENSOR_Init %d failed\n", device);
      ToF_Present[device] = 0; /* device not detected */
    }
    else
    {
      ToF_Present[device] = 1; /* device detected */
    }

    write_lowpower_pin(device, GPIO_PIN_RESET); /* turn off the device */
  }

  /* power on the devices one at a time and change their address
   * once the address is updated, the communication with the devices is checked
   * reading its ID.
   */
  for (device = 0; device < RANGING_SENSOR_INSTANCES_NBR; device++)
  {
    /* skip the sensor if init not successful */
    if (ToF_Present[device] == 0) { continue; }

    /* turn on the device */
    write_lowpower_pin(device, GPIO_PIN_SET);

    /* left: 0x54, right: 0x56 */
    i2c_addr = (RANGING_SENSOR_VL53L7CX_ADDRESS + (device + 1) * 2);
    VL53L7A1_RANGING_SENSOR_SetAddress(device, i2c_addr);

    /* check the communication with the device reading the ID */
    VL53L7A1_RANGING_SENSOR_ReadID(device, &id);
    printf("ToF sensor %d - ID: %04lX\n", device, (unsigned long)id);

//    VL53L7CX_SetSharpenerPercent(device, 50);
  }
}

static void MX_53L7A1_MultiSensorRanging_Process(void)
{
  uint8_t i;

  Profile.RangingProfile = RS_PROFILE_4x4_CONTINUOUS;
  Profile.TimingBudget = TIMING_BUDGET;
  Profile.Frequency = RANGING_FREQUENCY; /* Ranging frequency Hz (shall be consistent with TimingBudget value) */
  Profile.EnableAmbient = 0; /* Enable: 1, Disable: 0 */
  Profile.EnableSignal = 0; /* Enable: 1, Disable: 0 */

  for (i = 0; i < RANGING_SENSOR_INSTANCES_NBR; i++)
  {
    /* skip this device if not detected */
    if (ToF_Present[i] != 1) { continue; }

    VL53L7A1_RANGING_SENSOR_ConfigProfile(i, &Profile);
    status = VL53L7A1_RANGING_SENSOR_Start(i, RS_MODE_BLOCKING_CONTINUOUS);

    if (status != BSP_ERROR_NONE)
    {
      printf("VL53L7A1_RANGING_SENSOR_Start %d failed\n", i);
      while (1);
    }
  }

  while (1)
  {
    /* polling mode */
    for (i = 0; i < RANGING_SENSOR_INSTANCES_NBR; i++)
    {
      if (!ToF_Present[i]) { continue; }

      status = VL53L7A1_RANGING_SENSOR_GetDistance(i, &Result);

      if (status == BSP_ERROR_NONE)
      {
//        printf("%s\n", TofDevStr[i]);
//        print_result(&Result);

//        display_result(i, &Result);
//    	  display_result_cells(i, &Result);
    	  obstacle_avoidance(i, &Result);
        HAL_Delay(POLLING_PERIOD);
      }
    }
  }
}

static void print_result(RANGING_SENSOR_Result_t *Result)
{
  int8_t i;
  int8_t j;
  int8_t k;
  int8_t l;
  uint8_t zones_per_line;

  zones_per_line = ((Profile.RangingProfile == RS_PROFILE_8x8_AUTONOMOUS) ||
                    (Profile.RangingProfile == RS_PROFILE_8x8_CONTINUOUS)) ? 8 : 4;

  printf("Cell Format :\n\n");
  for (l = 0; l < RANGING_SENSOR_NB_TARGET_PER_ZONE; l++)
  {
    printf(" \033[38;5;10m%20s\033[0m : %20s\n", "Distance [mm]", "Status");
    if ((Profile.EnableAmbient != 0) || (Profile.EnableSignal != 0))
    {
      printf(" %20s : %20s\n", "Signal [kcps/spad]", "Ambient [kcps/spad]");
    }
  }

  printf("\n\n");

  for (j = 0; j < Result->NumberOfZones; j += zones_per_line)
  {
    for (i = 0; i < zones_per_line; i++) /* number of zones per line */
    {
      printf(" -----------------");
    }
    printf("\n");

    for (i = 0; i < zones_per_line; i++)
    {
      printf("|                 ");
    }
    printf("|\n");

    for (l = 0; l < RANGING_SENSOR_NB_TARGET_PER_ZONE; l++)
    {
      /* Print distance and status */
      for (k = (zones_per_line - 1); k >= 0; k--)
      {
        if (Result->ZoneResult[j + k].NumberOfTargets > 0)
          printf("| \033[38;5;10m%5ld\033[0m  :  %5ld ",
                 (long)Result->ZoneResult[j + k].Distance[l],
                 (long)Result->ZoneResult[j + k].Status[l]);
        else
          printf("| %5s  :  %5s ", "X", "X");
      }
      printf("|\n");

      if ((Profile.EnableAmbient != 0) || (Profile.EnableSignal != 0))
      {
        /* Print Signal and Ambient */
        for (k = (zones_per_line - 1); k >= 0; k--)
        {
          if (Result->ZoneResult[j + k].NumberOfTargets > 0)
          {
            if (Profile.EnableSignal != 0)
            {
              printf("| %5ld  :  ", (long)Result->ZoneResult[j + k].Signal[l]);
            }
            else
              printf("| %5s  :  ", "X");

            if (Profile.EnableAmbient != 0)
            {
              printf("%5ld ", (long)Result->ZoneResult[j + k].Ambient[l]);
            }
            else
              printf("%5s ", "X");
          }
          else
            printf("| %5s  :  %5s ", "X", "X");
        }
        printf("|\n");
      }
    }
  }

  for (i = 0; i < zones_per_line; i++)
  {
    printf(" -----------------");
  }
  printf("\n");
}

static void display_result(uint8_t device, RANGING_SENSOR_Result_t *Result)
{
	long tmp = 0;
	char buffer[6];

	// Average result
	for(int i = 0; i < Result->NumberOfZones; i++)
	{
		tmp += Result->ZoneResult[i].Distance[0];
	}
	tmp /= Result->NumberOfZones;

	// Display result
	ssd1306_SetCursor((device == 0) ? 0 : 64, 0);
	sprintf(buffer, "%4ld", tmp);
	ssd1306_WriteString(buffer, Font_11x18, White);
	ssd1306_UpdateScreen();
}

static void display_result_cells(uint8_t device, RANGING_SENSOR_Result_t *Result)
{
	for(int i = 0; i < Result->NumberOfZones; i++)
	{
		// Fill in cell based on distance
		// << 2 multiplies by 4 to scale 0-16 range to 0-64
		// >> 2 and % 4 operations map 1D 0-16 to 2D X and Y values 0-4
		// 112 -  and 48 - flip axes for sensor orientation; will need to change/remove these based on orientation
		uint8_t x_tmp = 112 - (((i >> 2) << 4) + ((device == 0) ? 64 : 0));
		uint8_t y_tmp = 48 - ((i % 4) << 4);

		if(Result->ZoneResult[i].Status[0] == 0)
		{
			display_cell(x_tmp, y_tmp, Result->ZoneResult[i].Distance[0]);
		}
	}

	ssd1306_UpdateScreen();
}

// Helper function for display_result_pixels that fills in a square on the display based on the distance provided
// x and y are the coordinates of the top left corner of the cell
// distance is the distance in millimeters measured by that zone of the TOF sensor
static void display_cell(uint8_t x, uint8_t y, long distance)
{
	// Scale and offset values to expected range
	int16_t pixelsToFill = 256 - (distance >> 3);

	// Clear old cell
	ssd1306_FillRectangle(x, y, x + 15, y + 15, Black);

	// Fill pixels from top left to bottom right of current cell
	for(uint8_t j = y; j < y + 16; j++)
	{
		for(uint8_t i = x; i < x + 16; i++)
		{
			if(pixelsToFill <= 0)
			{
				return;
			}

			pixelsToFill--;

			ssd1306_DrawPixel(i, j, White);
		}
	}
}

static void obstacle_avoidance(uint8_t device, RANGING_SENSOR_Result_t *Result)
{
	static uint16_t leftAverage = 0;
	static uint16_t rightAverage = 0;
	float sum = 0;
	float rightFraction = 0.0f;

	long tmp = 0;

	// Average result
	for(int i = 0; i < Result->NumberOfZones; i++)
	{
		tmp += Result->ZoneResult[i].Distance[0];
	}
	tmp /= Result->NumberOfZones;

	switch(device)
	{
		case 0:
			leftAverage = tmp;
			break;
		case 1:
			rightAverage = tmp;
			break;
	}

	// Sum averages
	sum = leftAverage + rightAverage;
	// Compute percentage of right side
	rightFraction = ((float) rightAverage) / sum;

	// Map percentage to turning angle and assign to servo
	// newValue = (-1 if flipped, 1 if not) * oldValue * (newRange / oldRange) + newRangeOffset
	// Normally I would divide by the old range (100), but I need to convert the fraction to a percentage
	// 50% is already 135 degrees, so we don't need an offset
//	TIM3->CCR1 = degreesToPWM(rightFraction * 270.0f);
	// Turn towards obstacle
//	targetTurningAnglePWM = degreesToPWM(turningAngleOffset + rightFraction * 270.0f);
	// Turn away from obstacle
	targetTurningAnglePWM = degreesToPWM(turningAngleOffset + (1.0f - rightFraction) * 270.0f);

//	printf("%d percent (%ld PWM) %ld target\n", (int)(rightFraction * 100), TIM3->CCR1, targetTurningAnglePWM);

	//TODO: Add safeguards against weird numerical shit

//	printf("%d %d\n", leftAverage, rightAverage);
}

static void write_lowpower_pin(uint8_t device, GPIO_PinState pin_state)
{
  switch (device)
  {
//    case VL53L7A1_DEV_CENTER:
//      HAL_GPIO_WritePin(VL53L7A1_LPn_C_PORT, VL53L7A1_LPn_C_PIN, pin_state);
//      break;

    case VL53L7A1_DEV_LEFT:
      HAL_GPIO_WritePin(VL53L7A1_LPn_L_PORT, VL53L7A1_LPn_L_PIN, pin_state);
      break;

    case VL53L7A1_DEV_RIGHT:
      HAL_GPIO_WritePin(VL53L7A1_LPn_R_PORT, VL53L7A1_LPn_R_PIN, pin_state);
      break;

    default:
      break;
  }
}

static void reset_all_sensors(void)
{
//  HAL_GPIO_WritePin(VL53L7A1_PWR_EN_C_PORT, VL53L7A1_PWR_EN_C_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VL53L7A1_PWR_EN_L_PORT, VL53L7A1_PWR_EN_L_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VL53L7A1_PWR_EN_R_PORT, VL53L7A1_PWR_EN_R_PIN, GPIO_PIN_RESET);
  HAL_Delay(2);

//  HAL_GPIO_WritePin(VL53L7A1_PWR_EN_C_PORT, VL53L7A1_PWR_EN_C_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VL53L7A1_PWR_EN_L_PORT, VL53L7A1_PWR_EN_L_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VL53L7A1_PWR_EN_R_PORT, VL53L7A1_PWR_EN_R_PIN, GPIO_PIN_SET);
  HAL_Delay(2);

//  HAL_GPIO_WritePin(VL53L7A1_LPn_C_PORT, VL53L7A1_LPn_C_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VL53L7A1_LPn_L_PORT, VL53L7A1_LPn_L_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VL53L7A1_LPn_R_PORT, VL53L7A1_LPn_R_PIN, GPIO_PIN_RESET);
  HAL_Delay(2);

//  HAL_GPIO_WritePin(VL53L7A1_LPn_C_PORT, VL53L7A1_LPn_C_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VL53L7A1_LPn_L_PORT, VL53L7A1_LPn_L_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VL53L7A1_LPn_R_PORT, VL53L7A1_LPn_R_PIN, GPIO_PIN_SET);
  HAL_Delay(2);
}

uint32_t degreesToPWM(float degrees)
{
	// Validate range
	if(degrees < 0 || degrees > 270)
	{
		// Home motor if angle is out of range
		degrees = 135;
		// Can use this as error value b/c pulse will never be this thin
		// return 0;
	}

	// newValue = (-1 if flipped, 1 if not) * oldValue * (newRange / oldRange) + newRangeOffset

	return ((float) degrees) * (2000.0f / 270.0f) + 500;
}

#ifdef __cplusplus
}
#endif
