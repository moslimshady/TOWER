/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBAL_H
#define __GLOBAL_H

#define PRINTCOUNTER 10
#define EMMA_WIDTH 78.75
#define EMMA_WHEEL_DIA 80
#define SENSOR_WIDTH 85
#define VECTOR_SIZE_FOR_MODE 20
#define FREQUENCY_GOAL 1000000
#define FWD_MEASURE_LIMIT 500
#define FWD_IDEAL 45
#define LAT_IDEAL 54
#define CELL_LENGTH 200 // in mm
#define WALL_WIDTH 14
#define MAX_YAW_ADJUST 10
#define DEADEND_ADJUST 50
#define MAX_FWD_ADJUST 70
#define ADJUSTS_PER_CELL 4
#define MAX_LAT_WALL_DIST 50
#define MAX_FRNT_WALL_DIST 70

#include "stm32f0xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stm32f0xx_it.h>
#include <stdio.h>    //EM for sprintf
#include <math.h>     //EM for pi
#include <ctype.h>    //For isdigit()
#include <inttypes.h> //to print uint32_t numbers
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// EM define variables globally
volatile uint16_t comm;
// Pins for the left motor
GPIO_TypeDef *type_GPIO_L;
uint16_t IN1_L;
uint16_t IN2_L;
uint16_t IN3_L;
uint16_t IN4_L;
// Pins for the right motor
GPIO_TypeDef *type_GPIO_R;
uint16_t IN1_R;
uint16_t IN2_R;
uint16_t IN3_R;
uint16_t IN4_R;

uint16_t indexOfRightMotorStep;
uint16_t indexOfLeftMotorStep;
uint16_t indexOfLeftMotorStepTotal;
uint16_t indexOfRightMotorStepTotal;

uint8_t Is_First_Captured_X;
uint8_t Is_First_Captured_L;
uint8_t Is_First_Captured_R;

int initialPositionX;
int initialPositionY;
int initialHeading;
int currentPositionX;
int currentPositionY;
int currentHeading;
double requiredYaw;
double ADJUST_SCALER;
char *stringContainsCRLF;
unsigned char rx_data[1];
char data_buffer[15];
char data_buffer_temp[15];

volatile uint8_t taskCompleted;
volatile uint8_t stepCompleted;
volatile uint8_t receivedFullCommand;
// Transmission variables:
// char uart_buf[50]; // buffer

// output for sprintf function
int messDisR;

// time stamps for US sensor distance calculation
uint16_t t0_X;
uint16_t t1_X;
uint16_t t0_L;
uint16_t t1_L;
uint16_t t0_R;
uint16_t t1_R;

// Distance to be driven
double desired_steps;
double desired_angle;
double desired_speed;
uint16_t desired_speed_left;
uint16_t desired_speed_right;

//Speed Autoreload
double tick;

// US sensors distances
float Diff_X; // intermediate variable for calculations
float Diff_L;
float Diff_R;
float dis_L;
float dis_X;
float dis_R;

float latestTwoDistanceMeasurements[2][3];
float currentYaw;

volatile uint64_t msPassed;
volatile uint64_t usPassed;
volatile uint64_t sPassed;

char uart_buf[150]; // buffer

void moveForward();
void moveForwardDistance(double *);
void setSpeedBothMotors(double *);
void moveBackward();
void moveBackwardDistance(double *);
void moveBackwardSpeed(double *);

void moveLeftWheelForward();
void moveLeftWheelForwardDistance(double *);
void setSpeedLeftMotor();
void moveLeftWheelBackward();
void moveLeftWheelBackwardDistance(double *);
void moveLeftWheelBackwardSpeed(double *);

void moveRightWheelForward();
void moveRightWheelForwardDistance(double *);
void setSpeedRightMotor();
void moveRightWheelBackward();
void moveRightWheelBackwardDistance(double *);
void moveRightWheelBackwardSpeed(double *);

void moveRotate();
void moveRotateOneWheel();
void moveRotateDiffSpeedsRight();
void moveRotateDiffSpeedsLeft();

void moveStop();
void moveSlowDownToStop();

void initializeGlobalVariables();
void initializePeripherals();
void initializeTimers();

void measureSurroundings();
void printUsValues();
void printWallValues();

// void MX_TSC_Init();

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint8_t labyrinth[7][7];

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;

TSC_HandleTypeDef htsc;

UART_HandleTypeDef huart1;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NCS_MEMS_SPI_Pin GPIO_PIN_0
#define NCS_MEMS_SPI_GPIO_Port GPIOC
#define MEMS_INT1_Pin GPIO_PIN_1
#define MEMS_INT1_GPIO_Port GPIOC
#define MEMS_INT1_EXTI_IRQn EXTI0_1_IRQn
#define EXT_RESET_Pin GPIO_PIN_5
#define EXT_RESET_GPIO_Port GPIOC
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
#define I2C2_SDA_Pin GPIO_PIN_11
#define I2C2_SDA_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define IN1_R_Pin GPIO_PIN_6
#define IN1_R_GPIO_Port GPIOC
#define IN2_R_Pin GPIO_PIN_7
#define IN2_R_GPIO_Port GPIOC
#define IN3_R_Pin GPIO_PIN_8
#define IN3_R_GPIO_Port GPIOC
#define IN4_R_Pin GPIO_PIN_9
#define IN4_R_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define IN1_L_Pin GPIO_PIN_3
#define IN1_L_GPIO_Port GPIOB
#define IN2_L_Pin GPIO_PIN_4
#define IN2_L_GPIO_Port GPIOB
#define IN3_L_Pin GPIO_PIN_5
#define IN3_L_GPIO_Port GPIOB
#define IN4_L_Pin GPIO_PIN_8
#define IN4_L_GPIO_Port GPIOB

// Transmission variables:
// char uart_buf[50]; // buffer

// output for sprintf function
int messDisR;

// time stamps for US sensor distance calculation
uint16_t t0_X;
uint16_t t1_X;
uint16_t t0_L;
uint16_t t1_L;
uint16_t t0_R;
uint16_t t1_R;

// distance Matrix containing 20 values of dis_L, dis_X, dis_R values.
int distanceMatrix[VECTOR_SIZE_FOR_MODE][3];
float medianDistanceVector[3];
int totalDistanceInMillimeters;
// EM function to convert float distance values to integer for sending data:

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
uint16_t commandCount;
uint16_t tempCommandCount;
struct remoteCommand
{
  char command1[2]; // e.g., mv
  char command2[2]; // e.g., rt
  int parameter1;          // all 6 parameters are integers.
  int parameter2;
  int parameter3;
  int parameter4;
  int parameter5;
  int parameter6;
};
struct remoteCommand rc;

#endif /* __GLOBAL_H */