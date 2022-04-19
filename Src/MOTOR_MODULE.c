#include "MOTOR_MODULE.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lre_stepper.h"
#include "math.h"
#include "global.h"

void MOTOR_MODULE()
{
    switch (comm)
    {
    case 1:
        desired_angle = 0;
        desired_speed = 0;
        desired_steps = 0;
        desired_speed_left = 0;
        desired_speed_right = 0;
        break;
    case 2:
        moveForwardDistance(&desired_steps);
        break;
    case 4:
        moveRotate();
        break;

    default:
        break;
    }
}

void moveLeftWheelForward()
{
    lre_stepper_setStep(indexOfLeftMotorStep, type_GPIO_L, IN1_L, IN2_L, IN3_L, IN4_L);
    indexOfLeftMotorStep++;
    if (indexOfLeftMotorStep == 7)
    {
        indexOfLeftMotorStep = 0;
    }
}
void moveLeftWheelForwardDistance(double *steps)
{
    if (indexOfLeftMotorStepTotal == 0)
    {
        indexOfLeftMotorStep = 0;
    }
    if (indexOfLeftMotorStepTotal <= *steps)
    {
        indexOfLeftMotorStep++;
        indexOfLeftMotorStepTotal++;
        lre_stepper_setStep(indexOfLeftMotorStep, type_GPIO_L, IN1_L, IN2_L, IN3_L, IN4_L);

        if (indexOfLeftMotorStep == 7)
        {
            indexOfLeftMotorStep = 0;
        }
        moveSlowDownToStop(indexOfLeftMotorStepTotal);
    }
    else
    {
        stepCompleted = 1;
        return;
    }
}
void moveLeftWheelBackwardDistance(double *steps)
{
    if ((int16_t)indexOfLeftMotorStepTotal == 0)
    {
        indexOfLeftMotorStep = 7;
    }
    if (indexOfLeftMotorStepTotal <= *steps)
    {
        indexOfLeftMotorStep--;
        indexOfLeftMotorStepTotal++;
        lre_stepper_setStep(indexOfLeftMotorStep, type_GPIO_L, IN1_L, IN2_L, IN3_L, IN4_L);
        if ((int16_t)indexOfLeftMotorStep <= 0)
        {
            indexOfLeftMotorStep = 7;
        }
        moveSlowDownToStop(indexOfLeftMotorStepTotal);
    }
    else
    {
        stepCompleted = 1;
        return;
    }
}
void setSpeedLeftMotor()
{
    int autoreload = 360 * 1000000 / (4096 * desired_speed_left);
    printToHterm(sprintf(uart_buf, "Setting left ARR to %i", autoreload));
    __HAL_TIM_SET_AUTORELOAD(&htim3, autoreload - 1);
}
void moveLeftWheelBackwardSpeed(double *speed)
{
}
void moveLeftWheelBackward()
{
}

void moveRightWheelForward()
{
    lre_stepper_setStep(indexOfRightMotorStep, type_GPIO_R, IN1_R, IN2_R, IN3_R, IN4_R);
    indexOfRightMotorStep++;
    if (indexOfRightMotorStep == 7)
    {
        indexOfRightMotorStep = 0;
    }
}
void moveRightWheelForwardDistance(double *steps)
{
    if (indexOfRightMotorStepTotal == 0)
    {
        indexOfRightMotorStep = 0;
    }
    if (indexOfRightMotorStepTotal <= *steps)
    {
        indexOfRightMotorStep++;
        indexOfRightMotorStepTotal++;
        lre_stepper_setStep(indexOfRightMotorStep, type_GPIO_R, IN1_R, IN2_R, IN3_R, IN4_R);
        if (indexOfRightMotorStep == 7)
        {
            indexOfRightMotorStep = 0;
        }
        moveSlowDownToStop(indexOfRightMotorStepTotal);
    }

    else
    {
        stepCompleted = 1;
        return;
    }
}
void moveRightWheelBackward()
{
}
void moveRightWheelBackwardDistance(double *steps)
{
    if (indexOfRightMotorStepTotal == 0)
    {
        indexOfRightMotorStep = 7;
    }
    if (indexOfRightMotorStepTotal <= *steps)
    {
        indexOfRightMotorStep--;
        indexOfRightMotorStepTotal++;
        lre_stepper_setStep(indexOfRightMotorStep, type_GPIO_R, IN1_R, IN2_R, IN3_R, IN4_R);
        if ((int16_t)indexOfRightMotorStep <= 0)
        {
            indexOfRightMotorStep = 7;
        }
        moveSlowDownToStop(indexOfRightMotorStepTotal);
    }
    else
    {
        stepCompleted = 1;
        return;
    }
}
void moveRightWheelBackwardDistanceSeparateTimer(double *steps)
{
    if (indexOfRightMotorStepTotal == 0)
    {
        indexOfRightMotorStep = 7;
    }
    if (indexOfRightMotorStepTotal <= *steps)
    {
        indexOfRightMotorStep--;
        indexOfRightMotorStepTotal++;
        lre_stepper_setStep(indexOfRightMotorStep, type_GPIO_R, IN1_R, IN2_R, IN3_R, IN4_R);
        if ((int16_t)indexOfRightMotorStep <= 0)
        {
            indexOfRightMotorStep = 7;
        }
        moveSlowDownToStop(indexOfRightMotorStepTotal);
    }
    else
    {
        stepCompleted = 1;
        return;
    }
}
void setSpeedRightMotor()
{
    int autoreload = 360 * 1000000 / (4096 * desired_speed_right);
    printToHterm(sprintf(uart_buf, "Setting right ARR to %i", autoreload));
    __HAL_TIM_SET_AUTORELOAD(&htim15, autoreload - 1);
}

void moveRotateDiffSpeedsLeft()
{
    int rx_distance = desired_angle * M_PI * EMMA_WIDTH / (2 * 180);
    desired_steps = fabs((4096 * rx_distance) / (M_PI * EMMA_WHEEL_DIA)); // EM: ( steps per rotation * input distance(mm) )/( pi * wheel diameter(mm) )

    // anti-clockwise angle, so negative is right, positive left. desired_angle is in degrees.
    if (desired_angle < 0) // RIGHT
    {
        moveLeftWheelForwardDistance(&desired_steps);
    }
    else if (desired_angle > 0) // LEFT
    {
        moveLeftWheelBackwardDistance(&desired_steps);
    }
}
void moveRotateDiffSpeedsRight()
{
    int rx_distance = desired_angle * M_PI * EMMA_WIDTH / (2 * 180);
    desired_steps = fabs((4096 * rx_distance) / (M_PI * EMMA_WHEEL_DIA)); // EM: ( steps per rotation * input distance(mm) )/( pi * wheel diameter(mm) )

    // anti-clockwise angle, so negative is right, positive left. desired_angle is in degrees.
    if (desired_angle < 0) // RIGHT
    {
        moveRightWheelBackwardDistance(&desired_steps);
    }
    else if (desired_angle > 0) // LEFT
    {
        moveRightWheelForwardDistance(&desired_steps);
    }
}
void moveRightWheelBackwardSpeed(double *speed)
{
}

void moveForward()
{
    moveLeftWheelForward();
    moveRightWheelForward();
}
void moveForwardDistance(double *steps)
{

    moveLeftWheelForwardDistance(steps);
    moveRightWheelForwardDistance(steps);
}
void setSpeedBothMotors(double *speed)
{
    uint16_t autoreload = (EMMA_WHEEL_DIA * M_PI) / (4096 * (int)speed) * 1000000; // us
    __HAL_TIM_SET_AUTORELOAD(&htim3, autoreload - 1);
}

void moveBackward()
{
    moveRightWheelBackward();
    moveLeftWheelBackward();
    // indefinitely
}
void moveBackwardDistance(double *steps)
{
    moveRightWheelBackwardDistance(steps);
    moveLeftWheelBackwardDistance(steps);
}
void moveBackwardSpeed(double *speed)
{
}

void moveRotate()
{
    int rx_distance = desired_angle * M_PI * EMMA_WIDTH / (2 * 180);
    desired_steps = fabs((4096 * rx_distance) / (M_PI * 80)); // EM: ( steps per rotation * input distance(mm) )/( pi * wheel diameter(mm) )

    // anti-clockwise angle, so negative is right, positive left. desired_angle is in degrees.
    if (desired_angle < 0) // RIGHT
    {
        moveRightWheelBackwardDistance(&desired_steps);
        moveLeftWheelForwardDistance(&desired_steps);
    }
    else if (desired_angle > 0) // LEFT
    {
        moveLeftWheelBackwardDistance(&desired_steps);
        moveRightWheelForwardDistance(&desired_steps);
    }
}

void moveRotateOneWheel()
{
    if (desired_angle < 0) // RIGHT
    {
        moveLeftWheelForwardDistance(&desired_steps);
    }
    else if (desired_angle > 0) // LEFT
    {
        moveRightWheelForwardDistance(&desired_steps);
    }
}

void moveStop()
{
    desired_angle = 0;
    desired_speed = 0;
    desired_steps = 0;
    desired_speed_left = 0;
    desired_speed_right = 0;
    indexOfLeftMotorStepTotal = 0;
    indexOfRightMotorStepTotal = 0;
    stepCompleted = 0;
    return;
}

void moveSlowDownToStop(double totalSteps)
{
    int already75Dis = 0;
    if (totalSteps / desired_steps >= 0.75 && already75Dis == 0)
    {

        tick = 1.5 * tick;                          // speed reduced by 50%
        __HAL_TIM_SET_AUTORELOAD(&htim3, tick - 1); // custom time between motor steps (speed adjustment)
        already75Dis = 1;
    }
    if (totalSteps / desired_steps >= 0.9 && already75Dis == 1)
    {

        tick = 1.5 * tick;                          // speed reduced by 50%
        __HAL_TIM_SET_AUTORELOAD(&htim3, tick - 1); // custom time between motor steps (speed adjustment)
        already75Dis = 2;
    }
}