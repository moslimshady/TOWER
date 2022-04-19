#include "lre_stepper.h"

/*
    Stepper connections to STM32F072 expected to be:
    BLUE   - 1: PC6
    PINK   - 2: PC7
    YELLOW - 3: PC8
*/

void lre_stepper_setStep(uint8_t step, GPIO_TypeDef*  type_GPIO, uint16_t IN1, uint16_t IN2, uint16_t IN3, uint16_t IN4 ){
    switch (step)
    {
        case 0:
            HAL_GPIO_WritePin(type_GPIO, IN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(type_GPIO, IN2 | IN3 | IN4, GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(type_GPIO, IN1 | IN2, GPIO_PIN_SET);
            HAL_GPIO_WritePin(type_GPIO, IN3 | IN4, GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(type_GPIO, IN2, GPIO_PIN_SET);
            HAL_GPIO_WritePin(type_GPIO, IN1 | IN3 | IN4, GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(type_GPIO, IN2 | IN3, GPIO_PIN_SET);
            HAL_GPIO_WritePin(type_GPIO, IN1 | IN4, GPIO_PIN_RESET);
            break;
        case 4:
            HAL_GPIO_WritePin(type_GPIO, IN3, GPIO_PIN_SET);
            HAL_GPIO_WritePin(type_GPIO, IN1 | IN2 | IN4, GPIO_PIN_RESET);
            break;
        case 5:
            HAL_GPIO_WritePin(type_GPIO, IN3 | IN4, GPIO_PIN_SET);
            HAL_GPIO_WritePin(type_GPIO, IN1 | IN2, GPIO_PIN_RESET);
            break;
        case 6:
            HAL_GPIO_WritePin(type_GPIO, IN4, GPIO_PIN_SET);
            HAL_GPIO_WritePin(type_GPIO, IN1 | IN2 | IN3, GPIO_PIN_RESET);
            break;
        case 7:
            HAL_GPIO_WritePin(type_GPIO, IN1 | IN4, GPIO_PIN_SET);
            HAL_GPIO_WritePin(type_GPIO, IN2 | IN3, GPIO_PIN_RESET);
            break;

        default:
            break;
    }
}