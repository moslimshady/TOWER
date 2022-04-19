#include "stm32f0xx_hal.h"
#include "utils.h"
void lre_stepper_setStep(uint8_t step, GPIO_TypeDef*  type_GPIO, uint16_t IN1, uint16_t IN2, uint16_t IN3, uint16_t IN4 );