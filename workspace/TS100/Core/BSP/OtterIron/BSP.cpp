#include <stdint.h>
#include "Pins.h"
#include "BSP.h"
#include "Setup.h"

const uint16_t powerPWM = 255;
uint16_t totalPWM;

// Called once from preRToSInit()
void BSPInit(void) {
	totalPWM = 255;
	return;
}

// Called periodically in the movement handling thread
// Can be used to check any details for the power system
void power_check() {

}

uint8_t usb_pd_detect() {
	return false;
}

bool getIsPoweredByDCIN() {
	return false;
}

void reboot() {
	NVIC_SystemReset();
}

void delay_ms(uint16_t count) {
	HAL_Delay(count);
}

void resetWatchdog() {
	#ifndef SWD_ENABLE
		HAL_IWDG_Refresh(&hiwdg);
	#endif
}


void unstick_I2C() {
	GPIO_InitTypeDef GPIO_InitStruct;
	int timeout = 100;
	int timeout_cnt = 0;

	// 1. Clear PE bit.
	hi2c1.Instance->CR1 &= ~(0x0001);
	/**I2C1 GPIO Configuration
	 PB6     ------> I2C1_SCL
	 PB7     ------> I2C1_SDA
	 */
	//  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	GPIO_InitStruct.Pin = OLED_SCL_Pin;
	HAL_GPIO_Init(OLED_SCL_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = OLED_SDA_Pin;
	HAL_GPIO_Init(OLED_SDA_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_SET);

	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin)) {
		//Move clock to release I2C
		HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_RESET);
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET);

		timeout_cnt++;
		if (timeout_cnt > timeout)
			return;
	}

	// 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	GPIO_InitStruct.Pin = OLED_SCL_Pin;
	HAL_GPIO_Init(OLED_SCL_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = OLED_SDA_Pin;
	HAL_GPIO_Init(OLED_SDA_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_SET);

	// 13. Set SWRST bit in I2Cx_CR1 register.
	hi2c1.Instance->CR1 |= 0x8000;

	asm("nop");

	// 14. Clear SWRST bit in I2Cx_CR1 register.
	hi2c1.Instance->CR1 &= ~0x8000;

	asm("nop");

	// 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
	hi2c1.Instance->CR1 |= 0x0001;

	// Call initialization function.
	HAL_I2C_Init(&hi2c1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}


// If the user has programmed in a bootup logo, draw it to the screen from flash
// Returns 1 if the logo was printed so that the unit waits for the timeout or button
uint8_t showBootLogoIfavailable() {
	return 0;
}

uint8_t getButtonA() {
	return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET ? 1 : 0;
}
uint8_t getButtonB() {
	return HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin) == GPIO_PIN_RESET ? 1 : 0;
}

uint16_t getHandleTemperature() {
	return 1;
}

uint16_t getTipRawTemp(uint8_t refresh) {
	return 1;
}


// Returns the main DC input voltage, using the adjustable divisor + sample flag
uint16_t getInputVoltageX10(uint16_t divisor, uint8_t sample) {
	return 1;
}

bool tryBetterPWM(uint8_t pwm) {
	return false;
}

void setTipPWM(uint8_t pulse) {
	return;
}
