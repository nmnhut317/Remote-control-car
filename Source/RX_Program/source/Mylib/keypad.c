#include "keypad.h"
volatile uint8_t keyBuff[5] = {20, 20, 20};

uint8_t key_code[KEYPAD_ROW][KEYPAD_COL] =
				{
					{'1', '2', '3', 'A'},
					{'4', '5', '6', 'B'},
					{'7', '8', '9', 'C'},
					{'*', '0', '#', 'D'}
				};

GPIO_TypeDef *KEYPAD_PORT_ROW[KEYPAD_ROW] = {GPIOA, GPIOA, GPIOA, GPIOA};
uint16_t KEYPAD_PIN_ROW[KEYPAD_ROW] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};

GPIO_TypeDef *KEYPAD_PORT_COL[KEYPAD_COL] = {GPIOA, GPIOA, GPIOA, GPIOA};
uint16_t KEYPAD_PIN_COL[KEYPAD_COL] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};

void KEYPAD_UnSelectRow(void)
{
	for (uint8_t i = 0; i < KEYPAD_ROW; i++)
	{
		HAL_GPIO_WritePin(KEYPAD_PORT_ROW[i], KEYPAD_PIN_ROW[i], GPIO_PIN_SET);
	}
}

void KEYPAD_SeclectRow(uint16_t row)
{
	HAL_GPIO_WritePin(KEYPAD_PORT_ROW[row], KEYPAD_PIN_ROW[row], GPIO_PIN_RESET);
}

uint8_t KEYPAD_GetKey(void)
{
	for (uint8_t row = 0; row< KEYPAD_ROW; row++)
	{
		KEYPAD_UnSelectRow();
		KEYPAD_SeclectRow(row);
		for (uint8_t col = 0; col < KEYPAD_COL; col++)
		{
			if (HAL_GPIO_ReadPin(KEYPAD_PORT_COL[col], KEYPAD_PIN_COL[col]) == 0)
			{
				return key_code[row][col];
			}
		}
	}
	return 0;
}

uint8_t key_scan_EOS(void)
{
	uint8_t kq = 20;
	uint8_t key_temp;
	key_temp = KEYPAD_GetKey();
	
	keyBuff[0] = keyBuff[1];
	keyBuff[1] = keyBuff[2];
	if ((keyBuff[0] == keyBuff[1]) && (keyBuff[2] == 20))
	{
		kq = keyBuff[0];
	}
	keyBuff[2] = key_temp;
	
	return kq;
}
