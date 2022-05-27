/*
 * keypad.c
 *
 *  Created on: Oct 13, 2021
 *      Author: Marcin
 */
#include"main.h"
#include"keypad.h"
void keypad_init(keypad_t* keypad,char* map)
{

	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			keypad->keypad[i][j]=*map;
			map++;
		}
	}
	keypad->timer=HAL_GetTick();
	keypad->cnt=0;
	keypad->flag=0;
	keypad->key=0;
	keypad->pins[0].GPIO_Port=out0_GPIO_Port;
	keypad->pins[0].Pin=out0_Pin;
	keypad->pins[1].GPIO_Port=out1_GPIO_Port;
	keypad->pins[1].Pin=out1_Pin;
	keypad->pins[2].GPIO_Port=out2_GPIO_Port;
	keypad->pins[2].Pin=out2_Pin;
	keypad->pins[3].GPIO_Port=out3_GPIO_Port;
	keypad->pins[3].Pin=out3_Pin;
	keypad->pins[4].GPIO_Port=in0_GPIO_Port;
	keypad->pins[4].Pin=in0_Pin;
	keypad->pins[5].GPIO_Port=in1_GPIO_Port;
	keypad->pins[5].Pin=in1_Pin;
	keypad->pins[6].GPIO_Port=in2_GPIO_Port;
	keypad->pins[6].Pin=in2_Pin;
	keypad->pins[7].GPIO_Port=in3_GPIO_Port;
	keypad->pins[7].Pin=in3_Pin;
}
char keypad_getKey(keypad_t* keypad)
{
	keypad->flag=2;
	return keypad->key;
}
void debounce_task(keypad_t* keypad)
{
	if(!HAL_GPIO_ReadPin(keypad->pins[4].GPIO_Port, keypad->pins[4].Pin)&&!HAL_GPIO_ReadPin(keypad->pins[5].GPIO_Port, keypad->pins[5].Pin)&&!HAL_GPIO_ReadPin(keypad->pins[6].GPIO_Port, keypad->pins[6].Pin)&&!HAL_GPIO_ReadPin(keypad->pins[7].GPIO_Port, keypad->pins[7].Pin))
	{
		keypad->timer=HAL_GetTick();
		keypad->state=idle;
	}
	else
	{
		if(HAL_GetTick()-keypad->timer>100)
		{
			if(keypad->flag==0)
			{
				if(HAL_GPIO_ReadPin(keypad->pins[4].GPIO_Port, keypad->pins[4].Pin))
				{
					keypad->key=keypad->keypad[keypad->cnt%4][0];
				}
				else if(HAL_GPIO_ReadPin(keypad->pins[5].GPIO_Port, keypad->pins[5].Pin))
				{
					keypad->key=keypad->keypad[keypad->cnt%4][1];
				}
				else if(HAL_GPIO_ReadPin(keypad->pins[6].GPIO_Port, keypad->pins[6].Pin))
				{
					keypad->key=keypad->keypad[keypad->cnt%4][2];
				}
				else
				{
					keypad->key=keypad->keypad[keypad->cnt%4][3];
				}
				keypad->flag=1;
			}
			else
			{
				keypad->key=0;
			}
		}
	}
}
void idle_task(keypad_t* keypad)
{
	if(HAL_GetTick()-keypad->timer>10)
	{
		if(HAL_GPIO_ReadPin(keypad->pins[4].GPIO_Port, keypad->pins[4].Pin)||HAL_GPIO_ReadPin(keypad->pins[5].GPIO_Port, keypad->pins[5].Pin)||HAL_GPIO_ReadPin(keypad->pins[6].GPIO_Port, keypad->pins[6].Pin)||HAL_GPIO_ReadPin(keypad->pins[7].GPIO_Port, keypad->pins[7].Pin))
		{
			keypad->timer=HAL_GetTick();
			keypad->state=debounce;
		}
		else
		{
			HAL_GPIO_WritePin(keypad->pins[keypad->cnt%4].GPIO_Port, keypad->pins[keypad->cnt%4].Pin, GPIO_PIN_RESET);
			keypad->cnt++;
			HAL_GPIO_WritePin(keypad->pins[keypad->cnt%4].GPIO_Port, keypad->pins[keypad->cnt%4].Pin, GPIO_PIN_SET);
			keypad->timer=HAL_GetTick();
		}
	}




	keypad->flag=0;
}
void keypad_task(keypad_t* keypad)
{
	switch(keypad->state)
	{
	case idle:
	{
		idle_task(keypad);
		break;
	}
	case debounce:
	{
		debounce_task(keypad);
		break;
	}
	}
}
