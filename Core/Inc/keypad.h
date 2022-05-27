/*
 * keypad.h
 *
 *  Created on: Oct 13, 2021
 *      Author: Marcin
 */

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_
/*
 	 	 ****************************************
 	 	 *   xx        xx      xx       xx      *
 	 	 *   xx        xx      xx       xx      *
 	 	 *                                      *
 	 	 *   xx        xx      xx       xx      *
 	 	 *   xx        xx      xx       xx      *
 	 	 *                                      *
 	 	 *   xx        xx      xx       xx      *
 	 	 *   xx        xx      xx       xx      *
 	 	 *                                      *
 	 	 *   xx        xx      xx       xx      *
 	 	 *   xx        xx      xx       xx      *
 	 	 *                                      *
 	 	 ****************************************
 	     *    *    *    *    *    *    *    *
 	     *    *    *    *    *    *    *    *
 	     *    *    *    *    *    *    *    *
 	     *    *    *    *    *    *    *    *
 	     *    *    *    *    *    *    *    *
 	     *    *    *    *    *    *    *    *
 	     *    *    *    *    *    *    *    *
 	     *    *    *    *    *    *    *    *
 	     *    *    *    *    *    *    *    *
 	     o    o    o    o    i    i    i    i
 	     u    u    u    u    n    n    n    n
 	     t    t    t    t    0    1    2    3
 	     0    1    2    3
*/


typedef enum
{
	idle=0,
	debounce
}keypad_state;
typedef struct
{
	GPIO_TypeDef* GPIO_Port;
	uint16_t Pin;
}pin;
typedef struct
{
	char keypad[4][4];
	pin pins[8];
	keypad_state state;
	uint32_t timer;
	char key;
	uint8_t cnt;
	volatile uint8_t flag;
}keypad_t;

void keypad_init(keypad_t* keypad,char* map);
void keypad_task(keypad_t* keypad);
char keypad_getKey(keypad_t* keypad);
#endif /* INC_KEYPAD_H_ */
