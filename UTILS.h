/*
 * UTILS.h
 *
 *  Created on: Feb 16, 2022
 *      Author: User
 */

#ifndef INC_UTILS_H_

/** INCLUDES***/
#include <stdio.h>

extern UART_HandleTypeDef huart2;

/** FUNCTIONS **/

	void espera (int time);



	int _write(int file,char *ptr, int len){
		int i=0;
		for(i=0; i<len; i++)
		   HAL_UART_Transmit(&huart2,(unsigned char *)ptr++,1,1000);

		return len;

	}

	void Bin2Ascii(int value, unsigned char *temp);







#endif /* INC_UTILS_H_ */
