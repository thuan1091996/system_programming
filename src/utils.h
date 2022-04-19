/*
 * utils.h
 *
 *  Created on: Jan 1, 2012
 *      Author: AJP
 */

#ifndef UTILS_H_
#define UTILS_H_

void vADC1_Init(void);
uint16_t sADC1_GetData(void); 

void vUSART2_Init(void);
void vUSART2_RX_IRQ_Init(void);

#endif /* UTILS_H_ */
