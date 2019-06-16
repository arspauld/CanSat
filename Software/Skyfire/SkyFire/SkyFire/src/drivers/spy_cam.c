/*
 * spy_cam.c
 *
 * Created: 4/14/2019 12:44:34 AM
 *  Author: Pat Smith
 */ 

#include "spy_cam.h"
#include <asf.h>

void cam_init(void){
	PORTA.DIR |= PIN3_bm; // Sets A3 to output
	delay_ms(100);
	//cam_switch();
}

void cam_switch(void){
	PORTA.OUT ^= PIN3_bm; // Changes A3 from high to low or vice-versa
}