/*
 * spy_cam.c
 *
 * Created: 4/14/2019 12:44:34 AM
 *  Author: Pat Smith
 */ 

#include "spy_cam.h"
#include <asf.h>

void cam_init(void){
	PORTA.DIR |= 0x08; // Sets A3 to output
}

void cam_switch(void){
	PORTA.OUT ^= 0x08; // Changes A3 from high to low or vice-versa
}