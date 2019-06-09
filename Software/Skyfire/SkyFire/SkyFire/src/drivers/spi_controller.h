/*
 * spi_controller.h
 *
 * Created: 9/3/2018 12:50:23 PM
 *  Author: cellis
 */ 


#ifndef SPI_CONTROLLER_H_
#define SPI_CONTROLLER_H_


void spi_init(void);

void spi_write(uint8_t data);

uint8_t spi_read(void);

void spi_select(uint8_t port);


#endif /* SPI_CONTROLLER_H_ */