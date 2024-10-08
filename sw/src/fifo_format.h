/*
 * fifo_format.h
 *
 *  Created on: Oct 8, 2024
 *      Author: alekxoo
 */

#ifndef FIFO_FORMAT_H_
#define FIFO_FORMAT_H_

#include <stdint.h>

#define FIFO_SIZE 64

typedef struct {
    uint8_t buffer[FIFO_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} Fifo;

void Fifo_Init(Fifo *fifo);
int Fifo_Put(Fifo *fifo, uint8_t data);
int Fifo_Get(Fifo *fifo, uint8_t *data);
uint8_t Fifo_Size(Fifo *fifo);

#endif
