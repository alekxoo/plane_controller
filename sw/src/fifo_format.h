/*
 * fifo_format.h
 *
 *  Created on: Oct 8, 2024
 *      Author: alekxoo
 */

#ifndef FIFO_FORMAT_H_
#define FIFO_FORMAT_H_

#include <stdint.h>

#define FIFO_SIZE 256

typedef struct {
    uint16_t buffer[FIFO_SIZE];
    int head;
    int tail;
    int count;
} Fifo;

void Fifo_Init(Fifo *fifo);
int Fifo_Put(Fifo *fifo, uint16_t data);
int Fifo_Get(Fifo *fifo, uint16_t *data);
uint16_t Fifo_Size(Fifo *fifo);

#endif
