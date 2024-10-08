/*
 * fifo_format.c
 *
 *  Created on: Oct 8, 2024
 *      Author: alekxoo
 */


#include "fifo_format.h"

void Fifo_Init(Fifo *fifo) {
    fifo->head = 0;
    fifo->tail = 0;
    fifo->count = 0;
}

int Fifo_Put(Fifo *fifo, uint8_t data) {
    if (fifo->count >= FIFO_SIZE) {
        return 0; // FIFO is full
    }
    fifo->buffer[fifo->tail] = data;
    fifo->tail = (fifo->tail + 1) % FIFO_SIZE;
    fifo->count++;
    return 1;
}

int Fifo_Get(Fifo *fifo, uint8_t *data) {
    if (fifo->count == 0) {
        return 0; // FIFO is empty
    }
    *data = fifo->buffer[fifo->head];
    fifo->head = (fifo->head + 1) % FIFO_SIZE;
    fifo->count--;
    return 1;
}

uint8_t Fifo_Size(Fifo *fifo) {
    return fifo->count;
}
