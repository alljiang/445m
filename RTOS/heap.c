// filename *************************heap.c ************************
// Implements memory heap for dynamic memory allocation.
// Follows standard malloc/calloc/realloc/free interface
// for allocating/unallocating memory.

// Jacob Egner 2008-07-31
// modified 8/31/08 Jonathan Valvano for style
// modified 12/16/11 Jonathan Valvano for 32-bit machine
// modified August 10, 2014 for C99 syntax

/* This example accompanies the book
   "Embedded Systems: Real Time Operating Systems for ARM Cortex M Microcontrollers",
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2015

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains

 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */


#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "RTOS/heap.h"

#define HEAP_SIZE  2048u

#define set16(array, index, value) (array[index] = (value) >> 8, array[index+1] = (value) & 0xFF)
#define get16(array, index) ((array[index] << 8) | array[index+1])

int heap[HEAP_SIZE];

//******** Heap_Init *************** 
// Initialize the Heap
// input: none
// output: always 0
// notes: Initializes/resets the heap to a clean state where no memory
//  is allocated.
int32_t Heap_Init(void){
  memset(heap, 0, HEAP_SIZE);  
  
  // initialize the free block, leaving room for the header
  set16(heap, 0, HEAP_SIZE - 2);

  return 0;   // replace
}


//******** Heap_Malloc *************** 
// Allocate memory, data not initialized
// input: 
//   desiredBytes: desired number of bytes to allocate
// output: void* pointing to the allocated memory or will return NULL
//   if there isn't sufficient space to satisfy allocation request
void* Heap_Malloc(int32_t desiredBytes){
  void* rv = 0;

  // find the first free block that is large enough
  uint16_t search_index = 0;
	bool foundFreeBlock = false;

  while (search_index < HEAP_SIZE) {
    int16_t free_size = get16(heap, search_index);
    
    if (free_size >= desiredBytes) {
      // found a free block that is large enough

      // if the remainder is <= 2, then we can just use the whole block
      if (free_size - desiredBytes<= 2) {
        // use entire block
        set16(heap, search_index, -free_size);
      } else {
        // only use desired number of bytes of the block
        set16(heap, search_index, -desiredBytes);

        // set the remainder to free
        set16(heap, search_index + desiredBytes + 2, free_size - desiredBytes - 2);
      }

			foundFreeBlock = true;
      break;
    } else {
      // keep searching
      if (free_size < 0) {
        // this block is already allocated
        search_index -= free_size;
      } else {
        // this block is free, but not large enough
        search_index += free_size;
      }
      // account for header
      search_index += 2;
    }
  }
  
  if (!foundFreeBlock) {
    // no free block large enough
    rv = NULL;
  } else {
    // found a free block large enough
    // allocate the block, account for header
    rv = &heap[search_index + 2];
  }

  return (void *) rv;
}


//******** Heap_Calloc *************** 
// Allocate memory, data are initialized to 0
// input:
//   desiredBytes: desired number of bytes to allocate
// output: void* pointing to the allocated memory block or will return NULL
//   if there isn't sufficient space to satisfy allocation request
//notes: the allocated memory block will be zeroed out
void* Heap_Calloc(int32_t desiredBytes){  
  uint8_t *arr = Heap_Malloc(desiredBytes);
  if(arr != NULL){
    memset(arr, 0, desiredBytes);
  }
  return arr;   // NULL
}


//******** Heap_Realloc *************** 
// Reallocate buffer to a new size
//input: 
//  oldBlock: pointer to a block
//  desiredBytes: a desired number of bytes for a new block
// output: void* pointing to the new block or will return NULL
//   if there is any reason the reallocation can't be completed
// notes: the given block may be unallocated and its contents
//   are copied to a new block if growing/shrinking not possible
void* Heap_Realloc(void* oldBlock, int32_t desiredBytes){
  uint8_t *newBlock = Heap_Malloc(desiredBytes);

  if(newBlock != NULL){
    memcpy(newBlock, oldBlock, desiredBytes);
    Heap_Free(oldBlock);
  }

  return newBlock;   // NULL
}

void mergeFreeBlocks() {
  uint16_t search_index = 0;
  uint16_t next_search_index = 0;
  while (search_index < HEAP_SIZE) {
    int16_t free_size = get16(heap, search_index);
    if (free_size < 0) {
      // this block is not free
      next_search_index = search_index - free_size + 2;
    } else {
      next_search_index = search_index + free_size + 2;

      // check if the next block is also free
      int16_t next_free_size = get16(heap, next_search_index);
      if (next_free_size > 0) {
        // merge the two blocks
        set16(heap, search_index, free_size + next_free_size + 2);
        next_search_index += next_free_size + 2;
      }
    }

    search_index = next_search_index;
  }
}

//******** Heap_Free *************** 
// return a block to the heap
// input: pointer to memory to unallocate
// output: 0 if everything is ok, non-zero in case of error (e.g. invalid pointer
//     or trying to unallocate memory that has already been unallocated
int32_t Heap_Free(void* pointer){
  int32_t rv = 0;

  int16_t blockIndex = ((uintptr_t) pointer - (uintptr_t) heap) - 2;
	int16_t blockSize;

  if (blockIndex < 0 || blockIndex >= HEAP_SIZE) {
    // invalid pointer
    rv = 1;
    goto exit;
  }

  blockSize = get16(heap, blockIndex);

  if (blockSize > 0) {
    // block is already unallocated
    rv = 2;
    goto exit;
  }

  // mark the block as free
  set16(heap, blockIndex, -blockSize);

  mergeFreeBlocks();
exit:
  return rv;
}


//******** Heap_Stats *************** 
// return the current status of the heap
// input: reference to a heap_stats_t that returns the current usage of the heap
// output: 0 in case of success, non-zeror in case of error (e.g. corrupted heap)
int32_t Heap_Stats(heap_stats_t *stats){
  int rv = 0;

  stats->size = HEAP_SIZE;

  uint16_t blockIndex = 0;
  int free_size = 0;
  int used_size = 0;

  while (blockIndex < HEAP_SIZE) {
    int16_t blockSize = get16(heap, blockIndex);

    if (blockSize > 0) {
      // this block is free
      free_size += blockSize;
			blockIndex += blockSize;
    } else {
      // this block is allocated
      used_size += -blockSize;
			blockIndex -= blockSize;
    }

    blockIndex += 2;
  }

  if (free_size < 0 || used_size < 0) {
    // corrupted heap
    rv = 1;
    goto exit;
  }

  if (blockIndex != HEAP_SIZE) {
    // corrupted heap
    rv = 1;
    goto exit;
  }

  stats->free = free_size;
  stats->used = used_size;

exit:
  return rv;   // replace
}
