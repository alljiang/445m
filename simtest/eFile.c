// filename ************** eFile.c *****************************
// High-level routines to implement a solid-state disk 
// Students implement these functions in Lab 4
// Jonathan W. Valvano 1/12/20
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "sim-compat.h"

bool filesystem_initialized = false;
uint8_t buffer1[512];
uint8_t buffer2[512];
uint8_t buffer3[512];
int16_t buffer1Size;
int16_t loadedDataBlock;
int16_t readIndex;
bool writing, reading;

extern Sema4Type SPIFree;

// links free blocks block1 and block2 in sequential order
// Output: 0 if successful and 1 on failure
int
linkFreeBlocks(int16_t block1, int16_t block2) {
    int rv = 0;

    rv = eDisk_ReadBlock(buffer1, block1);
    if (rv == 1) goto exit;

    rv = eDisk_ReadBlock(buffer2, block2);
    if (rv == 1) goto exit;

    buffer1[0] = block2 & 0xFFu;
    buffer1[1] = block2 >> 8u;
    buffer2[2] = block1 & 0xFFu;
    buffer2[3] = block1 >> 8u;

    rv = eDisk_WriteBlock(buffer1, block1);
    if (rv == 1) goto exit;

    rv = eDisk_WriteBlock(buffer2, block2);
    if (rv == 1) goto exit;

exit:
    return rv;
}

// inserts block2 and populates as free block
// Output: 0 if successful and 1 on failure
int
insertFreeBlock(int16_t block1, int16_t block2, int16_t block3) {
    int rv = 0;

    // insert block2 between block1 and block3
    rv = eDisk_ReadBlock(buffer1, block1);
    if (rv == 1) goto exit;

    rv = eDisk_ReadBlock(buffer2, block3);
    if (rv == 1) goto exit;

    buffer1[0] = block2 & 0xFFu;
    buffer1[1] = block2 >> 8u;
    buffer2[2] = block2 & 0xFFu;
    buffer2[3] = block2 >> 8u;

    rv = eDisk_WriteBlock(buffer1, block1);
    if (rv == 1) goto exit;

    rv = eDisk_WriteBlock(buffer2, block3);
    if (rv == 1) goto exit;

    // populate block2 as free block
    memset(buffer1, 0, 512);
    buffer1[0] = block3 & 0xFFu;
    buffer1[1] = block3 >> 8u;
    buffer1[2] = block1 & 0xFFu;
    buffer1[3] = block1 >> 8u;

    rv = eDisk_WriteBlock(buffer1, block2);
    if (rv == 1) goto exit;

exit:
    return rv;
}

//---------- eFile_Init-----------------
// Activate the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure (already initialized)
int
eFile_Init(void) { // initialize file system
    int rv = 0;

    if (filesystem_initialized) {
        rv = 1;
        goto exit;
    }

    filesystem_initialized = true;

    // initialize disk
    rv = eDisk_Init(0);
    if (rv == 1) goto exit;

    writing = false;
    reading = false;

    OS_InitSemaphore(&SPIFree, 1);

exit:
    return rv;
}

//---------- eFile_Format-----------------
// Erase all files, create blank directory, initialize free space manager
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int
eFile_Format(void) { // erase disk, add format
    int rv = 0;

    OS_Wait(&SPIFree);

    if (!filesystem_initialized) {
        rv = 1;
        goto exit;
    }

    // write zeros to buffer
    memset(buffer1, 0, sizeof(buffer1));

    // Initialize File Index List (Block 0)
    buffer1[0] = 1u & 0xFFu;
    buffer1[1] = 1u >> 8u;  // point first free block to block 1
    if (eDisk_WriteBlock(buffer1, 0)) {
        rv = 1;
        goto exit;
    }

    // Initialize Free Blocks (All other blocks)
    for (int block = 1; block < DISK_BLOCK_COUNT; block++) {
        int nextFreeBlock = block + 1;
        int prevFreeBlock = block - 1;

        // make it a circular doubly-linked list
        if (block == DISK_BLOCK_COUNT - 1) nextFreeBlock = 1;
        if (block == 1) prevFreeBlock = DISK_BLOCK_COUNT - 1;

        buffer1[0] = nextFreeBlock & 0xFFu;
        buffer1[1] = nextFreeBlock >> 8u;
        buffer1[2] = prevFreeBlock & 0xFFu;
        buffer1[3] = prevFreeBlock >> 8u;

        if (eDisk_WriteBlock(buffer1, block)) {
            rv = 1;
            goto exit;
        }
    }

exit:
    OS_Signal(&SPIFree);
    return rv;
}

//---------- eFile_Mount-----------------
// Mount the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure
int
eFile_Mount(void) { // initialize file system
    int rv = 0;

    OS_Wait(&SPIFree);

    if (!filesystem_initialized) {
        rv = 1;
        goto exit;
    }

    // get file index list
    rv = eDisk_ReadBlock(buffer1, 0);
    if (rv) goto exit;

exit:
    OS_Signal(&SPIFree);
    return rv;
}

//---------- eFile_Create-----------------
// Create a new, empty file with one allocated block
// Input: file name is an ASCII string up to seven characters 
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int
eFile_Create(const char name[]) {  // create new file, make it empty
    int rv = 0;
    int length;
    int metadataBlock;
    int dataBlock;
    int nextFreeBlock;
    int prevFreeBlock;
    int blockIndex;
    int i;

    OS_Wait(&SPIFree);

    if (!filesystem_initialized) {
        rv = 1;
        goto exit;
    }

    // verify length of name is in range [1, FILENAME_MAX_LENGTH]
    for (length = 0; length <= FILENAME_MAX_LENGTH; length++) {
        if (name[length] == 0) {
            break;
        }
    }
    if (length == 0 || length > FILENAME_MAX_LENGTH) {
        rv = 1;
        goto exit;
    }

    // read file index list into buffer
    rv = eDisk_ReadBlock(buffer1, 0);
    if (rv == 1) goto exit;

    // get next free block to allocate a metadata block
    metadataBlock = (buffer1[1] << 8u) | buffer1[0];

    if (metadataBlock == 0) {
        // no more free blocks, cannot write
        rv = 1;
        goto exit;
    }

    // search file index list to make sure file does not already exist
    // find file in file index list
    for (i = 2; i < sizeof(buffer1); i += 2) {
        int blockIndex = (buffer1[i + 1] << 8u) | buffer1[i];
        if (blockIndex == 0) {
            continue;
        }

        rv = eDisk_ReadBlock(buffer2, blockIndex);
        if (rv == 1) goto exit;

        if (strncmp(name, (const char*) (buffer2 + 2), length) == 0) {
            // file already exists
            rv = 1;
            goto exit;
        }
    }

    // add metadataBlock to file index list
    for (i = 2; i < sizeof(buffer1); i += 2) {
        blockIndex = (buffer1[i + 1] << 8u) | buffer1[i];
        if (blockIndex == 0) {
            break;
        }
    }
    if (i >= sizeof(buffer1)) {
        // all indices used up, cannot create any more new files
        rv = 1;
        goto exit;
    }
    buffer1[i] = metadataBlock & 0xFFu;
    buffer1[i + 1] = metadataBlock >> 8u;

    // get the next free block to allocate a data block
    rv = eDisk_ReadBlock(buffer2, metadataBlock);
    if (rv == 1) goto exit;

    dataBlock = (buffer2[1] << 8u) | buffer2[0];
    prevFreeBlock = (buffer2[3] << 8u) | buffer2[2];

    // populate metadata block
    buffer2[0] = dataBlock & 0xFFu;
    buffer2[1] = dataBlock >> 8u;
    memcpy(buffer2 + 2, name, length);

    // get the next free block to replace the one in file index list
    rv = eDisk_ReadBlock(buffer3, dataBlock);
    if (rv == 1) goto exit;

    nextFreeBlock = (buffer3[1] << 8u) | buffer3[0];

    // populate data block
    buffer3[0] = 0;
    buffer3[1] = 0;
    buffer3[2] = 508u & 0xFFu;
    buffer3[3] = 508u >> 8u;

    // update file index list free block
    buffer1[0] = nextFreeBlock & 0xFFu;
    buffer1[1] = nextFreeBlock >> 8u;

    // update file index list
    rv = eDisk_WriteBlock(buffer1, 0);
    if (rv == 1) goto exit;

    // write metadata block
    rv = eDisk_WriteBlock(buffer2, metadataBlock);
    if (rv == 1) goto exit;

    // write data block
    rv = eDisk_WriteBlock(buffer3, dataBlock);
    if (rv == 1) goto exit;

    rv = linkFreeBlocks(prevFreeBlock, nextFreeBlock);
    if (rv == 1) goto exit;

exit:
    OS_Signal(&SPIFree);
    return rv;
}

//---------- eFile_WOpen-----------------
// Open the file, read into RAM last block
// Input: file name is an ASCII string up to seven characters
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int
eFile_WOpen(const char name[]) {      // open a file for writing
    int rv = 0;
    int length;
    int dataBlockPtr = 0;
    int numberOfBytesRemaining;
    int blockIndex;

    OS_Wait(&SPIFree);

    if (!filesystem_initialized) {
        rv = 1;
        goto exit;
    }

    // check if file already open
    if (writing || reading) {
        rv = 1;
        goto exit;
    }

    // verify length of name is in range [1, FILENAME_MAX_LENGTH]
    for (length = 0; length <= FILENAME_MAX_LENGTH; length++) {
        if (name[length] == 0) {
            break;
        }
    }
    if (length == 0 || length > FILENAME_MAX_LENGTH) {
        rv = 1;
        goto exit;
    }

    // read file index list into buffer
    rv = eDisk_ReadBlock(buffer1, 0);
    if (rv == 1) goto exit;

    // find file in file index list
    for (int i = 2; i < sizeof(buffer1); i += 2) {
        blockIndex = (buffer1[i + 1] << 8u) | buffer1[i];
        if (blockIndex == 0) {
            continue;
        }

        rv = eDisk_ReadBlock(buffer2, blockIndex);
        if (rv == 1) goto exit;

        if (strncmp(name, (char*) (buffer2 + 2), length) == 0) {
            // found file, read last block into RAM
            dataBlockPtr = (buffer2[1] << 8u) | buffer2[0];
            break;
        }
    }

    if (dataBlockPtr == 0) {
        // could not find file
        rv = 1;
        goto exit;
    }

    rv = eDisk_ReadBlock(buffer3, dataBlockPtr);
    if (rv == 1) goto exit;

    // keep on getting the next data block in linked list until we reach
    // a point where we have bytes remaining (which means the last page)
    numberOfBytesRemaining = (buffer3[3] << 8u) | buffer3[2];
    while (numberOfBytesRemaining == 0) {
        dataBlockPtr = (buffer3[1] << 8u) | buffer3[0];

        rv = eDisk_ReadBlock(buffer3, dataBlockPtr);
        if (rv == 1) goto exit;

        numberOfBytesRemaining = (buffer3[3] << 8u) | buffer3[2];
    }

    // at this point, buffer3 will contain the last block of the file
    loadedDataBlock = dataBlockPtr;
    writing = true;

exit:
    OS_Signal(&SPIFree);
    return rv;
}

//---------- eFile_Write-----------------
// save at end of the open file
// Input: data to be saved
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int
eFile_Write(const char data) {
    int rv = 0;
    int bytesRemaining = (buffer3[3] << 8u) | buffer3[2];
    int dataBlock, prevFreeBlock, nextFreeBlock;
    int index;

    OS_Wait(&SPIFree);

    if (!filesystem_initialized) {
        rv = 1;
        goto exit;
    }

    if (!writing) {
        rv = 1;
        goto exit;
    }

    if (bytesRemaining == 0) {
        // we are at the end of the file, need to allocate a new block

        // read file index list into buffer
        rv = eDisk_ReadBlock(buffer1, 0);
        if (rv == 1) goto exit;

        // get next free block to allocate a data block
        rv = eDisk_ReadBlock(buffer1, 0);
        dataBlock = (buffer1[1] << 8u) | buffer1[0];

        if (dataBlock == 0) {
            // no more free blocks, cannot write
            rv = 1;
            goto exit;
        }

        // load in the free block
        rv = eDisk_ReadBlock(buffer2, dataBlock);
        if (rv == 1) goto exit;

        nextFreeBlock = (buffer2[1] << 8u) | buffer2[0];
        prevFreeBlock = (buffer2[3] << 8u) | buffer2[2];

        // update the next free block
        buffer1[0] = nextFreeBlock & 0xFFu;
        buffer1[1] = nextFreeBlock >> 8u;

        // update the next data block entry in the current data block
        buffer3[0] = dataBlock & 0xFFu;
        buffer3[1] = dataBlock >> 8u;

        // update the loaded data block
        rv = eDisk_WriteBlock(buffer3, loadedDataBlock);
        if (rv == 1) goto exit;

        // update the file index list
        rv = eDisk_WriteBlock(buffer1, 0);
        if (rv == 1) goto exit;

        loadedDataBlock = dataBlock;

        // add data to the new data block
        buffer2[4] = data;

        // update pointer to next data block
        buffer2[0] = 0;
        buffer2[1] = 0;

        // update bytes remaining to 508
        buffer2[2] = 508u & 0xFFu;
        buffer2[3] = 508u >> 8u;
        bytesRemaining = 508;

        memcpy(buffer3, buffer2, sizeof(buffer2));

        // link up prev and next free blocks
        rv = linkFreeBlocks(prevFreeBlock, nextFreeBlock);
        if (rv == 1) goto exit;
    }

    // write data to buffer
    index = 508 - bytesRemaining + 4;
    buffer3[index] = data;

    // update bytes remaining
    bytesRemaining -= 1;
    buffer3[2] = bytesRemaining & 0xFFu;
    buffer3[3] = bytesRemaining >> 8u;

exit:
    OS_Signal(&SPIFree);
    return rv;
}

//---------- eFile_WClose-----------------
// close the file, left disk in a state power can be removed
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int
eFile_WClose(void) { // close the file for writing
    int rv = 0;

    OS_Wait(&SPIFree);

    if (!filesystem_initialized) {
        rv = 1;
        goto exit;
    }

    if (!writing) {
        rv = 1;
        goto exit;
    }

    rv = eDisk_WriteBlock(buffer3, loadedDataBlock);
    if (rv == 1) goto exit;

    writing = false;

exit:
    OS_Signal(&SPIFree);
    return rv;
}

//---------- eFile_ROpen-----------------
// Open the file, read first block into RAM 
// Input: file name is an ASCII string up to seven characters
// Output: 0 if successful and 1 on failure (e.g., trouble read to flash)
int
eFile_ROpen(const char name[]) {      // open a file for reading
    int rv = 0;
    int length = 0;
    int dataBlockPtr;
    int i;

    OS_Wait(&SPIFree);

    if (!filesystem_initialized) {
        rv = 1;
        goto exit;
    }

    // check if file already open
    if (reading || writing) {
        rv = 1;
        goto exit;
    }

    // verify length of name is in range [1, FILENAME_MAX_LENGTH]
    for (length = 0; length <= FILENAME_MAX_LENGTH; length++) {
        if (name[length] == 0) {
            break;
        }
    }
    if (length == 0 || length > FILENAME_MAX_LENGTH) {
        rv = 1;
        goto exit;
    }

    rv = eDisk_ReadBlock(buffer1, 0);
    if (rv == 1) goto exit;

    // find file in file index list
    for (i = 2; i < sizeof(buffer1); i += 2) {
        int blockIndex = (buffer1[i + 1] << 8u) | buffer1[i];
        if (blockIndex == 0) {
            continue;
        }

        rv = eDisk_ReadBlock(buffer2, blockIndex);
        if (rv == 1) goto exit;

        if (strncmp(name, (const char*) (buffer2 + 2), length) == 0) {
            // found file, read first block into buffer3
            dataBlockPtr = (buffer2[1] << 8u) | buffer2[0];

            rv = eDisk_ReadBlock(buffer3, dataBlockPtr);
            if (rv == 1) goto exit;
            break;
        }
    }

    if (i == 512) {
        // file not found
        rv = 1;
        goto exit;
    }

    loadedDataBlock = dataBlockPtr;
    reading = true;
    readIndex = 0;

exit:
    OS_Signal(&SPIFree);
    return rv;
}

//---------- eFile_ReadNext-----------------
// retreive data from open file
// Input: none
// Output: return by reference data
//         0 if successful and 1 on failure (e.g., end of file)
int
eFile_ReadNext(char *pt) {       // get next byte
    int rv = 0;
    int bytesRemaining;
    int nextDataBlock;

    OS_Wait(&SPIFree);

    if (!filesystem_initialized) {
        rv = 1;
        goto exit;
    }

    if (!reading) {
        rv = 1;
        goto exit;
    }

    // check if at end of file
    if (readIndex == 508) {
        // read next data block
        nextDataBlock = (buffer3[1] << 8u) | buffer3[0];
        if (nextDataBlock == 0) {
            // no more blocks, end of file
            rv = 1;
            goto exit;
        }

        rv = eDisk_ReadBlock(buffer3, nextDataBlock);
        if (rv == 1) goto exit;

        loadedDataBlock = nextDataBlock;
        readIndex = 0;
    }

    // check for end of file
    bytesRemaining = (buffer3[3] << 8u) | buffer3[2];
    if (readIndex >= 508 - bytesRemaining) {
        rv = 1;
        goto exit;
    }

    // read next byte
    *pt = buffer3[readIndex + 4];
    readIndex++;

exit:
    OS_Signal(&SPIFree);
    return rv;
}

//---------- eFile_RClose-----------------
// close the reading file
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int
eFile_RClose(void) { // close the file for writing
    int rv = 0;

    OS_Wait(&SPIFree);

    if (!filesystem_initialized) {
        rv = 1;
        goto exit;
    }

    if (!reading) {
        rv = 1;
        goto exit;
    }

    reading = false;

exit:
    OS_Signal(&SPIFree);
    return rv;
}

//---------- eFile_Delete-----------------
// delete this file
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int
eFile_Delete(const char name[]) {  // remove this file
    int rv = 0;
    int length = 0;
    int dataBlockPtr, metadataBlockIndex;
    int nextDataBlock;
    int lastFreeBlock;
    int firstFreeBlock;
    int i;

    OS_Wait(&SPIFree);

    if (!filesystem_initialized) {
        rv = 1;
        goto exit;
    }

    // check if file already open
    if (reading || writing) {
        rv = 1;
        goto exit;
    }

    // verify length of name is in range [1, FILENAME_MAX_LENGTH]
    for (length = 0; length <= FILENAME_MAX_LENGTH; length++) {
        if (name[length] == 0) {
            break;
        }
    }
    if (length == 0 || length > FILENAME_MAX_LENGTH) {
        rv = 1;
        goto exit;
    }

    rv = eDisk_ReadBlock(buffer1, 0);
    if (rv == 1) goto exit;

    // find file in file index list
    for (i = 2; i < sizeof(buffer1); i += 2) {
        metadataBlockIndex = (buffer1[i + 1] << 8u) | buffer1[i];
        if (metadataBlockIndex == 0) {
            continue;
        }

        rv = eDisk_ReadBlock(buffer2, metadataBlockIndex);
        if (rv == 1) goto exit;

        if (strncmp(name, (char*) (buffer2 + 2), length) == 0) {
            // found file
            dataBlockPtr = (buffer2[1] << 8u) | buffer2[0];

            break;
        }
    }

    if (i == 512) {
        // file not found
        rv = 1;
        goto exit;
    }

    // delete file

    firstFreeBlock = (buffer1[1] << 8u) | buffer1[0];

    // remove file from file index list
    buffer1[i] = 0;
    buffer1[i + 1] = 0;

    // write file index list to flash
    rv = eDisk_WriteBlock(buffer1, 0);
    if (rv == 1) goto exit;

    // get last free block
    rv = eDisk_ReadBlock(buffer1, firstFreeBlock);
    if (rv == 1) goto exit;
    lastFreeBlock = (buffer1[3] << 8u) | buffer1[2];

    // delete metadata block
    rv = insertFreeBlock(lastFreeBlock, metadataBlockIndex, firstFreeBlock);
    if (rv == 1) goto exit;
    lastFreeBlock = metadataBlockIndex;

    // delete all data blocks
    while (true) {
        rv = eDisk_ReadBlock(buffer1, dataBlockPtr);
        if (rv == 1) goto exit;
        nextDataBlock = (buffer1[1] << 8u) | buffer1[0];

        rv = insertFreeBlock(lastFreeBlock, dataBlockPtr, firstFreeBlock);
        if (rv == 1) goto exit;
        lastFreeBlock = dataBlockPtr;

        if (nextDataBlock == 0) {
            break;
        }

        dataBlockPtr = nextDataBlock;
    }

exit:
    OS_Signal(&SPIFree);
    return rv;
}

int
eFile_PrintDirectory(void
(*print)(char*)) {
    int rv = 0;

    OS_Wait(&SPIFree);

    if (!filesystem_initialized) {
        rv = 1;
        goto exit;
    }

    rv = eDisk_ReadBlock(buffer1, 0);
    if (rv == 1) goto exit;

    for (int i = 2; i < sizeof(buffer1); i += 2) {
        int blockIndex = (buffer1[i + 1] << 8u) | buffer1[i];
        if (blockIndex == 0) {
            continue;
        }

        rv = eDisk_ReadBlock(buffer2, blockIndex);
        if (rv == 1) goto exit;

        print((char*) (buffer2 + 2));
        print("\r\n");
    }

exit:
    OS_Signal(&SPIFree);
    return rv;
}
