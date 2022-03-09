// filename ************** eFile.c *****************************
// High-level routines to implement a solid-state disk 
// Students implement these functions in Lab 4
// Jonathan W. Valvano 1/12/20
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <RTOS/eDisk.h>
#include <RTOS/eFile.h>
#include <RTOS/OS.h>

bool filesystem_initialized = false;
uint8_t buffer1[512];
uint8_t buffer2[512];
uint8_t buffer3[512];
int16_t buffer1Size;

// links free blocks block1 and block2 in sequential order
// Output: 0 if successful and 1 on failure
int
linkFreeBlocks(int16_t block1, int16_t block2) {
    int rv = 0;

    rv = eDisk_ReadBlock(buffer1, block1);
    if (rv == 1) goto exit;

    rv = eDisk_ReadBlock(buffer2, block2);
    if (rv == 1) goto exit;

    buffer1[0] = block2 >> 8u;
    buffer1[1] = block2 % 8u;
    buffer2[2] = block1 >> 8u;
    buffer2[3] = block1 % 8u;

    rv = eDisk_WriteBlock(buffer1, block1);
    if (rv == 1) goto exit;

    rv = eDisk_WriteBlock(buffer2, block2);
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

    rv = eFile_Format();
    if (rv == 1) goto exit;

    // initialize disk
    rv = eDisk_Init(0);
    if (rv == 1) goto exit;

exit:
    return rv;   // replace
}

//---------- eFile_Format-----------------
// Erase all files, create blank directory, initialize free space manager
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int
eFile_Format(void) { // erase disk, add format
    int rv = 0;

    if (!filesystem_initialized) {
        rv = 1;
    }

    // write zeros to buffer
    memset(buffer1, 0, sizeof(buffer1));

    // Initialize File Index List (Block 0)
    buffer1[0] = 1u >> 8u;
    buffer1[1] = 1u % 8u;  // point first free block to block 1
    if (eDisk_WriteBlock(buffer1, 0)) {
        rv = 1;
        goto exit;
    }

    // Initialize Free Blocks (All other blocks)
    for (int block = 1; block < DISK_SIZE_KB; block++) {
        int nextFreeBlock = block + 1;
        int lastFreeBlock = block - 1;

        // make it a circular doubly-linked list
        if (block == DISK_BLOCK_COUNT - 1) nextFreeBlock = 1;
        if (block == 1) lastFreeBlock = DISK_BLOCK_COUNT - 1;

        buffer1[0] = nextFreeBlock >> 8u;
        buffer1[1] = nextFreeBlock % 8u;
        buffer1[2] = lastFreeBlock >> 8u;
        buffer1[3] = lastFreeBlock % 8u;

        if (eDisk_WriteBlock(buffer1, block)) {
            rv = 1;
            goto exit;
        }
    }

exit:
    return rv;
}

//---------- eFile_Mount-----------------
// Mount the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure
int
eFile_Mount(void) { // initialize file system
    int rv = 0;

    // get file index list
    rv = eDisk_ReadBlock(buffer1, 0);
    if(rv) goto exit;

    return rv;   // replace
}

//---------- eFile_Create-----------------
// Create a new, empty file with one allocated block
// Input: file name is an ASCII string up to seven characters 
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int
eFile_Create(const char name[]) {  // create new file, make it empty
    int rv = 0;
    int length;
    int nextFreeBlock;
    int lastFreeBlock;
    int i;

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
    rv = eFile_Mount();
    if(rv == 1) goto exit;

    // get next free block to allocate a metadata block
    nextFreeBlock = (buffer1[1] << 8u) | buffer1[0];
    lastFreeBlock = (buffer2[1] << 8u) | buffer2[0];

    // add nextFreeBlock to file index list
    for (i = 1; i < sizeof(buffer1); i++) {
        if (buffer1[i] == 0) {
            break;
        }
    }
    if (i >= sizeof(buffer1)) {
        // all indices used up, cannot create any more new files
        rv = 1;
        goto exit;
    }
    buffer1[i] = nextFreeBlock;

    // get the next free block to allocate a data block
    rv = eDisk_ReadBlock(buffer2, nextFreeBlock);
    if (rv == 1) goto exit;

    nextFreeBlock = (buffer2[1] << 8u) | buffer2[0];

    // populate metadata block
    buffer2[0] = nextFreeBlock >> 8u;
    buffer2[1] = nextFreeBlock % 8u;
    memcpy(buffer2 + 2, name, length);

    // get the next free block to replace the one in file index list
    rv = eDisk_ReadBlock(buffer3, nextFreeBlock);
    if (rv == 1) goto exit;

    nextFreeBlock = (buffer3[1] << 8u) | buffer3[0];

    // populate data block
    buffer3[0] = 0;
    buffer3[1] = 0;
    buffer3[2] = 508u >> 8u;
    buffer3[3] = 508u % 8u;

    // update file index list
    rv = eDisk_WriteBlock(buffer1, 0);
    if(rv == 1) goto exit;

    // write metadata block
    rv = eDisk_WriteBlock(buffer2, 0);
    if(rv == 1) goto exit;

    // write data block
    rv = eDisk_WriteBlock(buffer3, 0);
    if(rv == 1) goto exit;

    rv = linkFreeBlocks(lastFreeBlock, nextFreeBlock);
    if(rv == 1) goto exit;
exit:
    return rv;
}

//---------- eFile_WOpen-----------------
// Open the file, read into RAM last block
// Input: file name is an ASCII string up to seven characters
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int
eFile_WOpen(const char name[]) {      // open a file for writing

    return 1;   // replace
}

//---------- eFile_Write-----------------
// save at end of the open file
// Input: data to be saved
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int
eFile_Write(const char data) {

    return 1;   // replace
}

//---------- eFile_WClose-----------------
// close the file, left disk in a state power can be removed
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int
eFile_WClose(void) { // close the file for writing

    return 1;   // replace
}

//---------- eFile_ROpen-----------------
// Open the file, read first block into RAM 
// Input: file name is an ASCII string up to seven characters
// Output: 0 if successful and 1 on failure (e.g., trouble read to flash)
int
eFile_ROpen(const char name[]) {      // open a file for reading

    return 1;   // replace
}

//---------- eFile_ReadNext-----------------
// retreive data from open file
// Input: none
// Output: return by reference data
//         0 if successful and 1 on failure (e.g., end of file)
int
eFile_ReadNext(char *pt) {       // get next byte

    return 1;   // replace
}

//---------- eFile_RClose-----------------
// close the reading file
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int
eFile_RClose(void) { // close the file for writing

    return 1;   // replace
}

//---------- eFile_Delete-----------------
// delete this file
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int
eFile_Delete(const char name[]) {  // remove this file

    return 1;   // replace
}

//---------- eFile_DOpen-----------------
// Open a (sub)directory, read into RAM
// Input: directory name is an ASCII string up to seven characters
//        (empty/NULL for root directory)
// Output: 0 if successful and 1 on failure (e.g., trouble reading from flash)
int
eFile_DOpen(const char name[]) { // open directory

    return 1;   // replace
}

//---------- eFile_DirNext-----------------
// Retreive directory entry from open directory
// Input: none
// Output: return file name and size by reference
//         0 if successful and 1 on failure (e.g., end of directory)
int
eFile_DirNext(char *name[], unsigned long *size) {  // get next entry

    return 1;   // replace
}

//---------- eFile_DClose-----------------
// Close the directory
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int
eFile_DClose(void) { // close the directory

    return 1;   // replace
}

//---------- eFile_Unmount-----------------
// Unmount and deactivate the file system
// Input: none
// Output: 0 if successful and 1 on failure (not currently mounted)
int
eFile_Unmount(void) {

    return 1;   // replace
}
