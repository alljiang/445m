// *************os.c**************
// EE445M/EE380L.6 Labs 1, 2, 3, and 4 
// High-level OS functions
// Students will implement these functions as part of Lab
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 
// Jan 12, 2020, valvano@mail.utexas.edu

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <RTOS/eFile.h>
#include <RTOS/OS.h>
#include <RTOS/ST7735.h>
#include <RTOS/UART0int.h>
#include "vware/tm4c123gh6pm.h"
#include "vware/CortexM.h"
#include "vware/PLL.h"
#include "vware/LaunchPad.h"
#include "vware/Timer4A.h"
#include "vware/WTimer0A.h"
#include "vware/ADCT0ATrigger.h"
#include "vware/InterruptFunctions.h"
#include "bit-utils.h"
#include "gpio.h"
#include "timers.h"

#define LAUNCHPAD_BUTTON_DEBOUNCE_MS 10u

#define OS_FIFO_SIZE 128u

#define NEXT_FIFO_INDEX(i) (((i) + 1) % OS_FIFO_SIZE)

extern void
StartOS(void);

void
(*PeriodicTask)(void);

// Performance Measurements 
int32_t MaxJitter;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
uint32_t const JitterSize = JITTERSIZE;
uint32_t JitterHistogram[JITTERSIZE] = { 0, };

uint64_t osTimeMs;
int id_counter = 0;

#define MAX_THREADS_COUNT 20
#define THREAD_STACK_SIZE 256
TCB_t tcb_list[MAX_THREADS_COUNT];
TCBPtr RunPt;
TCBPtr NextRunPt;

int16_t threadCount;

int32_t stack[MAX_THREADS_COUNT][THREAD_STACK_SIZE];

int32_t os_fifo[OS_FIFO_SIZE];
uintptr_t os_fifo_ptr_head;
uintptr_t os_fifo_ptr_tail;

/*------------------------------------------------------------------------------
 Systick Interrupt Handler
 SysTick interrupt happens every 10 ms
 used for preemptive thread switch
 *------------------------------------------------------------------------------*/
unsigned long
OS_LockScheduler(void) {
    // lab 4 might need this for disk formating
    return 0; // replace with solution
}
void
OS_UnLockScheduler(unsigned long previous) {
    // lab 4 might need this for disk formating
}

/**
 * @details  Initialize operating system, disable interrupts until OS_Launch.
 * Initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers.
 * Interrupts not yet enabled.
 * @param  none
 * @return none
 * @brief  Initialize OS
 */
void
OS_Init(void) {
    // put Lab 2 (and beyond) solution here

    // label all threads in list as dead
    for (int i = 0; i < MAX_THREADS_COUNT; i++) {
        tcb_list[i].id = -1;
    }

    threadCount = 0;

    // Set PendSV priority
    NVIC_SYS_PRI3_R = set_bit_field_u32(NVIC_SYS_PRI3_R, 24, 8, 0xD0); // priority 6

    // Start 1ms periodic timer
    Timer0Init();

    // Start high res system timer
    Timer2Init(0xFFFFFFFF, 7);
}

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void
OS_InitSemaphore(Sema4Type *semaPt, int32_t value) {
    // put Lab 2 (and beyond) solution here
    uint32_t sr = StartCritical();
    semaPt->Value = value;
    EndCritical(sr);
}

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void
OS_Wait(Sema4Type *semaPt) {
    // put Lab 2 (and beyond) solution here
    int32_t sr = StartCritical();

    while (semaPt->Value <= 0) {
        EndCritical(sr);
        OS_Suspend();
        sr = StartCritical();
    }

    semaPt->Value--;

    EndCritical(sr);
}

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void
OS_Signal(Sema4Type *semaPt) {
    // put Lab 2 (and beyond) solution here
    int32_t sr = StartCritical();

    semaPt->Value++;

    EndCritical(sr);
}

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void
OS_bWait(Sema4Type *semaPt) {
    // put Lab 2 (and beyond) solution here
    int32_t sr = StartCritical();

    while (semaPt->Value == 0) {
        EndCritical(sr);
        OS_Suspend();
        sr = StartCritical();
    }

    semaPt->Value = 0;

    EndCritical(sr);
}

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void
OS_bSignal(Sema4Type *semaPt) {
    // put Lab 2 (and beyond) solution here
    int32_t sr = StartCritical();

    semaPt->Value = 1;

    EndCritical(sr);
}

//******** OS_AddThread *************** 
// add a foreground thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisible by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int
OS_AddThread(void
(*task)(void), uint32_t stackSize, uint32_t priority) {
    // put Lab 2 (and beyond) solution here
    int rv = 1;
    int listIndex = -1;
    TCBPtr lastTCB;
    TCBPtr tcbPtr = NULL;

    // CRITICAL SECTION
    uint32_t sr = StartCritical();

    if (task == NULL) {
        // task is invalid
        rv = 0;
        goto exit;
    }

    if (stackSize % 8 != 0) {
        // stack size must be divisible by 8
        rv = 0;
        goto exit;
    }

    // search for a dead TCB to recycle (find the first occurrence of id == -1)
    for (int i = 0; i < MAX_THREADS_COUNT; i++) {
        if (tcb_list[i].id == -1) {
            listIndex = i;
            tcbPtr = &tcb_list[i];
            break;
        }
    }

    if (tcbPtr == NULL) {
        // no dead TCBs available, too many active threads
        // increase MAX_THREADS_COUNT
        rv = 0;
        goto exit;
    }

    // populate TCB entry
    tcbPtr->id = id_counter++;         // id is unique, monotonically generated
    tcbPtr->stack_pointer =
            (uintptr_t) &stack[listIndex][THREAD_STACK_SIZE - 16];
    tcbPtr->sleep_state = 0;
    tcbPtr->blocked_state = 0; // ???? TODO

    // initialize this newly generated task's stack
    stack[listIndex][THREAD_STACK_SIZE - 1] = 0x01000000;                // PSR
    stack[listIndex][THREAD_STACK_SIZE - 2] = (int32_t) task;             // PC
    stack[listIndex][THREAD_STACK_SIZE - 3] = 0x14141414;             // R14/LR
    stack[listIndex][THREAD_STACK_SIZE - 4] = 0x12121212;                // R12
    stack[listIndex][THREAD_STACK_SIZE - 5] = 0x03030303;                 // R3
    stack[listIndex][THREAD_STACK_SIZE - 6] = 0x02020202;                 // R2
    stack[listIndex][THREAD_STACK_SIZE - 7] = 0x01010101;                 // R1
    stack[listIndex][THREAD_STACK_SIZE - 8] = 0x00000000;                 // R0
    stack[listIndex][THREAD_STACK_SIZE - 9] = 0x11001100;                // R11
    stack[listIndex][THREAD_STACK_SIZE - 10] = 0x10001000;               // R10
    stack[listIndex][THREAD_STACK_SIZE - 11] = 0x09090909;                // R9
    stack[listIndex][THREAD_STACK_SIZE - 12] = 0x08080808;                // R8
    stack[listIndex][THREAD_STACK_SIZE - 13] = 0x07070707;                // R7
    stack[listIndex][THREAD_STACK_SIZE - 14] = 0x06060606;                // R6
    stack[listIndex][THREAD_STACK_SIZE - 15] = 0x05050505;                // R5
    stack[listIndex][THREAD_STACK_SIZE - 16] = 0x04040404;                // R4

    threadCount++;
    if (threadCount == 1) {
        // first thread, set RunPt
        RunPt = tcbPtr;
        RunPt->TCB_next = RunPt;
        RunPt->TCB_previous = RunPt;
    } else {
        // starting from RunPt, search for the last TCB in line
        lastTCB = RunPt;
        while (lastTCB->TCB_next != RunPt) {
            lastTCB = lastTCB->TCB_next;
        }

        // insert the new TCB behind lastTCB and in front of RunPt
        lastTCB->TCB_next = tcbPtr;
        tcbPtr->TCB_previous = lastTCB;
        tcbPtr->TCB_next = RunPt;
        RunPt->TCB_previous = tcbPtr;
    }

exit:
    EndCritical(sr);
    return rv; // replace this line with solution
}

void
OS_UpdateSleep(void) {
    int tcbIndex;

    for (tcbIndex = 0; tcbIndex < MAX_THREADS_COUNT; tcbIndex++) {
        TCBPtr tcbPtr = &tcb_list[tcbIndex];
        if (tcbPtr->sleep_state > 0) {
            tcbPtr->sleep_state--;

            // Check if just finished sleeping
            if (tcbPtr->sleep_state == 0) {
                //TODO
            }
        }
    }
}

//******** OS_AddProcess *************** 
// add a process with foregound thread to the scheduler
// Inputs: pointer to a void/void entry point
//         pointer to process text (code) segment
//         pointer to process data segment
//         number of bytes allocated for its stack
//         priority (0 is highest)
// Outputs: 1 if successful, 0 if this process can not be added
// This function will be needed for Lab 5
// In Labs 2-4, this function can be ignored
int
OS_AddProcess(void
(*entry)(void), void *text, void *data, unsigned long stackSize,
        unsigned long priority) {
    // put Lab 5 solution here

    return 0; // replace this line with Lab 5 solution
}

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
uint32_t
OS_Id(void) {
    // put Lab 2 (and beyond) solution here

    return RunPt->id;
}
;

//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In lab 1, this command will be called 1 time
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int
OS_AddPeriodicThread(void
(*task)(void), uint32_t period, uint32_t priority) {
    // put Lab 2 (and beyond) solution here
    uint32_t sr = StartCritical();
    int rv = 1;

    PeriodicTask = task;

    Timer1Init(period, priority);

    EndCritical(sr);
    return rv; // replace this line with solution
}
;

/*----------------------------------------------------------------------------
 PF1 Interrupt Handler
 *----------------------------------------------------------------------------*/
void
(*SW1Task)(void);
void
(*SW2Task)(void);
uint8_t SW1Task_Priority, SW2Task_Priority;
uint32_t lastPF0EdgeInterrupt = 0, lastPF4EdgeInterrupt = 0;
void
GPIOPortF_Handler(void) {
    uint32_t ms = OS_MsTime();
    if (GPIO_PORTF_RIS_R & (1u << 0u)) {
        // PF0
        GPIO_ClearInterruptStatus(PORT_F, 0);
        if (ms - lastPF0EdgeInterrupt > LAUNCHPAD_BUTTON_DEBOUNCE_MS) {
            lastPF0EdgeInterrupt = ms;

            if (SW2Task != NULL) {
//                OS_AddThread(SW2Task, 128, SW2Task_Priority);
                (*SW2Task)();
            }
        }
    } else if (GPIO_PORTF_RIS_R & (1u << 4u)) {
        // PF4
        GPIO_ClearInterruptStatus(PORT_F, 4);
        if (ms - lastPF4EdgeInterrupt > LAUNCHPAD_BUTTON_DEBOUNCE_MS) {
            lastPF4EdgeInterrupt = ms;

            if (SW1Task != NULL) {
//                OS_AddThread(SW1Task, 128, SW2Task_Priority);
                (*SW1Task)();
            }
        }
    }
}

//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int
OS_AddSW1Task(void
(*task)(void), uint32_t priority) {
    // put Lab 2 (and beyond) solution here
    SW1Task = task;
    SW1Task_Priority = priority;
    GPIO_EnableEdgeInterrupt(PORT_F, 4, FALLING_EDGE);
    return 0; // replace this line with solution
}

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int
OS_AddSW2Task(void
(*task)(void), uint32_t priority) {
    // put Lab 2 (and beyond) solution here
    SW2Task = task;
    SW2Task_Priority = priority;
    GPIO_EnableEdgeInterrupt(PORT_F, 0, FALLING_EDGE);
    return 0; // replace this line with solution
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void
OS_Sleep(uint32_t sleepTime) {
    // put Lab 2 (and beyond) solution here
    uint32_t sr = StartCritical();

    RunPt->sleep_state = sleepTime;

    OS_Suspend();

    EndCritical(sr);
}

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void
OS_Kill(void) {
    // put Lab 2 (and beyond) solution here
    uint32_t sr = StartCritical();

    // Remove RunPt from TCB Chain
    RunPt->TCB_previous->TCB_next = RunPt->TCB_next;
    RunPt->TCB_next->TCB_previous = RunPt->TCB_previous;

    // Deactivate RunPt's ID to indicate death
    RunPt->id = -1;

    OS_Suspend();
    EndCritical(sr);

}
;

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void
OS_Suspend(void) {
    // put Lab 2 (and beyond) solution here
    NVIC_ST_CURRENT_R = 0;          // clear timer
    NVIC_INT_CTRL_R = 0x04000000;   // trigger SysTick Handler
}
;

// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void
OS_Fifo_Init(uint32_t size) {
    // put Lab 2 (and beyond) solution here

    // reset ptr to head of fifo
    os_fifo_ptr_head = 0;
    os_fifo_ptr_tail = 0;
}
;

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int
OS_Fifo_Put(uint32_t data) {
    // put Lab 2 (and beyond) solution here
    int rv = 1;

    if (NEXT_FIFO_INDEX(os_fifo_ptr_head) == os_fifo_ptr_tail) {
        // full
        rv = 0;
        goto exit;
    }

    os_fifo[os_fifo_ptr_head] = data;
    os_fifo_ptr_head = NEXT_FIFO_INDEX(os_fifo_ptr_head);

exit:
    return rv;
}

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
uint32_t
OS_Fifo_Get(void) {
    // put Lab 2 (and beyond) solution here
    uint32_t rv;

    // empty
    while (os_fifo_ptr_tail == os_fifo_ptr_head);

    rv = os_fifo[os_fifo_ptr_tail];
    os_fifo_ptr_tail = NEXT_FIFO_INDEX(os_fifo_ptr_tail);

    return rv;
}

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
int32_t
OS_Fifo_Size(void) {
    // put Lab 2 (and beyond) solution here
    int32_t rv;

    if (os_fifo_ptr_head >= os_fifo_ptr_tail) {
        rv = os_fifo_ptr_head - os_fifo_ptr_tail;
    } else {
        rv = os_fifo_ptr_head + OS_FIFO_SIZE - os_fifo_ptr_tail;
    }

    return rv;
}

static Sema4Type Mailbox_DataValid;
static Sema4Type Mailbox_BoxFree;
static uint32_t Mailbox_Data;

// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void
OS_MailBox_Init(void) {
    // put Lab 2 (and beyond) solution here
    OS_InitSemaphore(&Mailbox_DataValid, 0);
    OS_InitSemaphore(&Mailbox_BoxFree, 1);
}
;

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void
OS_MailBox_Send(uint32_t data) {
    // put Lab 2 (and beyond) solution here
    OS_bWait(&Mailbox_BoxFree);
    Mailbox_Data = data;
    OS_bSignal(&Mailbox_DataValid);

}
;

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
uint32_t
OS_MailBox_Recv(void) {
    // put Lab 2 (and beyond) solution here
    uint32_t rv;

    OS_bWait(&Mailbox_DataValid);
    rv = Mailbox_Data;
    OS_bSignal(&Mailbox_BoxFree);

    return rv;
}
;

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
uint32_t
OS_Time(void) {
    // put Lab 2 (and beyond) solution here

    return TIMER2_TAR_R; // replace this line with solution
}
;

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
uint32_t
OS_TimeDifference(uint32_t start, uint32_t stop) {
    // put Lab 2 (and beyond) solution here
    uint32_t rv;

    if (stop > start) {
        rv = stop - start;
    } else {
        rv = start - stop;
    }
    return rv;
}
;

// ******** OS_ClearMsTime ************
// sets the system time to zero (solve for Lab 1), and start a periodic interrupt
// Inputs:  none
// Outputs: none
// You are free to change how this works
void
OS_ClearMsTime(void) {
    // put Lab 1 solution here
    osTimeMs = 0;
    Timer0Init();
}
;

// ******** OS_MsTime ************
// reads the current time in msec (solve for Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// For Labs 2 and beyond, it is ok to make the resolution to match the first call to OS_AddPeriodicThread
uint64_t
OS_MsTime(void) {
    // put Lab 1 solution here
    // osTimeMs is incremented in timers.c
    return osTimeMs;
}
;

//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void
OS_Launch(uint32_t theTimeSlice) {
    // put Lab 2 (and beyond) solution here

    // There must be at least one task to run
    if (threadCount == 0) {
        goto exit;
    }

    // TODO Set RunPt to point to highest priority

    SysTick_Init(theTimeSlice);
    StartOS();

exit:
    return;
}

void
OS_Scheduler(void) {
    // TODO Set NextRunPt to point to highest priority
    NextRunPt = RunPt->TCB_next;

    // Cannot run a sleeping thread, find a thread that is awake
    while (NextRunPt->sleep_state > 0) {
        NextRunPt = NextRunPt->TCB_next;
    }
}

//************** I/O Redirection *************** 
// redirect terminal I/O to UART or file (Lab 4)

int StreamToDevice = 0;                // 0=UART, 1=stream to file (Lab 4)

int
fputc(int ch, FILE *f) {
    if (StreamToDevice == 1) {  // Lab 4
        if (eFile_Write(ch)) {          // close file on error
            OS_EndRedirectToFile(); // cannot write to file
            return 1;                  // failure
        }
        return 0; // success writing
    }

    // default UART output
    UART_OutChar(ch);
    return ch;
}

int
fgetc(FILE *f) {
    char ch = UART_InChar();  // receive from keyboard
    UART_OutChar(ch);         // echo
    return ch;
}

int
OS_RedirectToFile(const char *name) {  // Lab 4
    eFile_Create(name);              // ignore error if file already exists
    if (eFile_WOpen(name)) return 1;  // cannot open file
    StreamToDevice = 1;
    return 0;
}

int
OS_EndRedirectToFile(void) {  // Lab 4
    StreamToDevice = 0;
    if (eFile_WClose()) return 1;    // cannot close file
    return 0;
}

int
OS_RedirectToUART(void) {
    StreamToDevice = 0;
    return 0;
}

int
OS_RedirectToST7735(void) {

    return 1;
}

