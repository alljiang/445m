;/*****************************************************************************/
;/* OSasm.s: low-level OS commands, written in assembly                       */
;/* derived from uCOS-II                                                      */
;/*****************************************************************************/
;Jonathan Valvano, OS Lab2/3/4/5, 1/12/20
;Students will implement these functions as part of EE445M/EE380L.12 Lab

        .text				;AREA |.text|, CODE, READONLY, ALIGN=2
        .align 2
        .thumb				;THUMB
        					;REQUIRE8
        					;PRESERVE8

        .ref RunPt			;EXTERN  RunPt            ; currently running thread
		.align 4
RunPt_Pt
		.field RunPt, 32
        .align 2

        .def StartOS		;EXPORT  StartOS
        .def ContextSwitch	;EXPORT  ContextSwitch
        .def PendSV_Handler	;EXPORT  PendSV_Handler
        .def SVC_Handler	;EXPORT  SVC_Handler

;
; https://users.ece.utexas.edu/~valvano/arm/ConvertKeilCCS.pdf
;
; .equ directly loads to symbol table
; .field puts data
;
; However, using LDR with symbol is not possible with TI Compiler.
; So, LDRs must be done by loading from a label after defining the data with .field.
;
; TLDR use .field for LDRs, and .equ for other stuff
;

;NVIC_INT_CTRL   EQU     0xE000ED04                              ; Interrupt control state register.
NVIC_INT_CTRL	.field	0xE000ED04, 32

;NVIC_SYSPRI14   EQU     0xE000ED22                              ; PendSV priority register (position 14).
NVIC_SYSPRI14	.field	0xE000ED22, 32

;NVIC_SYSPRI15   EQU     0xE000ED23                              ; Systick priority register (position 15).
NVIC_SYSPRI15	.field	0xE000ED23, 32

;NVIC_LEVEL14    EQU           0xEF                              ; Systick priority value (second lowest).
NVIC_LEVEL14	.equ	0xEF

;NVIC_LEVEL15    EQU           0xFF                              ; PendSV priority value (lowest).
NVIC_LEVEL15	.equ	0xFF

;NVIC_PENDSVSET  EQU     0x10000000                              ; Value to trigger PendSV exception.
NVIC_PENDSVSET	.equ	0x10000000



StartOS
; put your code here
    ; Load SP with RunPt.SP
    LDR R0, RunPt_Pt			; R0 <- Pointer to RunPt
    LDR R0, [R0]				; R0 <- RunPt
    LDR SP, [R0, #0]			; SP <- RunPt.stack_pointer
    
    ; Restore registers
    POP {R4-R11}
    POP {R0-R3}
    POP {R12}

    ADD SP, SP, #4				; discard LR
    POP {LR}					; LR <- PC
    ADD SP, SP, #4				; discard PSR

    CPSIE I						; enable interrupts

    BX      LR                 ; start first thread
	.endasmfunc

OSStartHang
    B       OSStartHang        ; Should never get here
	.endasmfunc


;********************************************************************************************************
;                               PERFORM A CONTEXT SWITCH (From task level)
;                                           void ContextSwitch(void)
;
; Note(s) : 1) ContextSwitch() is called when OS wants to perform a task context switch.  This function
;              triggers the PendSV exception which is where the real work is done.
;********************************************************************************************************

ContextSwitch
; edit this code
    
    BX      LR
	.endasmfunc
    

;********************************************************************************************************
;                                         HANDLE PendSV EXCEPTION
;                                     void OS_CPU_PendSVHandler(void)
;
; Note(s) : 1) PendSV is used to cause a context switch.  This is a recommended method for performing
;              context switches with Cortex-M.  This is because the Cortex-M3 auto-saves half of the
;              processor context on any exception, and restores same on return from exception.  So only
;              saving of R4-R11 is required and fixing up the stack pointers.  Using the PendSV exception
;              this way means that context saving and restoring is identical whether it is initiated from
;              a thread or occurs due to an interrupt or exception.
;
;           2) Pseudo-code is:
;              a) Get the process SP, if 0 then skip (goto d) the saving part (first context switch);
;              b) Save remaining regs r4-r11 on process stack;
;              c) Save the process SP in its TCB, OSTCBCur->OSTCBStkPtr = SP;
;              d) Call OSTaskSwHook();
;              e) Get current high priority, OSPrioCur = OSPrioHighRdy;
;              f) Get current ready thread TCB, OSTCBCur = OSTCBHighRdy;
;              g) Get new process SP from TCB, SP = OSTCBHighRdy->OSTCBStkPtr;
;              h) Restore R4-R11 from new process stack;
;              i) Perform exception return which will restore remaining context.
;
;           3) On entry into PendSV handler:
;              a) The following have been saved on the process stack (by processor):
;                 xPSR, PC, LR, R12, R0-R3
;              b) Processor mode is switched to Handler mode (from Thread mode)
;              c) Stack is Main stack (switched from Process stack)
;              d) OSTCBCur      points to the OS_TCB of the task to suspend
;                 OSTCBHighRdy  points to the OS_TCB of the task to resume
;
;           4) Since PendSV is set to lowest priority in the system (by OSStartHighRdy() above), we
;              know that it will only be run when no other exception or interrupt is active, and
;              therefore safe to assume that context being switched out was using the process stack (PSP).
;********************************************************************************************************

PendSV_Handler
; put your code here

    
    
    BX      LR                 ; Exception return will restore remaining context   
	.endasmfunc
    

;********************************************************************************************************
;                                         HANDLE SVC EXCEPTION
;                                     void OS_CPU_SVCHandler(void)
;
; Note(s) : SVC is a software-triggered exception to make OS kernel calls from user land. 
;           The function ID to call is encoded in the instruction itself, the location of which can be
;           found relative to the return address saved on the stack on exception entry.
;           Function-call paramters in R0..R3 are also auto-saved on stack on exception entry.
;********************************************************************************************************

        .ref OS_Id			;IMPORT    OS_Id
        .ref OS_Kill		;IMPORT    OS_Kill
       	.ref OS_Sleep		;IMPORT    OS_Sleep
        .ref OS_Time		;IMPORT    OS_Time
        .ref OS_AddThread	;IMPORT    OS_AddThread

SVC_Handler
; put your Lab 5 code here


    BX      LR                   ; Return from exception
	.endasmfunc


    .align 2	;ALIGN
    .end		;END
