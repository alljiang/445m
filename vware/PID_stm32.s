;/******************** (C) COPYRIGHT 2009  STMicroelectronics ********************
;* File Name          : PID_stm32.s
;* Author             : MCD Application Team
;* Version            : V2.0.0
;* Date               : 04/27/2009
;* Description        : This source file contains assembly optimized source code
;*                      of a PID controller.
;********************************************************************************
;* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
;* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
;* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
;* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
;* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
;* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
;*******************************************************************************/

	.thumb				;  THUMB
						;  REQUIRE8
						;  PRESERVE8

	.align 2
	.global PID_stm32	;  EXPORT PID_stm32
	.def IntTerm		;  IMPORT IntTerm
	.def PrevError	;  IMPORT PrevError
IntTerm:
	.field 16
PrevError:
	.field 16
	.align 2

	.text				;  AREA |.text|, CODE, READONLY, ALIGN=2


;Err     RN R0    				; 1st function input: Error
;Coeff   RN R1    				; 2nd fct input: Address of coefficient table
;Kd      RN R1
;Ki      RN R2
;Kp      RN R3

;Out     RN R4
;Result  RN R2
;Integ   RN R5
;PrevErr RN R12

;/*******************************************************************************
;* Function Name  : DoPID
;* Description    : PID in ASM, Error computed outside the routine
;* Input          : Error: difference between reference and measured value
;*                  Coeff: pointer to the coefficient table
;* Output         : None
;* Return         : PID output (command)
;*******************************************************************************/
PID_stm32:
	PUSH {R4, R5, R9} 		;  PUSH {R4, R5, R9}
	LDR R12, IntTerm 		;  LDR R12, =IntTerm
	LDR R9, PrevError		;  LDR R9, =PrevError
	B IntTerm

	LDRH R3, [R1, #0]  		;  LDRH Kp, [Coeff, #0]  		; Load Kp
	LDRH R2, [R1, #2]  		;  LDRH Ki, [Coeff, #2]  		; Load Ki
	LDRH R1, [R1, #4]  		;  LDRH Kd, [Coeff, #4]  		; Load Kd and destroy Coeff
	LDRH R5, [R12, #0] 		;  LDRH Integ, [R12, #0]  		; Last Integral Term
	LDRH R12, [R9, #0]		;  LDRH PrevErr, [R9, #0]  		; Previous Error
	
	MLA R5, R2, R0, R5   	;  MLA Integ, Ki, Err, Integ   	; IntTerm += Ki*error
	MLA R4, R3, R0, R5      ;  MLA Out, Kp, Err, Integ      	; Output = (Kp * error) + InTerm
	SUBS R12, R0, R12   	;  SUBS PrevErr, Err, PrevErr    ; PrevErr now holds DeltaError = Error - PrevError
	MLA R2, R1, R12, R4 	;  MLA Result, Kd, PrevErr, Out  ; Output += Kd * DeltaError
	
	LDR R12, IntTerm		;  LDR R12, =IntTerm
	STRH R5, [R12, #0]		;  STRH Integ, [R12, #0]       	; Write back InTerm
	STRH R0, [R9, #0]   	;  STRH Err, [R9, #0]         	; Write back PrevError
	
	MOV R0, R2				;  MOV R0, Result
	UXTH R0, R0				;  UXTH R0, R0
	POP {R4, R5, R9}		;  POP {R4, R5, R9}
	BX LR					;  BX LR
	
	.end					;  END

;/******************* (C) COPYRIGHT 2009  STMicroelectronics *****END OF FILE****/
