;******************** (C) COPYRIGHT 2009  STMicroelectronics ********************
;* File Name          : cr4_fft_64_stm32.s
;* Author             : MCD Application Team
;* Version            : V2.0.0
;* Date               : 04/27/2009
;* Description        : Optimized 64-point radix-4 complex FFT for Cortex-M3
;********************************************************************************
;* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
;* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
;* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
;* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
;* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
;* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
;*******************************************************************************/

.thumb        	;	THUMB
        		;  	REQUIRE8
        		;   PRESERVE8
        
	.align 2
	.text	        ;	AREA |.text|, CODE, READONLY, ALIGN=2
        
	.global cr4_fft_64_stm32        ;	EXPORT cr4_fft_64_stm32
	;		        ;	EXTERN TableFFT
        
        
        ;pssK      	RN R0
        ;pssOUT    	RN R0
        ;pssX      	RN R1
        ;pssIN     	RN R1
        ;butternbr 	RN R2
        ;Nbin      	RN R2
        ;index     	RN R3
        ;Ar        	RN R4
        ;Ai        	RN R5
        ;Br        	RN R6
        ;Bi        	RN R7
        ;Cr        	RN R8
        ;Ci        	RN R9
        ;Dr        	RN R10
        ;Di        	RN R11
        ;cntrbitrev 	RN R12
        ;tmp       	RN R12
        ;pssIN2    	RN R14
        ;tmp2      	RN R14
        
        ;NPT EQU 64
        
        ;----------------------------- MACROS ----------------------------------------
        
DEC		.macro REG        	;	 	MACRO
        					;	 	DEC  $reg
        SUB REG, REG, #1	;     	SUB  $reg,$reg,#1
        .endm				;     	MEND
        
INC		.macro REG        	;	 	MACRO
        					;     	INC  $reg
        ADD REG, REG, #1	;     	ADD  $reg,$reg,#1
        .endm				;     	MEND
        
        
QUAD	.macro REG        	; 	 	MACRO
        					;	 	QUAD  $reg
        MOV REG, REG, LSL#2	;     	MOV  $reg,$reg,LSL#2
        .endm				;     	MEND
        
        ;sXi = *(PssX+1); sXr = *PssX; PssX += offset; PssX= R1
LDR2Q	.macro sXr, sXi, PssX, offset        	;	  	MACRO
        										;	  	LDR2Q  $sXr,$sXi, $PssX, $offset
        LDRSH sXi, [PssX, #2]					;      	LDRSH $sXi, [$PssX, #2]
        LDRSH sXr, [PssX]						;      	LDRSH $sXr, [$PssX]
        ADD PssX, PssX, offset					;      	ADD $PssX, $PssX, $offset
        .endm									;      	MEND
        
        ;!! Same macro, to be used when passing negative offset value !!
LDR2Qm	.macro sXr, sXi, PssX, offset        	;	  	MACRO
        										;	  	LDR2Qm  $sXr,$sXi, $PssX, $offset
        LDRSH sXi, [PssX, #2]					;      	LDRSH $sXi, [$PssX, #2]
        LDRSH sXr, [PssX]						;      	LDRSH $sXr, [$PssX]
        SUB PssX, PssX, offset					;      	SUB $PssX, $PssX, $offset
        .endm									;      	MEND

        ;(PssX+1)= sXi;  *PssX=sXr; PssX += offset;
STR2Q	.macro sXr, sXi, PssX, offset	        ;		MACRO
        										;		STR2Q  $sXr, $sXi, $PssX, $offset
		STRH  sXi, [PssX, #2]					;        STRH  $sXi, [$PssX, #2]
        STRH  sXr, [PssX]						;        STRH  $sXr, [$PssX]
        ADD PssX, PssX, offset				;        ADD $PssX, $PssX, $offset
        .endm									;        MEND
        
        ; YY = Cplx_conjugate_mul(Y,K)
        ;  Y = YYr + i*YYi
        ; use the following trick
        ;  K = (Kr-Ki) + i*Ki
CXMUL_V7 .macro YYr, YYi, Yr, Yi, Kr, Ki, tmp, tmp2        	;		MACRO
        													;		CXMUL_V7  $YYr, $YYi, $Yr, $Yi, $Kr, $Ki,$tmp,$tmp2
        SUB  tmp2, Yi, Yr         							;       SUB  $tmp2, $Yi, $Yr         ; sYi-sYr
        MUL  tmp, tmp2, Ki        							;       MUL  $tmp, $tmp2, $Ki        ; (sYi-sYr)*sKi
        ADD  tmp2, Kr, Ki, LSL#1 							;       ADD  $tmp2, $Kr, $Ki, LSL#1  ; (sKr+sKi)
        MLA  YYi, Yi, Kr, tmp               				;       MLA  $YYi, $Yi, $Kr, $tmp     ; lYYi = sYi*sKr-sYr*sKi
        MLA  YYr, Yr, tmp2, tmp             				;       MLA  $YYr, $Yr, $tmp2, $tmp   ; lYYr = sYr*sKr+sYi*sKi
        .endm												;        MEND
        
        ; Four point complex Fast Fourier Transform		
CXADDA4	.macro s        						;		MACRO
        										;		CXADDA4  $s
        ; (C,D) = (C+D, C-D)
        ADD     R8, R8, R10              		;        ADD     Cr, Cr, Dr
        ADD     R9, R9, R11              		;        ADD     Ci, Ci, Di
        SUB     R10, R8, R10, LSL#1               ;        SUB     Dr, Cr, Dr, LSL#1
        SUB     R11, R9, R11, LSL#1               ;        SUB     Di, Ci, Di, LSL#1
        ; (A,B) = (A+(B>>s), A-(B>>s))/4
        MOV     R4, R4, ASR#2           		;        MOV     Ar, Ar, ASR#2
        MOV     R5, R5, ASR#2           		;        MOV     Ai, Ai, ASR#2
        ADD     R4, R4, R6, ASR#(2+s)          	;        ADD     Ar, Ar, Br, ASR#(2+$s)
        ADD     R5, R5, R7, ASR#(2+s)          	;        ADD     Ai, Ai, Bi, ASR#(2+$s)
        SUB     R6, R4, R6, ASR#(1+s)          	;        SUB     Br, Ar, Br, ASR#(1+$s)
        SUB     R7, R5, R7, ASR#(1+s)          	;        SUB     Bi, Ai, Bi, ASR#(1+$s)
        ; (A,C) = (A+(C>>s)/4, A-(C>>s)/4)
        ADD     R4, R4, R8, ASR#(2+s)          	;        ADD     Ar, Ar, Cr, ASR#(2+$s)
        ADD     R5, R5, R9, ASR#(2+s)          	;        ADD     Ai, Ai, R9, ASR#(2+$s)
        SUB     R8, R4, R8, ASR#(1+s)          	;        SUB     Cr, Ar, Cr, ASR#(1+$s)
        SUB     R9, R5, R9, ASR#(1+s)          	;        SUB     Ci, Ai, Ci, ASR#(1+$s)
        ; (B,D) = (B-i*(D>>s)/4, B+i*(D>>s)/4)
        ADD     R6, R6, R11, ASR#(2+s)          	;        ADD     Br, Br, Di, ASR#(2+$s)
        SUB     R7, R7, R10, ASR#(2+s)          	;        SUB     Bi, Bi, Dr, ASR#(2+$s)
        SUB     R11, R6, R11, ASR#(1+s)          	;        SUB     Di, Br, Di, ASR#(1+$s)
        ADD     R10, R7, R10, ASR#(1+s)          	;        ADD     Dr, Bi, Dr, ASR#(1+$s)
        .endm									;        MEND
        
        
BUTFLY4ZERO_OPT  .macro pIN, offset, pOUT       		;		MACRO
        												;		BUTFLY4ZERO_OPT  $pIN,$offset, $pOUT
        LDRSH R5, [pIN, #2]                    			;        LDRSH Ai, [$pIN, #2]
        LDRSH R4, [pIN],#64                   			;        LDRSH Ar, [$pIN],#NPT
        LDRSH R9, [pIN, #2]                    			;        LDRSH Ci, [$pIN, #2]
        LDRSH R8, [pIN],#64                   			;        LDRSH Cr, [$pIN],#NPT
        LDRSH R7, [pIN, #2]                    			;        LDRSH Bi, [$pIN, #2]
        LDRSH R6, [pIN],#64                   			;        LDRSH Br, [$pIN],#NPT
        LDRSH R11, [pIN, #2]                    			;        LDRSH Di, [$pIN, #2]
        LDRSH R10, [pIN],#64                   			;        LDRSH Dr, [$pIN],#NPT
		; (C,D) = (C+D, C-D)
        ADD     R8, R8, R10                      		;        ADD     Cr, Cr, Dr
        ADD     R9, R9, R11                      		;        ADD     Ci, Ci, Di
        SUB     R10, R8, R10, LSL#1                    	;        SUB     Dr, Cr, Dr, LSL#1  ; trick
        SUB     R11, R9, R11, LSL#1                    	;        SUB     Di, Ci, Di, LSL#1  ;trick
        ; (A,B) = (A+B)/4, (A-B)/4
        MOV     R4, R4, ASR#2                   		;        MOV     Ar, Ar, ASR#2
        MOV     R5, R5, ASR#2                   		;        MOV     Ai, Ai, ASR#2
        ADD     R4, R4, R6, ASR#2                       ;        ADD     Ar, Ar, Br, ASR#2
        ADD     R5, R5, R7, ASR#2                       ;        ADD     Ai, Ai, Bi, ASR#2
        SUB     R6, R4, R6, ASR#1                       ;        SUB     Br, Ar, Br, ASR#1
        SUB     R7, R5, R7, ASR#1                       ;        SUB     Bi, Ai, Bi, ASR#1
        ; (A,C) = (A+C)/4, (A-C)/4
        ADD     R4, R4, R8, ASR#2                       ;        ADD     Ar, Ar, Cr, ASR#2
        ADD     R5, R5, R9, ASR#2                       ;        ADD     Ai, Ai, Ci, ASR#2
        SUB     R8, R4, R8, ASR#1                       ;        SUB     Cr, Ar, Cr, ASR#1
        SUB     R9, R5, R9, ASR#1                       ;        SUB     Ci, Ai, Ci, ASR#1
        ; (B,D) = (B-i*D)/4, (B+i*D)/4
        ADD     R6, R6, R11, ASR#2                       ;        ADD     Br, Br, Di, ASR#2
        SUB     R7, R7, R10, ASR#2                       ;        SUB     Bi, Bi, Dr, ASR#2
        SUB     R11, R6, R11, ASR#1                       ;        SUB     Di, Br, Di, ASR#1
        ADD     R10, R7, R10, ASR#1                       ;        ADD     Dr, Bi, Dr, ASR#1

        STRH    R5, [pOUT, #2]                 			;        STRH    Ai, [$pOUT, #2]
        STRH    R4, [pOUT], #4                 			;        STRH    Ar, [$pOUT], #4
        STRH    R7, [pOUT, #2]                 			;        STRH    Bi, [$pOUT, #2]
        STRH    R6, [pOUT], #4                 			;        STRH    Br, [$pOUT], #4
        STRH    R9, [pOUT, #2]                 			;        STRH    Ci, [$pOUT, #2]
        STRH    R8, [pOUT], #4                 			;        STRH    Cr, [$pOUT], #4
        STRH    R10, [pOUT, #2]                  		;        STRH    Dr, [$pOUT, #2]  ; inversion here
        STRH    R11, [pOUT], #4                 			;        STRH    Di, [$pOUT], #4
        .endm                       					;        MEND
        
BUTFLY4_V7 .macro pssDin, offset, pssDout, qformat, pssK    ;		MACRO
        												;		BUTFLY4_V7   $pssDin,$offset,$pssDout,$qformat,$pssK
        LDR2Qm   R4,R5, pssDin, offset			        ;        LDR2Qm   Ar,Ai,$pssDin, $offset;-$offset
        LDR2Q    R10,R11, pssK, #4                        ;        LDR2Q    Dr,Di,$pssK, #4
        ; format CXMUL_V7 YYr, YYi, Yr, Yi, Kr, Ki,tmp,tmp2
        CXMUL_V7 R10,R11,R4,R5,R10,R11,tmp,tmp2             ;        CXMUL_V7 Dr,Di,Ar,Ai,Dr,Di,tmp,tmp2
        LDR2Qm   R4,R5, pssDin, offset			        ;        LDR2Qm   Ar,Ai,$pssDin,$offset;-$offset
        LDR2Q    R8,R9,pssK,#4	                 		;        LDR2Q    Cr,Ci,$pssK,#4
        CXMUL_V7 R8,R9,R4,R5,R8,R9,tmp,tmp2             ;        CXMUL_V7 Cr,Ci,Ar,Ai,Cr,Ci,tmp,tmp2
        LDR2Qm    R4,R5, pssDin, offset;				;        LDR2Qm    Ar,Ai, $pssDin, $offset;-$offset
        LDR2Q    R6,R7, pssK, #4                        ;        LDR2Q    Br,Bi, $pssK, #4
        CXMUL_V7  R6,R7,R4,R5,R6,R7,tmp,tmp2            ;        CXMUL_V7  Br,Bi,Ar,Ai,Br,Bi,tmp,tmp2
        LDR2Q    R4,R5, pssDin, #0                      ;        LDR2Q    Ar,Ai, $pssDin, #0
        CXADDA4  qformat                        		;        CXADDA4  $qformat
        STRH    R5, [pssDout, #2]                       ;        STRH    Ai, [$pssDout, #2]
        STRH    R4, [pssDout]                   		;        STRH    Ar, [$pssDout]
        ADD 	pssDout, pssDout, offset                ;        ADD 	$pssDout, $pssDout, $offset
        STRH    R7, [pssDout, #2]                       ;        STRH    Bi, [$pssDout, #2]
        STRH    R6, [pssDout]                   		;        STRH    Br, [$pssDout]
        ADD     pssDout, pssDout, offset                ;        ADD     $pssDout, $pssDout, $offset
        STRH    R9, [pssDout, #2]                       ;        STRH    Ci, [$pssDout, #2]
        STRH    R8, [pssDout]                   		;        STRH    Cr, [$pssDout]
        ADD     pssDout, pssDout, offset                ;        ADD     $pssDout, $pssDout, $offset
        STRH    R10, [pssDout, #2]                     	;        STRH    Dr, [$pssDout, #2]  ; inversion here
        STRH    R11, [pssDout], #4                       ;        STRH    Di, [$pssDout], #4
        .endm											;        MEND
        
        ;------------------- 			CODE 			--------------------------------
        ;===============================================================================
        ;*******************************************************************************
        ;* Function Name  : cr4_fft_64_stm32
        ;* Description    : complex radix-4 64 points FFT
        ;* Input          : - R0 = pssOUT: Output array .
        ;*                  - R1 = pssIN: Input array 
        ;*                  - R2 = Nbin: =64 number of points, this optimized FFT function  
        ;*                    can only convert 64 points.
        ;* Output         : None 
        ;* Return         : None
        ;*******************************************************************************
cr4_fft_64_stm32:						;cr4_fft_64_stm32
        
        STMFD   SP!, {R4-R11, LR}		;        STMFD   SP!, {R4-R11, LR}
                
        MOV R12, #0						;        MOV cntrbitrev, #0
        MOV R3,#0						;        MOV index,#0
                
preloop_v7:						        ;preloop_v7
        ADD     R14, R1, R12, LSR#26	;        ADD     pssIN2, pssIN, cntrbitrev, LSR#26 ;64-pts
        BUTFLY4ZERO_OPT R14,R2,R0		;        BUTFLY4ZERO_OPT pssIN2,Nbin,pssOUT
        INC R3							;        INC index
        RBIT R12,R3						;        RBIT cntrbitrev,index
        CMP R3, #16						;        CMP index,#16  ;64-pts
        BNE  preloop_v7					;        BNE  preloop_v7
        
        SUB     R1, R0, R2, LSL#2		;        SUB     pssX, pssOUT, Nbin, LSL#2
        MOV     R3, #16					;        MOV     index, #16
        MOVS    R2, R2, LSR#4			;        MOVS    butternbr, Nbin, LSR#4   ;dual use of register
                
        ;------------------------------------------------------------------------------
        ;   The FFT coefficients table can be stored into Flash or RAM. 
        ;   The following two lines of code allow selecting the method for coefficients 
        ;   storage. 
        ;   In the case of choosing coefficients in RAM, you have to:
        ;   1. Include the file table_fft.h, which is a part of the DSP library, 
        ;      in your main file.
        ;   2. Decomment the line LDR.W pssK, =TableFFT and comment the line 
        ;      ADRL    pssK, TableFFT_V7
        ;   3. Comment all the TableFFT_V7 data.
        ;------------------------------------------------------------------------------
        ;        ADRL    pssK, TableFFT_V7    ; Coeff in Flash 
        ;        ;LDR.W pssK, =TableFFT      ; Coeff in RAM 
        
        ;................................
        ;passloop_v7
        ;        STMFD   SP!, {pssX,R2}
        ;        ADD     tmp, index, index, LSL#1
        ;        ADD     pssX, pssX, tmp
        ;        SUB     R2, R2, #1<<16
        ;................
        ;grouploop_v7
        ;        ADD     R2,R2,index,LSL#(16-2)
        ;.......
        ;butterloop_v7
        ;        BUTFLY4_V7  pssX,index,pssX,14,pssK
        ;        SUBS        R2,R2, #1<<16
        ;        BGE     butterloop_v7
        ;.......
        ;        ADD     tmp, index, index, LSL#1
        ;        ADD     pssX, pssX, tmp
        ;        DEC     R2
        ;        MOVS    tmp2, R2, LSL#16
        ;        IT      NE
        ;        SUBNE   pssK, pssK, tmp
        ;        BNE     grouploop_v7
        ;................
        ;        LDMFD   sp!, {pssX, R2}
        ;        QUAD    index
        ;        MOVS    R2, R2, LSR#2    ; loop nbr /= radix
        ;        BNE     passloop_v7
        ;................................
        ;       LDMFD   SP!, {R4-R11, PC}
        
        ;=============================================================================
        
TableFFT_V7:                        				        ; TableFFT_V7
        ; N=16
        .short 0x4000,0x0000, 0x4000,0x0000, 0x4000,0x0000                 ;        DCW 0x4000,0x0000, 0x4000,0x0000, 0x4000,0x0000
        .short 0xdd5d,0x3b21, 0x22a3,0x187e, 0x0000,0x2d41                 ;        DCW 0xdd5d,0x3b21, 0x22a3,0x187e, 0x0000,0x2d41
        .short 0xa57e,0x2d41, 0x0000,0x2d41, 0xc000,0x4000                 ;        DCW 0xa57e,0x2d41, 0x0000,0x2d41, 0xc000,0x4000
        .short 0xdd5d,0xe782, 0xdd5d,0x3b21, 0xa57e,0x2d41                 ;        DCW 0xdd5d,0xe782, 0xdd5d,0x3b21, 0xa57e,0x2d41
        
        ;N=64
        .short 0x4000,0x0000, 0x4000,0x0000, 0x4000,0x0000                 ;        DCW 0x4000,0x0000, 0x4000,0x0000, 0x4000,0x0000
        .short 0x2aaa,0x1294, 0x396b,0x0646, 0x3249,0x0c7c                 ;        DCW 0x2aaa,0x1294, 0x396b,0x0646, 0x3249,0x0c7c
        .short 0x11a8,0x238e, 0x3249,0x0c7c, 0x22a3,0x187e                 ;        DCW 0x11a8,0x238e, 0x3249,0x0c7c, 0x22a3,0x187e
        .short 0xf721,0x3179, 0x2aaa,0x1294, 0x11a8,0x238e                 ;        DCW 0xf721,0x3179, 0x2aaa,0x1294, 0x11a8,0x238e
        .short 0xdd5d,0x3b21, 0x22a3,0x187e, 0x0000,0x2d41                 ;        DCW 0xdd5d,0x3b21, 0x22a3,0x187e, 0x0000,0x2d41
        .short 0xc695,0x3fb1, 0x1a46,0x1e2b, 0xee58,0x3537                 ;        DCW 0xc695,0x3fb1, 0x1a46,0x1e2b, 0xee58,0x3537
        .short 0xb4be,0x3ec5, 0x11a8,0x238e, 0xdd5d,0x3b21                 ;        DCW 0xb4be,0x3ec5, 0x11a8,0x238e, 0xdd5d,0x3b21
        .short 0xa963,0x3871, 0x08df,0x289a, 0xcdb7,0x3ec5                 ;        DCW 0xa963,0x3871, 0x08df,0x289a, 0xcdb7,0x3ec5
        .short 0xa57e,0x2d41, 0x0000,0x2d41, 0xc000,0x4000                 ;        DCW 0xa57e,0x2d41, 0x0000,0x2d41, 0xc000,0x4000
        .short 0xa963,0x1e2b, 0xf721,0x3179, 0xb4be,0x3ec5                 ;        DCW 0xa963,0x1e2b, 0xf721,0x3179, 0xb4be,0x3ec5
        .short 0xb4be,0x0c7c, 0xee58,0x3537, 0xac61,0x3b21                 ;        DCW 0xb4be,0x0c7c, 0xee58,0x3537, 0xac61,0x3b21
        .short 0xc695,0xf9ba, 0xe5ba,0x3871, 0xa73b,0x3537                 ;        DCW 0xc695,0xf9ba, 0xe5ba,0x3871, 0xa73b,0x3537
        .short 0xdd5d,0xe782, 0xdd5d,0x3b21, 0xa57e,0x2d41                 ;        DCW 0xdd5d,0xe782, 0xdd5d,0x3b21, 0xa57e,0x2d41
        .short 0xf721,0xd766, 0xd556,0x3d3f, 0xa73b,0x238e                 ;        DCW 0xf721,0xd766, 0xd556,0x3d3f, 0xa73b,0x238e
        .short 0x11a8,0xcac9, 0xcdb7,0x3ec5, 0xac61,0x187e                 ;        DCW 0x11a8,0xcac9, 0xcdb7,0x3ec5, 0xac61,0x187e
        .short 0x2aaa,0xc2c1, 0xc695,0x3fb1, 0xb4be,0x0c7c                 ;        DCW 0x2aaa,0xc2c1, 0xc695,0x3fb1, 0xb4be,0x0c7c
                                        
        .end															   ;       END

        ;******************* (C) COPYRIGHT 2009  STMicroelectronics *****END OF FILE****
