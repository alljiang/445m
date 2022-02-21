
	.text
	.align 2
	.thumb

    .def  DisableInterrupts
    .def EnableInterrupts
    .def StartCritical
    .def EndCritical
    .def WaitForInterrupt

DisableInterrupts
	CPSID I
	BX LR
	.endasmfunc

EnableInterrupts
	CPSIE I
	BX LR
	.endasmfunc

StartCritical
	MRS R0, PRIMASK
	CPSID I
	BX LR
	.endasmfunc

EndCritical
	MSR PRIMASK, R0
	BX LR

WaitForInterrupt
	WFI
	BX LR
