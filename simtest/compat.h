

typedef struct Sema4 {
} Sema4Type;

void OS_Signal(Sema4Type *semaPt);

void OS_Wait(Sema4Type *semaPt);

void OS_InitSemaphore(Sema4Type *semaPt, int value);