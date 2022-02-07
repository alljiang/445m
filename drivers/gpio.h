#ifndef GPIO_H_
#define GPIO_H_

enum Port {
    PORT_A, PORT_B, PORT_C, PORT_D, PORT_E, PORT_F
};

enum EdgeEvent {
    FALLING_EDGE, RISING_EDGE, BOTH_EDGES
};

extern void
GPIO_PortAHandler(void);
extern void
GPIO_PortBHandler(void);
extern void
GPIO_PortCHandler(void);
extern void
GPIO_PortDHandler(void);
extern void
GPIO_PortEHandler(void);
extern void
GPIO_PortFHandler(void);

extern void
GPIO_Initialize(void);

extern void
GPIO_PortFInitialize(void);

extern void
GPIO_EnableEdgeInterrupt(enum Port port, uint8_t pinNum,
        enum EdgeEvent risingEdge);

extern void
GPIO_ClearInterruptStatus(enum Port port, uint8_t pinNum);

#endif /* GPIO_H_ */
