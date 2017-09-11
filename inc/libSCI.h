#ifndef LIBSCI_H_
#define LIBSCI_H_

#define LIBSCI_BUFFER_LEN 200
#include "inc/ioBuffer.h"

//enums:
typedef enum libSCI_baudrate_enum { LIBSCI_230400, LIBSCI_115200, LIBSCI_9600 } libSCI_baudrate_t;
typedef enum libSCI_port_enum { A, B, C, D } libSCI_port_t;
//end enums

//generic typedefs:
typedef volatile struct SCI_REGS * sciPort_t;
//end typedefs

//structs:

typedef struct sciStateMachine_struct { //!< and one of these per port
    sciPort_t port;
    int enabled;
    iobuf_handle iobuf;
} sciState_t;

typedef struct libSCI_handle_struct { //!< one of these exists per CPU
	sciState_t A;
	sciState_t B;
	sciState_t C;
	sciState_t D;
} libSCI_handle_t;
//end structs

//Functions:
//extern __interrupt void SCI_TX_Handler(void);
extern int sciRead(sciState_t*);
extern libSCI_handle_t libSCI_init(libSCI_port_t libport, libSCI_baudrate_t baudrate);
extern int sciQueueMessage(sciState_t* portHandle, char* msg, unsigned int length);
extern int sciAvailable(sciState_t* portHandle);
extern int sciRead(sciState_t* p);
//end functions

//globals:
extern libSCI_handle_t libSCI_handle;
//end globals

#endif
