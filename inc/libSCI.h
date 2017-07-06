#ifndef LIBSCI_H_
#define LIBSCI_H_

#define LIBSCI_BUFFER_LEN 50

//enums:
typedef enum libSCI_baudrate_enum { LIBSCI_115200, LIBSCI_9600 } libSCI_baudrate_t;
typedef enum libSCI_port_enum { A, B, C, D } libSCI_port_t;
//end enums

//generic typedefs:
typedef volatile struct SCI_REGS * sciPort_t;
//end typedefs

//structs:

typedef struct libsci_buffer_struct { //!< two of these per SCI state machine
    char buffer[LIBSCI_BUFFER_LEN];
    int length; //!< length of message
    int progress; //!< how many bytes have already been sent?
    int inUse; //!< are we done? 0=ready to accept new message. 1=ready to tx. 2=being txed
} libSCI_buffer_t;

typedef struct sciStateMachine_struct { //!< and one of these per port
    sciPort_t port;
    int enabled;
    int transmitting; //!< which buffer is transmitting? zero for idle
    libSCI_buffer_t first;
    libSCI_buffer_t second;
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
