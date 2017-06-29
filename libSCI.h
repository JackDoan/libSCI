#ifndef LIBSCI_H_
#define LIBSCI_H_

#define LIBSCI_BUFFER_LEN 50

//enums:
typedef enum libSCI_baudrate_enum { LIBSCI_115200, LIBSCI_9600 } libSCI_baudrate_t;
//end enums

//generic typedefs:
typedef volatile struct SCI_REGS * sciPort_t;
//end typedefs

//structs:


typedef struct libSCI_handle_struct { //!< one of these exists per CPU
	sciState_t A;
	sciState_t B;
	sciState_t C;
	sciState_t D;
} libSCI_handle_t;
//end structs








//Functions:
extern __interrupt void SCI_TX_Handler(void);
//end functions

#endif