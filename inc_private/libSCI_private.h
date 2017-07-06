#ifndef LIBSCI_priv_H_
#define LIBSCI_priv_H_

#define CPU2
#define CPU_RATE   5.00L   // for a 200MHz CPU clock speed (SYSCLKOUT)
#define SCIA_TX_INT_ACK PieCtrlRegs.PIEACK.bit.ACK9
#define SCIB_TX_INT_ACK PieCtrlRegs.PIEACK.bit.ACK9
#define SCIC_TX_INT_ACK PieCtrlRegs.PIEACK.bit.ACK8
#define SCID_TX_INT_ACK PieCtrlRegs.PIEACK.bit.ACK8



//#include "inc_private/libSCI_F2837xD_device.h" //device definitions
//#include "inc_private/libSCI_F2837xD_sci.h"
//#include "inc_private/libSCI_F2837xD_sysctrl.h"
//#include "inc_private/libSCI_F2837xD_piectrl.h"             // PIE Control Registers
//#include "inc_private/libSCI_F2837xD_pievect.h"

#include "F2837xD_device.h" //device definitions
#include "F2837xD_sci.h"
#include "F2837xD_sysctrl.h"
#include "F2837xD_piectrl.h"             // PIE Control Registers
#include "F2837xD_pievect.h"




extern void F28x_usDelay(long LoopCount);
#define DELAY_US(A)  F28x_usDelay(((((long double) A * 1000.0L) / (long double)CPU_RATE) - 9.0L) / 5.0L)


#endif
