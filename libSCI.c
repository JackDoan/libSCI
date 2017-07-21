/*
 * libSCI.c
 *
 *  Created on: Jan 27, 2017
 *      Author: jad140230
 *	
 *	This is a driver for the TI C2000 SCI port. It allows the you to queue messages and have them transmit
 *	with minimal blocking/CPU intervention.
 *
 *	This driver can be instantiated once for each port available to the CPU it's running on.
 *	
 *	The port is double-buffered, with the assumption that writes will occur infrequently enough and be small
 *	enough such that one can finish, and a second one can be ready before a third one is queued.
 *	As such, there isn't a real queue table.
 *
 *      
 *
 *
 */
 
#include "inc/libSCI.h"
#include "inc_private/libSCI_private.h"

libSCI_handle_t libSCI_handle;

int sciCheckMsgLength(int len) {
	if(len <= LIBSCI_BUFFER_LEN)
		return 1;
	else
		return 0;
}

int sciEnqueue(libSCI_buffer_t* bufHandle, char* msg, int length) {
	int toReturn = 0;
	if(bufHandle->inUse == 0) { //buffer is available
		if(!sciCheckMsgLength(length)) {
			length = LIBSCI_BUFFER_LEN;
			toReturn = -2; //!< only the first LIBSCI_BUFFER_LEN bytes of your message were queued. Advance your pointer and try again.
		}
		else {
			toReturn = 1; //!< message queued successfully
		}
		bufHandle->length = length;
		bufHandle->progress = 0;
		memcpy( &(bufHandle->buffer), msg, length ); //!< copy the message into the buffer
		bufHandle->inUse = 1; //!<order here is CRITICAL! don't mark the buffer in use until it's ready, in case we get interrupted!
	}
	else {
		toReturn = -3; //!< both message buffers were full
	}
	return toReturn;
}

int sciQueueMessage(sciState_t* portHandle, char* msg, unsigned int length) {
	int i = 0;
	int toReturn = 0;
	if(portHandle->enabled != 1) {
		return -1; //!< error! this port isn't enabled.
	}
	if(portHandle->transmitting == 0) { //the port is idle
		//EDGE CASE: 
		if(length < 0x10) { //if the message is less than the width of our currently idle FIFO
			for(i = 0; i < length; i++) { //just put the bytes on the damn port
				portHandle->port->SCITXBUF.bit.TXDT = msg[i];
			}
			toReturn = 0; //!< message sent and not queued
		}
		else {
			sciEnqueue( &(portHandle->first), msg, length );
			portHandle->transmitting = 1; //!< set this buffer to be transmitted
			portHandle->first.inUse = 2; //!< set this buffer as being transmitted (because it will be)
			portHandle->port->SCIFFTX.bit.TXFFIENA = 1; //!< enable the TX FIFO interrupt for this port
			//portHandle->port->SCIFFTX.bit.TXFFINTCLR = 1;
		}
	}
	else if(portHandle->transmitting == 1) { //buffer one is transmitting
		toReturn = sciEnqueue( &(portHandle->second), msg, length ); //therefore, enqueue on two
	}
	else if(portHandle->transmitting == 2) { //second buffer is transmitting
		toReturn = sciEnqueue( &(portHandle->first), msg, length ); //therefore, enqueue on one
	}
	else {
	    asm ("      ESTOP0"); //something bad has happened
	}
	return toReturn;
}

static void sciProcessBuffer(sciState_t * sciState, sciPort_t port, libSCI_buffer_t * toTX, libSCI_buffer_t * other) {
	//now that we've established that we have work to do, let's get some bytes into that FIFO:
    while( 	(toTX->progress < toTX->length) && //!<we've sent fewer bytes than the length of the frame
			(port->SCIFFTX.bit.TXFFST != 0x10) ) { //!<AND we haven't filled the FIFO,   
        //!<upper byte of the int is masked out because we can't send it & bytes are 16 bits on this godforsaken platform
        port->SCITXBUF.bit.TXDT = ( (toTX->buffer)[toTX->progress] & 0x00FF); //!< send the byte:
        toTX->progress++; //!< mark the byte sent
    }
    if(toTX->progress == toTX->length) { //!< if we're done with this frame,
		toTX->inUse = 0; //!< mark it completed:
		if(other->inUse == 1) { //check to see if the second buffer needs to be transmitted:
			if(sciState->transmitting == 1) 		{ sciState->transmitting = 2; } //do a buffer swap
			else if(sciState->transmitting == 2) 	{ sciState->transmitting = 1; }
			other->inUse = 2;
		}
		else {
			sciState->transmitting = 0;
		}
        
    }
}

__interrupt void libSCI_TX_Handler(void) {
	sciState_t * sciState;
	sciPort_t port;
	int ack = 0;

#ifdef CPU2
	GpioDataRegs.GPBDAT.bit.GPIO60 = 1;
	GpioDataRegs.GPADAT.bit.GPIO22 = 1;
#endif
#ifdef CPU1
	GpioDataRegs.GPCDAT.bit.GPIO67 = 0;
	GpioDataRegs.GPDDAT.bit.GPIO111 = 1;
	GpioDataRegs.GPDDAT.bit.GPIO105 = 1;
#endif

	//logic to figure out which port fired this interrupt:
	if((libSCI_handle.A.enabled) && (PieCtrlRegs.PIEACK.bit.ACK9) &&
	  (libSCI_handle.A.port->SCIFFTX.bit.TXFFINT) ) {
	          sciState = &(libSCI_handle.A);
		      ack = 9;
	}
	else if ((libSCI_handle.B.enabled) &&
	        (PieCtrlRegs.PIEACK.bit.ACK9) &&
			(libSCI_handle.B.port->SCIFFTX.bit.TXFFINT) ) {
		        sciState = &(libSCI_handle.B);
		        ack = 9;
	}
	else if ((libSCI_handle.C.enabled) &&
	        (PieCtrlRegs.PIEACK.bit.ACK8) &&
			(libSCI_handle.C.port->SCIFFTX.bit.TXFFINT) ) {
		        sciState = &(libSCI_handle.C);
			    ack = 8;
	}
	else if ((libSCI_handle.D.enabled) &&
			(PieCtrlRegs.PIEACK.bit.ACK8) &&
			(libSCI_handle.D.port->SCIFFTX.bit.TXFFINT) ) {
		        sciState = &(libSCI_handle.D);
		        ack = 8;
	}
	else {
		//this interrupt was called without any ports enabled. What the fuck?
	    asm ("      ESTOP0");
	}
	port = sciState->port;
	//end port finding logic
	
    //is a transmission in progress?
	if(sciState->transmitting == 1) {
		sciProcessBuffer(sciState, port, &(sciState->first), &(sciState->second) );
	}
	else if(sciState->transmitting == 2) {
		sciProcessBuffer(sciState, port, &(sciState->second), &(sciState->first) );
	}
	//we aren't sending anything right now
    else if( (sciState->first.inUse == 0) && (sciState->second.inUse == 0) ) {
        //there is legitimately nothing to do.
        //we need to do something to keep this interrupt from firing
		sciState->transmitting = 0; //!< mark that we aren't transmitting anything, just in case.
        port->SCIFFTX.bit.TXFFIENA = 0; //!< disable FIFO-empty interrupt. This will be re-enabled by sciQueueMessage()
    }
	else {
		//something bad, but non-critical has occurred. This shouldn't ever execute, so if it does, halt.
	    asm ("      ESTOP0");

	}

    //ack the interrupt and return
    port->SCIFFTX.bit.TXFFINTCLR = 1; //!< ack the interrupt at the peripheral level
    switch(ack) { //!< ack the interrupt at the interrupt controller level
    case 9:
        PieCtrlRegs.PIEACK.bit.ACK9 = 1;
        break;
    case 8:
        PieCtrlRegs.PIEACK.bit.ACK8 = 1;
        break;
    default:
        break;
    }
#ifdef CPU2
    GpioDataRegs.GPBDAT.bit.GPIO60 = 0;
    GpioDataRegs.GPADAT.bit.GPIO22 = 0;
#endif
#ifdef CPU1
    GpioDataRegs.GPCDAT.bit.GPIO67 = 0;
    GpioDataRegs.GPDDAT.bit.GPIO111 = 0;
    GpioDataRegs.GPDDAT.bit.GPIO105 = 0;
#endif
}



libSCI_handle_t libSCI_init(libSCI_port_t libport, libSCI_baudrate_t baudrate) {
	DINT;
	EALLOW;
	sciPort_t port;
	switch(libport) {
		case A:
		PieVectTable.SCIA_TX_INT = &libSCI_TX_Handler; //!< add our interrupt to the table
		CpuSysRegs.PCLKCR7.bit.SCI_A = 1; //!< SCIA clock enable
		PieCtrlRegs.PIEIER9.bit.INTx2 = 1; //!< SCIA_TX is on INT9.2
		IER |= M_INT9; //!< M_INT9 = SCIA_RX/TX and SCIB_RX/TX
		port = &SciaRegs;
		libSCI_handle.A.enabled = 1;
		libSCI_handle.A.first.inUse = 0; //!< set buffers to empty
		libSCI_handle.A.second.inUse = 0;
		libSCI_handle.A.port = port;
		break;
		
		case B:
		PieVectTable.SCIB_TX_INT = &libSCI_TX_Handler; //!< add our interrupt to the table
		CpuSysRegs.PCLKCR7.bit.SCI_B = 1; //!< SCIB clock enable
		PieCtrlRegs.PIEIER9.bit.INTx4 = 1; //!< SCIB_TX is on INT9.4
		IER |= M_INT9; //!< M_INT9 = SCIA_RX/TX and SCIB_RX/TX
		port = &SciaRegs;
		libSCI_handle.B.enabled = 1;
		libSCI_handle.B.first.inUse = 0; //!< set buffers to empty
		libSCI_handle.B.second.inUse = 0;
		libSCI_handle.B.port = port;
		break;
		
		case C:
		PieVectTable.SCIC_TX_INT = &libSCI_TX_Handler; //!< add our interrupt to the table
		CpuSysRegs.PCLKCR7.bit.SCI_C = 1; //!< SCIC clock enable
		PieCtrlRegs.PIEIER8.bit.INTx6 = 1; //!< SCIC_TX is on INT8.6
		IER |= M_INT8; //!< M_INT8 = SCIC_RX/TX and SCID_RX/TX
		port = &SciaRegs;
		libSCI_handle.C.enabled = 1;
		libSCI_handle.C.first.inUse = 0; //!< set buffers to empty
		libSCI_handle.C.second.inUse = 0;
		libSCI_handle.C.port = port;
		break;
		
		case D:
		PieVectTable.SCID_TX_INT = &libSCI_TX_Handler; //!< add our interrupt to the table
		CpuSysRegs.PCLKCR7.bit.SCI_D = 1; //!< SCIC clock enable
		PieCtrlRegs.PIEIER8.bit.INTx8 = 1; //!< SCIC_TX is on INT8.8
		IER |= M_INT8; //!< M_INT8 = SCIC_RX/TX and SCID_RX/TX
		port = &SciaRegs;
		libSCI_handle.D.enabled = 1;
		libSCI_handle.D.first.inUse = 0; //!< set buffers to empty
		libSCI_handle.D.second.inUse = 0;
		libSCI_handle.D.port = port;
		break;
	
		default:
		//do an error thing
		break;
	}
	DELAY_US(100); //!<delay here. Fucking pipelines.	
	//port->SCICCR.all = 0x0007;
	port->SCICCR.bit.STOPBITS = 0; // 1 stop bit,
	port->SCICCR.bit.PARITYENA = 0; // No parity
	port->SCICCR.bit.LOOPBKENA = 0; //No loopback
	port->SCICCR.bit.ADDRIDLE_MODE = 0; //async mode, idle-line protocol
	port->SCICCR.bit.SCICHAR = 7; //8 char bits

    //port->SCICTL1.all = 0x0003; // enable TX, RX, internal SCICLK,
    port->SCICTL1.bit.RXERRINTENA = 0; //no RX error interrupt
    port->SCICTL1.bit.SWRESET = 0; //put the SCI in reset for configuration
    port->SCICTL1.bit.TXWAKE = 0; //don't use this
    port->SCICTL1.bit.SLEEP = 0; //don't sleep
    port->SCICTL1.bit.TXENA = 1; //enable TX
    port->SCICTL1.bit.RXENA = 1; //enable RX

    switch(baudrate) {
        case LIBSCI_115200:
        port->SCIHBAUD.all = 0x0000;
        port->SCILBAUD.all = 0x0036;
        break;

        default:
        //do an error thing
        break;
    }

	port->SCICTL2.bit.TXINTENA = 1; //enable TX interrupt?
	port->SCICTL2.bit.RXBKINTENA = 0; //no break-detection needed

	port->SCICTL1.bit.SWRESET = 1; // Relinquish SCI from Reset
	//fifos:
	//port->SCIFFTX.all = 0xE040;
	port->SCIFFTX.bit.SCIFFENA = 1; //enable FIFO
	port->SCIFFTX.bit.SCIRST = 1; //don't reset
	port->SCIFFTX.bit.TXFIFORESET = 1; //dont reset this either
	port->SCIFFTX.bit.TXFFIL = 1; //fire TX FIFO interrupt when the FIFO is almost empty
	port->SCIFFTX.bit.TXFFIENA = 1; //enable TX FIFO interrupt
	//port->SCIFFRX.all = 0x2044;
	port->SCIFFRX.bit.RXFIFORESET = 1; //come out of reset
	port->SCIFFRX.bit.RXFFIL = 1; //interrupt when we have this many words available
	port->SCIFFRX.bit.RXFFIENA = 0; //enable setting for RXFFIL

	port->SCIFFCT.all = 0x0;
		
	EDIS;
	EINT;
	return libSCI_handle;
}

void sciExpectRX(sciState_t* libport, int length) {
    sciPort_t port = libport->port;
    //we don't care about the contents of the RX buffer, just how much is in it.
    port->SCIFFRX.bit.RXFFIL = length; //fire isr when this many bytes are here
    port->SCIFFRX.bit.RXFFIENA = 1;    //enable isr
    port->SCIFFRX.bit.RXFFINTCLR = 1;  //clear the interrupt latch so that new ISRs can fire
}

int sciRead(sciState_t* p) {
    return p->port->SCIRXBUF.bit.SAR;
}

int sciAvailable(sciState_t* portHandle) { //returns number of bytes in the rx buffer
    return portHandle->port->SCIFFRX.bit.RXFFST;
}
/* the RX state machine
 *
 * State 0 / default: fire an ISR when a byte is rxed. If the byte marks the start of a packet:
 *          move to state 1
 *          return from ISR
 *          else: return from ISR. This was an invalid byte.
 * State 1: still single byte, but this will mark the length of the incoming packet. Save this to $length.
 *          port->SCIFFRX.bit.RXFFIL = max($length, 14). If $length > 16, save $length-14 to $more
 *          14 is chosen instead of 16 to 100% avoid overflows
 *          advance to State 2
 *          set a CPU timer for $timeout milliseconds. If this timer fires, return to state 0.
 *          return from ISR
 * State 2: copy the RX FIFO into a buffer, up to $length+$more, decrementing $more as appropriate
 *          clear timer from State 1
 *          if $more > 0, adjust RXFFIL and return to State 1
 *          else, alert main() that there is a packet ready in buffer N and return to State 0
 *          return from ISR
 *
 */




