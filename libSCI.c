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
 
#include "libSCI.h"
#include "libSCI_private.h"

libSCI_handle_t libSCI_handle;

typedef struct libsci_buffer_struct { //!< two of these per SCI state machine
	char buffer[LIBSCI_BUFFER_LEN];
	int length; //!< length of message
	int progress; //!< how many bytes have already been sent?
	int inUse; //!< are we done? 0=ready to accept new message. 1=ready to tx. 2=being txed
} libSCI_buffer_t;

typedef struct sciStateMachine_struct { //!< and one of these per port
	sciPort_t port;
	int* txIntAck; //!< write a one here to tell the CPU we've processed our interrupt
	int enabled;
	int transmitting; //!< which buffer is transmitting? zero for idle
	libSCI_buffer_t first; 
	libSCI_buffer_t second;
} sciState_t;

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
			length = LIBSCI_BUFFER_LEN
			toReturn = -2; //!< only the first LIBSCI_BUFFER_LEN bytes of your message were queued. Advance your pointer and try again.
		}
		else {
			toReturn = 1; //!< message queued successfully
		}
		bufHandle->length = length;
		bufHandle->progress = 0;
		memcpy( &(bufHandle->buffer), msg, length ); //!< copy the message into the buffer
		bufHandle->inUse = 1 //!<order here is CRITICAL! don't mark the buffer in use until it's ready, in case we get interrupted!
	}
	else {
		toReturn = -3; //!< both message buffers were full
	}
	return toReturn;
}

int sciQueueMessage(sciState_t* portHandle, char* msg, int length) {
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
			portHandle->first->inUse = 2; //!< set this buffer as being transmitted (because it will be)
			portHandle->port->SCIFFTX.bit.TXFFIENA = 1; //!< enable the TX FIFO interrupt for this port
		}
	}
	else if(portHandle->transmitting == 1) { //buffer one is transmitting
		toReturn = sciEnqueue( &(portHandle->second), msg, length ); //therefore, enqueue on two
	}
	else if(portHandle->transmitting == 2) { //second buffer is transmitting
		toReturn = sciEnqueue( &(portHandle->first), msg, length ); //therefore, enqueue on one
	}
	else {
		asm("ESTOP0"); //something bad has happened
	}
	return toReturn;
}

static void sciProcessBuffer(sciPort_t port, libSCI_buffer_t * toTX, libSCI_buffer_t * other) {
	//now that we've established that we have work to do, let's get some bytes into that FIFO:
    while( 	(toTX->progress < toTX->length) && //!<we've sent fewer bytes than the length of the frame
			(port->SCIFFTX.bit.TXFFST != 0x10) ) { //!<AND we haven't filled the FIFO,   
        //!<upper byte of the int is masked out because we can't send it & bytes are 16 bits on this godforsaken platform
        port->SCITXBUF.bit.TXDT = ( (toTX->buffer)[toTX->charsSent] & 0x00FF); //!< send the byte:
        toTX->progress++; //!< mark the byte sent
    }
    if(toTX->charsSent == toTX->length) { //!< if we're done with this frame,
		toTX->inUse = 0; //!< mark it completed:
		if(other->inUse == 1) { //check to see if the second buffer needs to be transmitted:
			if(sciState->transmitting == 1) 		{ sciState->transmitting = 2; } //do a buffer swap
			else if(sciState->transmitting == 2) 	{ sciState->transmitting = 1; }
			other->inUse == 2;
		}
		else {
			sciState->transmitting = 0;
		}
        
    }
}

__interrupt void libSCI_TX_Handler(void) {
	sciState_t * sciState;
	sciPort_t port;
	//logic to figure out which port fired this interrupt:
	if(libSCI_handle.A.enabled) {
		if( (PieCtrlRegs.PIEIFR9.bit.INTx2) && 
			(libSCI_handle.A.port->SCIFFTX.bit.TXFFINT) ) 
			{ sciState = &(libSCI_handle.A); }
	}
	else if(libSCI_handle.B.enabled) {
		if( (PieCtrlRegs.PIEIFR9.bit.INTx4) && 
			(libSCI_handle.B.port->SCIFFTX.bit.TXFFINT) ) 
			{ sciState = &(libSCI_handle.B); }
	}
	else if(libSCI_handle.C.enabled) {
		if( (PieCtrlRegs.PIEIFR8.bit.INTx6) && 
			(libSCI_handle.C.port->SCIFFTX.bit.TXFFINT) ) 
			{ sciState = &(libSCI_handle.C); }
	}
	else if(libSCI_handle.D.enabled) {
		if(	(PieCtrlRegs.PIEIFR8.bit.INTx8) && 
			(libSCI_handle.D.port->SCIFFTX.bit.TXFFINT) ) 
			{ sciState = &(libSCI_handle.D); }
	}
	else {
		//this interrupt was called without any ports enabled. What the fuck?
		asm("ESTOP0");
	}
	port = sciState->port;
	//end port finding logic
	
    //is a transmission in progress?
	if(sciState->transmitting == 1) {
		sciProcessBuffer( port, &(sciState->first), &(sciState->second) );
	}
	else if(sciState->transmitting == 2) {
		sciProcessBuffer( port, &(sciState->second), &(sciState->first) );
	}
	//we aren't sending anything right now
    else if( (sciState->first.inUse == 0) && (sciState->second.inUse == 0) ) {
        //there is legitimately nothing to do.
        //we need to do something to keep this interrupt from firing
		sciState->transmitting = 0; //!< mark that we aren't transmitting anything, just in case.
        port->SCIFFTX.bit.TXFFIENA = 0; //!< disable FIFO-empty interrupt. This will be re-enabled by sciQueueMessage()
    }
	else {
		//something bad, but non-critical has occured.
		//todo something about it?
	}

    //ack the interrupt and return
    port->SCIFFTX.bit.TXFFINTCLR = 1; //!< ack the interrupt at the peripheral level
    *(sciState->txIntAck) = 1; //!< ack the interrupt at the interrupt controller level
}



libSCI_handle_t libSCI_init(sciPort_t port, libSCI_baudrate_t) {
	DINT;
	EALLOW;
	switch(port) {
		case &SciaRegs:
		PieVectTable.SCIA_TX_INT = &libSCI_TX_Handler; //!< add our interrupt to the table
		CpuSysRegs.PCLKCR7.bit.SCI_A = 1; //!< SCIA clock enable
		PieCtrlRegs.PIEIER9.bit.INTx2 = 1; //!< SCIA_TX is on INT9.2
		IER |= M_INT9; //!< M_INT9 = SCIA_RX/TX and SCIB_RX/TX
		libSCI_handle.A.enabled = 1;
		libSCI_handle.A.first.inUse = 0; //!< set buffers to empty
		libSCI_handle.A.second.inUse = 0;
		libSCI_handle.A.port = port;
		libSCI_handle.A.txIntAck = &PieCtrlRegs.PIEACK.bit.ACK9;
		break;
		
		case &ScibRegs:
		PieVectTable.SCIB_TX_INT = &libSCI_TX_Handler; //!< add our interrupt to the table
		CpuSysRegs.PCLKCR7.bit.SCI_B = 1; //!< SCIB clock enable
		PieCtrlRegs.PIEIER9.bit.INTx4 = 1; //!< SCIB_TX is on INT9.4
		IER |= M_INT9; //!< M_INT9 = SCIA_RX/TX and SCIB_RX/TX
		libSCI_handle.B.enabled = 1;
		libSCI_handle.B.first.inUse = 0; //!< set buffers to empty
		libSCI_handle.B.second.inUse = 0;
		libSCI_handle.B.port = port;
		libSCI_handle.B.txIntAck = &PieCtrlRegs.PIEACK.bit.ACK9;
		break;
		
		case &ScicRegs:
		PieVectTable.SCIC_TX_INT = &libSCI_TX_Handler; //!< add our interrupt to the table
		CpuSysRegs.PCLKCR7.bit.SCI_C = 1; //!< SCIC clock enable
		PieCtrlRegs.PIEIER8.bit.INTx6 = 1; //!< SCIC_TX is on INT8.6
		IER |= M_INT8; //!< M_INT8 = SCIC_RX/TX and SCID_RX/TX
		libSCI_handle.C.enabled = 1;
		libSCI_handle.C.first.inUse = 0; //!< set buffers to empty
		libSCI_handle.C.second.inUse = 0;
		libSCI_handle.C.port = port;
		libSCI_handle.C.txIntAck = &PieCtrlRegs.PIEACK.bit.ACK8;
		break;
		
		case &ScidRegs:
		PieVectTable.SCID_TX_INT = &libSCI_TX_Handler; //!< add our interrupt to the table
		CpuSysRegs.PCLKCR7.bit.SCI_D = 1; //!< SCIC clock enable
		PieCtrlRegs.PIEIER8.bit.INTx8 = 1; //!< SCIC_TX is on INT8.8
		IER |= M_INT8; //!< M_INT8 = SCIC_RX/TX and SCID_RX/TX
		libSCI_handle.D.enabled = 1;
		libSCI_handle.D.first.inUse = 0; //!< set buffers to empty
		libSCI_handle.D.second.inUse = 0;
		libSCI_handle.D.port = port;
		libSCI_handle.D.txIntAck = &PieCtrlRegs.PIEACK.bit.ACK8;
		break;
	
		default:
		//do an error thing
		break;
	}
	DELAY_US(100); //!<delay here. Fucking pipelines.	
	port->SCICCR.all = 0x0007;  // 1 stop bit,  No loopback No parity,8 char bits, async mode, idle-line protocol
    port->SCICTL1.all = 0x0003; // enable TX, RX, internal SCICLK,
	port->SCICTL2.all = 0x0003; // Disable RX ERR, SLEEP, TXWAKE
	port->SCICTL2.bit.TXINTENA = 1; //enable TX interrupt?
	port->SCICTL2.bit.RXBKINTENA = 1;
	port->SCICTL1.all = 0x0023; // Relinquish SCI from Reset
	//fifos:
	port->SCIFFTX.all = 0xE040;
	port->SCIFFTX.bit.TXFFIL = 0; //fire TX FIFO interrupt when the FIFO is empty
	port->SCIFFTX.bit.TXFFIENA = 1; //enable TX FIFO interrupt
	port->SCIFFRX.all = 0x2044;
	port->SCIFFCT.all = 0x0;
		
	switch(libSCI_baudrate_t) {
		case LIBSCI_115200:
		port->SCIHBAUD.all = 0x0000;
		port->SCILBAUD.all = 0x0036;
		break;
		
		default:
		//do an error thing
		break;
	}
	
	EDIS;
	EINT;
	return libSCI_handle;
}

