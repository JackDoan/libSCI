/*
 * ioBuffer.c
 *
 * Author: Jack Doan
 * 
 * This is a general IO buffer for the C2000's serial peripherals. It provides a minimum of two slots
 * for outgoing messages such that they can be sent by multiple threads without mutual exclusion problems.
 *
 */

#include <string.h>

#include "inc/ioBuffer.h"

static int iobuf_internal_check_msg_length(int len) {
    if(len <= IOBUF_STORAGE_LEN)
        return 1;
    else
        return 0;
}

static int iobuf_place_into_storage(iobuf_storage_t* storage, char* msg, int length) {
    //todo: return value enums
    int toReturn = -1;
    if(storage->status == IOBUF_STORAGE_EMPTY) { //buffer is available
        if(!iobuf_internal_check_msg_length(length)) {
            length = IOBUF_STORAGE_LEN;
            toReturn = -2; //!< only the first IOBUF_STORAGE_LEN bytes of your message were queued.       Advance your pointer and try again.
        }
        else {
            toReturn = 1; //!< message queued successfully
        }
        storage->length = length;
        storage->progress = 0;
        memcpy( &(storage->buffer), msg, length ); //!< copy the message into the buffer
        storage->status = IOBUF_STORAGE_QUEUED; //!<order here is CRITICAL! don't mark the buffer in use until it's ready,  in case we get interrupted!
    }
    else {
        toReturn = -3; //!< both message buffers were full
    }
    return toReturn;
}

static char iobuf_extract_byte(iobuf_storage_t* storage) {
    /*
     * This function takes a storage unit and does the following:
     * extracts the next byte
     * increments progress by one
     */
    char toReturn = 0; 
    //todo: should we check that we've sent fewer bytes than the length of the frame?
    toReturn = ( (storage->buffer)[storage->progress] ); //send the byte:
    storage->progress++; //mark the byte sent
    return toReturn;
}

/*
//returns a pointer internal to the iobuf, and a legal amount of bytes to read
iobuf_mass_reply_t iobuf_mass_read(iobuf_handle* handle, int desiredLen) {
    iobuf_mass_reply_t toReturn;
    iobuf_storage_t* storage;
    if( (handle->transmitting > -1) && (handle->transmitting < IOBUF_NUM_STORAGE) ){ //someone is txing
        storage = &( handle->storage[handle->transmitting] );
        int remaining = storage->length - storage->progress;
        if(remaining <= desiredLen) {
            //we need to rotate the storage, since we either exactly finished it or came back short

        }
    }
    else {
        //there is nothing to do
        toReturn.status = IOBUF_SERVICE_NONE;
        toReturn.data = 0;
    }
}
*/
iobuf_service_reply_t iobuf_service(iobuf_handle* handle) {
    iobuf_service_reply_t toReturn;
    iobuf_storage_t* storage;
    iobuf_storage_t* next_storage;
    int next_storage_id = handle->transmitting+1 % IOBUF_NUM_STORAGE;
    
    if( (handle->transmitting > -1) && (handle->transmitting < IOBUF_NUM_STORAGE) ){ //someone is txing
        storage = &( handle->storage[handle->transmitting] );
        next_storage = &( handle->storage[next_storage_id] );

        toReturn.status = IOBUF_SERVICE_GOOD;
        toReturn.data = iobuf_extract_byte(storage);

        if(storage->progress >= storage->length) { //!< if we're done with this frame,
            storage->status = IOBUF_STORAGE_EMPTY; //!< mark it completed:
            if(next_storage->status == IOBUF_STORAGE_QUEUED) { //check to see if the second buffer needs to be transmitted:
                handle->transmitting = next_storage_id;
                handle->storage[next_storage_id].status = IOBUF_STORAGE_PROCESSING;
            }
            else {
                handle->transmitting = -1;
            }
        }
    }
    else {
        //there is nothing to do
        toReturn.status = IOBUF_SERVICE_NONE;
        toReturn.data = 0;
    }
    return toReturn;
}


int iobuf_enqueue(iobuf_handle* handle, char* msg, int length) {
    //this method chooses a storage to put the message in, and adjusts the ISR to tx it
    //todo: return value enums
    int toReturn = -1; //assume failure
    
    /*
     * todo: improve performance by not enQ'ing when the port is idle and the message is short
     * this is hard to generalize between port types though.
     *
     * if(handle->transmitting == 0) { //the port is idle
        if(length < handle->hw.tx_fifo_width) { //the message is less that the hardware's buffer width
            //todo: just send the message, don't buffer.
        }
     */

    if(handle->transmitting == -1) { //the port is idle
        handle->transmitting = 0; //we'll use the first storage unit then
        toReturn = iobuf_place_into_storage( &(handle->storage[0]), msg, length );
        handle->storage[0].status = IOBUF_STORAGE_PROCESSING;
        //*(handle->hw_ISR_enable) = 1;  //this is handled in a layer up
        //!< enable the TX FIFO interrupt for this port
    }
    else if(handle->transmitting == 0) { //buffer one is transmitting
        toReturn = iobuf_place_into_storage( &(handle->storage[1]), msg, length ); //therefore, enqueue on two
    }
    else if(handle->transmitting == 1) { //second buffer is transmitting, and first isn't.
        toReturn = iobuf_place_into_storage( &(handle->storage[0]), msg, length ); //therefore, enqueue on one
    }
    else {
//        asm ("      ESTOP0"); //something bad has happened
    }
    return toReturn;
}    



void iobuf_init(iobuf_handle * handle) {
    handle->transmitting = -1;
    for(int i = 0; i < IOBUF_NUM_STORAGE; i++) {
        handle->storage[i].length = 0;
        handle->storage[i].progress = 0;
        handle->storage[i].status = IOBUF_STORAGE_EMPTY;
    }
}
