/*
 * ioBuffer.h
 *
 * Author: Jack Doan
 * 
 * This is a general IO buffer for the C2000's serial peripherals. It provides a minimum of two slots
 * for outgoing messages such that they can be sent by multiple threads without mutual exclusion problems.
 *
 */
#ifndef IOBUF_H_
#define IOBUF_H_

#include <string.h>

#define IOBUF_STORAGE_LEN 200
#define IOBUF_NUM_STORAGE 2 //todo: currently only 2 are supported


typedef enum iobuf_service_status_enum {
    IOBUF_SERVICE_ERROR,
    IOBUF_SERVICE_NONE,
    IOBUF_SERVICE_GOOD
} iobuf_service_status_t;

typedef struct iobuf_service_reply_struct {
    iobuf_service_status_t status;
    char data;
} iobuf_service_reply_t;

typedef enum iobuf_storage_status_enum {
    IOBUF_STORAGE_EMPTY,
    IOBUF_STORAGE_QUEUED,
    IOBUF_STORAGE_PROCESSING
} iobuf_storage_status;

typedef struct iobuf_storage_struct { //!< two of these per iobuf state machine
    char buffer[IOBUF_STORAGE_LEN];
    int length; //!< length of message
    int progress; //!< how many bytes have already been sent?
    iobuf_storage_status status; //!< are we done? 0=ready to accept new message. 1=ready to tx. 2=being txed
 } iobuf_storage_t;


typedef struct iobuf_handle_struct {
    iobuf_storage_t storage[IOBUF_NUM_STORAGE];
    int transmitting; //which buffer is the port currently processing? -1=idle.
} iobuf_handle;


int iobuf_enqueue(iobuf_handle* handle, char* msg, int length);
iobuf_service_reply_t iobuf_service(iobuf_handle* handle);
void iobuf_init(iobuf_handle* handle);

#endif
