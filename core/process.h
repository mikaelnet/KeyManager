/*
 * process.h
 *
 * Created: 2016-08-23 22:11:44
 *  Author: mikael
 */ 


#ifndef PROCESS_H_
#define PROCESS_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum DeviceId_enum {
	DEVICE_CLOCK_ID,
	DEVICE_BUTTON_ID,
	DEVICE_USB_ID,
} DeviceId_t;

typedef enum EventId_enum {
	DEFAULT,
	UNKNOWN,
} EventId_t;

typedef struct EventArgs_struct {
	DeviceId_t senderId;
	EventId_t eventId;
	uint16_t eventData;
} EventArgs_t;

typedef struct Process_struct {
	void (*executeLoopMethod)();
	void (*eventHandlerMethod)(EventArgs_t *args);
	struct Process_struct *next;
} Process_t;

void process_raise_event (EventArgs_t *event);
void process_execute_loop ();

#endif /* PROCESS_H_ */
