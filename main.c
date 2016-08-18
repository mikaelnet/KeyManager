/*
 * KeyManager.c
 *
 * Created: 2016-07-28 23:23:35
 * Author : mikael
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include "usbdrv/usbdrv.h"
#include "usbdrv/oddebug.h"        /* This is also an example for using debug macros */

#define PASS_LENGTH 10 // password length for generated password
#define SEND_ENTER 0 // define to 1 if you want to send ENTER after password

const PROGMEM uchar measuring_message[] = "Starting generation...";
const PROGMEM uchar finish_message[] = " New password saved.";

// The buffer needs to accommodate the messages above and the password
#define MSG_BUFFER_SIZE 32

EEMEM uchar stored_password[MSG_BUFFER_SIZE];


// ************************
// *** USB HID ROUTINES ***
// ************************

// From Frank Zhao's USB Business Card project
// http://www.frank-zhao.com/cache/usbbusinesscard_details.php
const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
	0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	0x09, 0x06,                    // USAGE (Keyboard)
	0xa1, 0x01,                    // COLLECTION (Application)
	0x75, 0x01,                    //   REPORT_SIZE (1)
	0x95, 0x08,                    //   REPORT_COUNT (8)
	0x05, 0x07,                    //   USAGE_PAGE (Keyboard)(Key Codes)
	0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)(224)
	0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)(231)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs) ; Modifier byte
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x81, 0x03,                    //   INPUT (Cnst,Var,Abs) ; Reserved byte
	0x95, 0x05,                    //   REPORT_COUNT (5)
	0x75, 0x01,                    //   REPORT_SIZE (1)
	0x05, 0x08,                    //   USAGE_PAGE (LEDs)
	0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
	0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
	0x91, 0x02,                    //   OUTPUT (Data,Var,Abs) ; LED report
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x75, 0x03,                    //   REPORT_SIZE (3)
	0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs) ; LED report padding
	0x95, 0x06,                    //   REPORT_COUNT (6)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
	0x05, 0x07,                    //   USAGE_PAGE (Keyboard)(Key Codes)
	0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))(0)
	0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)(101)
	0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
	0xc0                           // END_COLLECTION
};

typedef struct {
	uint8_t modifier;
	uint8_t reserved;
	uint8_t keycode[6];
} keyboard_report_t;

static keyboard_report_t keyboard_report; // sent to PC
volatile static uchar LED_state = 0xff; // received from PC
static uchar idleRate; // repeat rate for keyboards

#define NUM_LOCK 1
#define CAPS_LOCK 2
#define SCROLL_LOCK 4

#define STATE_SEND 1
#define STATE_DONE 0

static uchar messageState = STATE_DONE;
static uchar messageBuffer[MSG_BUFFER_SIZE] = "";
static uchar messagePtr = 0;
static uchar messageCharNext = 1;

#define MOD_SHIFT_LEFT (1<<1)


usbMsgLen_t usbFunctionSetup(uchar data[8]) 
{
	usbRequest_t *rq = (void *)data;

	if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {
		switch(rq->bRequest) {
			case USBRQ_HID_GET_REPORT:
				// send "no keys pressed" if asked here
				usbMsgPtr = (void *)&keyboard_report;
				keyboard_report.modifier = 0;
				keyboard_report.keycode[0] = 0;
				return sizeof(keyboard_report);

			case USBRQ_HID_SET_REPORT:
				return (rq->wLength.word == 1) ? USB_NO_MSG : 0;

			case USBRQ_HID_GET_IDLE:
				usbMsgPtr = &idleRate;
				return 1;

			case USBRQ_HID_SET_IDLE:
				idleRate = rq->wValue.bytes[1];
				return 0;
		}
	}
	return 0;
}

void buildReport(char ch) {
	keyboard_report.modifier = 0;

	if (ch >= '0' && ch <= '9') {
		keyboard_report.modifier = 0;
		keyboard_report.keycode[0] = (ch == '0') ? 39 : 30+(ch-'1');
	}
	else if (ch >= 'a' && ch <= 'z') {
		keyboard_report.modifier = 0;
		keyboard_report.keycode[0] = 4 + (ch-'a');
	}
	else if (ch >= 'A' && ch <= 'Z') {
		keyboard_report.modifier = MOD_SHIFT_LEFT;
		keyboard_report.keycode[0] = 4 + (ch-'A');
	}
	else {
		keyboard_report.modifier = 0;
		uint8_t keyCode = 0;
		
		switch (ch) {
			case '.':
				keyCode = 0x37; break;
			case '_':
				keyboard_report.modifier = MOD_SHIFT_LEFT;	// No break here!
			case '-':
				keyCode = 0x2D; break;
			case ' ':
				keyCode = 0x2C; break;
			case '\t':
				keyCode = 0x2B; break;
			case '\n':
				keyCode = 0x28; break;
		}
		keyboard_report.keycode[0] = keyCode;
	}
}


#define STATE_WAIT 0
#define STATE_SEND_KEY 1
#define STATE_RELEASE_KEY 2


int main(void)
{
	DDRB = _BV(PB1) | _BV(PB4);	// RED LED + WS2812 LED
	PORTB = _BV(PB1);

	wdt_enable(WDTO_1S);
	usbInit();

	usbDeviceDisconnect();	// enforce re-enumeration
	for (uint8_t i = 0 ; i < 250 ; i ++) {	// Wait 500ms
		wdt_reset();
		_delay_ms(2);
	}
	usbDeviceConnect();

	sei();

    uint8_t state = STATE_WAIT;
	uint8_t button_release_counter = 0;
    while (1) 
    {
		wdt_reset();
		usbPoll();

		if ( !(PINB & _BV(PB3)) ) {
			if (state == STATE_WAIT && button_release_counter == 255)
				state = STATE_SEND_KEY;

			button_release_counter = 0;
		}

		if (button_release_counter < 255)
			button_release_counter ++;

		if (usbInterruptIsReady() && state != STATE_WAIT) {
			switch (state) {
				case STATE_SEND_KEY:
					buildReport('x');
					state = STATE_RELEASE_KEY;
					break;

				case STATE_RELEASE_KEY:
					buildReport(0);
					state = STATE_WAIT;
					break;

				default:
					state = STATE_WAIT;
			}

			usbSetInterrupt((void*)&keyboard_report, sizeof(keyboard_report));
		}

    }
}

