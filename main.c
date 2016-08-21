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
#include <stdlib.h>

extern void hasUsbReset();

#include "usbconfig.h"
#include "usbdrv/usbdrv.h"
#include "usbdrv/oddebug.h"        /* This is also an example for using debug macros */

#include "ws2812/ws2812.h"

#define PASS_LENGTH 10 // password length for generated password
#define SEND_ENTER 0 // define to 1 if you want to send ENTER after password

// The buffer needs to accommodate the messages above and the password
#define MSG_BUFFER_SIZE 32
EEMEM uchar stored_passwords[MSG_BUFFER_SIZE * 7];

/*EEMEM uchar stored_passwords[7][MSG_BUFFER_SIZE] = {
    { "012345678901234567890123456789\n" },
    { "abcdefghijklmnopqrstuvwxyz1234\n" },
    { "ABCDEFGHIJKLMNOPQRSTUVWXYZ5678\n" },

    { "abcde01234ABCDE56789abcdefghij\n" },
    { "aabbccddeeffgghhiijjkkllmmnnoo\n" },
    { "AABBCCDDEEFFGGHHIIJJKKLLMMNNOO\n" },
    { "12345_12345-12345_12345-12345_\n" },
};*/

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

#define MOD_SHIFT_LEFT (1<<1)

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t *rq = (void *)data;

	if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {
		switch(rq->bRequest) {
			case USBRQ_HID_GET_REPORT:
				// send "no keys pressed" if asked here
				usbMsgPtr = (usbMsgPtr_t)&keyboard_report;  //was cast to void*
				keyboard_report.modifier = 0;
				keyboard_report.keycode[0] = 0;
				return sizeof(keyboard_report);

			case USBRQ_HID_SET_REPORT:
				return (rq->wLength.word == 1) ? USB_NO_MSG : 0;

			case USBRQ_HID_GET_IDLE:
				usbMsgPtr = (usbMsgPtr_t)&idleRate; //was no cast
				return 1;

			case USBRQ_HID_SET_IDLE:
				idleRate = rq->wValue.bytes[1];
				return 0;
		}
	}
	return 0;
}

#define i_abs(x) ((x) > 0 ? (x) : (-x))
void hadUsbReset()
{
	int frameLength;
	int targetLength = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);
	int bestDeviation = 9999;
	uchar trialCal;
	uchar bestCal = OSCCAL;

	// do a binary search in regions 0-127 and 128-255 to get optimum OSCCAL
	for (int region = 0 ; region <= 1 ; region ++) {
		frameLength = 0;
		trialCal = (region == 0) ? 0 : 128;

		for (int step = 64; step > 0; step >>= 1) {
			if (frameLength < targetLength)	// true for initial iteration
				trialCal += step;
			else
				trialCal -= step;

			OSCCAL = trialCal;
			frameLength = usbMeasureFrameLength();

			if (i_abs(frameLength-targetLength) < bestDeviation) {
				bestCal = trialCal;
				bestDeviation = i_abs(frameLength - targetLength);
			}
		}
	}

	OSCCAL = bestCal;
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
            //default:
                //keyCode = 0x37; break;
		}
		keyboard_report.keycode[0] = keyCode;
	}
}


#define STATE_WAIT 0
#define STATE_SEND_KEY 1
#define STATE_RELEASE_KEY 2

struct cRGB led[8];
uint8_t ledIndex = 0;
void setup() {
	DDRB = _BV(PB1) | _BV(PB4);	// RED LED + WS2812 LED
	PORTB = _BV(PB1);

	led[0].r = 0x0F; led[0].g = 0x00; led[0].b = 0x00;
	led[1].r = 0x00; led[1].g = 0x1F; led[1].b = 0x00;
	led[2].r = 0x00; led[2].g = 0x00; led[2].b = 0x0F;
	led[3].r = 0x0F; led[3].g = 0x0F; led[3].b = 0x00;
	led[4].r = 0x0F; led[4].g = 0x00; led[4].b = 0x0F;
	led[5].r = 0x00; led[5].g = 0x0F; led[5].b = 0x0F;
	led[6].r = 0x0F; led[6].g = 0x0F; led[6].b = 0x0F;
	led[7].r = 0x00; led[7].g = 0x00; led[7].b = 0x00;

    ws2812_setleds(&led[ledIndex], 1);

    TCCR1 = 0x0F;   // Divide 16.5MHz in 16384 -> 1007 ticks/second
    TCNT1 = 155;
    TIMSK |= _BV(TOIE1); // | _BV(TOIE0);
}

volatile uint16_t global_timer; // Counts upwards once every 0.1s
ISR(TIMER1_OVF_vect) {
    global_timer ++;
    TCNT1 = 155;
}

char messageBuffer[MSG_BUFFER_SIZE+3];  // 2 extra bytes for newline and null termination

char *generateNewKeys() {
    PORTB |= _BV(PB1);

    srand(global_timer);
    for (int i=0 ; i < MSG_BUFFER_SIZE * 7 ; i ++) {
        uchar ch = rand() % 63;
        if (ch < 26)
            ch = 'a' + ch;
        else if (ch < 52)
            ch = 'A' + ch - 26;
        else if (ch < 62)
            ch = '0' + ch - 52;
        else if (ch == 62)
            ch = '-';
        //else if (ch == 63)
        //    ch = '_';

        wdt_reset();
        eeprom_write_byte(&stored_passwords[i], ch);
    }

    strcpy_P(messageBuffer, PSTR("New keys generated\n"));
    return messageBuffer;
}

char *toHex (char *ptr, char ch) {
    uint8_t nibble = (ch >> 4) & 0x0F;
    *ptr++ = (nibble < 10 ? '0' : 'A'-10) + nibble;
    nibble = ch & 0x0F;
    *ptr++ = (nibble < 10 ? '0' : 'A'-10) + nibble;
    return ptr;
}

int main(void)
{
	setup();

    //eeprom_write_byte(eeTestChar, 0x5A);
    //eeprom_write_byte(eeTestChar+1, 0x3D);

	wdt_enable(WDTO_1S);
	usbInit();

	usbDeviceDisconnect();	// enforce re-enumeration
	for (uint8_t i = 0 ; i < 250 ; i ++) {	// Wait 500ms
		wdt_reset();
		_delay_ms(2);
	}
	usbDeviceConnect();

	PORTB &= ~_BV(PB1);
	sei();

    uint8_t state = STATE_WAIT;

	uint8_t lastState = !(PINB & _BV(PB3)), btnState;
	uint8_t timer_start = global_timer, timeout;

    char *bufPtr = NULL;

    while (1)
    {
		wdt_reset();
		usbPoll();

		btnState = !(PINB & _BV(PB3));
        if (state == STATE_WAIT && bufPtr == NULL) {
		    if (btnState != lastState) {
			    if (btnState) {
				    timer_start = global_timer;
			    }
			    else {
				    // Button is released
                    timeout = global_timer - timer_start;
                    if (timeout > 10 && timeout < 50) {
                        //state = STATE_SEND_KEY;
                    }
                    else if (timeout > 1 && timeout <= 10) {
                        ledIndex = (ledIndex+1) & 0x07;
                        PORTB &= ~_BV(PB1);  // LED off (if it was turned on by a re-gen
                		wdt_reset();
                        ws2812_setleds(&led[ledIndex], 1);
		                wdt_reset();
                    }
			    }
		    }
            else if (btnState) {
                timeout = global_timer - timer_start;
                if (timeout == 10 && ledIndex != 7) {
                    //eeprom_read_block(messageBuffer, stored_passwords[ledIndex], MSG_BUFFER_SIZE);
                    char *ptr = messageBuffer;
                    *ptr ++ = '0'+ledIndex;
                    for (int idx = 0 ; idx < MSG_BUFFER_SIZE ; idx ++) {
                        char eeChar = eeprom_read_byte((uint8_t *)&stored_passwords[ledIndex * MSG_BUFFER_SIZE + idx]);
                        //ptr = toHex(ptr, eeChar);
                        *ptr++ = eeChar;
                    }
                    *ptr ++ = '\n';
                    *ptr = 0;

                    wdt_reset();
                    bufPtr = messageBuffer;
                    state = STATE_SEND_KEY;
                }
                else if (timeout == 50 && ledIndex == 7) {
                    bufPtr = generateNewKeys();
                    if (bufPtr != NULL)
                        state = STATE_SEND_KEY;
                }
            }
        }
        else
            timer_start = global_timer;
		lastState = btnState;

		if (usbInterruptIsReady() && state != STATE_WAIT) {
			switch (state) {
				case STATE_SEND_KEY:
                    if (bufPtr != NULL) {
    					buildReport(*bufPtr);

                        bufPtr ++;
                        if (*bufPtr != 0) {
                            state = STATE_SEND_KEY;
                        }
                        else {
                            state = STATE_RELEASE_KEY;
                            bufPtr = NULL;
                        }

    					//state = STATE_RELEASE_KEY;
                    }
                    else
                        state = STATE_WAIT;
					break;

				case STATE_RELEASE_KEY:
					buildReport(0);
					state = STATE_WAIT;
                    if (bufPtr != NULL) {
                        bufPtr ++;
                        if (*bufPtr != 0)
                            state = STATE_SEND_KEY;
                        else
                            bufPtr = NULL;
                    }
					break;

				default:
					state = STATE_WAIT;
			}

			usbSetInterrupt((void*)&keyboard_report, sizeof(keyboard_report));
		}

    }
}

