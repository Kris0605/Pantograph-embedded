/*******************************************************************************
Copyright 2016 Microchip Technology Inc. (www.microchip.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

To request to license the code under the MLA license (www.microchip.com/mla_license), 
please contact mla_licensing@microchip.com
 *******************************************************************************/

/** INCLUDES *******************************************************/
#include "system.h"

#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "usb.h"

#include "app_device_cdc_basic.h"
#include "usb_config.h"
#include "umogi2.h"

#include "mechanics.h"
#include "HX711.h"

/** VARIABLES ******************************************************/

static uint8_t readBuffer[CDC_DATA_OUT_EP_SIZE];
static uint8_t writeBuffer[CDC_DATA_IN_EP_SIZE];
uint8_t numBytes;

/*********************************************************************
 * Function: void APP_DeviceCDCBasicDemoInitialize(void);
 *
 * Overview: Initializes the demo code
 *
 * PreCondition: None
 *
 * Input: None
 *
 * Output: None
 *
 ********************************************************************/
void APP_DeviceCDCBasicDemoInitialize() {
    line_coding.bCharFormat = 0;
    line_coding.bDataBits = 8;
    line_coding.bParityType = 0;
    line_coding.dwDTERate = 38400;
}

/*********************************************************************
 * Function: void APP_DeviceCDCBasicDemoTasks(void);
 *
 * Overview: Keeps the demo running.
 *
 * PreCondition: The demo should have been initialized and started via
 *   the APP_DeviceCDCBasicDemoInitialize() and APP_DeviceCDCBasicDemoStart() demos
 *   respectively.
 *
 * Input: None
 *
 * Output: None
 *
 ********************************************************************/
void APP_DeviceCDCBasicDemoTasks() {
    /* If the USB device isn't configured yet, we can't really do anything
     * else since we don't have a host to talk to.  So jump back to the
     * top of the while loop. */
    if (USBGetDeviceState() < CONFIGURED_STATE) {
        return;
    }

    /* If we are currently suspended, then we need to see if we need to
     * issue a remote wakeup.  In either case, we shouldn't process any
     * keyboard commands since we aren't currently communicating to the host
     * thus just continue back to the start of the while loop. */
    if (USBIsDeviceSuspended() == true) {
        return;
    }

    if (USBUSARTIsTxTrfReady() == true) {

        numBytes = getsUSBUSART(readBuffer, sizeof (readBuffer)); //until the buffer is free.
        if (numBytes > 0) {
            if (readBuffer[0] == 48) { /*char '0' in int, wait for Simulation*/
                SimSelect = 0;
                OC1R = 0;
                OC2R = 0;
                sprintf(lcd, "HAPTIC          ");
                sprintf(lcd + lcd_cpl, "PANTOGRAPH      ");
                lcd_update();
            } else if (readBuffer[0] == 49) { /*char '1' in int, Billiard*/
                SimSelect = 1;
                wl = -0.125f;
                wr = 0.125f;
                wt = 0.080f;
                wb = -0.080f;
                setTargetPoint(0, 0);
                sprintf(lcd, "Billiard        ");
                sprintf(lcd + lcd_cpl, "                ");
                lcd_update();
            } else if (readBuffer[0] == 50) { /*char '2' in int, Air hockey*/
                SimSelect = 2;
                wl = -0.120f;
                wr = 0.120f;
                wt = 0.377f;
                wb = -0.077f;
                setTargetPoint(0, 0);

                sprintf(lcd, "Air Hockey      ");
                sprintf(lcd + lcd_cpl, "0:0             ");
                lcd_update();
            } else if (readBuffer[0] == 51) { /*char '3' in int, Spinner*/
                SimSelect = 3;
                setTargetPoint(0.06f, 0);

                sprintf(lcd, "Spinner         ");
                sprintf(lcd + lcd_cpl, "                ");
                lcd_update();
            } else if (readBuffer[0] == 52) { /*char '4' in int, Labirinth*/
                SimSelect = 4;
                wl = -0.143f;
                wr = 0.143f;
                wt = 0.079f;
                wb = -0.079f;
                setTargetPoint(-0.128f, 0.064f);

                sprintf(lcd, "Labyrinth       ");
                sprintf(lcd + lcd_cpl, "                ");
                lcd_update();
            }else if (readBuffer[0] == 53) { /*char '5' in int, Coffee*/
                SimSelect = 5;
                setTargetPoint(0.06f, 0);

                sprintf(lcd, "Coffee          ");
                sprintf(lcd + lcd_cpl, "                ");
                lcd_update();
            } else if (readBuffer[0] == 99) { /*char 'c' in int, calibrate*/
                enc1_counter = 0;
                enc2_counter = 0;
            }
            reset_simulation();
        }
        if (SimSelect == 3 || SimSelect == 5) { /*Spinner, Coffee*/
            if (loadTick % 12 == 0) {
                sprintf(writeBuffer, "e;%.4f;%.4f;s;%.4f;%.4f;%.4f\n", e.x, e.y, s.alfa, Fx, Fy);
                loadTick = 0;
            } else {
                //sprintf(writeBuffer, "\n");
                sprintf(writeBuffer, "e;%.4f;%.4f;s;%.4f;%.4f\n", e.x, e.y, s.alfa);
            }
        } else if(SimSelect == 2) {
            if (loadTick % 12 == 0) {
                sprintf(writeBuffer, "e;%.4f;%.4f;b;%.4f;%.4f;o;%.4f;%.4f;%.4f;%.4f\n", e.x, e.y, b.x, b.y, o.x, o.y, Fx, Fy);
                loadTick = 0;
            } else {
                //sprintf(writeBuffer, "\n");
                sprintf(writeBuffer, "e;%.4f;%.4f;b;%.4f;%.4f;o;%.4f;%.4f\n", e.x, e.y, b.x, b.y, o.x, o.y);
            }
        }else {
            if (loadTick % 12 == 0) {
                sprintf(writeBuffer, "e;%.4f;%.4f;b;%.4f;%.4f;%.4f;%.4f\n", e.x, e.y, b.x, b.y, Fx, Fy);
                loadTick = 0;
            } else {
                //sprintf(writeBuffer, "\n");
                sprintf(writeBuffer, "e;%.4f;%.4f;b;%.4f;%.4f\n", e.x, e.y, b.x, b.y);
            }
        }
        loadTick++;
        //sprintf(writeBuffer,"e;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;b;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f\n", e.x,e.y,e.vx,e.vy,e.fx,e.fy,b.x,b.y,b.vx,b.vy,b.fx,b.fy);
        putUSBUSART(writeBuffer, strlen((char *) writeBuffer));
    }
    CDCTxService();
}