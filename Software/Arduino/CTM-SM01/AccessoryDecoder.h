/**
 * CT-Modelisme (ct-modelisme.fr) - Decodeur Servo-Moteur 01 (CTM-SM01.ino)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 * 
 * 
 * Cyrille TOULET <cyrille.toulet@linux.com>
 * Thu 22 Jul 10:20:42 CEST 2021
 */

#ifndef ACCESSORY_DECODER_H
#define ACCESSORY_DECODER_H

// Include libraries
#include "Arduino.h"                                // Arduino library
#include <Servo.h>                                  // Servo library
#include "DCCInterface.h"                           // DCC interface


// General purpose constants
#define FIRMWARE "1.2"                              // The decoder firmware release


// Arduino pins definition
#define POS_A0_REL_OUT 3                            // The A0 relay pin
#define POS_A1_REL_OUT 4                            // The A1 relay pin
#define POS_B0_REL_OUT 5                            // The B0 relay pin
#define POS_B1_REL_OUT 6                            // The B1 relay pin

#define MODE_BTN_IN 7                               // The mode button
#define A_BTN_IN 8                                  // The pos A button
#define B_BTN_IN 9                                  // The pos B button

#define SERVO_A_OUT 10                              // The servo A pin (pwm)
#define SERVO_B_OUT 11                              // The servo B pin (pwm)

#define POS_A0_LED_OUT A0                           // The A0 LED pin
#define POS_A1_LED_OUT A1                           // The A1 LED pin
#define POS_B0_LED_OUT A2                           // The B0 LED pin
#define POS_B1_LED_OUT A3                           // The B1 LED pin
#define MODE_LED_OUT LED_BUILTIN                    // The mode LED pin


// Factory CVs definition (CVs 47-64 & CVs 112-256 reserved for manufacturers)
#define CV_ACCESSORY_DECODER_ADDRESS_2_LSB 47       // DCC address 2 CVs
#define CV_ACCESSORY_DECODER_ADDRESS_2_MSB 48
#define CV_ACCESSORY_DECODER_POS_A0_LSB    49       // First position of A CVs
#define CV_ACCESSORY_DECODER_POS_A0_MSB    50
#define CV_ACCESSORY_DECODER_POS_A1_LSB    51       // Last position of A CVs
#define CV_ACCESSORY_DECODER_POS_A1_MSB    52
#define CV_ACCESSORY_DECODER_POS_B0_LSB    53       // First position of B CVs
#define CV_ACCESSORY_DECODER_POS_B0_MSB    54
#define CV_ACCESSORY_DECODER_POS_B1_LSB    55       // Last position of B CVs
#define CV_ACCESSORY_DECODER_POS_B1_MSB    56
#define CV_ACCESSORY_DECODER_RELAY_PULSE   57       // Relay pulse duration CV
#define CV_ACCESSORY_DECODER_SERVO_DELAY   58       // Servo wait delay CV
#define CV_ACCESSORY_DECODER_STATE_A       112      // State A CV
#define CV_ACCESSORY_DECODER_STATE_B       113      // State B CV


/**
 * Accessory decoder class
 */
class AccessoryDecoder
{
  private:
  
    // Private attributes
    DCCInterface *dcc;                              // The dcc object
    Servo servoA;                                   // The first servo object
    Servo servoB;                                   // The second servo object
    
    unsigned long timerStartTime;                   // Used for timers
    unsigned long timerEndTime;                     // Used for timers
    byte timerRunning;                              // Used for timers
    
    byte posProgRequested;                          // Used in pos programming mode
    int posA0;                                      // First position of A
    int posA1;                                      // Last position of A
    int posB0;                                      // First position of B
    int posB1;                                      // Last position of B
    
    byte stateA;                                    // State of A
    byte stateB;                                    // State of B

    // Private methods
    AccessoryDecoder();                             // Singleton pattern
    AccessoryDecoder(AccessoryDecoder const&);      // Disable object copy
    void operator = (AccessoryDecoder const&);      // Disable assigment
    
    void blinkLed(int led, int nb=1, int freq=200); // Blink a led
    
    void defineMode();                              // Define the decoder mode
    void informationMode();                         // The information mode (Serial)
    void addressProgrammingMode();                  // The address programming mode
    void positionProgrammingMode();                 // The servo positions programming mode


  public:
  
    // Public types
    enum MODES
    {
      RUNNING,
      INFO,
      ADDR_PROG,
      POS_PROG,
    };
    
    // Public attributes
    int dccAddressA;                                // The first DCC address
    int dccAddressB;                                // The second DCC address
    enum MODES mode;                                // The current decoder mode
    byte waitForSecondAddress;                      // Used in addr programming mode
    
    // Public methods
    static AccessoryDecoder* getInstance()
    {                                               // Singleton getter
        static AccessoryDecoder decoder;
        return &decoder;
    };

    void setup();                                   // Setup decoder
    void loop();                                    // Run decoder operations
    void switchStateA(byte state);                  // Switch state of servo A
    void switchStateB(byte state);                  // Switch state of servo B
};


void notifyDccAccTurnoutOutput(uint16_t addr, 
  uint8_t direction, uint8_t outputPower);          // The DCC accessory packet handler

#endif
