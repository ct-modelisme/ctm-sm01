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

#include "AccessoryDecoder.h"


// The factory CV values for this decoder
CV factoryCVs [] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, 3},      // Less signifiant bits of long address 1
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},      // Most signifiant bits of long address 1
  {CV_ACCESSORY_DECODER_ADDRESS_2_LSB, 4},    // Less signifiant bits of long address 2
  {CV_ACCESSORY_DECODER_ADDRESS_2_MSB, 0},    // Most signifiant bits of long address 2
  {CV_ACCESSORY_DECODER_POS_A0_LSB, 80},      // Less signifiant bits of the first position of A servo
  {CV_ACCESSORY_DECODER_POS_A0_MSB, 0},       // Less signifiant bits of the first position of A servo
  {CV_ACCESSORY_DECODER_POS_A1_LSB, 120},     // Less signifiant bits of the end position of A servo
  {CV_ACCESSORY_DECODER_POS_A1_MSB, 0},       // Less signifiant bits of the end position of A servo
  {CV_ACCESSORY_DECODER_POS_B0_LSB, 80},      // Less signifiant bits of the first position of B servo
  {CV_ACCESSORY_DECODER_POS_B0_MSB, 0},       // Less signifiant bits of the first position of B servo
  {CV_ACCESSORY_DECODER_POS_B1_LSB, 120},     // Less signifiant bits of the end position of B servo
  {CV_ACCESSORY_DECODER_POS_B1_MSB, 0},       // Less signifiant bits of the end position of B servo
  {CV_ACCESSORY_DECODER_RELAY_PULSE, 75},     // The pulse duration to switch relays
  {CV_ACCESSORY_DECODER_SERVO_DELAY, 50},     // The delay between each servo step
  {CV_ACCESSORY_DECODER_STATE_A, 0},          // The state of A servo
  {CV_ACCESSORY_DECODER_STATE_B, 0},          // The state of B servo
};


/**
 * Generic constructor for AccessoryDecoder
 */
AccessoryDecoder::AccessoryDecoder() 
{
  this->dcc = DCCInterface::getInstance();
  this->dcc->registerFactoryCVs(*factoryCVs, sizeof(factoryCVs) / sizeof(CV));
  
  this->mode = AccessoryDecoder::RUNNING;
  this->posProgRequested = 0;
  this->waitForSecondAddress = 0;
  this->timerRunning = 0;
  
  // Read settings from decoder CVs
  this->dccAddressA = this->dcc->readIntFromCVs(CV_ACCESSORY_DECODER_ADDRESS_LSB, CV_ACCESSORY_DECODER_ADDRESS_MSB);
  this->dccAddressB = this->dcc->readIntFromCVs(CV_ACCESSORY_DECODER_ADDRESS_2_LSB, CV_ACCESSORY_DECODER_ADDRESS_2_MSB);
  this->posA0 = this->dcc->readIntFromCVs(CV_ACCESSORY_DECODER_POS_A0_LSB, CV_ACCESSORY_DECODER_POS_A0_MSB);
  this->posA1 = this->dcc->readIntFromCVs(CV_ACCESSORY_DECODER_POS_A1_LSB, CV_ACCESSORY_DECODER_POS_A1_MSB);
  this->posB0 = this->dcc->readIntFromCVs(CV_ACCESSORY_DECODER_POS_B0_LSB, CV_ACCESSORY_DECODER_POS_B0_MSB);
  this->posB1 = this->dcc->readIntFromCVs(CV_ACCESSORY_DECODER_POS_B1_LSB, CV_ACCESSORY_DECODER_POS_B1_MSB);
  this->stateA = this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_STATE_A);
  this->stateB = this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_STATE_B);

  // Apply saved states
  this->switchStateA(this->stateA);
  this->switchStateB(this->stateB);  
}


/**
 * The deecoder setup function
 */
void 
AccessoryDecoder::setup()
{
  // Set pin modes
  pinMode(MODE_LED_OUT, OUTPUT);
  pinMode(POS_A0_REL_OUT, OUTPUT);
  pinMode(POS_A1_REL_OUT, OUTPUT);
  pinMode(POS_B0_REL_OUT, OUTPUT);
  pinMode(POS_B1_REL_OUT, OUTPUT);
  pinMode(POS_A0_LED_OUT, OUTPUT);
  pinMode(POS_A1_LED_OUT, OUTPUT);
  pinMode(POS_B0_LED_OUT, OUTPUT);
  pinMode(POS_B1_LED_OUT, OUTPUT);
  pinMode(MODE_BTN_IN, INPUT);
  pinMode(A_BTN_IN, INPUT);
  pinMode(B_BTN_IN, INPUT);
  
  // Initialize servos
  this->servoA.attach(SERVO_A_OUT);
  this->servoB.attach(SERVO_B_OUT);



  Serial.println("-- Running mode --");
}


/**
 * The deecoder loop
 */
void 
AccessoryDecoder::loop()
{
  if(this->mode == RUNNING)
  {
    digitalWrite(MODE_LED_OUT, LOW);
    this->defineMode();

    if(digitalRead(A_BTN_IN))
    {
      this->switchStateA(! this->stateA);
      
      // Software anti-rebound
      do {
        delay(200);
      } while(digitalRead(A_BTN_IN));
    }
    
    if(digitalRead(B_BTN_IN))
    {
      this->switchStateB(!this->stateB);
      
      // Software anti-rebound
      do {
        delay(200);
      } while(digitalRead(B_BTN_IN));
    }
  }

  else if(this->mode == ADDR_PROG)
    this->addressProgrammingMode();
    
  else if(this->mode == POS_PROG)
    this->positionProgrammingMode();
    
  else if(this->mode == INFO)
    this->informationMode();
}



/**
 * Switch A to new state
 * 
 * @param state: The new state of A
 */
void
AccessoryDecoder::switchStateA(byte state)
{
  Serial.print("Switching A to state ");
  Serial.print(state);
  Serial.println(".");

  digitalWrite(POS_A0_LED_OUT, LOW);
  digitalWrite(POS_A1_LED_OUT, LOW);
  digitalWrite(POS_A0_REL_OUT, LOW);
  digitalWrite(POS_A1_REL_OUT, LOW);

  if(state)
  {
    int delta = (this->posA1 - this->servoA.read()) / 2;
    for(int i = this->servoA.read(); i < this->posA1 - delta; i++)
    {
      this->servoA.write(i);
      delay(this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_SERVO_DELAY));
    }

    digitalWrite(POS_A1_REL_OUT, HIGH);
    delay(this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_RELAY_PULSE));
    digitalWrite(POS_A1_REL_OUT, LOW);

    for(int i = this->servoA.read(); i < this->posA1; i++)
    {
      this->servoA.write(i);
      delay(this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_SERVO_DELAY));
    }
  }
  else
  {
    int delta = (this->servoA.read() - this->posA0) / 2;
    for(int i = this->servoA.read(); i > this->posA0 + delta; i--)
    {
      this->servoA.write(i);
      delay(this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_SERVO_DELAY));
    }

    digitalWrite(POS_A0_REL_OUT, HIGH);
    delay(this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_RELAY_PULSE));
    digitalWrite(POS_A0_REL_OUT, LOW);
    
    for(int i = this->servoA.read(); i > this->posA0; i--)
    {
      this->servoA.write(i);
      delay(this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_SERVO_DELAY));
    }
  }

  digitalWrite(POS_A0_LED_OUT, !state);
  digitalWrite(POS_A1_LED_OUT, state);

  this->stateA = state;
  delay(this->dcc->nmraDCC.setCV(CV_ACCESSORY_DECODER_STATE_A, state));
}



/**
 * Switch B to new state
 * 
 * @param state: The new state of B
 */
void
AccessoryDecoder::switchStateB(byte state)
{
  Serial.print("Switching B to state ");
  Serial.print(state);
  Serial.println(".");

  digitalWrite(POS_B0_LED_OUT, LOW);
  digitalWrite(POS_B1_LED_OUT, LOW);
  digitalWrite(POS_B0_REL_OUT, LOW);
  digitalWrite(POS_B1_REL_OUT, LOW);

  if(state)
  {
    int delta = (this->posB1 - this->servoB.read()) / 2;
    for(int i = this->servoB.read(); i < this->posB1 - delta; i++)
    {
      this->servoB.write(i);
      delay(this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_SERVO_DELAY));
    }

    digitalWrite(POS_B1_REL_OUT, HIGH);
    delay(this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_RELAY_PULSE));
    digitalWrite(POS_B1_REL_OUT, LOW);
    
    for(int i = this->servoB.read(); i < this->posB1; i++)
    {
      this->servoB.write(i);
      delay(this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_SERVO_DELAY));
    }
  }
  else
  {
    int delta = (this->servoB.read() - this->posB0) / 2;
    for(int i = this->servoB.read(); i > this->posB0 + delta; i--)
    {
      this->servoB.write(i);
      delay(this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_SERVO_DELAY));
    }

    digitalWrite(POS_B0_REL_OUT, HIGH);
    delay(this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_RELAY_PULSE));
    digitalWrite(POS_B0_REL_OUT, LOW);
    
    for(int i = this->servoB.read(); i > this->posB0; i--)
    {
      this->servoB.write(i);
      delay(this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_SERVO_DELAY));
    }
  }

  digitalWrite(POS_B0_LED_OUT, !state);
  digitalWrite(POS_B1_LED_OUT, state);

  this->stateB = state;
  this->dcc->nmraDCC.setCV(CV_ACCESSORY_DECODER_STATE_B, state);
}



/**
 * Analyse buttons pressed or not and define the current mode
 */
void 
AccessoryDecoder::defineMode()
{
  if(this->mode == RUNNING)
  {
    int modeBtnValue = digitalRead(MODE_BTN_IN);
  
    // Start pressing mode button
    if(!this->timerRunning && modeBtnValue)
    {
      this->timerStartTime = millis();
      this->timerRunning = 1;
    }
    
    // Continue pressing mode button
    else if(this->timerRunning && modeBtnValue)
    {
      if(digitalRead(A_BTN_IN) or digitalRead(B_BTN_IN))
        this->posProgRequested = 1;
    }
    
    // Stop pressing mode button
    else if(this->timerRunning && !modeBtnValue)
    {
      this->timerEndTime = millis();
      this->timerRunning = 0;
      
      unsigned long duration = timerEndTime - timerStartTime;
  
      if(duration < 1000)
      {
        Serial.println("-- Information mode --");
        this->mode = INFO;
      }
      else if(posProgRequested)
      {
        Serial.println("-- Position programming mode --");
        this->mode = POS_PROG;
      }
      else
      {
        Serial.println("-- DCC Address programming mode --");
        this->mode = ADDR_PROG;
      }
  
      this->posProgRequested = 0;
    }
  }
}



/**
 * Information mode
 * Display settings on serial console.
 */
void 
AccessoryDecoder::informationMode()
{
  int modeBtnValue;
  
  Serial.print("DCC address A: ");
  Serial.println(this->dccAddressA);
  Serial.print("DCC address B: ");
  Serial.println(this->dccAddressB);

  delay(200);
  do {
    modeBtnValue = digitalRead(MODE_BTN_IN);
  } while(modeBtnValue);
  
  // Software anti-rebound
  do {
    delay(200);
  } while(!digitalRead(MODE_BTN_IN));

  Serial.print("First position of A: ");
  Serial.print(this->posA0);
  Serial.println("°");
  Serial.print("Last position of A: ");
  Serial.print(this->posA1);
  Serial.println("°");

  delay(200);
  do {
    modeBtnValue = digitalRead(MODE_BTN_IN);
  } while(modeBtnValue);
  
  // Software anti-rebound
  do {
    delay(200);
  } while(!digitalRead(MODE_BTN_IN));

  Serial.print("First position of B: ");
  Serial.print(this->posB0);
  Serial.println("°");
  Serial.print("Last position of B: ");
  Serial.print(this->posB1);
  Serial.println("°");

  delay(200);
  do {
    modeBtnValue = digitalRead(MODE_BTN_IN);
  } while(modeBtnValue);
  
  // Software anti-rebound
  do {
    delay(200);
  } while(!digitalRead(MODE_BTN_IN));

  Serial.print("Servo steps delay: ");
  Serial.print(this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_SERVO_DELAY));
  Serial.println("ms");
  Serial.print("Relay pulse duration: ");
  Serial.print(this->dcc->nmraDCC.getCV(CV_ACCESSORY_DECODER_RELAY_PULSE));
  Serial.println("ms");  

  delay(200);
  do {
    modeBtnValue = digitalRead(MODE_BTN_IN);
  } while(modeBtnValue);
  
  // Software anti-rebound
  do {
    delay(200);
  } while(!digitalRead(MODE_BTN_IN));

  Serial.print("Firmware version: v");
  Serial.println(FIRMWARE);
  Serial.println("CTM-SM01");

  delay(200);
  do {
    modeBtnValue = digitalRead(MODE_BTN_IN);
  } while(modeBtnValue);

  Serial.println("Back to running mode.\n");
  Serial.println("-- Running mode --");
  this->mode = RUNNING;
  
  // Software anti-rebound
  do {
    delay(200);
  } while(digitalRead(MODE_BTN_IN));
}



/**
 * DCC address programmation mode
 * Learn the first address received on DCC frames as it first address
 */
void 
AccessoryDecoder::addressProgrammingMode()
{
  int modeBtnValue;
  digitalWrite(MODE_LED_OUT, HIGH);

  // Learning done in DCC handler...
  
  if(digitalRead(MODE_BTN_IN))
  {
    Serial.println("Aborted! Leaving programming mode.\n");
    Serial.println("-- Running mode --");
    this->mode = RUNNING;

    // Software anti-rebound
    do {
      delay(200);
    } while(digitalRead(MODE_BTN_IN));
  }
}



/**
 * Servo motor positions programmation mode
 * Each servo (A & B) as a start and end position (in deg).
 */
void 
AccessoryDecoder::positionProgrammingMode()
{
  int modeBtnValue;
  int aBtnValue;
  int bBtnValue;

  digitalWrite(MODE_LED_OUT, HIGH);
  digitalWrite(POS_A0_LED_OUT, LOW);
  digitalWrite(POS_A1_LED_OUT, LOW);
  digitalWrite(POS_B0_LED_OUT, LOW);
  digitalWrite(POS_B1_LED_OUT, LOW);

  // A0
  Serial.print("First position of A: ");
  Serial.print(this->posA0);
  Serial.println("°");
  this->switchStateA(0);

  delay(250);
  do {
    blinkLed(POS_A0_LED_OUT);
    
    aBtnValue = digitalRead(A_BTN_IN);
    bBtnValue = digitalRead(B_BTN_IN);
    
    if(aBtnValue)
    {
      this->posA0 = max(0, this->posA0 - 2);
      Serial.print(this->posA0);
      Serial.println("°");
      this->servoA.write(this->posA0);
    }
    else if(bBtnValue)
    {
      this->posA0 = min(180, this->posA0 + 2);
      Serial.print(this->posA0);
      Serial.println("°");
      this->servoA.write(this->posA0);
    }
    
    modeBtnValue = digitalRead(MODE_BTN_IN);
  } while(!modeBtnValue);

  // Software anti-rebound
  do {
    delay(200);
  } while(digitalRead(MODE_BTN_IN));

  // A1
  Serial.print("Last position of A: ");
  Serial.print(this->posA1);
  Serial.println("°");
  this->switchStateA(1);

  delay(250);
  do {
    blinkLed(POS_A1_LED_OUT);

    aBtnValue = digitalRead(A_BTN_IN);
    bBtnValue = digitalRead(B_BTN_IN);
    
    if(aBtnValue)
    {
      this->posA1 = max(0, this->posA1 - 2);
      Serial.print(this->posA1);
      Serial.println("°");
      this->servoA.write(this->posA1);
    }
    else if(bBtnValue)
    {
      this->posA1 = min(180, this->posA1 + 2);
      Serial.print(this->posA1);
      Serial.println("°");
      this->servoA.write(this->posA1);
    }
    
    modeBtnValue = digitalRead(MODE_BTN_IN);
  } while(!modeBtnValue);

  // Software anti-rebound
  do {
    delay(200);
  } while(digitalRead(MODE_BTN_IN));

  // B0
  Serial.print("First position of B: ");
  Serial.print(this->posB0);
  Serial.println("°");
  this->switchStateB(0);

  delay(250);
  do {
    blinkLed(POS_B0_LED_OUT);
    
    aBtnValue = digitalRead(A_BTN_IN);
    bBtnValue = digitalRead(B_BTN_IN);
    
    if(aBtnValue)
    {
      this->posB0 = max(0, this->posB0 - 2);
      Serial.print(this->posB0);
      Serial.println("°");
      this->servoB.write(this->posB0);
    }
    else if(bBtnValue)
    {
      this->posB0 = min(180, this->posB0 + 2);
      Serial.print(this->posB0);
      Serial.println("°");
      this->servoB.write(this->posB0);
    }
    
    modeBtnValue = digitalRead(MODE_BTN_IN);
  } while(!modeBtnValue);

  // Software anti-rebound
  do {
    delay(200);
  } while(digitalRead(MODE_BTN_IN));

  // B1
  Serial.print("Last position of B: ");
  Serial.print(this->posB1);
  Serial.println("°");
  this->switchStateB(1);

  delay(250);
  do {
    blinkLed(POS_B1_LED_OUT);

    aBtnValue = digitalRead(A_BTN_IN);
    bBtnValue = digitalRead(B_BTN_IN);
    
    if(aBtnValue)
    {
      this->posB1 = max(0, this->posB1 - 2);
      Serial.print(this->posB1);
      Serial.println("°");
      this->servoB.write(this->posB1);
    }
    else if(bBtnValue)
    {
      this->posB1 = min(180, this->posB1 + 2);
      Serial.print(this->posB1);
      Serial.println("°");
      this->servoB.write(this->posB1);
    }
    
    modeBtnValue = digitalRead(MODE_BTN_IN);
  } while(!modeBtnValue);
  
  Serial.println("Saving settings...");
  this->dcc->writeIntToCVs(CV_ACCESSORY_DECODER_POS_A0_LSB, CV_ACCESSORY_DECODER_POS_A0_MSB, this->posA0);
  this->dcc->writeIntToCVs(CV_ACCESSORY_DECODER_POS_A1_LSB, CV_ACCESSORY_DECODER_POS_A1_MSB, this->posA1);
  this->dcc->writeIntToCVs(CV_ACCESSORY_DECODER_POS_B0_LSB, CV_ACCESSORY_DECODER_POS_B0_MSB, this->posB0);
  this->dcc->writeIntToCVs(CV_ACCESSORY_DECODER_POS_B1_LSB, CV_ACCESSORY_DECODER_POS_B1_MSB, this->posB1);

  Serial.println("Back to running mode.\n");
  Serial.println("-- Running mode --");
  this->switchStateA(this->stateA);
  this->switchStateB(this->stateB);
  digitalWrite(POS_A0_LED_OUT, !this->stateA);
  digitalWrite(POS_A1_LED_OUT, this->stateA);
  digitalWrite(POS_B0_LED_OUT, !this->stateB);
  digitalWrite(POS_B1_LED_OUT, this->stateB);
  this->mode = RUNNING;

  // Software anti-rebound
  do {
    delay(200);
  } while(digitalRead(MODE_BTN_IN));
}



/**
 * Blink a given LED
 * 
 * @param led: The LED pin
 * @param nb: The number of flashes (default to 1)
 * @param freq: The time between flashes in ms (default to 200)
 */
void 
AccessoryDecoder::blinkLed(int led, int nb, int freq)
{
  for(int i=0; i<nb; i++)
  {
    delay(freq);
    digitalWrite(led, HIGH);
    delay(freq);
    digitalWrite(led, LOW);
  }
}



/** 
 * DCC turnout packet handler
 * 
 * @param addr: The DCC address
 * @param direction: The requested state
 * @param outputPower: The output state
 */
void notifyDccAccTurnoutOutput(uint16_t addr, uint8_t direction, uint8_t outputPower)
{
  DCCInterface *dcc = DCCInterface::getInstance();
  AccessoryDecoder *decoder = AccessoryDecoder::getInstance();
  
  // Learning mode
  if(decoder->mode == AccessoryDecoder::ADDR_PROG)
  {
    if(! decoder->waitForSecondAddress)
    {
      decoder->dccAddressA = addr;
      dcc->writeIntToCVs(CV_ACCESSORY_DECODER_ADDRESS_LSB, CV_ACCESSORY_DECODER_ADDRESS_MSB, addr);
      Serial.print("New address learned for A: ");
      Serial.println(addr, DEC);

      decoder->waitForSecondAddress = 1;
      delay(350);
    }
    else
    {
      decoder->dccAddressB = addr;
      dcc->writeIntToCVs(CV_ACCESSORY_DECODER_ADDRESS_2_LSB, CV_ACCESSORY_DECODER_ADDRESS_2_MSB, addr);
      Serial.print("New address learned for B: ");
      Serial.println(addr, DEC);
      Serial.println("Back to running mode.\n");
      
      decoder->waitForSecondAddress = 0;
      decoder->mode = AccessoryDecoder::RUNNING;
    }
  }
  
  // Normal mode
  else if(decoder->mode == AccessoryDecoder::RUNNING)
  {
    if(addr == decoder->dccAddressA)
      decoder->switchStateA(direction);
    else if(addr == decoder->dccAddressB)
      decoder->switchStateB(direction);
  }
}
