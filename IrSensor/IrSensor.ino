/**
   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters. Each
   repeater and gateway builds a routing tables in EEPROM which keeps track of the
   network topology allowing messages to be routed to nodes.

   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2015 Sensnology AB
   Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors

   Documentation: http://www.mysensors.org
   Support Forum: http://forum.mysensors.org

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.

 *******************************

   REVISION HISTORY
   Version 1.0 - Henrik EKblad
   Version 1.1 - rknipp and rejoe2
   Version 1.1.1 - rknipp

   DESCRIPTION
   Example sketch showing how to control ir devices
   An IR LED must be connected to Arduino PWM pin 3.
   An optional ir receiver can be connected to PWM pin 8.
   All received ir signals from PIN 8 will be sent to gateway device stored in IR_RECEIVED.
   NEW: The Node expects real IR commands to process via the IR_SEND variable (instead of "on/off" via V_LIGHT).
   Variable format to be sent form controller side is: "protocol code irbits", eg. "1 0x1EE17887 32"
   for Vol up yamaha ysp-900 (assuming NEC-codes are mapped to protocol #1 in your IRLib).

   IMPORTANT:
   The IRLib used here is Gabriel Staples version available at https://github.com/ElectricRCAircraftGuy/IRLib,
   so IR commands are different from the standard IRLib delivered with MySensors!
   http://www.mysensors.org/build/ir
*/

// Enable debug prints
//#define MY_DEBUG
//#define USE_DUMP //should print out lots of info to serial (ElectricRCAircraftGuy feature?)

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#include <SPI.h>
#include <MySensor.h>
#include <IRLib.h> //Gabriel Staples version!


int RECV_PIN = 8;

#define CHILD_ID_IR  1  // childId

IRsend irsend;
IRrecv My_Receiver(RECV_PIN);
IRdecode My_Decoder;
MyMessage msgIr(CHILD_ID_IR, V_IR_RECEIVE);

void setup()
{
  // My_Decoder.useDoubleBuffer(Buffer); //uncomment to use; requires the "extraBuffer" to be uncommented above
  //Try different values here for Mark_Excess. 50us is a good starting guess. See detailed notes above for more info.
  My_Receiver.Mark_Excess = 50; //us; mark/space correction factor
  //Optional: set LED to blink while IR codes are being received
  // My_Receiver.blink13(true); //blinks whichever LED is connected to LED_BUILTIN on your board, usually pin 13
  //                            //-see here for info on LED_BUILTIN: https://www.arduino.cc/en/Reference/Constants
  My_Receiver.setBlinkLED(13, true); //same as above, except you can change the LED pin number if you like
  My_Receiver.enableIRIn(); // Start the receiver

}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("IR Sensor", "1.1.1");

  // Register the sensor als IR Node
  present(CHILD_ID_IR, S_IR);
}

void loop()
{
  if (My_Receiver.getResults(&My_Decoder)) //if IR data is ready to be decoded
  {
    //1) decode it
    My_Decoder.decode();
    Serial.println("decoding");
    Serial.print("Protocol:");
    Serial.print(Pnames(My_Decoder.decode_type));
    Serial.print(",");
    Serial.print(My_Decoder.decode_type);
    Serial.print(" ");
    Serial.print(My_Decoder.value, HEX);
    Serial.print(" ");
    Serial.println(My_Decoder.bits);
    
    //FOR EXTENSIVE OUTPUT:
#ifdef USE_DUMP
    My_Decoder.dumpResults();
#endif

    //filter out zeros for not recognized codes and NEC repeats
    const char rec_value = My_Decoder.value;
    if (rec_value != 0xffffffff && rec_value != 0x0) {
      char buffer[24];
      uint8_t IrBits = My_Decoder.bits;
      String IRType_string = Pnames(My_Decoder.decode_type);
      char IRType[IRType_string.length() + 1];
      IRType_string.toCharArray(IRType, IRType_string.length() + 1);
      sprintf(buffer, "%i 0x%08lX %i, %s", My_Decoder.decode_type, My_Decoder.value, IrBits, IRType);
      // Send ir result to gw
      send(msgIr.set(buffer));
      
      //somedebugging output
#ifdef MY_DEBUG
      //2) print results
      //FOR BASIC OUTPUT ONLY:
      Serial.println(buffer);
#endif
    }
    //3) resume receiver (ONLY CALL THIS FUNCTION IF SINGLE-BUFFERED); comment this line out if double-buffered
    /*for single buffer use; do NOT resume until AFTER done calling all decoder
      functions that use the last data acquired, such as decode and dumpResults; if using a double
      buffer, don't use resume() at all unless you called My_Receiver.detachInterrupt() previously
      to completely stop receiving, and now want to resume IR receiving.*/
    My_Receiver.resume();

  }
}

void receive(const MyMessage &message) {
    const char *irData;
    // Complete send command from controller side is needed, e.g. "1 0x1EE17887 32"
    if (message.type == V_IR_SEND) {
    irData = message.getString();
    
    //some debugging output
#ifdef MY_DEBUG
    Serial.println(F("Received IR send command..."));
    Serial.println(irData);
#endif

    int i = 0;
    char* arg[3]; //pointer array to save the three arguments
    unsigned char protocol;
    unsigned long code;
    unsigned int bits;
   
    //copy irData to a temporary variable because strtok() is changing the string its working on.
    char* irString = strdup(irData);
    
    //seperating the String into the three arguments.
    char* token = strtok(irString, " ");
    while (token != NULL) {
      arg[i] = token;
      token = strtok(NULL, " ");
      i++;
    }
    
    //some debugging output
#ifdef MY_DEBUG
    Serial.print("Protocol:"); Serial.print(arg[0]);
    Serial.print(" Code:"); Serial.print(arg[1]);
    Serial.print(" Bits:"); Serial.println(arg[2]);
#endif

    protocol = atoi(arg[0]);
    code = strtoul(arg[1], NULL, 0);
    bits = atoi(arg[2]);
    irsend.send(protocol, code, bits);
    free(irString);
  }

  // Start receiving ir again...
  My_Receiver.resume();
}
