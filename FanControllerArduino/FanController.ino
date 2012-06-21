//#include <pnew.cpp> //Before arduino 1.01, this include was needed to use Andy Brown's ported STL library
//Many thanks to him for porting the iterator and algorithm header files to Arduino!
#include <stdio.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <iterator>
#include <algorithm>
/*
 Serial governed PC fan controller
 
 This script will control one or more PC fans plugged into its pwm enabled ports by setting them to the speeds fed to the controller via serial. 
 These speeds are calculated from a simple bash script run every minute on the host computer from CPU and HDD integrated temp sensors.
 This allows for the overall control code to be easily changed on the host computer without forcing the user to touch the arduino and its code.
 Additionally, this method does not require the use of additional sensors to monitor the temperature of each individual HDD or CPU being actively cooled by the arduino.
 --The computer (using superuser privleges) can access the realtime temp of each HDD using the HDD's SMART data.
 Although this host-centric method of monitoring is convenient, it also opens up more point of failure in the controller system. What if the host stops responding and fails to monitor the HDDs? 
 Since HDDs are rated up to, and can surpass 40C without very much danger (besides a drastically degraded lifespan), we can somewhat mitigate this problem by instituting heartbeats and a conservative (fast) fallback fanspeed.
 If the host fails to check in once every minute, the arduino will bump up the fan speed to the max until the connection is restablished.
 
 The controller is hooked up to the 12V rail of the PC's power supply and therefore will turn on when the computer is booted, then turn off on halt.
 
 This arduino script will take wake the arduino on serial input, read the packet, then set the fans to the required speed, then sleep again.
 
 COMMAND PACKET:
 Header:
 1st byte: 0xFF
 2nd byte: 0xFF
 Packet:
 3rd byte: Packet length
 4th byte: Command identifier
 All other bytes: Data
 
 Comand Identifiers can be:
 Utilities:
 0x0 Nothing, default packet status. Should be ignored as no packet received
 
 0x1 Heartbeat: Keep all fans at their last refreshed settings 
 Fan controls:
 0x10 Set fan[0] pwm to payload
 0x11 Set fan[1] pwm to payload
 [...]
 
 
 */

//Constants:
const uint8_t NUMBER_OF_FANS = 1;

const uint8_t RESERVED_PACKET_LENGTH = 3;
const uint8_t HEARTBEAT_PACKET_LENGTH = 1;
const uint8_t FAN_PACKET_LENGTH = 2;
const uint8_t NUM_HEADER_BYTES = 2;

const uint8_t COMMANDS[] = {
  0x1, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15};
const uint8_t COMMAND_LENGTHS[] = {
  HEARTBEAT_PACKET_LENGTH, FAN_PACKET_LENGTH, FAN_PACKET_LENGTH, FAN_PACKET_LENGTH, FAN_PACKET_LENGTH, FAN_PACKET_LENGTH, FAN_PACKET_LENGTH};

//Communication timeout vars
//IMPORTANT, DOES MILLIS() STILL WORK IN SLEEP_MODE_PWR_SAVE MODE?
//If so, we can just compare the times
uint32_t lastMillis = 0;

const uint16_t timeoutSetting = 1000*60*2; //Timout is set to 2 minutes
//If not, we can set a interrupt timer and increment and check a overflow varible
uint16_t timeoutOverflowCounter = 0;

const uint8_t timeoutTimerSetting; //What number should this be set to? This can be either a 1 or 2 byte number depending whether we can use timer 0, or timer 1
const uint8_t timeoutOverflowSetting = 128;

//Pins
const uint8_t serialWakePin = 2;
const uint8_t FAN_PINS[] = {
  3, 5, 6, 9, 10, 11};
const uint8_t FAN_PIN_ARRAY_OFFSET = 1;

//Packet varibles
uint8_t currentPacket[RESERVED_PACKET_LENGTH];
boolean newPacket = false;

const uint8_t HEADER_BYTE = 0xFF;
const uint8_t LENGTH_POS = 0;
const uint8_t COMMAND_BYTE_POS = 1;
const uint8_t PAYLOAD_BYTE_POS = 2;


void setup() {
  //Init wakeup pins
  pinMode(serialWakePin, INPUT);

  //Init fan pins
  for (int i = 0; i < NUMBER_OF_FANS; i++){
    pinMode(FAN_PINS[i], OUTPUT);
  }

  //Zero out packet varible
  cleanPacket();

  //Enable power saving modes
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();

  // initialize serial:
  Serial.begin(9600);

  Serial.println("Hello World!");
  //Kickstart fans into motion by setting them to HIGH for a moment.

  delay(10);

}

void loop() {
  //Sleep
  //sleepNow();
  // Wake from sleep on heartbeat fraction timeout, or serial data

  //If serial data (check the serial boolean)
  if (newPacket) {
    //reset
    newPacket = false;
    //Get command
    uint8_t command = currentPacket[COMMAND_BYTE_POS];
    
    //if command is 0x1
    if (command == 0x1) {
      //Heartbeat (This probably will be removed and replaced with the regular update commands)
      //reset the timer and reset all fans to previous values 
    } 
    //Got a fan command
    else if (command >= 0x10) {
      //Get fanpin and data
      Serial.println("Got a fan command");
      uint8_t fanPin = getFanPinFromCommand(command);
      uint8_t data = getDataFromPacket();
      Serial.print("Setting fanpin:");
      Serial.print(fanPin, DEC);
      Serial.print(" to PWM:");
      Serial.println(data, DEC);
      //Set fan to data
      analogWrite(fanPin, data);
      //Reset heartbeat timer
    }

    //Check the heartbeat timeout
    //If heatbeat timeout has incremented more than 2 minutes total:
    //set all fans to HIGH
  }
delay(1000);
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  receivePacket();

}

void receivePacket() {
  //Get ready for new packet
  uint8_t headerBytes = 0;
  uint8_t index = 0;
  uint8_t packetLength = 0;

  delay(5); //Wait for data

  while (Serial.available()) {
    //get a new byte to work with
    uint8_t inByte = (uint8_t)Serial.read();

    if (headerBytes < NUM_HEADER_BYTES) {
      //If packet has not started yet,
      //Check for beginning of new packet
      if (inByte == HEADER_BYTE) { 
        headerBytes++;
      } 
      else {
        //discard byte
      }
    } 
    else {
      //Packet has started,
      if (packetLength == 0) {
        //We have not recieved a valid packet length yet
        //Take the next byte and use it as the packet length if it is valid
        if (inByte != 0) {
          //Use as length 
          packetLength = inByte;

          //Clean the packet before using
          cleanPacket();
          //Copy to currentPacket var
          currentPacket[index] = packetLength;
          index++;
        } 
        else {
          //Not valid, aborting
          newPacket = false;
          return;
        }
      } 
      else {
        //We are in the middle of reading a packet
        //Check that we aren't done with our packet

        if (index <= packetLength){
          //Place the current data into the packet
          currentPacket[index] = inByte;
          index++;
        } 
        else {
          //Reached length specified, done with packet.
          checkPacket();
          return;
        }
      }
    }
  }

  //Done reading packet (no more data to read)
  //Check that the lengths match up
  if (index == packetLength+1) {
    //Do more in-depth testing
    Serial.println("Got a good packet! Testing now");
    checkPacket(); 
  } 
  else {
    //Our recieved packet was hopelessly inadequate
    Serial.println("Bad packet!");
    cleanPacket();
    newPacket = false;
    return; 
  }
}

boolean checkPacket() {
  //Will return true or false in addition to setting the global newPacket var.
  Serial.println("Checking in depth packet: NA");

  //Check that the command byte is valid

  //Command array length:
  uint16_t commandArrayLength = sizeof(COMMANDS)/sizeof(uint8_t);

  //Check to see if the command from the packet is our vocabulary of commands
  if (std::binary_search(COMMANDS, &COMMANDS[commandArrayLength], currentPacket[COMMAND_BYTE_POS])){
    //We can do that command, lets just check that our lengths match up
    //Get the mem address of the command in the command array
    const uint8_t* pointer = std::lower_bound(COMMANDS, &COMMANDS[commandArrayLength], currentPacket[COMMAND_BYTE_POS]);
    //figure out based on the mem position of the array and the found command, the command's index position in the array
    uint8_t index = (pointer - COMMANDS);
    //Now we can check the lengths
    if (COMMAND_LENGTHS[index] == currentPacket[LENGTH_POS]) {
      Serial.println("Good packet!");
      newPacket = true;
      return true;
    } 
    else {
      Serial.println("In-Depth check FAIL on lengths!");
      newPacket = false;
        cleanPacket();
      return false;
    }

  } 
  else {
    //We don't know how to do that command
    Serial.println("In-depth checking FAIL");
    newPacket = false;
    cleanPacket();
    return false; 
  }


}

void cleanPacket() {
  memset(currentPacket, 0x0, RESERVED_PACKET_LENGTH);
}

void sleepNow() // here we put the arduino to sleep
{
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
   * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
   * there is a list of sleep modes which explains which clocks and
   * wake up sources are available in which sleep mode.
   *
   * In the avr/sleep.h file, the call names of these sleep modes are to be found:
   *
   * The 5 different modes are:
   *     SLEEP_MODE_IDLE         -the least power savings
   *     SLEEP_MODE_ADC
   *     SLEEP_MODE_PWR_SAVE //Timer is on, main clock off
   *     SLEEP_MODE_STANDBY  //Timer off, but clock is held running (crystal still occilates), but not used. Fast turnaround. (6 cycles)
   *     SLEEP_MODE_PWR_DOWN     -the most power savings (but does not allow for timer2, PWM, or fast turnaround)
   *
   * For now, we want as much power savings as possible, so we
   * choose the according
   * sleep mode: SLEEP_MODE_PWR_DOWN
   *
   */
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);   // sleep mode is set here

  sleep_enable();          // enables the sleep bit in the mcucr register
  // so sleep is possible. just a safety pin

  /* Now it is time to enable an interrupt. We do it here so an
   * accidentally pushed interrupt button doesn't interrupt
   * our running program. if you want to be able to run
   * interrupt code besides the sleep function, place it in
   * setup() for example.
   *
   * In the function call attachInterrupt(A, B, C)
   * A   can be either 0 or 1 for interrupts on pin 2 or 3.  
   *
   * B   Name of a function you want to execute at interrupt for A.
   *
   * C   Trigger mode of the interrupt pin. can be:
   *             LOW        a low level triggers
   *             CHANGE     a change in level triggers
   *             RISING     a rising edge of a level triggers
   *             FALLING    a falling edge of a level triggers
   *
   * In all but the IDLE sleep modes only LOW can be used.
   */

  attachInterrupt(0,wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function
  // wakeUpNow when pin 2 gets LOW

    sleep_mode();            // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  sleep_disable();         // first thing after waking from sleep:
  // disable sleep...
  detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
  // wakeUpNow code will not be executed
  // during normal running time.

}
void wakeUpNow()        // here the interrupt is handled after wakeup
{
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
}

//Returns the fanpin that is referenced in the command byte of the most recently received packet
uint8_t getFanPinFromCommand(uint8_t command) {
  //calulate command offset 
      //Get command array length
      uint16_t commandArrayLength = sizeof(COMMANDS)/sizeof(uint8_t);
      //Get pos in memory of the recieved command
      const uint8_t* pointer = std::lower_bound(COMMANDS, &COMMANDS[commandArrayLength], command);
      //figure out based on the mem position of the array and the found command, the command's index position in the array
      uint8_t index = (pointer - COMMANDS);
      //adjust index
      index -= FAN_PIN_ARRAY_OFFSET;
      return FAN_PINS[index];
}

//Returns the databyte of the packet
uint8_t getDataFromPacket(){ 
  return currentPacket[PAYLOAD_BYTE_POS];
}
