/*
 Program publishes data from two ADC outputs
 LTC1865 ADC
 16-bit, 250ksps, SPI, MSBFIRST

 Shows the output of ADC
 Uses the SPI library.

 Circuit:
 LTC1865 ADC attached to pins 34, 50 - 52:
 CONV: pin 34/14 - pin 1 ADC
 MISO: pin 50/16 - pin 6 ADC
 MOSI: pin 51/15 - pin 5 ADC
 SCK: pin 52/17 - pin 7 ADC
 
 */
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <rosserial_arduino/Adc.h>

ros::NodeHandle nh;

rosserial_arduino::Adc adc_msg;
ros::Publisher p("adc_zlc", &adc_msg);

// the ADC communicates using SPI, so include the library:
#include <SPI.h>

//ADC register addresses:
const byte READ_CH0 = 0x80;     // CH0 read command
const byte READ_CH1 = 0xC0;     // CH1 read command

// pins used for the connection with the sensor
// the other you need are controlled by the SPI library:
const int convPin = 10;
//pins used for synchronization
const int syncOutPin = A0;
const int syncInPin = A1;

// set up the speed, data order and data mode
SPISettings settingsADC(250000, MSBFIRST, SPI_MODE3); 

void setup() {
  nh.getHardware()->setBaud(115200); // set a custom baud rate for communication with PC
  nh.initNode();
  nh.advertise(p);

  // start the SPI library:
  SPI.begin();

  // initalize the  data ready and chip select pins:
  pinMode(convPin, OUTPUT);
  // initialize digital pins for synchronization 
  pinMode(syncOutPin, OUTPUT);
  pinMode(syncInPin, INPUT);
  digitalWrite(syncOutPin, HIGH);
}

void loop() {
  unsigned int arr_lc_force[10];
  unsigned int arr_z_force[10];
  unsigned int tot_lc = 0;
  unsigned int tot_z = 0;
  for (int i=0; i<5; i++){
    
    while (digitalRead(syncInPin) == LOW) {
      // wait for start conversion signal
    }
    
    digitalWrite(syncOutPin, LOW); // data conversion in process
    
    SPI.beginTransaction(settingsADC);
    //Read the ADC CH0 data
    unsigned int LCForceData = readRegister(READ_CH1, 3);
    arr_lc_force[i] = LCForceData;
  
    //Read the ADC CH1 data
    unsigned int ZForceData = readRegister(READ_CH0, 3);
    arr_z_force[i] = ZForceData;
    
    SPI.endTransaction();
    
    digitalWrite(syncOutPin, HIGH); // data conversion finished
    //delayMicroseconds(3);
  }
  for (int i=0; i<5; i++){
    tot_lc += arr_lc_force[i]/10;
    tot_z += arr_z_force[i]/10;
  }
  adc_msg.adc0 = tot_lc*2;
  adc_msg.adc1 = tot_z*2;
  p.publish(&adc_msg);
  tot_lc = 0;
  tot_z  = 0;
  nh.spinOnce();
}

//Read from or write to register from the ADC:
unsigned int readRegister(byte thisRegister, int bytesToRead) {
  byte inByte = 0;           // incoming byte from the SPI
  unsigned int result = 0;   // result to return

  // take the CONV high to start convertion cycle:
  digitalWrite(convPin, HIGH);
  delayMicroseconds(3);        // pauses for 3 microseconds
  digitalWrite(convPin, LOW);

  // send a value of next register to be readed and get 2 bytes returned:
  result = SPI.transfer(thisRegister);
  
  // decrement the number of bytes left to read:
  bytesToRead--;
  // if you still have another byte to read:
  if (bytesToRead > 0) {
    inByte = SPI.transfer(0x00);
    // shift the first byte left, then get the second byte:
    result = result << 8;
    // combine the byte you just got with the previous one:
    result = result | inByte;
    // decrement the number of bytes left to read:
    bytesToRead--;
  }
  // take the CONV high and go to sleep mode:
  digitalWrite(convPin, HIGH);
  // return the result:
  return (result);
}




