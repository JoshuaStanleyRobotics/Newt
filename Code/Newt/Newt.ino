/*
   Description: This program is used to control Newt: a four wheeled omni-directional robot using mecanum wheels 

        Wiring: The required components are 4x Nema17 stepper motors, 4x TMC2208 stepper drivers, a NRF24L01 radio module, an Arduino CNC shield, and an Arduino Mega
        The stepper motors are connected to the X, Y, Z, and A axis pinouts on the CNC Shield and jumpers are used to connect D12 and D13 to the A axis
        The NRF24L01 is connected 3.3V to 3v3, GND to GND, CSN to D46, MOSI to D51, CE to D48, SCK to D52, MISO to D50
*/

#include "AccelStepper.h"

AccelStepper stepperFL(1, 3, 6);
AccelStepper stepperFR(1, 12, 13);
AccelStepper stepperBL(1, 2, 5);
AccelStepper stepperBR(1, 4, 7);

#include "RF24.h"
RF24 radio(48, 46);
const byte chan[6] = "00007";
byte data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int trig = 22;
int echo = 24;

int xDir = 0;
int yDir = 0;
int zDir = 0;
int spd = 0;

unsigned long millisPrev = 0;

const int MaxSpeed = 5000;

void setup() {
  Serial.begin(9600);
  Serial.println("Serial Communication Initialized");
  
  radio.begin();                                                                                    //Begin radio communication
  radio.openReadingPipe(1, chan);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("Radio Communication Initialized");

  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);                                                                             //Enable stepper motors
  stepperFL.setMaxSpeed(MaxSpeed);                                                                  //Configure stepper motors
  stepperFR.setMaxSpeed(MaxSpeed);
  stepperBL.setMaxSpeed(MaxSpeed);
  stepperBR.setMaxSpeed(MaxSpeed);
  Serial.println("Stepper Motors Set");

  delay(1000);
}

void loop() {
  if (radio.available()) {  
    radio.read(&data, sizeof(data));                                                                //Read in data from remote control  

    if (data[0] == 0) xDir = 0;                                                                     //Left and right motion controlled by left to right motion of left joystick
    else xDir = map(data[0], 1, 255, -10, 10);

    if (data[1] == 0) yDir = 0;                                                                     //Forward and reverse controlled by front to back motion of left joystick
    else yDir = map(data[1], 1, 255, -10, 10);

    if (data[4] == 0) zDir = 0;                                                                     //Turning controlled by left to right motion of right joystick
    else zDir = map(data[4], 1, 255, -5, 5);

    spd = map(data[9], 0, 255, 20, 500);                                                            //Driving speed controlled by right potentiometer
  }

  else if ((millis() - millisPrev) > 100){                                                          //Recalculate wheel speeds every 100 milliseconds
      stepperFL.setSpeed(spd * ( xDir + yDir + zDir));                                              //Set each wheel speed as a function of directional controls
      stepperFR.setSpeed(spd * (-xDir + yDir - zDir));
      stepperBL.setSpeed(spd * (-xDir + yDir + zDir));
      stepperBR.setSpeed(spd * ( xDir + yDir - zDir));

      millisPrev = millis();
  }

  stepperFL.runSpeed();
  stepperFR.runSpeed();
  stepperBL.runSpeed();
  stepperBR.runSpeed();
 }  

 int Ping() {                                                                                       //Function to return distance indicated by sensor in inches
  long duration, inches;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);                                                                         //Sends out pulse
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);                                                                   //Measures how long it take for the pulse to return
  inches = duration / 74 / 2;                                                                       //Calculates how many inches sound would travel in this time and divides by 2 for round trip
  inches = constrain(inches, 0, 120);                                                               //Limits readouts to be from 0 to 120 inches
  return inches;                                                                                    //Returns the distance in inches
}
