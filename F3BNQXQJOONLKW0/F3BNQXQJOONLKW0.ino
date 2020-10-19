/*
  Degree_of_Roast_Infrared_Analyzer.ino 

  The Degree of Roast Infrared Analyzer uses Infrared reflectance to measure
  the degree of roast of coffee beans. 

  The circuits:
   1)QWIIC Port --> AS7263 NIR Spectral Sensor

  `2)5V (Supply) --> Power Switch --> VIN
     Ground (Supply) --> GND (Board)

   3)5V (Supply)--> 4 Incandesent Lamps (parallel) --> SSR (Load +)
     SSR (Load -) --> Ground (Supply)

     Digital 5 --> SSR (Control +)
     GND --> SSR (Control -)

   4)HC-05
      VCC (BT) --> 5V (Board)
      GND (BT) --> GND (Board)
      RXD (BT) --> Digital 11 (Board)
      TXD (BT) --> Digital 10 (Board)

   5)Button 1
      5V (Board) --> B1 (+)
      Digital 3 --> B1 (-)
      GND (Board) --> R1 (10kOhm) --> B1(-)     

   6)Button 2
      5V (Board) --> B2 (+)
      Digital 2 --> B2 (-)
      GND (Board) --> R2 (10kOhm) --> B2(-)    
      
  Created 01 JUL 2018
  By Spencer Corry
  Modified 01 NOV 2018
  By Spencer Corry

*/


/////////////////////////////////////
// Chapter 1. Libraries            //
/////////////////////////////////////

#include "AS726X.h" //Library for InfraRed Sensor
#include <Wire.h>  // Include Wire if you're using I2C
#include <SPI.h>  // Include SPI if you're using SPI
#include <SoftwareSerial.h>

//////////////////////////////
// Chapter 2. Declarations  //
//////////////////////////////


//////////////////////////////
// Chapter 2.1 Definitions  //
//////////////////////////////

#define calibrationPin   2  // Connect Button to pin 2
#define tryerPin         3  // Connect Button to pin 3
#define RELAY_pin        5  // Connect Relay to pin 4
#define BT_TX            10 // COnnect BT module TX to D10
#define BT_RX            11 // Connect BT module RX to D11

///////////////////////////
// Chapter 2.2  Objects  //
///////////////////////////

AS726X sensor; // IR sensor Object Declaration
SoftwareSerial BT(BT_TX, BT_RX); //Used to communicate with BlueTooth

///////////////////////////
// Chapter 2.3 Variables  //
///////////////////////////

int measurement_delay = 10; //Length of time in ms between sensor measurements
bool calibrated; //Tracks if the calibration process has been completed

//Final Calibrated Percentage
//Values for all Sensor Channels
float R = 0.0;
float S = 0.0;
float T = 0.0;
float U = 0.0;
float V = 0.0;
float W = 0.0;

//Sum of X Calibration data
//Taken with white plate
float R_white_sum;
float S_white_sum;
float T_white_sum;
float U_white_sum;
float V_white_sum;
float W_white_sum;

//Average Value of White Calibration
//Used as a maximum value of reflectance for calibration
float R_white_avg;
float S_white_avg;
float T_white_avg;
float U_white_avg;
float V_white_avg;
float W_white_avg;

//Sum of X Coffee Bean reflectance data
//Used to take an average ove muliple readings
float R_sum;
float S_sum;
float T_sum;
float U_sum;
float V_sum;
float W_sum;

//Average Value of Muliple Readings
float R_avg;
float S_avg;
float T_avg;
float U_avg;
float V_avg;
float W_avg;

//Global Variables for interrupts
volatile int tryer_flag; //tracks if the tryer button has been pressed
volatile int calibration_flag; //tracks if the calibration button has been pressed

//////////////////////
// Chapter 3 Setup  //
//////////////////////

void setup() {

  pinMode(calibrationPin, INPUT); //Attach Calibration Button and set as input
  pinMode(tryerPin, INPUT); //Attach Tryer Button and set as input
  pinMode(RELAY_pin, OUTPUT); //Attach Relay and set as output

  digitalWrite(RELAY_pin, LOW); //Begin with relay off

  attachInterrupt(digitalPinToInterrupt(calibrationPin), ISR_calibrate, RISING); //Interrupt Flag for when Calibration button pushed
  attachInterrupt(digitalPinToInterrupt(tryerPin), ISR_tryer, RISING); //Interrupt Flag for when Tryer button pushed


  sensor.begin(); //Attach the IR Sensor

  calibrated = false; //Sensor has not been calibrated yet

  Serial.begin(115200);
  BT.begin(115200);

}

////////////////////////////////////////////
// Chapter 3 External Interrupt Functions //
////////////////////////////////////////////


void ISR_calibrate() { //Function is triggered by interrupt on calibrationPin

  calibration_flag = 1; //Signal that the calibration button has been triggered
  Serial.println("Calibration Button Pushed");

}

void ISR_tryer() { //Function is triggered by the interrupt on tryerPin

  tryer_flag = 1; //Signals that the tryer button has been triggered
  Serial.println("Tryer Button Pushed");
}

/////////////////////////
// Chapter 4 Main Loop //
/////////////////////////

void loop() {

  ///////////////////////////////////////
  // Chapter 4.1 Calibration Procedure //
  ///////////////////////////////////////

  if (calibration_flag) {   //Runs if the calibration button has been triggered

    //Clear the summing variables to be used again
    R_white_sum = 0;
    S_white_sum = 0;
    T_white_sum = 0;
    U_white_sum = 0;
    V_white_sum = 0;
    W_white_sum = 0;

    ///////////////////////////////////////////////////
    // Chapter 4.2 Calibration ForLoop and Averaging //
    ///////////////////////////////////////////////////

    //Takes X measurements from the sensor and sums them
    for (int i = 0; i < 4; i++) {
      BT.print("*CR255G255B255*");
      digitalWrite(RELAY_pin, HIGH);
      //delay(measurement_delay);
      sensor.takeMeasurements();
      //delay(measurement_delay);
      digitalWrite(RELAY_pin, LOW);

      R_white_sum += sensor.getCalibratedR();
      S_white_sum += sensor.getCalibratedS();
      T_white_sum += sensor.getCalibratedT();
      U_white_sum += sensor.getCalibratedU();
      V_white_sum += sensor.getCalibratedV();
      W_white_sum += sensor.getCalibratedW();
      BT.print("*CR0G0B0*");
    }

    //Average Calculation by dividing the sum by the number of measurements
    R_white_avg = R_white_sum / 4;
    S_white_avg = S_white_sum / 4;
    T_white_avg = T_white_sum / 4;
    U_white_avg = U_white_sum / 4;
    V_white_avg = V_white_sum / 4;
    W_white_avg = W_white_sum / 4;

    //Serial Print Value
    Serial.print("R_cal: ");
    Serial.print(R_white_avg);
    Serial.print(", S_cal: ");
    Serial.print(S_white_avg);
    Serial.print(", T_cal: ");
    Serial.print(T_white_avg);
    Serial.print(", U_cal: ");
    Serial.print(U_white_avg);
    Serial.print(", V_cal: ");
    Serial.print(V_white_avg);
    Serial.print(", W_cal: ");
    Serial.println(W_white_avg);



    calibration_flag = 0; //Reset Calibration Flag to Zero (Button unpressed)
    calibrated = true; //Indicate that the sensor has been calibrated

    delay(500); //Delay program for 500ms
  }

  ////////////////////////////////////////////////////
  // Chapter 4.3 Measurement ForLoop and Averaging  //
  ////////////////////////////////////////////////////

  if (calibrated && tryer_flag) {

    tryer_flag = 0;

    R_sum = 0;
    S_sum = 0;
    T_sum = 0;
    U_sum = 0;
    V_sum = 0;
    W_sum = 0;

    //Takes X measurements from the sensor and sums them
    for (int i = 0; i < 4; i++) { //Take
      BT.print("*MR255G255B255*");  //Sets Measurement light on bluetooth device to white
      digitalWrite(RELAY_pin, HIGH);
      //delay(measurement_delay);
      sensor.takeMeasurements();
      //delay(measurement_delay);
      digitalWrite(RELAY_pin, LOW);

      R_sum += sensor.getCalibratedR();
      S_sum += sensor.getCalibratedS();
      T_sum += sensor.getCalibratedT();
      U_sum += sensor.getCalibratedU();
      V_sum += sensor.getCalibratedV();
      W_sum += sensor.getCalibratedW();
      BT.print("*MR0G0B0*");  //Sets Measurement light on bluetooth device to black
    }

    //Average Calculation by dividing the sum by the number of measurements
    R_avg = R_sum / 4;
    S_avg = S_sum / 4;
    T_avg = T_sum / 4;
    U_avg = U_sum / 4;
    V_avg = V_sum / 4;
    W_avg = W_sum / 4;

    //MAP sensor data to percent reflectance when compared to white tile
    R = R_avg * (100.0 / R_white_avg);
    S = S_avg * (100.0 / S_white_avg);
    T = T_avg * (100.0 / T_white_avg);
    U = U_avg * (100.0 / U_white_avg);
    V = V_avg * (100.0 / V_white_avg);
    W = W_avg * (100.0 / W_white_avg);

    ////////////////////////////////////////////////////
    // Chapter 4.4 Printing the Data and Making Graph //
    ////////////////////////////////////////////////////

    //Print Collected Data through serial
    Serial.print(R);
    Serial.print("\t");
    Serial.print(S);
    Serial.print("\t");
    Serial.print(T);
    Serial.print("\t");
    Serial.print(U);
    Serial.print("\t");
    Serial.print(V);
    Serial.print("\t");
    Serial.print(W);
    Serial.print("\t");
    Serial.println(sensor.getTemperature());

    //Print Collected Data and Transmit through bluetooth
    BT.print("*R" + String(R) + "*");
    BT.print("*S" + String(S) + "*");
    BT.print("*T" + String(T) + "*");
    BT.print("*U" + String(U) + "*");
    BT.print("*V" + String(V) + "*");
    BT.print("*W" + String(W) + "*");


  }



}
