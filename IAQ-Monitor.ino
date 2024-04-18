/*
  INCS 3610 Final Project: Indoor Air Quality Monitor

  The code in this project has been modified and compiled from the following 
  sample code written by the following authors:

  *** Libraries and Example code used ***:
  MQUnifiedsensor by Miguel A Califa, Yersson Carrillo, Ghiordy Contreras, Mario Rodriguez - for the MQ135
  Seeed_HM330X by Downey at Seeed Technology Co., Ltd. Copyright (c) 2018 - for the HM3301 PM2.5
  SensirionI2CSgp41 by Sensirion AG Copyright (c) 2021 - for the SPG41

  modified and compiled by: Marian Aquilizan and Brhamjot Singh

*/

//Include sensor libraries
#include <MQUnifiedsensor.h>
#include <Seeed_HM330X.h>
#include <SensirionI2CSgp41.h>

// Include Dependencies
//___SPG41___//
#include <Arduino.h>
#include <Wire.h>
//___________//

// Definitions
//___MQ135___//
#define placa "Arduino UNO"
#define Voltage_Resolution 5
#define pin A0 //Analog input 0 of your arduino
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 10 // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm 
//___________//

//___HM3301__//
#ifdef  ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define Serial SerialUSB
#else
    #define Serial Serial
#endif
//___________//

// Declarations
//___MQ135___//
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
//___________//

//___HM3301__//
HM330X sensor;
uint8_t buf[30];

const char* str[] = {"\"sensor num\":", 
                     "\"PM1.0Std\":",
                     "\"PM2.5Std\":",
                     "\"PM10Std\":",
                     "\"PM1.0Atm\":",
                     "\"PM2.5Atm\":",
                     "\"PM10Atm\":",
                    };
//___________//

//___SPG41___//
SensirionI2CSgp41 sgp41;
uint16_t conditioning_s = 10; // time in seconds needed for NOx conditioning
//___________//

// Functions
//___HM3301__//
HM330XErrorCode print_result(const char* str, uint16_t value) {
    if (NULL == str) {
        return ERROR_PARAM;
    }
    Serial.print(str);
    Serial.print(value);
    if (str != "\"PM10Atm\":"){
      Serial.print(",");
    } else {
      Serial.print("}");
    }
    return NO_ERROR;
}

/*parse buf with 29 uint8_t-data*/
HM330XErrorCode parse_result(uint8_t* data) {
    uint16_t value = 0;
    if (NULL == data) {
        return ERROR_PARAM;
    }
    for (int i = 1; i < 8; i++) {
        value = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];
        print_result(str[i - 1], value);


    }

    return NO_ERROR;
}

HM330XErrorCode parse_result_value(uint8_t* data) {
    if (NULL == data) {
        return ERROR_PARAM;
    }
    for (int i = 0; i < 28; i++) {
        Serial.print(data[i], HEX);
        Serial.print("  ");
        if ((0 == (i) % 5) || (0 == i)) {
            Serial.println("");
        }
    }
    uint8_t sum = 0;
    for (int i = 0; i < 28; i++) {
        sum += data[i];
    }
    if (sum != data[28]) {
        Serial.println("wrong checkSum!!");
    }
    Serial.println("");
    return NO_ERROR;
}
//___________//



void setup() {
//___MQ135___//
//Init the serial port communication - to debug the library
  Serial.begin(115200); //Init serial port

  //Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  
  /*****************************  MQ Init ********************************************/ 
  //Remarks: Configure the pin of arduino as input.
  /************************************************************************************/ 
  MQ135.init(); 
  /* 
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ135.setRL(10);
  */
  /*****************************  MQ CAlibration ********************************************/ 
  // Explanation: 
  // In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
  // and on clean air (Calibration conditions), setting up R0 value.
  // We recomend executing this routine only on setup in laboratory conditions.
  // This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  MQ135.setRL(17.5);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
//___________//

//___HM3301__//
    delay(100);
    Serial.println("Serial start");
    if (sensor.init()) {
        Serial.println("HM330X init failed!!");
        while (1);
    }
//___________//

//___SPG41___//
  while (!Serial) {
      delay(100);
  }

  Wire.begin();
  sgp41.begin(Wire);

  uint8_t serialNumberSize = 3;
  uint16_t serialNumber[serialNumberSize];

  // Serial.print("SerialNumber:");
  // Serial.print("0x");
  //   for (size_t i = 0; i < serialNumberSize; i++) {
  //     uint16_t value = serialNumber[i];
  //     Serial.print(value < 4096 ? "0" : "");
  //     Serial.print(value < 256 ? "0" : "");
  //     Serial.print(value < 16 ? "0" : "");
  //     Serial.print(value, HEX);
  //   }
  uint16_t testResult;
  Serial.println();
  Serial.println(testResult);
//___________//
}

void loop() {
//___MQ135___//
MQ135.update(); // Update data, the arduino will read the voltage from the analog pin

  MQ135.setA(605.18); MQ135.setB(-3.937); // Configure the equation to calculate CO concentration value
  float CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(77.255); MQ135.setB(-3.18); //Configure the equation to calculate Alcohol concentration value
  float Alcohol = MQ135.readSensor(); // SSensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
  float CO2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(44.947); MQ135.setB(-3.445); // Configure the equation to calculate Toluen concentration value
  float Toluen = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  
  MQ135.setA(102.2 ); MQ135.setB(-2.473); // Configure the equation to calculate NH4 concentration value
  float NH4 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(34.668); MQ135.setB(-3.369); // Configure the equation to calculate Aceton concentration value
  float Aceton = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  Serial.print("\"CO\":"); Serial.print(CO); Serial.print(",");
  Serial.print("\"Alcohol\":"); Serial.print(Alcohol); Serial.print(",");
  // Note: 400 Offset for CO2 source: https://github.com/miguel5612/MQSensorsLib/issues/29
  /*
  Motivation:
  We have added 400 PPM because when the library is calibrated it assumes the current state of the
  air as 0 PPM, and it is considered today that the CO2 present in the atmosphere is around 400 PPM.
  https://www.lavanguardia.com/natural/20190514/462242832581/concentracion-dioxido-cabono-co2-atmosfera-bate-record-historia-humanidad.html
  */
  Serial.print("\"CO2\":"); Serial.print(CO2 + 400); Serial.print(",");
  Serial.print("\"Toluene\":"); Serial.print(Toluen); Serial.print(",");
  Serial.print("\"NH4\":"); Serial.print(NH4); Serial.print(",");
  Serial.print("\"Acetone\":"); Serial.print(Aceton); Serial.print(",");

  /*
    Exponential regression:
  GAS      | a      | b
  CO       | 605.18 | -3.937  
  Alcohol  | 77.255 | -3.18 
  CO2      | 110.47 | -2.862
  Toluen   | 44.947 | -3.445
  NH4      | 102.2  | -2.473
  Acetone  | 34.668 | -3.369
  */

  delay(500); //Sampling frequency
//___________//

//___HM3301__//
    if (sensor.read_sensor_value(buf, 29)) {
        Serial.println("HM330X read result failed!!");
    }
    //parse_result_value(buf); //ErrorCode function
    parse_result(buf); //ErrorCode function
    Serial.println("");
    delay(5000);
//___________//

//___SPG41___//
    uint16_t error;
    char errorMessage[256];
    uint16_t defaultRh = 0x8000;
    uint16_t defaultT = 0x6666;
    uint16_t srawVoc = 0;
    uint16_t srawNox = 0;

    delay(1000);

    // During NOx conditioning (10s) SRAW NOx will remain 0
    error = sgp41.measureRawSignals(defaultRh, defaultT, srawVoc, srawNox);
    conditioning_s--;

    Serial.print("{\"SRAW_VOC\":");
    Serial.print(srawVoc);
    Serial.print(",");

    Serial.print("\"SRAW_NOx\":");
    Serial.print(srawNox);
    Serial.print(",");
//___________//
}