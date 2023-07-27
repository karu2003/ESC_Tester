#include <Arduino.h>
#include <Servo.h>
#include "Streaming.h"
#include "Average.h"
//#define SERIAL

// Set your scale factor
int mVperAmp = 40; // See Scale Factors Below

/* Scale Factors
   50A bi-directional = 40
   50A uni-directional = 60
   100A bi-directional = 20
   100A uni-directional = 40
   150A bi-directional = 13.3
   150A uni-directioal = 26.7
   200A bi-directional = 10
   200A uni-directional = 20
 */

// Set you Offset
int ACSoffset = 2500; // See offsets below

/* Offsets
   If bi-directional = 2500
   If uni- directional = 600
 */

boolean state_1 = 0; // manual mode
boolean state_10 = 0;
boolean state_2 = 0; // auto mode
boolean state_3 = 0; // set Measured_Time
boolean state_4 = 0; // set PWM Step
boolean state_5 = 0; // set Motor Paar
boolean state_6 = 0; // set Gear
boolean digit = 0;
uint8_t calibrate_mode = 0;
uint8_t incomingByte = 0;
String incomingString = "";


//Variables for voltage divider
float denominator;
float resistor1 = 39000;
float resistor2 = 4700;

// HALL or Reflective Sensor
#define RPMin 3
#define PWMen 9
// Analog input

#if defined(ARDUINO_ARCH_RP2040)
#define VDCin A0
#define ADCin A1
#else
#define VDCin A4
#define ADCin A2
#endif

unsigned long interval = 2000; //Default 1s

#define time_min 50
#define time_max 10000
unsigned long micros1 = 0;
unsigned long lastmillis = 0;

volatile float Motor_Paar = 1; //7
#define paar_min 1
#define paar_max 14

volatile float Gear = 6.75; //6.75
#define gear_min 1
#define gear_max 14

volatile float rpm = 0;
volatile int rpm_present = 0;
long lastTime = 0;

struct Calibration {
        float incoming1;
        float incoming2;
        float volt1;
        float volt2;
        float slope;
        float intercept;
}; // Calibration

Calibration cali;

float acs_read;
int acs_offset = 0;
volatile float amp;
volatile float incoming;
volatile float volt;
volatile float watt;

int RawValue= 0;
double Voltage = 0;
double Amps = 0;

float slope_V = 1.004025;
float intercept_V = -0.132385;
float slope_A = 1.026; // 1.026
float intercept_A = 0.326; // 0.305

//esc
#define pwm_min 900
#define pwm_max 2100
#define ESCout 4
Servo esc;
int PWMValue = 1500;
int Step = 10;
#define step_min 10
#define step_max 500
int rpmcount = 0;
unsigned long lastmillis_rpm = 0;

#define BUFFER 32
Average<float> ave_VDC(BUFFER);
Average<float> ave_ADC(BUFFER);
Average<float> ave_RPM(10);

void getMultimeter() {
        //volt
        //volt = (analogRead(VDCin) / 1023.0) * 46.73; //
        volt = (analogRead(VDCin) * (5.0 / 1023.0));
        volt = volt / denominator;
        //volt = (slope_V * volt) + intercept_A;
        ave_VDC.push(volt);

        //ampere
        while (acs_offset == 0) {
                for (int i = 0; i < 10; i++) {
                        acs_offset += analogRead(ADCin);
                }
                acs_offset /= 10; //create the offset
                acs_offset = (acs_offset / 1023.0) * 5000; // Gets you mV
        }

        RawValue = analogRead(ADCin);
        Voltage = (RawValue / 1023.0) * 5000; // Gets you mV
        Amps = ((Voltage - ACSoffset) / mVperAmp);
        //Amps = ((Voltage - acs_offset) / mVperAmp);
        Amps = (slope_A * Amps) + intercept_A;
        //Serial << "Current:\t" << _FLOAT(Amps,3) << endl;
        ave_ADC.push(abs(Amps));

        //watt
        watt = ave_VDC.mean() * ave_ADC.mean();
}

void RPM (){
        // rpm = (1000000.0/(micros() - micros1))*60;
        // ave_RPM.push(rpm);
        // micros1 = micros();
        // rpm_present = 1;
        rpmcount++;
}

void SerialFlush(){
        incomingString = "";
        incomingByte = 0;
        Serial.flush();
}

void slope_intercept(Calibration cali){
        Serial << "Measured Value1: " << _FLOAT(cali.incoming1,2) << " Sensor Value1: " << _FLOAT(cali.volt1,2) << endl;
        Serial << "Measured Value2: " << _FLOAT(cali.incoming2,2) << " Sensor Value2: " << _FLOAT(cali.volt2,2) << endl;

        cali.slope = (cali.volt2 - cali.volt1)/(cali.incoming2 - cali.incoming1); // Slope = (Sensor Value 2 – Sensor Value 1)/(V2 – V1)
        cali.intercept = cali.volt2 - (cali.slope * cali.incoming2);   // Intercept = Sensor Value 2 – slope * V2
        Serial << "slope:\t" << _FLOAT(cali.slope,6)<< endl;
        Serial << "intercept:\t" << _FLOAT(cali.intercept,6)<< endl;
}


void setup()
{
        Serial.begin(115200);
        //pinMode(ESCout, OUTPUT);
        //digitalWrite(RPMin, 1); //
        pinMode(PWMen, OUTPUT);
        digitalWrite(PWMen, 1);
        esc.attach(ESCout, pwm_min, pwm_max);
        esc.writeMicroseconds(PWMValue);
        denominator = (float)resistor2 / (resistor1 + resistor2);
        attachInterrupt(1, RPM, FALLING);
        ave_VDC.clear();
        ave_ADC.clear();
        ave_RPM.clear();
        Serial << "Enter comand m, a, c, k, d, s, p, g, q, h" << endl;
        Serial << "> ";
}

void loop(){

        if (Serial.available() > 0) {
                incomingByte = Serial.read();
                Serial.print((char)incomingByte);

                if (digit) {
                        incomingString += (char)incomingByte;
                        if (incomingByte == '\n' or incomingByte == '\r') {
                                incoming = incomingString.toFloat();
                                //Serial << "incoming:\t" << _FLOAT(incoming,2) << endl;
                                SerialFlush();
                                digit = 0;
                        }

                }

                switch (incomingByte) {
                case 'c':
                        Serial << endl;
                        Serial << "Two-point Voltage calibration" << endl;
                        Serial << "Set min Voltage on Power Supply (~9V)" << endl;
                        Serial << "Enter the measured voltage - xx.xx" << endl;
                        Serial << "> ";
                        slope_V = 1.000;
                        intercept_V = 0.0;
                        calibrate_mode = 1;
                        digit = 1;
                        SerialFlush();
                        break;
                case 'k':
                        Serial << endl;
                        Serial << "Two-point Current calibration" << endl;
                        Serial << "Set min load (~100mA)" << endl;
                        Serial << "Enter the measured current - xx.xx" << endl;
                        Serial << "> ";
                        slope_A = 1.000;
                        intercept_A = 0.0;
                        calibrate_mode = 11;
                        digit = 1;
                        SerialFlush();
                        break;
                case 'm':
                        Serial << endl;
                        Serial << "Enter PWM - uS or Enter" << endl;
                        Serial << "> ";
                        state_1 = 1;
                        digit = 1;
                        break;
                case 'a':
                        Serial << endl;
                        PWMValue = 1500;
                        Serial << "Start Test: PWM " << (PWMValue) << " > " << (pwm_max) << "; step " << (Step) <<  "; Duration " << (interval) << " mS"<<endl;
                        Serial << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<endl;
                        esc.writeMicroseconds(PWMValue);
                        lastmillis = millis();
                        //delay(interval);
                        state_2 = 1;
                        break;
                case 'd':
                        Serial << endl;
                        Serial << "Enter Duration - mS or Enter" << endl;
                        Serial << "> ";
                        state_3 = 1;
                        digit = 1;
                        break;
                case 's':
                        Serial << endl;
                        Serial << "Enter PWM Step - mS or Enter" << endl;
                        Serial << "> ";
                        state_4 = 1;
                        digit = 1;
                        break;
                case 'p':
                        Serial << endl;
                        Serial << "Enter Motor Paar or Enter" << endl;
                        Serial << "> ";
                        state_5 = 1;
                        digit = 1;
                        break;
                case 'g':
                        Serial << endl;
                        Serial << "Enter Gear or Enter" << endl;
                        Serial << "> ";
                        state_6 = 1;
                        digit = 1;
                        break;
                case 'q':   //'\x'
                        Serial << endl;
                        state_1 = 0; // manual mode
                        state_10 = 0;
                        state_2 = 0; // auto mode
                        state_3 = 0; // set Measured_Time
                        state_4 = 0; // set PWM Step
                        digit = 0;
                        calibrate_mode = 0;
                        PWMValue = 1500;
                        ave_ADC.clear();
                        ave_VDC.clear();
                        ave_RPM.clear();
                        esc.writeMicroseconds(PWMValue);
                        Serial << "> ";
                        break;
                case 'h':
                        Serial << endl;
                        Serial << " m - Manual mode" << endl;
                        Serial << " a - Auto mode" << endl;
                        Serial << " c - Voltage calibration" << endl;
                        Serial << " k - Current calibration" << endl;
                        Serial << " d - Set Duration of one measurement" << endl;
                        Serial << " s - Set PWM Step" << endl;
                        Serial << " p - Set Motor Paar" << endl;
                        Serial << " p - Set Gear" << endl;
                        Serial << " q - Stop" << endl;
                        Serial << " h - Help" << endl;
                        Serial << "> ";
                        break;
                        //  default:
                        // statements
                }

                if (calibrate_mode == 1 and !digit) {
                        cali.incoming1 = incoming;
                        cali.volt1 = ave_VDC.mean();
                        //Serial << "first point: " << _FLOAT(incoming1,2) << "V " << "Voltage: " << _FLOAT(volt1,3) << "V" << endl;
                        Serial << "Set max Voltage on Power Supply (~33V)" << endl;
                        Serial << "Enter the measured voltage - xx.xx" << endl;
                        Serial << "> ";
                        digit = 1;
                        calibrate_mode = 2;
                }

                if (calibrate_mode == 2 and !digit) {
                        cali.incoming2 = incoming;
                        cali.volt2 = ave_VDC.mean();
                        //Serial << "second point: " << _FLOAT(incoming2,2) << "V " << "Voltage: " << _FLOAT(volt2,3) << "V" << endl;
                        calibrate_mode = 3;
                }

                if (calibrate_mode == 11 and !digit) {
                        cali.incoming1 = incoming;
                        cali.volt1 = ave_ADC.mean();
                        //Serial << "first point: " << _FLOAT(incoming1,2) << "A " << "Current: " << _FLOAT(volt1,3) << "A"<< endl;
                        Serial << "Set max load (~10A)" << endl;
                        Serial << "Enter the measured voltage - xx.xx" << endl;
                        Serial << "> ";
                        digit = 1;
                        calibrate_mode = 12;
                }

                if (calibrate_mode == 12 and !digit) {
                        cali.incoming2 = incoming;
                        cali.volt2 = ave_ADC.mean();
                        //Serial << "second point:\t" << _FLOAT(incoming2,2) << "A" << "Current:\t" << _FLOAT(volt2,3) << "A" << endl;
                        calibrate_mode = 13;
                }
                if (state_1 == 1 and !digit) {
                        if (incoming >= (float)pwm_min and incoming <= (float)pwm_max) {
                                PWMValue = (int)incoming;
                        }
                        // if (incoming !=(float)0.0) {
                        //         Serial << "ESC - " << (PWMValue) << endl;
                        // }
                        esc.writeMicroseconds(PWMValue);
                        lastmillis = millis();
                }
                if (state_3 == 1 and !digit) {
                        if (incoming >= (float)time_min and incoming <= (float)time_max) {
                                interval = (unsigned long)incoming;
                        }
                        Serial << "Duration - " << (interval) << endl;
                        state_3 = 0;
                        Serial << "> ";
                }
                if (state_4 == 1 and !digit) {
                        if (incoming >= (float)step_min and incoming <= (float)step_max) {
                                Step = (int)incoming;
                        }
                        Serial << "PWM Step - " << (Step) << endl;
                        state_4 = 0;
                        Serial << "> ";
                }
                if (state_5 == 1 and !digit) {
                       if (incoming >= (float)paar_min and incoming <= (float)paar_max) {
                               Motor_Paar = (float)incoming;
                        }
                        Serial << "Motor Paar - " << _FLOAT(Motor_Paar,1) << endl;
                        state_5 = 0;
                        Serial << "> ";
                }
                if (state_6 == 1 and !digit) {
                       if (incoming >= (float)gear_min and incoming <= (float)gear_max) {
                               Gear = (float)incoming;
                        }
                        Serial << "Gear - " << _FLOAT(Gear,2) << endl;
                        state_5 = 0;
                        Serial << "> ";
                }
        }

        if (calibrate_mode == 3) {
                slope_intercept(cali);
                slope_V = cali.slope;
                intercept_V = cali.intercept;
                calibrate_mode = 0;
                Serial << "> ";
        }

        if (calibrate_mode == 13) {
                slope_intercept(cali);
                slope_A = cali.slope;
                intercept_A = cali.intercept;
                calibrate_mode = 0;
                Serial << "> ";
        }
        if (millis() - lastmillis_rpm >= 100){ //1000
                detachInterrupt(1);
                rpm = (float)rpmcount * 600;
                if (Motor_Paar > (float)1.0){
                  rpm =  rpm / (Motor_Paar * 2);
                }
                rpm = rpm / Gear;
                ave_RPM.push(rpm);
                rpmcount = 0;
                lastmillis_rpm = millis();
                //Serial << "RPM mean:\t" << (ave_RPM.mean()) << endl;
                //Serial << "RPM:\t" << (rpm) << endl;
                attachInterrupt(1, RPM, FALLING);
        }

        getMultimeter();

        // if (rpm_present) {
        //         if (millis() - lastmillis >= interval) {
        //                 lastmillis = millis();
        //                 Serial << "RPM:\t" << (ave_RPM.mean()) << endl;
        //         }
        //         rpm_present = 0;
        // }

        if (state_1 and !digit) {
                if (millis() - lastmillis >= interval) {
                        lastmillis = millis();
                        Serial << "PWM:\t" << (PWMValue) << endl;
                        //Serial << "RPM:\t" << (rpm) << endl;
                        Serial << "RPM:\t" << _FLOAT(ave_RPM.mean(),1) << endl;
                        Serial << "Current:\t" << _FLOAT(ave_ADC.mean(),3) << endl;
                        Serial << "Volt:\t" << _FLOAT(ave_VDC.mean(),3) << endl;
                        Serial << "Watt:\t" << _FLOAT(watt,3) << endl;
                        state_1 = 0;
                        Serial << "> ";
                }
        }
        if (state_2) {
                if (millis() - lastmillis >= interval) {
                        lastmillis = millis();
                        //Serial << (PWMValue) <<","<< _FLOAT(ave_VDC.mean(),3) <<","<< _FLOAT(ave_ADC.mean(),3) <<","<< (rpm) << endl;
                        Serial << (PWMValue) <<","<< _FLOAT(ave_VDC.mean(),3) <<","<< _FLOAT(ave_ADC.mean(),3) <<","<< (ave_RPM.mean()) << endl;                       PWMValue += Step;
                        if (PWMValue >= pwm_max) {
                                PWMValue = 1500;
                                Serial << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
                                Serial << "Test Completed" << endl;
                                ave_ADC.clear();
                                ave_VDC.clear();
                                ave_RPM.clear();
                                state_2 = 0;
                                Serial << "> ";
                                //break;
                        }
                        //ave_ADC.clear();
                        //ave_VDC.clear();
                        //ave_RPM.clear();
                        esc.writeMicroseconds(PWMValue);
                }
        }
        //delay(10);
}
