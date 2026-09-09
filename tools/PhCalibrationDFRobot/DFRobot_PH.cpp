/*!
 * @file DFRobot_PH.cpp
 * @brief Arduino library for Gravity: Analog pH Sensor / Meter Kit V2, SKU: SEN0161-V2
 *
 * Adapted from https://github.com/DFRobot/DFRobot_PH (master). Three
 * deviations from upstream:
 *   1. EEPROM.commit() added everywhere upstream writes - the ESP32
 *      Arduino core's EEPROM library is a flash-backed emulation, not
 *      real EEPROM, and only persists on an explicit commit() (unlike
 *      AVR, where EEPROM.write() commits immediately). Without this,
 *      every value this library "saves" would be lost on reset. The
 *      caller must also call EEPROM.begin(size) once before
 *      DFRobot_PH::begin() - see the .ino's setup().
 *   2. begin() pre-seeds a blank EEPROM with Basilience's own
 *      already-verified two-point calibration (pH 6.86 -> 2562.40 mV,
 *      pH 4.01 -> 2931.53 mV - see Calibration.h) instead of DFRobot's
 *      generic 1500.0/2032.44 mV "typical" defaults, which assume their
 *      own 5V/1024-count reference circuit and don't apply here.
 *   3. readPH()'s slope/intercept now derive from whatever two (pH,
 *      voltage) anchors are actually loaded (_neutralPh/_acidPh),
 *      instead of hardcoding 7.0/4.0 - required for #2 to work, since
 *      Basilience's calibration used 6.86/4.01 buffers, not 7.0/4.0.
 *
 * phCalibration()'s own buffer-recognition windows (1322-1678 mV for
 * 7.0, 1854-2210 mV for 4.0) are untouched from upstream and still
 * assume DFRobot's reference circuit, so "calph" likely won't recognize
 * a real buffer on this hardware. Use setCalibrationPoint() instead (see
 * its own comment) to set a new point without going through that check.
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [Jiawei Zhang](jiawei.zhang@dfrobot.com)
 * @version  V1.0
 * @date  2018-11-06
 * @url https://github.com/DFRobot/DFRobot_PH
 */


#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "DFRobot_PH.h"
#include <EEPROM.h>

#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]); EEPROM.commit();}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}

#define PHVALUEADDR 0x00    //the start address of the pH calibration parameters stored in the EEPROM

char* DFRobot_PH::strupr(char* str) {
    if (str == NULL) return NULL;
    char *ptr = str;
    while (*ptr != ' ') {
        *ptr = toupper((unsigned char)*ptr);
        ptr++;
    }
    return str;
}

DFRobot_PH::DFRobot_PH()
{
    this->_temperature    = 25.0;
    this->_phValue        = 6.86;
    // Basilience's own already-verified two-point calibration (see
    // Calibration.h), in the normal orientation - matches DFRobot's own
    // reference convention (higher voltage = lower pH; their defaults
    // are 2032.44 mV @ pH 4.0 vs 1500.0 mV @ pH 7.0, same pattern).
    this->_acidVoltage    = 2931.53;    //buffer solution 4.01 at 25C
    this->_neutralVoltage = 2562.40;    //buffer solution 6.86 at 25C
    this->_acidPh         = 4.01;
    this->_neutralPh      = 6.86;
    this->_voltage        = 2562.40;
}

DFRobot_PH::~DFRobot_PH()
{

}

void DFRobot_PH::begin()
{
    EEPROM_read(PHVALUEADDR, this->_neutralVoltage);  //load the higher-pH point's voltage from the EEPROM
    if(EEPROM.read(PHVALUEADDR)==0xFF && EEPROM.read(PHVALUEADDR+1)==0xFF && EEPROM.read(PHVALUEADDR+2)==0xFF && EEPROM.read(PHVALUEADDR+3)==0xFF){
        this->_neutralVoltage = 2562.40;  // blank EEPROM: Basilience's verified pH 6.86 point
        EEPROM_write(PHVALUEADDR, this->_neutralVoltage);
    }
    EEPROM_read(PHVALUEADDR+4, this->_acidVoltage);//load the lower-pH point's voltage from the EEPROM
    if(EEPROM.read(PHVALUEADDR+4)==0xFF && EEPROM.read(PHVALUEADDR+5)==0xFF && EEPROM.read(PHVALUEADDR+6)==0xFF && EEPROM.read(PHVALUEADDR+7)==0xFF){
        this->_acidVoltage = 2931.53;  // blank EEPROM: Basilience's verified pH 4.01 point
        EEPROM_write(PHVALUEADDR+4, this->_acidVoltage);
    }
    EEPROM_read(PHVALUEADDR+8, this->_neutralPh);  //load the higher-pH point's own pH value
    if(EEPROM.read(PHVALUEADDR+8)==0xFF && EEPROM.read(PHVALUEADDR+9)==0xFF && EEPROM.read(PHVALUEADDR+10)==0xFF && EEPROM.read(PHVALUEADDR+11)==0xFF){
        this->_neutralPh = 6.86;
        EEPROM_write(PHVALUEADDR+8, this->_neutralPh);
    }
    EEPROM_read(PHVALUEADDR+12, this->_acidPh);  //load the lower-pH point's own pH value
    if(EEPROM.read(PHVALUEADDR+12)==0xFF && EEPROM.read(PHVALUEADDR+13)==0xFF && EEPROM.read(PHVALUEADDR+14)==0xFF && EEPROM.read(PHVALUEADDR+15)==0xFF){
        this->_acidPh = 4.01;
        EEPROM_write(PHVALUEADDR+12, this->_acidPh);
    }
}

float DFRobot_PH::readPH(float voltage, float temperature)
{
    // Same two-point line as upstream, but generalized: the "-1500.0" and
    // "/3.0" are DFRobot's original normalization constants and cancel
    // out algebraically regardless of what the two anchor points are, so
    // this reduces to a standard two-point fit through
    // (_neutralVoltage,_neutralPh) and (_acidVoltage,_acidPh) - no longer
    // hardcoded to (x,7.0)/(x,4.0).
    float slope = (this->_neutralPh - this->_acidPh)/((this->_neutralVoltage-1500.0)/3.0 - (this->_acidVoltage-1500.0)/3.0);
    float intercept = this->_neutralPh - slope*(this->_neutralVoltage-1500.0)/3.0;
    this->_phValue = slope*(voltage-1500.0)/3.0+intercept;  //y = k*x + b
    return _phValue;
}


// Our own addition (not upstream) - see the header comment for why. No
// window check, no enterph/exitph state machine - just writes the point
// (and its actual buffer pH, not assumed to be 7.0/4.0) and commits it.
void DFRobot_PH::setCalibrationPoint(bool isNeutral, float voltage, float bufferPh)
{
    if (isNeutral)
    {
        this->_neutralVoltage = voltage;
        this->_neutralPh = bufferPh;
        EEPROM_write(PHVALUEADDR, this->_neutralVoltage);
        EEPROM_write(PHVALUEADDR+8, this->_neutralPh);
    }
    else
    {
        this->_acidVoltage = voltage;
        this->_acidPh = bufferPh;
        EEPROM_write(PHVALUEADDR+4, this->_acidVoltage);
        EEPROM_write(PHVALUEADDR+12, this->_acidPh);
    }
    Serial.print(F(">>>Stored pH "));
    Serial.print(bufferPh, 2);
    Serial.print(F(" point at "));
    Serial.print(voltage, 0);
    Serial.println(F(" mV<<<"));
}

void DFRobot_PH::calibration(float voltage, float temperature,char* cmd)
{
    this->_voltage = voltage;
    this->_temperature = temperature;
    String sCmd = String(cmd);
    sCmd.toUpperCase();
    phCalibration(cmdParse(sCmd.c_str()));  // if received Serial CMD from the serial monitor, enter into the calibration mode
}

void DFRobot_PH::calibration(float voltage, float temperature)
{
    this->_voltage = voltage;
    this->_temperature = temperature;
    if(cmdSerialDataAvailable() > 0){
        phCalibration(cmdParse());  // if received Serial CMD from the serial monitor, enter into the calibration mode
    }
}

boolean DFRobot_PH::cmdSerialDataAvailable()
{
    char cmdReceivedChar;
    static unsigned long cmdReceivedTimeOut = millis();
    while(Serial.available()>0){
        if(millis() - cmdReceivedTimeOut > 500U){
            this->_cmdReceivedBufferIndex = 0;
            memset(this->_cmdReceivedBuffer,0,(ReceivedBufferLength));
        }
        cmdReceivedTimeOut = millis();
        cmdReceivedChar = Serial.read();
        if (cmdReceivedChar == '\n' || this->_cmdReceivedBufferIndex==ReceivedBufferLength-1){
            this->_cmdReceivedBufferIndex = 0;
            strupr(this->_cmdReceivedBuffer);
            return true;
        }else{
            this->_cmdReceivedBuffer[this->_cmdReceivedBufferIndex] = cmdReceivedChar;
            this->_cmdReceivedBufferIndex++;
        }
    }
    return false;
}

byte DFRobot_PH::cmdParse(const char* cmd)
{
    byte modeIndex = 0;
    if(strstr(cmd, "ENTERPH")      != NULL){
        modeIndex = 1;
    }else if(strstr(cmd, "EXITPH") != NULL){
        modeIndex = 3;
    }else if(strstr(cmd, "CALPH")  != NULL){
        modeIndex = 2;
    }
    return modeIndex;
}

byte DFRobot_PH::cmdParse()
{
    byte modeIndex = 0;
    if(strstr(this->_cmdReceivedBuffer, "ENTERPH")      != NULL){
        modeIndex = 1;
    }else if(strstr(this->_cmdReceivedBuffer, "EXITPH") != NULL){
        modeIndex = 3;
    }else if(strstr(this->_cmdReceivedBuffer, "CALPH")  != NULL){
        modeIndex = 2;
    }
    return modeIndex;
}

void DFRobot_PH::phCalibration(byte mode)
{
    char *receivedBufferPtr;
    static boolean phCalibrationFinish  = 0;
    static boolean enterCalibrationFlag = 0;
    switch(mode){
        case 0:
        if(enterCalibrationFlag){
            Serial.println(F(">>>Command Error<<<"));
        }
        break;

        case 1:
        enterCalibrationFlag = 1;
        phCalibrationFinish  = 0;
        Serial.println();
        Serial.println(F(">>>Enter PH Calibration Mode<<<"));
        Serial.println(F(">>>Please put the probe into the 4.0 or 7.0 standard buffer solution<<<"));
        Serial.println();
        break;

        case 2:
        if(enterCalibrationFlag){
            if((this->_voltage>1322)&&(this->_voltage<1678)){        // buffer solution:7.0{
                Serial.println();
                Serial.print(F(">>>Buffer Solution:7.0"));
                this->_neutralVoltage =  this->_voltage;
                Serial.println(F(",Send EXITPH to Save and Exit<<<"));
                Serial.println();
                phCalibrationFinish = 1;
            }else if((this->_voltage>1854)&&(this->_voltage<2210)){  //buffer solution:4.0
                Serial.println();
                Serial.print(F(">>>Buffer Solution:4.0"));
                this->_acidVoltage =  this->_voltage;
                Serial.println(F(",Send EXITPH to Save and Exit<<<"));
                Serial.println();
                phCalibrationFinish = 1;
            }else{
                Serial.println();
                Serial.print(F(">>>Buffer Solution Error Try Again<<<"));
                Serial.println();                                    // not buffer solution or faulty operation
                phCalibrationFinish = 0;
            }
        }
        break;

        case 3:
        if(enterCalibrationFlag){
            Serial.println();
            if(phCalibrationFinish){
                if((this->_voltage>1322)&&(this->_voltage<1678)){

                    EEPROM_write(PHVALUEADDR, this->_neutralVoltage);
                }else if((this->_voltage>1854)&&(this->_voltage<2210)){
                    EEPROM_write(PHVALUEADDR+4, this->_acidVoltage);
                }
                Serial.print(F(">>>Calibration Successful"));
            }else{
                Serial.print(F(">>>Calibration Failed"));
            }
            Serial.println(F(",Exit PH Calibration Mode<<<"));
            Serial.println();
            phCalibrationFinish  = 0;
            enterCalibrationFlag = 0;
        }
        break;
    }
}
