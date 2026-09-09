/*!
 * @file DFRobot_PH.h
 * @brief Arduino library for Gravity: Analog pH Sensor / Meter Kit V2, SKU: SEN0161-V2
 *
 * Based on https://github.com/DFRobot/DFRobot_PH (master), adapted for
 * this bench tool. Two deviations from upstream, both documented at their
 * definitions in DFRobot_PH.cpp:
 *   1. EEPROM.commit() added (ESP32's EEPROM is flash-backed and needs an
 *      explicit commit - AVR's EEPROM.write() persists immediately).
 *   2. The two calibration anchors are no longer hardcoded to pH 7.0/4.0.
 *      setCalibrationPoint() now takes the buffer's actual pH value, and
 *      readPH() derives its slope/intercept from whatever two pH values
 *      were actually used - so Basilience's own already-verified
 *      calibration (pH 6.86 / pH 4.01 buffers, see Calibration.h) can be
 *      loaded directly, with no new physical calibration run required.
 *      Upstream's own enterph/calph/exitph flow (which does assume
 *      7.0/4.0) is left in place for anyone who wants to redo a
 *      calibration with true 7.0/4.0 buffers instead.
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [Jiawei Zhang](jiawei.zhang@dfrobot.com)
 * @version  V1.0
 * @date  2018-11-06
 * @url https://github.com/DFRobot/DFRobot_PH
 */

#ifndef _DFROBOT_PH_H_
#define _DFROBOT_PH_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define ReceivedBufferLength 10  //length of the Serial CMD buffer

class DFRobot_PH
{
public:
  DFRobot_PH();
  ~DFRobot_PH();
  /**
   * @fn calibration
   * @brief Calibrate the calibration data
   *
   * @param voltage     : Voltage value
   * @param temperature : Ambient temperature
   * @param cmd         : enterph -> enter the PH calibration mode
   * @n                   calph   -> calibrate with the standard buffer solution, two buffer solutions(4.0 and 7.0) will be automaticlly recognized
   * @n                   exitph  -> save the calibrated parameters and exit from PH calibration mode
   */
  void    calibration(float voltage, float temperature,char* cmd);  //calibration by Serial CMD
  void    calibration(float voltage, float temperature);
  /**
   * @fn readPH
   * @brief Convert voltage to PH with temperature compensation
   * @note voltage to pH value, with temperature compensation. Uses
   * whatever two (pH, voltage) anchors are currently loaded - see
   * setCalibrationPoint() - not a hardcoded 7.0/4.0.
   *
   * @param voltage     : Voltage value
   * @param temperature : Ambient temperature
   * @return The PH value
   */
  float   readPH(float voltage, float temperature);
  /**
   * @fn begin
   * @brief Initialization The Analog pH Sensor. Loads both calibration
   * points (voltage + their buffer pH) from EEPROM; on a blank/first-run
   * EEPROM, pre-seeds Basilience's own already-verified calibration
   * (pH 6.86 -> 2562.40 mV, pH 4.01 -> 2931.53 mV, see Calibration.h)
   * instead of DFRobot's generic 1500/2032.44 defaults, so this reads
   * correctly immediately, with no new physical calibration required.
   */
  void begin();
  /**
   * @fn setCalibrationPoint
   * @brief Our own addition, not upstream DFRobot code: bypasses
   * phCalibration()'s fixed voltage-recognition windows (tuned for
   * DFRobot's own 5V/1024-count reference circuit, not this ESP32/GPIO35
   * circuit) and writes a captured (pH, voltage) point straight to
   * EEPROM. The caller is trusting the probe was actually settled in the
   * stated buffer when this is called - there is no recognition check.
   *
   * @param isNeutral true to set the higher-pH point, false for the lower-pH point
   * @param voltage   the settled raw voltage (mV) to store for that point
   * @param bufferPh  the buffer solution's actual pH (e.g. 6.86 or 4.01) - does not have to be 7.0/4.0
   */
  void setCalibrationPoint(bool isNeutral, float voltage, float bufferPh);

private:
    float  _phValue;
    float  _acidVoltage;
    float  _neutralVoltage;
    float  _acidPh;
    float  _neutralPh;
    float  _voltage;
    float  _temperature;

    char   _cmdReceivedBuffer[ReceivedBufferLength];  //store the Serial CMD
    byte   _cmdReceivedBufferIndex;

private:
    boolean cmdSerialDataAvailable();
    void    phCalibration(byte mode); // calibration process, wirte key parameters to EEPROM
    byte    cmdParse(const char* cmd);
    byte    cmdParse();
	char* strupr(char* str);
};

#endif
