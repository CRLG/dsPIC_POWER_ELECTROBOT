#ifndef _INPUTS_CALIBRATION_H_
#define _INPUTS_CALIBRATION_H_

void Init_Calibration();

void calib_computeCoefsBatteryVoltage();
void calib_setBatteryVoltagePoint1(unsigned short raw_value, unsigned short phys_value_mV);
void calib_setBatteryVoltagePoint2(unsigned short raw_value, unsigned short phys_value_mV);
unsigned short rawToPhysBatteryVoltage(unsigned short raw);


void calib_computeCoefsGlobalCurrent();
void calib_setGlobalCurrentPoint1(unsigned short raw_value, unsigned short phys_value_mA);
void calib_setGlobalCurrentPoint2(unsigned short raw_value, unsigned short phys_value_mA);
unsigned short rawToPhysGlobalCurrent(unsigned short raw);

void calib_computeCoefsCurrentOut1();
void calib_setCurrentOut1Point1(unsigned short raw_value, unsigned short phys_value_mA);
void calib_setCurrentOut1Point2(unsigned short raw_value, unsigned short phys_value_mA);
unsigned short rawToPhysCurrentOut1(unsigned short raw);

void calib_computeCoefsCurrentOut2();
void calib_setCurrentOut2Point1(unsigned short raw_value, unsigned short phys_value_mA);
void calib_setCurrentOut2Point2(unsigned short raw_value, unsigned short phys_value_mA);
unsigned short rawToPhysCurrentOut2(unsigned short raw);

#endif // _INPUTS_CALIBRATION_H_
