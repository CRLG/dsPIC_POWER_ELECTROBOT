#include "inputs_calibration.h"
#include "eeprom.h"

// ===========================================

// ===========================================
void Init_Calibration()
{
   calib_computeCoefsBatteryVoltage();
   calib_computeCoefsGlobalCurrent();
   calib_computeCoefsCurrentOut1();
   calib_computeCoefsCurrentOut2();
}



// ===========================================
//                BATTERY VOLTAGE
// ===========================================
// Point1 : [raw1 ; U_phys1]
// Point2 : [raw2 ; U_phys2]
// U_phys = a*raw_value + b   U_phys [mV] / raw [0-4096]]
// a = (U_phys1 - U_phys2) / (raw1 - raw2)
// b = U_phys1 - raw1 * a
static float BatteryVoltage_CoefA = 1.23f;
static float BatteryVoltage_CoefB;

void calib_computeCoefsBatteryVoltage()
{
    BatteryVoltage_CoefA = ((float)EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_1_PHYS_mV] - (float)EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_2_PHYS_mV]) / ((float)EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_1_RAW] - (float)EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_2_RAW]);
    BatteryVoltage_CoefB = (float)EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_1_PHYS_mV] - (BatteryVoltage_CoefA * EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_1_RAW]);
}

void calib_setBatteryVoltagePoint1(unsigned short raw_value, unsigned short phys_value_mV)
{
    EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_1_RAW] = raw_value;
    EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_1_PHYS_mV] = phys_value_mV;
    calib_computeCoefsBatteryVoltage();
    saveEEPROM();
}

void calib_setBatteryVoltagePoint2(unsigned short raw_value, unsigned short phys_value_mV)
{
    EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_2_RAW] = raw_value;
    EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_2_PHYS_mV] = phys_value_mV;
    calib_computeCoefsBatteryVoltage();
    saveEEPROM();
}

unsigned short rawToPhysBatteryVoltage(unsigned short raw)
{
    return (BatteryVoltage_CoefA * raw) + BatteryVoltage_CoefB;
}

// ___________________________________________



// ===========================================
//                GLOBAL CURRENT
// ===========================================
// Point1 : [raw1 ; I_phys1]
// Point2 : [raw2 ; I_phys2]
// I_phys = a*raw_value + b   I_phys [mA] / raw [0-4096]]
// a = (I_phys1 - I_phys2) / (raw1 - raw2)
// b = I_phys1 - raw1 * a
static float GlobalCurrent_CoefA;
static float GlobalCurrent_CoefB;

void calib_computeCoefsGlobalCurrent()
{
    GlobalCurrent_CoefA = (float)(EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_1_PHYS_mA] - EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_2_PHYS_mA]) / (EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_1_RAW] - EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_2_RAW]);
    GlobalCurrent_CoefB = (float)EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_1_PHYS_mA] - (GlobalCurrent_CoefA * EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_1_RAW]);
}

void calib_setGlobalCurrentPoint1(unsigned short raw_value, unsigned short phys_value_mA)
{
    EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_1_RAW] = raw_value;
    EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_1_PHYS_mA] = phys_value_mA;
    calib_computeCoefsGlobalCurrent();
    saveEEPROM();
}

void calib_setGlobalCurrentPoint2(unsigned short raw_value, unsigned short phys_value_mA)
{
    EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_2_RAW] = raw_value;
    EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_2_PHYS_mA] = phys_value_mA;
    calib_computeCoefsGlobalCurrent();
    saveEEPROM();
}

unsigned short rawToPhysGlobalCurrent(unsigned short raw)
{
    return (GlobalCurrent_CoefA * raw) + GlobalCurrent_CoefB;
}


// ===========================================
//                CURRENT OUT 1
// ===========================================
// Point1 : [raw1 ; I_phys1]
// Point2 : [raw2 ; I_phys2]
// I_phys = a*raw_value + b   I_phys [mA] / raw [0-4096]]
// a = (I_phys1 - I_phys2) / (raw1 - raw2)
// b = I_phys1 - raw1 * a
static float CurrentOut1_CoefA;
static float CurrentOut1_CoefB;

void calib_computeCoefsCurrentOut1()
{
    CurrentOut1_CoefA = (float)(EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_1_PHYS_mA] - EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_2_PHYS_mA]) / (EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_1_RAW] - EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_2_RAW]);
    CurrentOut1_CoefB = (float)EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_1_PHYS_mA] - (CurrentOut1_CoefA * EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_1_RAW]);
}

void calib_setCurrentOut1Point1(unsigned short raw_value, unsigned short phys_value_mA)
{
    EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_1_RAW] = raw_value;
    EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_1_PHYS_mA] = phys_value_mA;
    calib_computeCoefsCurrentOut1();
    saveEEPROM();
}

void calib_setCurrentOut1Point2(unsigned short raw_value, unsigned short phys_value_mA)
{
    EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_2_RAW] = raw_value;
    EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_2_PHYS_mA] = phys_value_mA;
    calib_computeCoefsCurrentOut1();
    saveEEPROM();
}

unsigned short rawToPhysCurrentOut1(unsigned short raw)
{
    return (CurrentOut1_CoefA * raw) + CurrentOut1_CoefB;
}

// ===========================================
//                CURRENT OUT 2
// ===========================================
// Point1 : [raw1 ; I_phys1]
// Point2 : [raw2 ; I_phys2]
// I_phys = a*raw_value + b   I_phys [mA] / raw [0-4096]]
// a = (I_phys1 - I_phys2) / (raw1 - raw2)
// b = I_phys1 - raw1 * a
static float CurrentOut2_CoefA;
static float CurrentOut2_CoefB;

void calib_computeCoefsCurrentOut2()
{
    CurrentOut2_CoefA = (float)(EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_1_PHYS_mA] - EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_2_PHYS_mA]) / (EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_1_RAW] - EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_2_RAW]);
    CurrentOut2_CoefB = (float)EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_1_PHYS_mA] - (CurrentOut2_CoefA * EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_1_RAW]);
}

void calib_setCurrentOut2Point1(unsigned short raw_value, unsigned short phys_value_mA)
{
    EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_1_RAW] = raw_value;
    EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_1_PHYS_mA] = phys_value_mA;
    calib_computeCoefsCurrentOut2();
    saveEEPROM();
}

void calib_setCurrentOut2Point2(unsigned short raw_value, unsigned short phys_value_mA)
{
    EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_2_RAW] = raw_value;
    EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_2_PHYS_mA] = phys_value_mA;
    calib_computeCoefsCurrentOut2();
    saveEEPROM();
}

unsigned short rawToPhysCurrentOut2(unsigned short raw)
{
    return (CurrentOut2_CoefA * raw) + CurrentOut2_CoefB;
}