#include "eeprom.h"
#include "DEE_Emulation_16-bit.h"

unsigned short EEPROM_values[EEPROM_SIZE];

// _______________________________________________________
void forceEEPROMDefaultValues()
{
 EEPROM_values[EEPADDR_MAGIC_NUMBER] = EEPROM_MAGIC_NUMBER;

 // TODO : mettre des valeurs par défaut cohérentes prises sur une carte
 EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_1_RAW] = 0;
 EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_1_PHYS_mV] = 0;
 EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_2_RAW] = 0;
 EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_2_PHYS_mV] = 0;

 EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_1_RAW] = 0;
 EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_1_PHYS_mA] = 0;
 EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_2_RAW] = 0;
 EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_2_PHYS_mA] = 0;

 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_1_RAW] = 0;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_1_PHYS_mA] = 0;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_2_RAW] = 0;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_2_PHYS_mA] = 0;

 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_1_RAW] = 0;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_1_PHYS_mA] = 0;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_2_RAW] = 0;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_2_PHYS_mA] = 0;
}

// _______________________________________________________
void Init_EEPROM()
{
 DataEEInit();
 dataEEFlags.val = 0;

 readEEPROM(); 
}

// ___________________________________________
void readEEPROM()
{
 int i; 
 for (i=0; i<EEPROM_SIZE; i++) {
   EEPROM_values[i] = DataEERead(i);
 }
 // EEPROM corrupted or never initialized
 if (EEPROM_values[EEPADDR_MAGIC_NUMBER] != EEPROM_MAGIC_NUMBER) {
    forceEEPROMDefaultValues();
    saveEEPROM();
 }
}

// ___________________________________________
void saveEEPROM()
{
 int i; 
 for (i=0; i<EEPROM_SIZE; i++) {
   DataEEWrite(EEPROM_values[i], i);
 }
}
