#include "General.h"
#include "eeprom.h"
#include "DEE_Emulation_16-bit.h"

unsigned short EEPROM_values[EEPROM_SIZE];

// _______________________________________________________
void forceEEPROMDefaultValues()
{
 EEPROM_values[EEPADDR_MAGIC_NUMBER] = EEPROM_MAGIC_NUMBER;

 EEPROM_values[EEPADDR_I2C_ADDRESS_8bits] = I2C_DEFAULT_ADDRESS_8bits;

 EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_1_RAW] = 1648;
 EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_1_PHYS_mV] = 8000;
 EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_2_RAW] = 3097;
 EEPROM_values[EEPADDR_CALIB_BATT_VOLTAGE_POINT_2_PHYS_mV] = 15000;

 EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_1_RAW] = 2068;
 EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_1_PHYS_mA] = 38;
 EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_2_RAW] = 2856;
 EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_2_PHYS_mA] = 10060;

 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_1_RAW] = 2069;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_1_PHYS_mA] = 0;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_2_RAW] = 2844;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_2_PHYS_mA] = 5014;

 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_1_RAW] = 2064;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_1_PHYS_mA] = 0;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_2_RAW] = 2835;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_2_PHYS_mA] = 5013;
}

// _______________________________________________________
void Init_EEPROM()
{
 DataEEInit();
 dataEEFlags.val = 0;

 readEEPROM(); 
}


// ___________________________________________
void resetFactoryEEPROM()
{
  forceEEPROMDefaultValues();
  saveEEPROM();
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
    dsPIC_reg[REG_EEPROM_WRITE_UNPROTECT].val = EEPROM_WRITE_UNPROTECT;   // Autorise les écritures en EEPROM
    resetFactoryEEPROM();
    dsPIC_reg[REG_EEPROM_WRITE_UNPROTECT].val = 0;  // Protège les écritures en EEPROM
 }
}

// ___________________________________________
void saveEEPROM()
{
 // Vérifie si l'EEPROM est bien dévérouillée avant d'écrire
 if (dsPIC_reg[REG_EEPROM_WRITE_UNPROTECT].val != EEPROM_WRITE_UNPROTECT) return;
 int i; 
 for (i=0; i<EEPROM_SIZE; i++) {
   DataEEWrite(EEPROM_values[i], i);
 }
}
