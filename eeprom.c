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

 EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_1_RAW] = 2066;
 EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_1_PHYS_mA] = 33;
 EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_2_RAW] = 2082;
 EEPROM_values[EEPADDR_CALIB_GLOBAL_CURRENT_POINT_2_PHYS_mA] = 200;

 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_1_RAW] = 2069;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_1_PHYS_mA] = 0;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_2_RAW] = 2100;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT1_POINT_2_PHYS_mA] = 200;

 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_1_RAW] = 2064;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_1_PHYS_mA] = 0;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_2_RAW] = 2096;
 EEPROM_values[EEPADDR_CALIB_CURRENT_OUT2_POINT_2_PHYS_mA] = 200;
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
  unsigned char protect_code_save;
  protect_code_save = dsPIC_reg[REG_EEPROM_WRITE_UNPROTECT].val;
  forceEEPROMDefaultValues();
  dsPIC_reg[REG_EEPROM_WRITE_UNPROTECT].val = EEPROM_WRITE_UNPROTECT;   // Autorise les �critures en EEPROM
  saveEEPROM();
  dsPIC_reg[REG_EEPROM_WRITE_UNPROTECT].val = protect_code_save;  // Prot�ge les �critures en EEPROM
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
     resetFactoryEEPROM();
 }
}

// ___________________________________________
void saveEEPROM()
{
 // V�rifie si l'EEPROM est bien d�v�rouill�e avant d'�crire
 if (dsPIC_reg[REG_EEPROM_WRITE_UNPROTECT].val != EEPROM_WRITE_UNPROTECT) return;
 int i; 
 for (i=0; i<EEPROM_SIZE; i++) {
   DataEEWrite(EEPROM_values[i], i);
 }
}
