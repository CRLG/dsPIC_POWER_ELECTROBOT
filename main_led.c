/**********************************************************************
* © 2010 Microchip Technology Inc.
*
* FileName:        main_led.c
* Dependencies:    p24HJ64GP502.h
* Processor:       PIC24H
* Compiler:        MPLAB® C30 v2.01 or higher
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Inc. (“Microchip”) licenses this software to you
* solely for use with Microchip dsPIC® digital signal controller
* products. The software is owned by Microchip and is protected under
* applicable copyright laws.  All rights reserved.
*
* SOFTWARE IS PROVIDED “AS IS.”  MICROCHIP EXPRESSLY DISCLAIMS ANY
* WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP
* BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
* DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
* PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
* BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
* ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
* ADDITIONAL NOTES:  
*
* Simple demo program showing a flashing LED on each port
*
**********************************************************************/

#if defined(__dsPIC33F__)
#include "p33Fxxxx.h"
#elif defined(__PIC24H__)
#include "p24Hxxxx.h"
#endif
#include <libpic30.h>

#include "I2CSlaveDrv.h"	//Include file for I2C1 Driver
#include "General.h"
#include "eeprom.h"
#include "inputs_calibration.h"
/******************************* 

Set device configuration values 

********************************/
#ifdef __PIC24HJ64GP502__
_FOSCSEL(FNOSC_FRC);								// set oscillator mode for FRC ~ 8 Mhz
_FOSC(FCKSM_CSDCMD & OSCIOFNC_ON & POSCMD_NONE);	// use OSCIO pin for RA3
_FWDT(FWDTEN_OFF);									// turn off watchdog
#elif defined(__dsPIC33FJ128MC802__)
_FOSCSEL(FNOSC_FRCPLL);								
_FOSC(FCKSM_CSDCMD & OSCIOFNC_ON & POSCMD_NONE);
_FWDT(FWDTEN_OFF);									
#endif


#define PERIOD  0x3FFF						// sets the default interval flash rate
#define FLASH_RATE_NORMAL 1						// smaller value yields faster rate (1-100)
#define FLASH_RATE_ERR_COM_MASTER 30        // flash rate if master is absent
#define FOREVER 1							// endless 

/* function prototypes */
void InitTimer1();							 

/* globals */
unsigned int Counter = 0;
volatile unsigned int timer_expired;

unsigned short AnalogInput[4];

void Init_Ports(void);
void Init_Analog(void);
void Init_Registers(void);

unsigned short getAnalog(unsigned char voie);
unsigned short LectureEanaMoyennee(unsigned char voie);

void ControleSTOR1(unsigned char val);
void ControleSTOR2(unsigned char val);
void ControleSTOR3(unsigned char val);
void ControleSTOR4(unsigned char val);
void ControleSTOR5(unsigned char val);
void ControleSTOR6(unsigned char val);
void ControleSTOR7(unsigned char val);
void ControleSTOR8(unsigned char val);

T_dsPIC_REGISTER dsPIC_reg[MAX_REGISTRES_NUMBER];
    
    
#define SIZE_SAMPLE_MOY 50
typedef struct {
    unsigned short samples_tab[SIZE_SAMPLE_MOY];
    unsigned short value;
}tMoyenneGlissante;   
  
tMoyenneGlissante Moyenne_BatteryVoltage;    
tMoyenneGlissante Moyenne_GlobalCurrent;    
tMoyenneGlissante Moyenne_CurrentOut1;    
tMoyenneGlissante Moyenne_CurrentOut2;    

void CalculMoyenneGlissante(tMoyenneGlissante *moy, unsigned short new_value);

unsigned int value1, value2, value3;


/********************************* 

	main entry point

*********************************/

int main ( void )
{
 unsigned char ucval;
 unsigned short usval;
 unsigned short flash_speed;
 unsigned char perteComMaster=0;

 
 Init_EEPROM(); 
 Init_Calibration();
 Init_Ports();
 Init_Registers();
 Init_Analog();
 InitTimer1();
 i2c1_init(dsPIC_reg[REG_I2C_8BITS_ADDRESS].val);  // restitution de la valeur configurée en EEPROM
 
 // A l'init, tous les moteurs sont éteints
 ControleSTOR1(0);
 ControleSTOR2(0);
 ControleSTOR3(0);
 ControleSTOR4(0);
 ControleSTOR5(0);
 ControleSTOR6(0);
 ControleSTOR7(0);
 ControleSTOR8(0);

 value1 = LectureEanaMoyennee(5);
 value3 = LectureEanaMoyennee(4);
	/*  endless loop*/
	while (FOREVER)
	{

        // Diag de perte de comm avec le master
        if (timer_expired) {
ENTER_CRITICAL_SECTION_I2C()
            if (cptPerteComMaster!=0xFFFF) { cptPerteComMaster++; }
            perteComMaster = (cptPerteComMaster > 100);
LEAVE_CRITICAL_SECTION_I2C()
            flash_speed = (perteComMaster==0)?FLASH_RATE_NORMAL:FLASH_RATE_ERR_COM_MASTER;
        }
        
		if (timer_expired && (Counter >= flash_speed) )
		{
            LATAbits.LATA4 = !LATAbits.LATA4; 
		    Counter = 0;
			timer_expired = 0;
		}

        // TODO : 
        // Sur perte de communication, couper toutes les sorties
        
		//AnalogInput[0] = getAnalog(0);
        CalculMoyenneGlissante(&Moyenne_BatteryVoltage, getAnalog(0));
        AnalogInput[0] = rawToPhysBatteryVoltage(Moyenne_BatteryVoltage.value);
        
		//AnalogInput[1] = getAnalog(1);
        CalculMoyenneGlissante(&Moyenne_GlobalCurrent, getAnalog(1));
        AnalogInput[1] = rawToPhysGlobalCurrent(Moyenne_GlobalCurrent.value);
                
		//AnalogInput[2] = getAnalog(4);
        //AnalogInput[2] = value3;
        //LectureEanaMoyennee(4);
        CalculMoyenneGlissante(&Moyenne_CurrentOut1, getAnalog(4));
        AnalogInput[2] = rawToPhysCurrentOut1(Moyenne_CurrentOut1.value);

        //AnalogInput[3] = getAnalog(5);
        CalculMoyenneGlissante(&Moyenne_CurrentOut2, getAnalog(5));
        AnalogInput[3] = rawToPhysCurrentOut2(Moyenne_CurrentOut2.value);
        
ENTER_CRITICAL_SECTION_I2C()
        dsPIC_reg[REG_EANA_VBAT_H].val = AnalogInput[0]>>8;
        dsPIC_reg[REG_EANA_VBAT_L].val = AnalogInput[0];
        dsPIC_reg[REG_EANA_MEAS_CURR_VBAT_H].val = AnalogInput[1]>>8;
        dsPIC_reg[REG_EANA_MEAS_CURR_VBAT_L].val = AnalogInput[1];
        dsPIC_reg[REG_EANA_MEAS_CURR_1_H].val = AnalogInput[2]>>8;
        dsPIC_reg[REG_EANA_MEAS_CURR_1_L].val = AnalogInput[2];
        dsPIC_reg[REG_EANA_MEAS_CURR_2_H].val = AnalogInput[3]>>8;
        dsPIC_reg[REG_EANA_MEAS_CURR_2_L].val = AnalogInput[3];
LEAVE_CRITICAL_SECTION_I2C()
        // _____________________________________________________
        // Configuration adresse I2C
        if (dsPIC_reg[REG_I2C_8BITS_ADDRESS].new_data) {
ENTER_CRITICAL_SECTION_I2C()            
            ucval = dsPIC_reg[REG_I2C_8BITS_ADDRESS].val;
            dsPIC_reg[REG_I2C_8BITS_ADDRESS].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            EEPROM_values[EEPADDR_I2C_ADDRESS_8bits] = ucval;  // un reboot de la carte sera nécessaire. Pas de prise en compte immédiat pour ce changement
            saveEEPROM();  // l'EEPROM devra avoir été dévérouillée en écriture préalablement
         }    
        // _____________________________________________________
        // Configuration adresse I2C
        if (dsPIC_reg[REG_I2C_8BITS_ADDRESS].new_data) {
ENTER_CRITICAL_SECTION_I2C()            
            ucval = dsPIC_reg[REG_I2C_8BITS_ADDRESS].val;
            dsPIC_reg[REG_I2C_8BITS_ADDRESS].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            EEPROM_values[EEPADDR_I2C_ADDRESS_8bits] = ucval;  // un reboot de la carte sera nécessaire. Pas de prise en compte immédiat pour ce changement
            saveEEPROM();  // l'EEPROM devra avoir été dévérouillée en écriture préalablement
         }    
        // _____________________________________________________
        // Recherche une nouvelle demande de RAZ EEPROM à la configuration par défaut
        if (dsPIC_reg[REG_EEPROM_RESET_FACTORY].new_data) {
ENTER_CRITICAL_SECTION_I2C()            
            ucval = dsPIC_reg[REG_EEPROM_RESET_FACTORY].val;
            dsPIC_reg[REG_EEPROM_RESET_FACTORY].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            if (ucval == EEPROM_RESET_FACTORY_CODE) {
                resetFactoryEEPROM();   // l'EEPROM devra avoir été dévérouillée en écriture préalablement
            }            
         }    
        // _____________________________________________________
        // Recherche une nouvelle demande de sortie TOR
        if (dsPIC_reg[REG_STOR_2].new_data) {
ENTER_CRITICAL_SECTION_I2C()            
            ucval = dsPIC_reg[REG_STOR_2].val;
            dsPIC_reg[REG_STOR_2].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR2(ucval);            
         }    
        // _____________________________________________________
        // Recherche une nouvelle demande de sortie TOR
        if (dsPIC_reg[REG_STOR_3].new_data) {
ENTER_CRITICAL_SECTION_I2C()            
            ucval = dsPIC_reg[REG_STOR_3].val;
            dsPIC_reg[REG_STOR_3].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR3(ucval);            
         }    
        // _____________________________________________________
        // Recherche une nouvelle demande de sortie TOR
        if (dsPIC_reg[REG_STOR_4].new_data) {
ENTER_CRITICAL_SECTION_I2C()            
            ucval = dsPIC_reg[REG_STOR_4].val;
            dsPIC_reg[REG_STOR_4].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR4(ucval);            
         }    
        // _____________________________________________________
        // Recherche une nouvelle demande de sortie TOR
        if (dsPIC_reg[REG_STOR_5].new_data) {
ENTER_CRITICAL_SECTION_I2C()            
            ucval = dsPIC_reg[REG_STOR_5].val;
            dsPIC_reg[REG_STOR_5].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR5(ucval);            
         }    
        // _____________________________________________________
        // Recherche une nouvelle demande de sortie TOR
        if (dsPIC_reg[REG_STOR_6].new_data) {
ENTER_CRITICAL_SECTION_I2C()            
            ucval = dsPIC_reg[REG_STOR_6].val;
            dsPIC_reg[REG_STOR_6].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR6(ucval);            
         }    
        // _____________________________________________________
        // Recherche une nouvelle demande de sortie TOR
        if (dsPIC_reg[REG_STOR_7].new_data) {
ENTER_CRITICAL_SECTION_I2C()            
            ucval = dsPIC_reg[REG_STOR_7].val;
            dsPIC_reg[REG_STOR_7].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR7(ucval);            
         }    
        // _____________________________________________________
        // Recherche une nouvelle demande de sortie TOR
        if (dsPIC_reg[REG_STOR_8].new_data) {
ENTER_CRITICAL_SECTION_I2C()            
            ucval = dsPIC_reg[REG_STOR_8].val;
            dsPIC_reg[REG_STOR_8].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR8(ucval);            
         }
        // _____________________________________________________
        // Recherche une nouvelle demande de calibration 
        if (dsPIC_reg[REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_1_H].new_data && dsPIC_reg[REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_1_L].new_data){
ENTER_CRITICAL_SECTION_I2C()            
            usval = ((dsPIC_reg[REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_1_H].val)<<8) + dsPIC_reg[REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_1_L].val ;
            dsPIC_reg[REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_1_H].new_data = 0;
            dsPIC_reg[REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_1_L].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR1(1);
            calib_setBatteryVoltagePoint1(Moyenne_BatteryVoltage.value, usval);            
         }
        // _____________________________________________________
        // Recherche une nouvelle demande de calibration 
        if (dsPIC_reg[REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_2_H].new_data && dsPIC_reg[REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_2_L].new_data){
ENTER_CRITICAL_SECTION_I2C()            
            usval = ((dsPIC_reg[REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_2_H].val)<<8) + dsPIC_reg[REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_2_L].val ;
            dsPIC_reg[REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_2_H].new_data = 0;
            dsPIC_reg[REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_2_L].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR2(1);
            calib_setBatteryVoltagePoint2(Moyenne_BatteryVoltage.value, usval);            
         }
        // _____________________________________________________
        // Recherche une nouvelle demande de calibration 
        if (dsPIC_reg[REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_1_H].new_data && dsPIC_reg[REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_1_L].new_data){
ENTER_CRITICAL_SECTION_I2C()            
            usval = ((dsPIC_reg[REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_1_H].val)<<8) + dsPIC_reg[REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_1_L].val ;
            dsPIC_reg[REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_1_H].new_data = 0;
            dsPIC_reg[REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_1_L].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR3(1);
            calib_setGlobalCurrentPoint1(Moyenne_GlobalCurrent.value, usval);            
         }
        // _____________________________________________________
        // Recherche une nouvelle demande de calibration 
        if (dsPIC_reg[REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_2_H].new_data && dsPIC_reg[REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_2_L].new_data){
ENTER_CRITICAL_SECTION_I2C()            
            usval = ((dsPIC_reg[REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_2_H].val)<<8) + dsPIC_reg[REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_2_L].val ;
            dsPIC_reg[REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_2_H].new_data = 0;
            dsPIC_reg[REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_2_L].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR4(1);
            calib_setGlobalCurrentPoint2(Moyenne_GlobalCurrent.value, usval);            
         }
        // _____________________________________________________
        // Recherche une nouvelle demande de calibration 
        if (dsPIC_reg[REG_CALIB_CURRENT_OUT1_PHYS_POINT_1_H].new_data && dsPIC_reg[REG_CALIB_CURRENT_OUT1_PHYS_POINT_1_L].new_data){
ENTER_CRITICAL_SECTION_I2C()            
            usval = ((dsPIC_reg[REG_CALIB_CURRENT_OUT1_PHYS_POINT_1_H].val)<<8) + dsPIC_reg[REG_CALIB_CURRENT_OUT1_PHYS_POINT_1_L].val ;
            dsPIC_reg[REG_CALIB_CURRENT_OUT1_PHYS_POINT_1_H].new_data = 0;
            dsPIC_reg[REG_CALIB_CURRENT_OUT1_PHYS_POINT_1_L].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR5(1);
            calib_setCurrentOut1Point1(Moyenne_CurrentOut1.value, usval);            
         }
        // _____________________________________________________
        // Recherche une nouvelle demande de calibration 
        if (dsPIC_reg[REG_CALIB_CURRENT_OUT1_PHYS_POINT_2_H].new_data && dsPIC_reg[REG_CALIB_CURRENT_OUT1_PHYS_POINT_2_L].new_data){
ENTER_CRITICAL_SECTION_I2C()            
            usval = ((dsPIC_reg[REG_CALIB_CURRENT_OUT1_PHYS_POINT_2_H].val)<<8) + dsPIC_reg[REG_CALIB_CURRENT_OUT1_PHYS_POINT_2_L].val ;
            dsPIC_reg[REG_CALIB_CURRENT_OUT1_PHYS_POINT_2_H].new_data = 0;
            dsPIC_reg[REG_CALIB_CURRENT_OUT1_PHYS_POINT_2_L].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR6(1);
            calib_setCurrentOut1Point2(Moyenne_CurrentOut1.value, usval);            
         }
        // _____________________________________________________
        // Recherche une nouvelle demande de calibration 
        if (dsPIC_reg[REG_CALIB_CURRENT_OUT2_PHYS_POINT_1_H].new_data && dsPIC_reg[REG_CALIB_CURRENT_OUT2_PHYS_POINT_1_L].new_data){
ENTER_CRITICAL_SECTION_I2C()            
            usval = ((dsPIC_reg[REG_CALIB_CURRENT_OUT2_PHYS_POINT_1_H].val)<<8) + dsPIC_reg[REG_CALIB_CURRENT_OUT2_PHYS_POINT_1_L].val ;
            dsPIC_reg[REG_CALIB_CURRENT_OUT2_PHYS_POINT_1_H].new_data = 0;
            dsPIC_reg[REG_CALIB_CURRENT_OUT2_PHYS_POINT_1_L].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR7(1);
            calib_setCurrentOut2Point1(Moyenne_CurrentOut2.value, usval);            
         }
        // _____________________________________________________
        // Recherche une nouvelle demande de calibration 
        if (dsPIC_reg[REG_CALIB_CURRENT_OUT2_PHYS_POINT_2_H].new_data && dsPIC_reg[REG_CALIB_CURRENT_OUT2_PHYS_POINT_2_L].new_data){
ENTER_CRITICAL_SECTION_I2C()            
            usval = ((dsPIC_reg[REG_CALIB_CURRENT_OUT2_PHYS_POINT_2_H].val)<<8) + dsPIC_reg[REG_CALIB_CURRENT_OUT2_PHYS_POINT_2_L].val ;
            dsPIC_reg[REG_CALIB_CURRENT_OUT2_PHYS_POINT_2_H].new_data = 0;
            dsPIC_reg[REG_CALIB_CURRENT_OUT2_PHYS_POINT_2_L].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleSTOR8(1);
            calib_setCurrentOut2Point2(Moyenne_CurrentOut2.value, usval);            
         }
	}
}

/*---------------------------------------------------------------------
  Function Name: InitTimer1
  Description:   Initialize Timer1 for 1 second intervals
  Inputs:        None
  Returns:       None
-----------------------------------------------------------------------*/
void InitTimer1( void )
{
	T1CON = 0;						/* ensure Timer 1 is in reset state */
 	IFS0bits.T1IF = 0;				/* reset Timer 1 interrupt flag */
	IPC0bits.T1IP = 4;				/* set Timer1 interrupt priority level to 4 */
 	IEC0bits.T1IE = 1;				/* enable Timer 1 interrupt */
	PR1 = PERIOD;					/* set Timer 1 period register */
	T1CONbits.TCKPS = 2;			/* select Timer1 Input Clock Prescale */
	T1CONbits.TCS = 0;			 	/* select external timer clock */
	T1CONbits.TON = 1;			 	/* enable Timer 1 and start the count */ 
	
}


/*---------------------------------------------------------------------
  Function Name: _T1Interrupt
  Description:   Timer1 Interrupt Handler
  Inputs:        None
  Returns:       None
-----------------------------------------------------------------------*/
void __attribute__((interrupt, auto_psv)) _T1Interrupt( void )
{


	timer_expired = 1;				/* flag */
	Counter++;						/* keep a running counter */
 	IFS0bits.T1IF = 0;				/* reset timer interrupt flag	*/

}	
	

// ___________________________________________________________
void Init_Ports(void)
{
    /* 	Initialize ports */
    LATA  = 0x0000;             // set latch levels
    TRISA = 0xFFFF;             // set IO as inputs
    // Configuration de la LED sur la carte MICTOSTICK
    TRISAbits.TRISA4 = 0;       // set IO as outputs

    // STOR1
    LATBbits.LATB6  = 0;       // set latch levels
    TRISBbits.TRISB6 = 0;      // set IO as outputs

    // STOR2
    LATBbits.LATB7  = 0;       // set latch levels
    TRISBbits.TRISB7 = 0;      // set IO as outputs

    // STOR3
    LATBbits.LATB10  = 0;       // set latch levels
    TRISBbits.TRISB10 = 0;      // set IO as outputs

    // STOR4
    LATBbits.LATB11  = 0;       // set latch levels
    TRISBbits.TRISB11 = 0;      // set IO as outputs

    // STOR5
    LATBbits.LATB12  = 0;       // set latch levels
    TRISBbits.TRISB12 = 0;      // set IO as outputs

    // STOR6
    LATBbits.LATB13  = 0;       // set latch levels
    TRISBbits.TRISB13 = 0;      // set IO as outputs

    // STOR7
    LATBbits.LATB14  = 0;       // set latch levels
    TRISBbits.TRISB14 = 0;      // set IO as outputs

    // STOR8
    LATBbits.LATB15  = 0;       // set latch levels
    TRISBbits.TRISB15 = 0;      // set IO as outputs
}



// ___________________________________________________________
void Init_Analog(void)
{
//  AD1CON1bits.AD12B = 0;        // Select 10-bit mode
  AD1CON1bits.AD12B = 1;        // Select 12-bit mode

  AD1CON1bits.SSRC = 0;         // Conversion manuelle
  AD1CON1bits.ASAM = 0;         // Sample manuel
   
  AD1CON3bits.ADCS = 10;
  
  AD1PCFGL = 0xCC;              // AN0/AN1/AN4/AN5 sont des entrées analogiques
  
  AD1CON1bits.ADON = 1;         // Active le module ADC   
}


// ___________________________________________________________
unsigned short getAnalog(unsigned char voie)
{
  unsigned short ADCValue;
 // TODO Sélectionne l'entrée à convertir
 AD1CHS0bits.CH0SA = voie;
 __delay32(15000); // essai
 AD1CON1bits.SAMP = 1; // Start sampling
 // for (i=0; i<3000; i++);
 __delay32(85000);
 //DelayUs(10); // Wait for sampling time (10us)
 AD1CON1bits.SAMP = 0; // Start the conversion
 while (!AD1CON1bits.DONE); // Wait for the conversion to complete
 ADCValue = ADC1BUF0; // Read the conversion result
 return(ADCValue);  
}      

// _______________________________________________________
unsigned short LectureEanaMoyennee(unsigned char voie)
{
    const unsigned char TAILLE_MOY = 100;
    unsigned long sum=0;
    unsigned int i;
    
    for (i=0; i< TAILLE_MOY; i++) {
        sum += getAnalog(voie);
        
        __delay32(3000);
    }
    return (sum/TAILLE_MOY);
}



// ___________________________________________________________
void Init_Registers(void)
{
  unsigned short i=0;
  
  // Initialise de manière massive tous les registres
  for (i=0; i<MAX_REGISTRES_NUMBER; i++) {
     dsPIC_reg[i].val               = 0;
     dsPIC_reg[i].new_data          = 0;
     dsPIC_reg[i].type_read_write   = READ_ONLY; 
  }
  // Les registres Read/write
  for (i=REG_STOR_1; i<MAX_REGISTRES_NUMBER; i++) {
    dsPIC_reg[i].type_read_write = READ_WRITE;     
  }
  
  // Initialise les valeurs par défaut
  dsPIC_reg[REG_VERSION_SOFT_MAJ].val               = VERSION_SOFT_MAJ;
  dsPIC_reg[REG_VERSION_SOFT_MIN].val               = VERSION_SOFT_MIN;
  dsPIC_reg[REG_PTR_REG_LECTURE_I2C].val            = REG_EANA_VBAT_H;
  dsPIC_reg[REG_NBRE_REGISTRES_LECTURE_I2C].val     = 8; // Nombre de registres lus par le MBED lors d'une opération de lecture
  dsPIC_reg[REG_EEPROM_WRITE_UNPROTECT].val         = 0;  // EEPROM protégée en écriture
  dsPIC_reg[REG_I2C_8BITS_ADDRESS].val              = EEPROM_values[EEPADDR_I2C_ADDRESS_8bits]; // restitution de la valeur configurée en eeprom
}   



// _______________________________________________________
void ControleSTOR1(unsigned char val)
{
    LATBbits.LATB6 = val;
}
// _______________________________________________________
void ControleSTOR2(unsigned char val)
{
    LATBbits.LATB7 = val;
}
// _______________________________________________________
void ControleSTOR3(unsigned char val)
{
    LATBbits.LATB10 = val;
}
// _______________________________________________________
void ControleSTOR4(unsigned char val)
{
    LATBbits.LATB11 = val;
}
// _______________________________________________________
void ControleSTOR5(unsigned char val)
{
    LATBbits.LATB12 = val;
}
// _______________________________________________________
void ControleSTOR6(unsigned char val)
{
    LATBbits.LATB13 = val;
}
// _______________________________________________________
void ControleSTOR7(unsigned char val)
{
    LATBbits.LATB14 = val;
}
// _______________________________________________________
void ControleSTOR8(unsigned char val)
{
    LATBbits.LATB15 = val;
}

// _______________________________________________________
void CalculMoyenneGlissante(tMoyenneGlissante *moy, unsigned short new_value)
{
    int i;
    for (i=0; i<SIZE_SAMPLE_MOY-1; i++) {
        moy->samples_tab[i] = moy->samples_tab[i+1];
    }
    moy->samples_tab[SIZE_SAMPLE_MOY-1] = new_value;

    unsigned long sum=0;
    for (i=0; i<SIZE_SAMPLE_MOY; i++) {
        sum += moy->samples_tab[i];
    }
    moy->value = sum/SIZE_SAMPLE_MOY;
}

