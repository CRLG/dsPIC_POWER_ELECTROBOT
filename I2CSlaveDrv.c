/**********************************************************************
* � 2005 Microchip Technology Inc.
*
* FileName:        main.c
* Dependencies:    Header (.h) files if applicable, see below
* Processor:       dsPIC33Fxxxx/PIC24Hxxxx
* Compiler:        MPLAB� C30 v3.00 or higher
* Tested On:	   dsPIC33FJ256GP710
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author            	Date      Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Pinakin K. Makwana	09/02/2007	  First release of source file
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* ADDITIONAL NOTES: 
* This code is tested on Explorer 16 board with dsPIC33FJ256GP710
* Read "CE145_Readme.txt" file for more information on how to use this code example in your
* application.
**********************************************************************/

#if defined(__dsPIC33F__)
#include "p33Fxxxx.h"
#elif defined(__PIC24H__)
#include "p24Hxxxx.h"
#endif

#include "I2CSlaveDrv.h"
#include "General.h"

#define SIZE_RAM_BUFFER 200

unsigned char ucRAMBuffer[SIZE_RAM_BUFFER];	//RAM area which will work as EEPROM for Master I2C device
unsigned char *RAMPtr;			//Pointer to RAM memory locations
unsigned short cptPerteComMaster=0;
struct FlagType Flag;

// Prototype des fonctions locales
void PrepareBufferToSend(void);
void GestionReceptionI2C(unsigned char first, unsigned char data);
void FinReceptionTrameValideI2C(void);

/*****************************************************************
		Init I2C1 Bus
 * Param i2c_addr is 8 bits address. 
 * dsPIC register expect 7 bits address
*****************************************************************/
void i2c1_init(unsigned char i2c_addr)
{
	#if !defined(USE_I2C_Clock_Stretch)
		I2C1CON = 0x8000;	//Enable I2C1 module
	#else
		I2C1CON = 0x9040;	//Enable I2C1 module, enable clock stretching
	#endif
	

	I2C1ADD = i2c_addr>>1; // 7-bit I2C slave address must be initialised in register.
	
	IFS1=0;
	Flag.AddrFlag = 0;	//Initlize AddFlag
	Flag.DataFlag = 0;	//Initlize DataFlag
	_SI2C1IE = 1;
}

/*
Function Name: SI2C1Interrupt
Description : This is the ISR for I2C1 Slave interrupt.
Arguments	 : None
*/
void __attribute__((interrupt,no_auto_psv)) _SI2C1Interrupt(void)
{
	unsigned char Temp;	//used for dummy read
    static unsigned int indexRAM=0;
    
    // Requ�te d'�criture: juste apr�s l'adresse I2C reconnue
	if((I2C1STATbits.R_W == 0)&&(I2C1STATbits.D_A == 0))	//Address matched
	{
    	GestionReceptionI2C(1, I2C1RCV);  // Initialise la machine d'�tat + dummy read (rien � lire, il faut attendre la r�ception des octets suivants)
	}
	else if((I2C1STATbits.R_W == 0)&&(I2C1STATbits.D_A == 1))	//check for data	
	{
		GestionReceptionI2C(0, I2C1RCV);
		#if defined(USE_I2C_Clock_Stretch)
			I2C1CONbits.SCLREL = 1;	//Release SCL1 line
		#endif
	}
	// Requ�te de lecture : juste apr�s l'adresse I2C reconnue	
	else if((I2C1STATbits.R_W == 1)&&(I2C1STATbits.D_A == 0))
	{
		Temp = I2C1RCV;
		PrepareBufferToSend();  // Pr�pare un buffer � envoyer
        indexRAM = 0;
		I2C1TRN = ucRAMBuffer[indexRAM++];  // Renvoie le 1er octet du buffer
        I2C1CONbits.SCLREL = 1;	//Release SCL1 line
		while(I2C1STATbits.TBF);//Wait till all
        cptPerteComMaster = 0; // requ�te lecture -> le master i2c est bien pr�sent 
	}
	// Requ�te de lecture: la suite des octets
	else
	{
		I2C1TRN = ucRAMBuffer[indexRAM++];
        I2C1CONbits.SCLREL = 1;	//Release SCL1 line
		while(I2C1STATbits.TBF);//Wait till all
	}
	_SI2C1IF = 0;	//clear I2C1 Slave interrupt flag
}	




// ________________________________________________________
// Fonction appel�e sur une requ�te de lecture
// Juste apr�s que l'adresse I2C ai �t� reconnue
// Pr�pare le buffer � renvoyer au maitre
// ucRAMBuffer est le buffer
// Certaines valeurs comme les entr�s analogiques sont d�j� renseign�e dans dsPIC_reg[] par la tache de fond
//  
void PrepareBufferToSend(void)
{
 unsigned short i=0;  
 unsigned char checksum=0; 
 unsigned short indexBuffer = 0;
 unsigned char indexRegistre;
 
 indexRegistre = dsPIC_reg[REG_PTR_REG_LECTURE_I2C].val;
 for (i=0; i<dsPIC_reg[REG_NBRE_REGISTRES_LECTURE_I2C].val; i++) {    
    // Ajouter ici les autres cas particulier s'il y en a
    
    ucRAMBuffer[indexBuffer] = dsPIC_reg[indexRegistre].val;
    checksum += ucRAMBuffer[indexBuffer];
    indexBuffer++;
    indexRegistre++;
 }
 ucRAMBuffer[indexBuffer] = checksum; // Le dernier octet � transf�rer est le checksum      
}  

// ____________________________________________________________  
typedef enum {
    RX_NOMBRE_OCTETS_TRANSFERES = 0,
    RX_ADRESSE_REGSTRE,
    RX_VALEURS_REGISTRES,
    RX_CHECKSUM,
    RX_ERREUR   
}T_ETAT_RX_I2C;   

// _______________________________________________________________
void GestionReceptionI2C(unsigned char first, unsigned char data)
{
  static unsigned char etat = RX_NOMBRE_OCTETS_TRANSFERES;
  static unsigned char checksum  = 0;
  static unsigned char nbre_octets_recus = 0;
  static unsigned char nbre_octets_a_recevoir = 0;  
  // RAZ de la machine d'�tat en d�but de communication
  if (first) { 
     etat = RX_NOMBRE_OCTETS_TRANSFERES; 
     checksum = 0;
     nbre_octets_recus = 0;
     return;
  }
  
  switch(etat) {
    // _________________________________
    case RX_NOMBRE_OCTETS_TRANSFERES : 
      ucRAMBuffer[0] = data;
      nbre_octets_a_recevoir = data;
      nbre_octets_recus = 0;
      etat = RX_ADRESSE_REGSTRE; 
      checksum = data;
      // Cas anormal, on ne peut pas recevoir plus de donn�es que la taille du buffer de r�ception.
      if (nbre_octets_a_recevoir > SIZE_RAM_BUFFER) { etat = RX_ERREUR; }
      // Cas anormal, il faut  recevoir au moins 3 octets (l'adresse du registre + valeur + checksum)
      if (nbre_octets_a_recevoir < 3) { etat = RX_ERREUR; }
    break;   
    // _________________________________
    case RX_ADRESSE_REGSTRE : 
      ucRAMBuffer[nbre_octets_recus + 1] = data;
      nbre_octets_recus++;
      etat = RX_VALEURS_REGISTRES; 
      checksum += data;
    break;   
   // _________________________________
    case RX_VALEURS_REGISTRES : 
      ucRAMBuffer[nbre_octets_recus + 1] = data;
      nbre_octets_recus++;
      checksum += data;  
      if (nbre_octets_recus >= (nbre_octets_a_recevoir-1)) {
         etat = RX_CHECKSUM; 
      }           
    break;   
    // _________________________________
    case RX_CHECKSUM : 
      if (data == checksum) {
        FinReceptionTrameValideI2C();
        cptPerteComMaster = 0; // requ�te d'�criture valide -> le master i2c est bien pr�sent 
      } 
      else {
        etat = RX_ERREUR;
      }   
    break;  
    // _________________________________
    case RX_ERREUR:
    default :
        // Ne rien faire, attendre le d�but de la prochaine communication 
    break;  
      
  }      
}    
  
// ____________________________________________________________ 
// Si on arrive ici : 
//  - ucRAMBuffer[0] contient le nombre d'octets qui suit le transfert
//  - ucRAMBuffer[1] contient l'adresse du premier registre reg affect� par l'�criture
//  - ucRAMBuffer[2] contient la valeur du registre reg
//  - ucRAMBuffer[3] contient la valeur du registre reg+1
void FinReceptionTrameValideI2C(void)
{
 unsigned short i=0;
 unsigned char indexReg;
 
 indexReg = ucRAMBuffer[1];
 for (i=0; i<(ucRAMBuffer[0]-2); i++) { // -2 car il faut supprimer le checksum et le num�ro de registre
    if (dsPIC_reg[indexReg+i].type_read_write == READ_WRITE) {
       dsPIC_reg[indexReg+i].val = ucRAMBuffer[i+2];
       dsPIC_reg[indexReg+i].new_data = 1; // indique � la t�che de fond que le registe a �t� modifi�     
    }
    // else : le registre est read only, pas d'�criture     
 }
}   



// 

 