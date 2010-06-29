/*
 * radio_ini.h
 *
 * Copyright(c) 1998 - 2010 Texas Instruments. All rights reserved.      
 * All rights reserved.      
 * 
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License v1.0 or BSD License which accompanies
 * this distribution. The Eclipse Public License is available at
 * http://www.eclipse.org/legal/epl-v10.html and the BSD License is as below.                                   
 *                                                                       
 * Redistribution and use in source and binary forms, with or without    
 * modification, are permitted provided that the following conditions    
 * are met:                                                              
 *                                                                       
 *  * Redistributions of source code must retain the above copyright     
 *    notice, this list of conditions and the following disclaimer.      
 *  * Redistributions in binary form must reproduce the above copyright  
 *    notice, this list of conditions and the following disclaimer in    
 *    the documentation and/or other materials provided with the         
 *    distribution.                                                      
 *  * Neither the name Texas Instruments nor the names of its            
 *    contributors may be used to endorse or promote products derived    
 *    from this software without specific prior written permission.      
 *                                                                       
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



/** \file  radio_ini.h 
 *  \brief This file is a draft version and it contains the parameters needed for the Radio ini configuration
 *		   User must fill the TAutoRadioIniParams structure and append it in the aData structure
 *  \see   
 */


/////////////////////////////////////////////////////////////////////////
// Database:	IniFileGeneralParam
// Command:		TEST_CMD_INI_FILE_GENERAL_PARAM
/////////////////////////////////////////////////////////////////////////

#define MAX_SMART_REFLEX_PARAM 16
 
typedef struct 
{
	unsigned char RefClk;                                 
	unsigned char SettlingTime;                                                                 
	unsigned char ClockValidOnWakeup;                      
	unsigned char DC2DCMode;                               
	unsigned char Single_Dual_Band_Solution;               

	unsigned char	TXBiPFEMAutoDetect;
	unsigned char	TXBiPFEMManufacturer; 
    unsigned char   GeneralSettings;

    unsigned char   SRState;
    char  SRF1[MAX_SMART_REFLEX_PARAM];
    char  SRF2[MAX_SMART_REFLEX_PARAM];
    char  SRF3[MAX_SMART_REFLEX_PARAM];

    unsigned char padding[2];

}IniFileGeneralParam;  

typedef enum 
{		
	FEM_MANUAL_DETECT_MODE_E,
	FEM_AUTO_DETECT_MODE_E

}FEM_DETECT_MODE_ENM;

typedef enum 
{		

	FEM_RFMD_TYPE_E,
	FEM_TRIQUINT_TYPE_E,
	NUMBER_OF_FEM_TYPES_E

}FEM_TYPE_ENM;

typedef enum 
{		
	REF_CLK_19_2_E,
	REF_CLK_26_E,
	REF_CLK_38_4_E,
	REF_CLK_52_E

}REF_CLK_ENM;

typedef enum 
{		
	REF_CLK_NOT_VALID_E,
	REF_CLK_VALID_AND_STABLE_E

}CLK_VALID_ON_WAKEUP_ENM;

typedef enum 
{		
	BT_SPI_IS_NOT_USED_E,
	MUX_DC2DC_TO_BT_FUNC2_E

}DC2DC_MODE_ENM;

typedef enum 
{		
	SINGLE_BAND_SOLUTION_E,
	DUAL_BAND_SOLUTION_E

}SINGLE_DUAL_BAND_SOLUTION_ENM;

/////////////////////////////////////////////////////////////////////////
// Database:	IniFileRadioParam
// Command:		TEST_CMD_INI_FILE_RADIO_PARAM
/////////////////////////////////////////////////////////////////////////

#define RSSI_AND_PROCESS_COMPENSATION_TABLE_SIZE   (15)
#define NUMBER_OF_SUB_BANDS_IN_5G_BAND_E           (7) 
#define NUMBER_OF_RATE_GROUPS_E                    (6)
#define NUMBER_OF_2_4_G_CHANNELS                   (14)
#define NUMBER_OF_5G_CHANNELS                      (35)

typedef struct 
{	
	unsigned char RxTraceInsertionLoss_2_4G;																							
	unsigned char TXTraceLoss_2_4G;																							
	char  RxRssiAndProcessCompensation_2_4G[RSSI_AND_PROCESS_COMPENSATION_TABLE_SIZE];						

    unsigned char padding;
    
}TStatRadioParams;  

typedef struct 
{
	// SECTION 1: 2.4G parameters
	short TXBiPReferencePDvoltage_2_4G;
	char  TxBiPReferencePower_2_4G;
	char  TxBiPOffsetdB_2_4G;
	char  TxPerRatePowerLimits_2_4G_Normal[NUMBER_OF_RATE_GROUPS_E];
	char  TxPerRatePowerLimits_2_4G_Degraded[NUMBER_OF_RATE_GROUPS_E];
    char  TxPerRatePowerLimits_2_4G_Extreme[NUMBER_OF_RATE_GROUPS_E];
	char  TxPerChannelPowerLimits_2_4G_11b[NUMBER_OF_2_4_G_CHANNELS];
	char  TxPerChannelPowerLimits_2_4G_OFDM[NUMBER_OF_2_4_G_CHANNELS];
	char  TxPDVsRateOffsets_2_4G[NUMBER_OF_RATE_GROUPS_E];
	unsigned char TxIbiasTable_2_4G[NUMBER_OF_RATE_GROUPS_E];
	unsigned char RxFemInsertionLoss_2_4G;
    unsigned char DegradedLowToNormalThr_2_4G;
    unsigned char NormalToDegradedHighThr_2_4G;
    
    unsigned char padding;
     
}TDynRadioParams;  


/////////////////////////////////////////////////////////////////////////
// Addition to: WHA and TRIOSCOPE (not public_radion.h !!!)
/////////////////////////////////////////////////////////////////////////

typedef struct 
{
 IniFileGeneralParam  tGeneralParams;
 TStatRadioParams	  tStatRadioParams;
 TDynRadioParams	  tDynRadioParams[NUMBER_OF_FEM_TYPES_E];

}TAutoRadioIniParams;  
