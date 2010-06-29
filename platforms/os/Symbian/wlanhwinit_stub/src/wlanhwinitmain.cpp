/*
 * wlanhwinitmain.cpp
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


/**********************************************************************************************************************

  FILENAME:       wlanhwinitmain.cpp
*/


#include "WiLink6_nvs.h"
#ifdef _FW_FROMFILE
#include "wilink6_firmware.h"
#endif
#include "radio_ini.h"
#include "gendebug.h"

#include <e32base.h>
#include <d32dbms.h> // RDbStoreDatabase
#include <f32file.h> // RFs
#include <s32file.h> // CFileStore

#include "wlanhwinitmain.h"
#define __FILE_ID__								FILE_ID_136

#define NVS_LEN     468
#define RADIO_LEN   208

/****************************************************************************
 *                      CWlanHwInitMain::CWlanHwInitMain()
 ****************************************************************************
 * DESCRIPTION: class constructor
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: 
 ****************************************************************************/

CWlanHwInitMain::CWlanHwInitMain() :
	ipaData ( 0 )
{
//	TraceDump( INFO_LEVEL, ( _L( "CWlanHwInitMain:CWlanHwInitMain()" ) ) );
}

/****************************************************************************
 *                      CWlanHwInitMain::ConstructL()
 ****************************************************************************
 * DESCRIPTION: class constructor
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: 
 ****************************************************************************/
void CWlanHwInitMain::ConstructL()
{

}

/****************************************************************************
 *                      CWlanHwInitMain::NewL()
 ****************************************************************************
 * DESCRIPTION: create CWlanHwInitMain 
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: 
 ****************************************************************************/
CWlanHwInitMain* CWlanHwInitMain::NewL()
{
	CWlanHwInitMain* self = new( ELeave ) CWlanHwInitMain;
	CleanupStack::PushL( self );
	self->ConstructL();
	CleanupStack::Pop( self );
	return self;

}
    
/****************************************************************************
 *                      ~CWlanHwInitMain()
 ****************************************************************************
 * DESCRIPTION: class deconstructor
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: 
 ****************************************************************************/
CWlanHwInitMain::~CWlanHwInitMain()
{
//	TraceDump( INFO_LEVEL, ( _L( "CWlanHwInitMain:~CWlanHwInitMain()" ) ) );
	delete ipaData; 
}

/****************************************************************************
 *                   GetHwTestInitData
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: 
 ****************************************************************************/   
void CWlanHwInitMain::GetHwTestInitData(const TUint8** aInitData, TUint& aInitLength, const TUint8** aFwData, TUint& aFwLength)
{
	//TraceDump( INFO_LEVEL, ( _L( "CWlanHwInitMain:GetMacAddressL()" ) ) );   
	
    /* Update the pointer to the whole FW data (NVS + FW file) */
   /* *aFwData = ipNvsData;
    aFwLength = sizeof(WiLink6m_nvs);*/
	
	//TraceDump( INFO_LEVEL, ( _L( "CWlanHwInitMain:GetHwTestInitData()" ) ) );
}



/****************************************************************************
 *                    GetHwTestData
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: 
 ****************************************************************************/
TInt CWlanHwInitMain::GetHwTestData(TUint aId, TDes8& aData)
{
//	TraceDump( INFO_LEVEL, ( _L( "CWlanHwInitMain:GetHwTestData()" ) ) );
	
	return KErrNone;
}

/****************************************************************************
 *                     SetHwTestData
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:   
 * 
 * OUTPUT:  None
 * 
 * RETURNS: 
 ****************************************************************************/

TInt CWlanHwInitMain::SetHwTestData(TUint aId, TDesC8& aData)
{

	return KErrNone;
}

/** 
 * \fn     IsMMCFirmwareFound() 
 * \brief  Checks if a firmware file can be found from
* the memory card.
 * 
 * To success read the FW file from a card the FW bin file should be
 * in firmware directory in the root of the card
 * 
 * \param  None
 * \return alse if not found or error occured, True if 
 * the firmware was successfully read.
 * The firmware is loaded only once.
 * \sa     IsMMCFirmwareFound
 */ 
TBool CWlanHwInitMain::IsMMCFirmwareFound()
{
#ifdef LOAD_FW_FROM_MMC
   TraceDump( INFO_LEVEL, ( _L( "CWlanHwInitMain::IsMMCFirmwareFound()" ) ) );
   
   _LIT(KSearchPath,"E:\\firmware\\*.*");
   _LIT(KFilePath,"E:\\firmware\\");
   
   // Check if the firmware is already loaded,
   // free the memory and continue
   
   if (iFirmwareMC)
   {
    TraceDump( INFO_LEVEL, ( _L( "Memory for MMC Firmware already reserved" ) ) );
    return ETrue;
   }
   
   // Init store
   /* Define file only if FW is not loaded */
   RFs fs;
   RFile file;
   CDir* dirList;
   TInt fileSize = -1;
   TBuf<60> fileName;
   fileName = KFilePath;
   
    
   // Connect to the file system 
   if ( fs.Connect() != KErrNone)
   {
	TraceDump( CRIT_LEVEL, ( _L( "IsMMCFirmwareFound() fs.Connect()" ) ) );
    return EFalse;
   }
   
   // If returns an error, the folder is not found
   // -> return false;
   if (fs.GetDir(KSearchPath,
                 KEntryAttMaskSupported,
                 ESortByName,
                  dirList) != KErrNone )
	{
		TraceDump( CRIT_LEVEL, ( _L( "IsMMCFirmwareFound() fs.GetDir" ) ) );  	
		fs.Close();
		delete dirList;
		return EFalse;
	}
                
    // If no file is not found, return false.
   if (dirList->Count() == 0)
	{
		TraceDump( CRIT_LEVEL, ( _L( "IsMMCFirmwareFound() No file found" ) ) );  	
		fs.Close();
		delete dirList;
		return EFalse;
	}         
      
      // Take the first file in the list, further files
      // are not handled
   fileName.Append ((*dirList)[0].iName);  // Assume there is enough space, otherwise panics...
   
   // Try to open the firmware file
   if ( file.Open(fs, fileName, EFileStream) != KErrNone)
   {
		TraceDump( CRIT_LEVEL, ( _L( "IsMMCFirmwareFound() can't open file" ) ) );  	
		fs.Close();
		delete dirList;
		return EFalse;
   }
    
   // Get the size of the file 
   if (file.Size(fileSize) != KErrNone)
   {
       TraceDump( CRIT_LEVEL, ( _L( "IsMMCFirmwareFound() can't Get File size" ) ) ); 
    file.Close();
    fs.Close();
       delete dirList;
    return EFalse;
   }
    
    // Reserve memory from heap for it
    TRAPD(err, iFirmwareMC = HBufC8::NewL(fileSize));
    if (err != KErrNone)
    {
		TraceDump( CRIT_LEVEL, ( _L( "IsMMCFirmwareFound() can't reserve heap memory" ) ) );  	
     file.Close();
      fs.Close();
        delete dirList;
     return EFalse;
    }
 
    // Get a pointer and read the contents
    // of the file.
   TPtr8 pBuf = iFirmwareMC->Des();
   if (file.Read(pBuf) != KErrNone)
   {
	   TraceDump( CRIT_LEVEL, ( _L( "IsMMCFirmwareFound() can't read file" ) ) );  	
     file.Close();
      fs.Close();
       delete dirList;
     return EFalse;
   }
   
   // Successful
   TraceDump( CRIT_LEVEL, ( _L( "IsMMCFirmwareFound() Success" ) ) );  	
   
   file.Close();
    fs.Close();
    delete dirList;
   return ETrue; 
#else
 /* In case we are not supporting read file from card */
 return EFalse;
#endif // LOAD_FW_FROM_MMC
}



/** 
 * \fn     IsMMCNvsFound() 
 * \brief  Checks if a NVS file can be found from the memory card.
 * 
 * To success read the NVS file from a card the NVS bin file should be
 * in nvs directory in the root of the card
 * 
 * In case that the NVS is taken from MMC card it will contain
 * only the NVS parameters without the Radio parameters
 * therefor the Radio parmeters will be taken from the extend NVS array
 * that include from the WiLink_nvs.h file
 * 
 * \param  None
 * \return false if not found or error occured, True if 
 * the nvs was successfully read.
 * The nvs is loaded only once.
 * \sa     IsMMCNvsFound
 */ 
TBool CWlanHwInitMain::IsMMCNvsFound()
{
#ifdef LOAD_NVS_FROM_MMC
   TraceDump( INFO_LEVEL, ( _L( "CWlanHwInitMain::IsMMCNvsFound()" ) ) );
   
   _LIT(KSearchPath,"E:\\nvs\\*.*");
   _LIT(KFilePath,"E:\\nvs\\");
   
   /* 
    * Check if the nvs is already loaded,
    * free the memory and continue 
    */
   if (pNvsMC)
   {
    TraceDump( INFO_LEVEL, ( _L( "Memory for MMC NVS already reserved" ) ) );
    return ETrue;
   }
   
   /* Init store */
   /* Define file only if NVS is not loaded */
   RFs cFs;
   RFile cFile;
   CDir* pDirList;
   TInt iFileSize = -1;
   TBuf<60> aFileName;
   aFileName = KFilePath;
   
   /* Connect to the file system */
   if ( cFs.Connect() != KErrNone)
   {
	TraceDump( CRIT_LEVEL, ( _L( "IsMMCNvsFound() cFs.Connect()" ) ) );
    return EFalse;
   }
   
   /* If returns an error, the folder is not found */
   /* -> return false; */
   if (cFs.GetDir(KSearchPath,
                 KEntryAttMaskSupported,
                 ESortByName,
                  pDirList) != KErrNone )
	{
		TraceDump( CRIT_LEVEL, ( _L( "IsMMCNvsFound() cFs.GetDir" ) ) );  	
		cFs.Close();
		delete pDirList;
		return EFalse;
	}
                
    /* If no file is not found, return false. */
   if (pDirList->Count() == 0)
	{
		TraceDump( CRIT_LEVEL, ( _L( "IsMMCNvsFound() No file found" ) ) );  	
		cFs.Close();
		delete pDirList;
		return EFalse;
	}         
      
   /* Take the first file in the list, further files are not handled */
   aFileName.Append ((*pDirList)[0].iName);  /* Assume there is enough space, otherwise panics... */
   
   /* Try to open the NVS file */
   if ( cFile.Open(cFs, aFileName, EFileStream) != KErrNone)
   {
		TraceDump( CRIT_LEVEL, ( _L( "IsMMCNvsFound() can't open file" ) ) );  	
		cFs.Close();
		delete pDirList;
		return EFalse;
   }
    
   /* Get the size of the file */
   if (cFile.Size(iFileSize) != KErrNone)
   {
       TraceDump( CRIT_LEVEL, ( _L( "IsMMCNvsFound() can't Get File size" ) ) ); 
       cFile.Close();
       cFs.Close();
       delete pDirList;
       return EFalse;
   }
    
    /* Reserve memory from heap for it */
    TRAPD(err, pNvsMC = HBufC8::NewL(iFileSize));
    if (err != KErrNone)
    {
        TraceDump( CRIT_LEVEL, ( _L( "IsMMCNvsFound() can't reserve heap memory" ) ) );  	
        cFile.Close();
        cFs.Close();
        delete pDirList;
        return EFalse;
    }
 
    /* Get a pointer and read the contents */
    /* of the file. */
   TPtr8 pBuf = pNvsMC->Des();
   if (cFile.Read(pBuf) != KErrNone)
   {
	   TraceDump( CRIT_LEVEL, ( _L( "IsMMCNvsFound() can't read file" ) ) );  	
       cFile.Close();
       cFs.Close();
       delete pDirList;
       return EFalse;
   }
   
   /* Successful */
   TraceDump( CRIT_LEVEL, ( _L( "IsMMCNvsFound() Success" ) ) );  	
   
   cFile.Close();
   cFs.Close();
   delete pDirList;
   return ETrue; 
#else
 /* In case we are not supporting read file from card */
 return EFalse;
#endif /* LOAD_FW_FROM_MMC */
}



/****************************************************************************
 *                     GetHwInitData()
 ****************************************************************************
 * DESCRIPTION: init data NVS, Radio , FW file pointer
 * 
 * INPUTS:  data pointer    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: 
 ****************************************************************************/
void CWlanHwInitMain::GetHwInitData(const TUint8** ppConfigData, TUint& configLength, const TUint8** ppNvsData, TUint& nvsLength )
{
	/*TraceDump( INFO_LEVEL, ( _L( "CWlanHwInitMain:GetHwInitData()" ) ) );*/

	/* Update Config data pointer */
	*ppConfigData = NULL;
	configLength = 0;

	/* Temporary pointer for firmware */
	const TUint8*   pWiLink6fw;
	TUint32		WiLink6fwSize = 0;
	const TUint8*   pWiLink6nvs;
	TUint32		WiLink6nvsSize = 0 ;
    TUint8*         pWiLink6radio;
    TUint32		WiLink6radioSize = 0 ;
    TUint8*         pWiLink6NvsRadio;

    #ifdef _FW_FROMFILE
	if (IsMMCFirmwareFound())
	{
		TraceDump( CRIT_LEVEL, ( _L( "CWlanHwInitMain::GetHwInitData(): Firmware loaded from MMC card !!!" ) ) ); 
		pWiLink6fw = reinterpret_cast<const TUint8*>( iFirmwareMC->Ptr() );
		WiLink6fwSize = iFirmwareMC->Length(); 
	}
	else
	{
	    /* Init FW pointer & FW size */
		pWiLink6fw = reinterpret_cast<const TUint8*>( wilink6_firmware);
        WiLink6fwSize = sizeof( wilink6_firmware );
	} 
    #else
        WiLink6fwSize = 0;
    #endif
  

    if (IsMMCNvsFound()) {
		TraceDump( CRIT_LEVEL, ( _L( "CWlanHwInitMain::GetHwInitData(): NVS loaded from MMC card !!!" ) ) ); 
        pWiLink6nvs = reinterpret_cast<const TUint8*>( pNvsMC->Ptr() );
        WiLink6nvsSize = pNvsMC->Length(); 
    }
    else
    {
        /* Init NVS pointer & size */
        pWiLink6nvs = reinterpret_cast<const TUint8*>( WiLink6_nvs );
        /* Chaged from: sizeof(WiLink6_nvs) */
        WiLink6nvsSize = NVS_LEN;
    }
    pWiLink6NvsRadio = (TUint8*)reinterpret_cast<const TUint8*>( WiLink6_nvs ) + NVS_LEN;

    /* Init Radio pointer & size */
    /* Chaged from: sizeof(TAutoRadioIniParams) */
    WiLink6radioSize = RADIO_LEN;

    /* 
    aData structue :
    ____________________________________________________________________________________________
    |         |               |            |              |          |                          |
    |NVS Len  | NVS File      |  Radio Len | Radio File   | FW Len   |  FW File                 |
    |4 Bytes  | WiLink6_nvs.h | 4 Bytes    | radio_ini.h  | 4 Bytes  |  wilink6_firmware.h      |
    |_________|_______________|____________|______________|__________|__________________________|

    */
    
    /* Alloc lenght for the whole FW Data that is composed of 1 DWORD for NVS len + 1 DWORD for FW Len  + NVS array + FW Array + Radio len + Radio params*/
	nvsLength  = sizeof(TUint32) + WiLink6nvsSize + sizeof(TUint32) + WiLink6radioSize + sizeof(TUint32) + WiLink6fwSize;
	
	/* Reserve memory if it has not yet already been reserved */	
	if (ipaData == NULL)
	{
		ipaData = (TUint8*)User::Alloc(nvsLength); 

		if (!ipaData)
		{
			/* Out of memory*/
			ASSERT(0);
		}	
	}	
    /* Length of NVS*/
    TUint8   *TempPtr = ipaData + 4;
	
    /*Write Nvs Length*/    
    *(TUint32*)ipaData = WiLink6nvsSize;

    /* Copy NVS data to correct position*/
    Mem::Copy(ipaData + sizeof(TUint32), pWiLink6nvs, WiLink6nvsSize);
  	
    TempPtr +=  WiLink6nvsSize;

    /*Write Radio length*/
    *(TUint32*)TempPtr = WiLink6radioSize;
    TempPtr+= 4;
    /* Get Radio position pointer*/
    pWiLink6radio = ipaData + 2*sizeof(TUint32)+ WiLink6nvsSize;
    /*This values need to be impoerted from  Radio module team*/
    /* Changed from: FillRadioData(pWiLink6radio); */
    Mem::Copy(pWiLink6radio, pWiLink6NvsRadio, WiLink6radioSize);

    TempPtr += WiLink6radioSize;

    /*Write FW length*/
    *(TUint32*)TempPtr = WiLink6fwSize;
    TempPtr+= 4;
       
	/* Copy Firmware to correct position*/
    if (WiLink6fwSize != 0)
    {
    Mem::Copy(ipaData + 3*sizeof(TUint32) + WiLink6nvsSize + WiLink6radioSize, pWiLink6fw, WiLink6fwSize);
    }
    
    /* Updated given pointer to just built pointer */
	*ppNvsData = ipaData;
}


/****************************************************************************
 *                     GetMacAddress
 ****************************************************************************
 * DESCRIPTION: get MAC address from NVS 
 * 
 * INPUTS:   
 * 
 * OUTPUT:  MacAddress
 * 
 * RETURNS: OK
 ****************************************************************************/
TInt CWlanHwInitMain::GetMacAddress(TMacAddr& aMacAddress)
{
    const TUint8*   pWiLink6nvs;

     if (IsMMCNvsFound()) 
     {
         pWiLink6nvs = reinterpret_cast<const TUint8*>( pNvsMC->Ptr() );    
     }
     else
     {  
         pWiLink6nvs = WiLink6_nvs;
     }

    aMacAddress.iMacAddress[0] = pWiLink6nvs[11];
    aMacAddress.iMacAddress[1] = pWiLink6nvs[10];
    aMacAddress.iMacAddress[2] = pWiLink6nvs[6];
    aMacAddress.iMacAddress[3] = pWiLink6nvs[5];
    aMacAddress.iMacAddress[4] = pWiLink6nvs[4];
    aMacAddress.iMacAddress[5] = pWiLink6nvs[3];

	return KErrNone;
}


/****************************************************************************
 *                     FillRadioData
 ****************************************************************************
 * DESCRIPTION: fill Radio params , in future will be done by Radio scope
 * 
 * INPUTS:   
 * 
 * OUTPUT:  radio params
 * 
 * RETURNS: OK
 ****************************************************************************/
void CWlanHwInitMain::FillRadioData(TUint8 *WiLink6radio)
{
TAutoRadioIniParams *pRadioParams;

   pRadioParams = (TAutoRadioIniParams*)(WiLink6radio);

   pRadioParams->tGeneralParams.TXBiPFEMAutoDetect = FEM_AUTO_DETECT_MODE_E;
   pRadioParams->tGeneralParams.TXBiPFEMManufacturer = FEM_TRIQUINT_TYPE_E;   

   pRadioParams->tGeneralParams.RefClk = REF_CLK_38_4_E;                                 
   pRadioParams->tGeneralParams.SettlingTime = 5;                                                                 
   pRadioParams->tGeneralParams.ClockValidOnWakeup = REF_CLK_NOT_VALID_E;                      
   pRadioParams->tGeneralParams.DC2DCMode = BT_SPI_IS_NOT_USED_E;                               
   pRadioParams->tGeneralParams.Single_Dual_Band_Solution = SINGLE_BAND_SOLUTION_E;  
   
   pRadioParams->tStatRadioParams.RxTraceInsertionLoss_2_4G = 0;
   pRadioParams->tStatRadioParams.TXTraceLoss_2_4G  = 0;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[0] = 0xEC;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[1] = 0xF6;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[2] = 0x00;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[3] = 0x0C;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[4] = 0x18;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[5] = 0xF8;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[6] = 0xFC;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[7] = 0x00;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[8] = 0x08;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[9] = 0x10;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[10] = 0xF0;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[11] = 0xF8;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[12] = 0x00;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[13] = 0x0A;
   pRadioParams->tStatRadioParams.RxRssiAndProcessCompensation_2_4G[14] = 0x14;
   
/* RMFD deafult value */
   if (pRadioParams->tGeneralParams.Single_Dual_Band_Solution == DUAL_BAND_SOLUTION_E)
   {   
        pRadioParams->tDynRadioParams[0].TXBiPReferencePDvoltage_2_4G = 0 ;									
        pRadioParams->tDynRadioParams[0].TxBiPReferencePower_2_4G = 0;																				
        pRadioParams->tDynRadioParams[0].TxBiPOffsetdB_2_4G = 0;																							
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[0] = 0x1E;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[1] = 0x1F;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[2] = 0x22;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[3] = 0x24;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[4] = 0x28;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[5] = 0x29;							
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[0] = 0x1B;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[1] = 0x1C;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[2] = 0x1E;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[3] = 0x20;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[4] = 0x24;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[5] = 0x25;							
        for (int i=0; i<NUMBER_OF_2_4_G_CHANNELS;i++)
        {
            pRadioParams->tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_11b[i] = 0x50;
            pRadioParams->tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_OFDM[i] = 0x50;
        }
           
        pRadioParams->tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_OFDM[0] = 0x20;
        pRadioParams->tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_OFDM[10] = 0x20;
        memset(&pRadioParams->tDynRadioParams[0].TxPDVsRateOffsets_2_4G[0],0,NUMBER_OF_RATE_GROUPS_E);												
        for (int i=0; i<NUMBER_OF_RATE_GROUPS_E;i++)
        {
            pRadioParams->tDynRadioParams[0].TxIbiasTable_2_4G[i] = 0x0E;
        }
        pRadioParams->tDynRadioParams[0].TxIbiasTable_2_4G[5] = 0x17;
        pRadioParams->tDynRadioParams[0].RxFemInsertionLoss_2_4G = 0x0D;    								
   }
   else
   {
        pRadioParams->tDynRadioParams[0].TXBiPReferencePDvoltage_2_4G =0x24E ;									
        pRadioParams->tDynRadioParams[0].TxBiPReferencePower_2_4G = 0x78;																				
        pRadioParams->tDynRadioParams[0].TxBiPOffsetdB_2_4G = 0;																							
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[0] = 0x1E;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[1] = 0x1F;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[2] = 0x22;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[3] = 0x24;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[4] = 0x28;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[5] = 0x29;							
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[0] = 0x1B;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[1] = 0x1C;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[2] = 0x1E;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[3] = 0x20;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[4] = 0x24;
        pRadioParams->tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[5] = 0x25;							
        for (int i =0;i<NUMBER_OF_2_4_G_CHANNELS;i++)
        {
            pRadioParams->tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_11b[i] = 0x50;
            pRadioParams->tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_OFDM[i] = 0x50;
        }
        pRadioParams->tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_OFDM[0] = 0x20;
        pRadioParams->tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_OFDM[10] = 0x20;   
        memset(&pRadioParams->tDynRadioParams[0].TxPDVsRateOffsets_2_4G[0],0,NUMBER_OF_RATE_GROUPS_E);	
        for (int i = 0 ;i <NUMBER_OF_RATE_GROUPS_E;i++) 
        {
            pRadioParams->tDynRadioParams[0].TxIbiasTable_2_4G[i] = 0x1A;
        }
       
        pRadioParams->tDynRadioParams[0].TxIbiasTable_2_4G[5] = 0x2F;
        pRadioParams->tDynRadioParams[0].RxFemInsertionLoss_2_4G = 0;	
    
        /* TriQuint default value */
        pRadioParams->tDynRadioParams[1].TXBiPReferencePDvoltage_2_4G= 0x168;									
        pRadioParams->tDynRadioParams[1].TxBiPReferencePower_2_4G= 0x83;																				
        pRadioParams->tDynRadioParams[1].TxBiPOffsetdB_2_4G=0;																							
        pRadioParams->tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Normal[0] = 0x1E;
        pRadioParams->tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Normal[1] = 0x1F;
        pRadioParams->tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Normal[2] = 0x22;
        pRadioParams->tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Normal[3] = 0x24;
        pRadioParams->tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Normal[4] = 0x28;
        pRadioParams->tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Normal[5] = 0x29;							
        pRadioParams->tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Degraded[0] = 0x1B;
        pRadioParams->tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Degraded[1] = 0x1C;
        pRadioParams->tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Degraded[2] = 0x1E;
        pRadioParams->tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Degraded[3] = 0x20;
        pRadioParams->tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Degraded[4] = 0x24;
        pRadioParams->tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Degraded[5] = 0x25;	
        for (int i =0;i<NUMBER_OF_2_4_G_CHANNELS;i++)
        {
            pRadioParams->tDynRadioParams[1].TxPerChannelPowerLimits_2_4G_11b[i] = 0x50;
            pRadioParams->tDynRadioParams[1].TxPerChannelPowerLimits_2_4G_OFDM[i] = 0x50;
        }
        pRadioParams->tDynRadioParams[1].TxPerChannelPowerLimits_2_4G_OFDM[0] = 0x20;
        pRadioParams->tDynRadioParams[1].TxPerChannelPowerLimits_2_4G_OFDM[10] = 0x20;  
        memset(&pRadioParams->tDynRadioParams[1].TxPDVsRateOffsets_2_4G[0],0,NUMBER_OF_RATE_GROUPS_E); 
         for (int i = 0 ;i <NUMBER_OF_RATE_GROUPS_E;i++) 
        {
            pRadioParams->tDynRadioParams[1].TxIbiasTable_2_4G[i] = 0x11;
        }
      
        pRadioParams->tDynRadioParams[1].TxIbiasTable_2_4G[5] = 0x12; 													
        pRadioParams->tDynRadioParams[1].RxFemInsertionLoss_2_4G = 0x12;  
   }
}

