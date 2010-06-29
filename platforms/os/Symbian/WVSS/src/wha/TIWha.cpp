/*
 * TIWha.cpp
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


/** \file  TIWha.cpp 
 *  \brief  Interface between the Symbian to the WVSS WLAN WL6.1 Driver
 *
 *  \see   
*/

#include "TIWha.h"
#include "TIWhaAdaptCb.h"
#include "wlanhwbusaccesslayer.h"
#ifdef GEM_SUPPORT
#include "TIWhaGemDef.h"
#endif /* GEM_SUPPORT */

/* The fw that we are using in case it isn't contained in aData */
#include "wilink6_firmware.h"

#ifdef WLAN_SDIO
#include "SdioClient.h"
#endif //WLAN_SDIO


const TInt KWlanDriverMajorVersion = 1;
const TInt KWlanDriverMinorVersion = 0;
const TInt KWlanDriverBuildVersion = 0;
const TUint KWlanUnitsAllowedMask = 0x0000000F;


/*******************************************************************************
 * stuff for C file linking.                                                   *
 *******************************************************************************/

extern "C" 
{
#include "tidef.h"
#include "report.h"
#include "timer.h"
#include "TWDriver.h"
#include "version.h"
#include "osApi.h"
#include "context.h"
#include "public_commands.h"
//#include "public_radio.h"
#include "TxnQueue.h"
#include "BusDrv.h"
#define __FILE_ID__								FILE_ID_144
}

#define MAX_NUM_OF_TX_QUEUES	4 
#define TX_TOTAL_OFFSET_BEFORE_DATA (WSPI_PAD_LEN_WRITE + TX_DESCRIPTOR_SIZE)

/* Stalling the CPU of OAMP3430 for 2Ms */
#define STALL_ON_CPU 2000

/* In case of a KFailed we use the Assert() function as indicated in the spec, but return KSuccess */
#define ASSERT_MI2(hrep,mib,exp,file,func,line)              \
        if ((exp) != WHA::KSuccess) {                             \
            TRACE2(hrep, REPORT_SEVERITY_ERROR ,  " failed, mib=%d, file=, line=%d\n", mib, line);    \
            MWlanOsa::Assert( (const TInt8 *)file, line, EFalse ); \
            return WHA::KSuccess;                                  \
        }
        
#define ASSERT_MIB(hrep,mib,exp)                             \
        ASSERT_MI2(hrep,mib,exp, __FUNCTION__,__FUNCTION__,__LINE__)


#define ASSERT_ER2(hrep,fmsg,par,line)             \
        TRACE1(hrep, REPORT_SEVERITY_ERROR,  fmsg, par);    \
        //MWlanOsa::Assert( (const TInt8 *)file, line, EFalse ); 

#define ASSERT_ERR(hrep,fmsg,par)                            \
        ASSERT_ER2(hrep,fmsg,par,__LINE__)


#define FREF_CLK_FREQ_MASK      0x7


/************************************************************************************/

/** 
* \fn     Create
* \brief  static create
* 
* Just call constructor 
*
* \note   
* return   handle to TIWha class. 
* \sa     
*/ 
WHA::Wha* WHA::Wha::Create(MWlanOsa& aOsa, WlanHpa& aHpa, const SHwBusAccessLayer& aTransPortLayer)
{
    return new TIWha (aOsa, aHpa, *(aTransPortLayer.iSpia));
}


/** 
* \fn     Destroy
* \brief  static destroy
* 
* Just call destructor 
*
* \note   
* \sa     
*/ 
void WHA::Wha::Destroy(Wha* aWha)
{
    delete aWha;
}

/** 
* \fn     TIWha
* \brief  Constructor
* 
* This method is the default constructor,all modules mamory alocation 
* and software configuration (not sending any thing via BUS to the firmware)
* 
* \note   
* return   handle to TIWha class. 
* \sa     
*/ 
TIWha::TIWha(MWlanOsa& aOsa, WlanHpa& aHpa, WlanSpia& aSpia)
: Wha(aOsa, aHpa, aSpia) /* Construct the base class */  
{
    /* Before initializing the report module, we should use the WLAN_OS_REPORT macro */
    WLAN_INIT_REPORT (("TIWha constructor ++\n"));
    WLAN_INIT_REPORT(("Driver Version  : %s\n", SW_VERSION_STR));

    /* tOsContext (hOS is made of MWlanOsa and WlanSpia */
    iTwdCtrl.tOsContext.hOsa = &aOsa;
    iTwdCtrl.tOsContext.hSpia = &aSpia;
    iTwdCtrl.tOsContext.hHpa = &aHpa;

    /* Nullify handles to mark that it wasn't created yet */
    iTwdCtrl.hTWD = NULL;
    iTwdCtrl.hReport = NULL;
    iTwdCtrl.hReport = NULL;
    iTwdCtrl.hContext = NULL;
    iTiWlanHpaCb = NULL;    
    /* indicate that no error indication was called */
    bErrorIndication = TI_FALSE;

    bConnectionTimerRunning = TI_FALSE;
    /* indicate that TIWha::Release() should be called directly if we have error indication */
    bCallRelease = TI_TRUE;
	bFreeDriver	 = TI_FALSE;
	bRxMemFailTimerRunning = TI_FALSE;
    uRxMemFailCount = 0;

    bFailureIndication = TI_FALSE;

	/* Initilaize Timer handle */
	readDot11StationIdMibTmr = NULL;

    /* Set the join flag to false */
    bJoined = 0;
    
    iConnectionCounter = 0;

    #ifdef HT_SUPPORT
	    /* Reset the BA Vectors */
	    iTxBlockAckUsageLast = 0;
        iRxBlockAckUsageLast = 0;
    #endif

    
    pFailureDfcClient = new TIFailureDfcClient(*iTwdCtrl.tOsContext.hOsa);
    if (pFailureDfcClient == NULL)
    {
        WLAN_OS_REPORT (("ERROR: CreateDriver TIFailureDfcClient failure\n"));        
    }

    pConnectDfcClient = new TIConnectDfcClient(*iTwdCtrl.tOsContext.hOsa);
    if (pConnectDfcClient == NULL)
    {
        WLAN_OS_REPORT (("ERROR: CreateDriver TIConnectDfcClient failure\n"));        
    }
    
    WLAN_INIT_REPORT (("TIWha constructor -- \n"));
}


TIWha::TIWha() 
: Wha (*iTwdCtrl.tOsContext.hOsa,*iTwdCtrl.tOsContext.hHpa,*iTwdCtrl.tOsContext.hSpia)
{
    iVersion = TVersion( KWlanDriverMajorVersion, 
        KWlanDriverMinorVersion,KWlanDriverBuildVersion );
    iUnitsMask = KWlanUnitsAllowedMask;
    bDriverCreated = TI_TRUE;
/* Nullify handles to mark that it wasn't created yet */
    iTwdCtrl.hTWD = NULL;
    iTwdCtrl.hReport = NULL;
    iTwdCtrl.hReport = NULL;
    iTwdCtrl.hContext = NULL;
    iTiWlanHpaCb = NULL; 
    /* indicate that no error indication was called */
    bErrorIndication = TI_FALSE;

    bConnectionTimerRunning = TI_FALSE;
    /* indicate that TIWha::Release() should be called directly if we have error indication */
    bCallRelease = TI_TRUE;
	bFreeDriver	 = TI_FALSE;
	bRxMemFailTimerRunning = TI_FALSE;
    uRxMemFailCount = 0;

    bFailureIndication = TI_FALSE;

	/* Initilaize Timer handle */
	readDot11StationIdMibTmr = NULL;

    /* Set the join flag to false */
    bJoined = 0;
    
    iConnectionCounter = 0;

    #ifdef HT_SUPPORT
	    /* Reset the BA Vectors */
	    iTxBlockAckUsageLast = 0;
        iRxBlockAckUsageLast = 0;
    #endif

    WLAN_INIT_REPORT (("TIWha constructor -- \n"));
}


/** 
* \fn     CreateDriver
* \brief  Create driver modules
* 
* Create 'report', 'context', 'TWD', 'HpaCb'
*
* \note   In case of failure, The modules will be released later in DestroyDriver();
* \sa     
*/ 
TI_STATUS TIWha::CreateDriver()
{
    WLAN_INIT_REPORT (("CreateDriver ++\n"));
    

    /* Create utils and TWD */
    iTwdCtrl.hReport  = report_Create ((TI_HANDLE)&iTwdCtrl.tOsContext);
    if (iTwdCtrl.hReport == NULL)
    {
        WLAN_OS_REPORT (("ERROR: CreateDriver report_Create failure\n"));
        return TI_NOK;
    }
    InitReportParamTable ();    
    report_SetDefaults (iTwdCtrl.hReport, &(iTwdCtrl.report_init));
    
    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INIT,  "CreateDriver");

    /* Create context module */
    iTwdCtrl.hContext       = context_Create ((TI_HANDLE)&iTwdCtrl.tOsContext);
    if (iTwdCtrl.hContext == NULL)
    {
        WLAN_OS_REPORT (("ERROR:CreateDriver context_Create failure\n"));
        return TI_NOK;
    }

    /* Create Timer module */
    iTwdCtrl.hTimer = tmr_Create ((TI_HANDLE)&iTwdCtrl.tOsContext);
    if (iTwdCtrl.hTimer == NULL)
    {
        WLAN_OS_REPORT (("ERROR:CreateDriver tmr_Create failure\n"));
        return TI_NOK;
    }

    /* Create TxnQ */
    iTwdCtrl.hTxnQ = txnQ_Create ((TI_HANDLE)&iTwdCtrl.tOsContext);
    if (iTwdCtrl.hTxnQ == NULL)
    {
        WLAN_OS_REPORT (("ERROR:CreateDriver txnQ_Create failure\n"));
        return TI_NOK;
    }

    /* Create TWD */
    iTwdCtrl.hTWD = TWD_Create ((TI_HANDLE)&iTwdCtrl.tOsContext);
    if (iTwdCtrl.hTWD == NULL)
    {
        WLAN_OS_REPORT (("ERROR:CreateDriver TWD_Create failure\n"));
        return TI_NOK;
    }

    /* Create HpaCb */
    iTiWlanHpaCb = new TIWlanHpaCB (iTwdCtrl.hTWD);
    if (iTiWlanHpaCb == NULL)
    {
        WLAN_OS_REPORT (("ERROR:CreateDriver new TIWlanHpaCB failure\n"));
        return TI_NOK;
    }

    return TI_OK;
}

/** 
* \fn     DestroyDriver
* \brief  destroy driver modules
* 
* Destroy 'report', 'context', 'TWD', 'HpaCb' only if it is not NULL
*
* \sa     
*/ 
void TIWha::DestroyDriver()
{
    WLAN_OS_REPORT(("DestroyDriver\n"));

    /* Delete TWD and utils */
    if (iTwdCtrl.hTWD != NULL)
    {
        TWD_Destroy (iTwdCtrl.hTWD);
        /* Avoid future destroy of TWD */
        iTwdCtrl.hTWD = NULL;
    }

	if (iTwdCtrl.hTxnQ)
	{
		txnQ_Destroy(iTwdCtrl.hTxnQ);
		iTwdCtrl.hTxnQ = NULL;
	}

    if (iTwdCtrl.hContext != NULL)
    {
        context_Destroy (iTwdCtrl.hContext);
        /* Avoid future destroy of context */
        iTwdCtrl.hContext = NULL;
    }
    if (iTwdCtrl.hTimer != NULL)
    {
        tmr_Destroy (iTwdCtrl.hTimer);
        /* Avoid future destroy of timer */
        iTwdCtrl.hTimer = NULL;
    }
    if (iTiWlanHpaCb != NULL)
    {
        delete iTiWlanHpaCb;
        /* Avoid future destroy of HpaCb */
        iTiWlanHpaCb = NULL;    
    }
    if (iTwdCtrl.hReport != NULL)
    {
        report_Unload (iTwdCtrl.hReport);
        /* Avoid future destroy of report */
        iTwdCtrl.hReport = NULL;
    }
	
    if (readDot11StationIdMibTmr != NULL)
    {
        os_timerDestroy(0,readDot11StationIdMibTmr);
        readDot11StationIdMibTmr = NULL;
    }    

    if (hConnectionTimer != NULL)
    {
        os_timerDestroy(0,hConnectionTimer);
        hConnectionTimer = NULL;
    }

    if (hInitializeTimer != NULL) 
    {
        os_timerDestroy(0,hInitializeTimer);
        hInitializeTimer = NULL;
    }

    if (hRxMemFailTimer != NULL)
    {
        os_timerDestroy(0,hRxMemFailTimer);
        hRxMemFailTimer = NULL;
    }

}

/** 
* \fn     ~TIWha
* \brief  destructor
* 
* 
* \note   Release is called here if it was not called from UMAC
* \param  
* \return   
* \sa     
*/ 
TIWha::~TIWha()
{  
    /* Delete iFailureDfcClient */
    if (pFailureDfcClient != NULL)
    {        
        /* Either succeeds or has no effect */
        pFailureDfcClient->pFailureDfc->Dequeue();
        /* Osa owns this. Will be deleted there. */
        pFailureDfcClient->pFailureDfc = NULL;
        
        delete pFailureDfcClient;        
    }
    
    /* Delete iConnectDfcClient */
    if (pConnectDfcClient != NULL)
    {        
        /* Either succeeds or has no effect */
        pConnectDfcClient->pConnectDfc->Dequeue();
        /* Osa owns this. Will be deleted there. */
        pConnectDfcClient->pConnectDfc = NULL;

        delete pConnectDfcClient;        
    }
    
    WLAN_INIT_REPORT(("~TIWha\n"));   
}

/** 
* \fn     Initialize
* \brief  Initialize the WLAN driver (memory allocation and modules software init. )
*
* \note   
* Downloads the firmware code to the WLAN device.
* /param aData - firmware data
* /param aLength - length of the data in bytes
* /return  
*/
void TIWha::Initialize(const void* aData, TUint32 aLength)
{
    iData = aData;
    iLength = aLength;

    WLAN_INIT_REPORT (("Initialize ++ \n"));
    #ifdef TI_TEST
	    iQueueId = WHA::ELegacy;
    #endif /* TI_TEST*/

    /* configure HPA polarity according to CHIP configuration */
    #ifdef USE_IRQ_ACTIVE_HIGH
        WlanHpa::TConfig tConfig = {WlanHpa::EIsrPolarityHigh};
    #else
        WlanHpa::TConfig tConfig = {WlanHpa::EIsrPolarityLow};
    #endif

    iRxPacketsAllocated = 0;

    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INIT,  " IRQ is ");

    /* if we were called from failure indication then we don't reconfigure the HPA */
    if (bFailureIndication == TI_FALSE )
    {
        ((WlanHpa*)iTwdCtrl.tOsContext.hHpa)->Configure(tConfig);
    }    


    /* Power up the chip */
    ((WlanHpa*)iTwdCtrl.tOsContext.hHpa)->PowerOnDevice();
#ifdef WLAN_SDIO
    os_StalluSec ((TI_HANDLE)&iTwdCtrl.tOsContext, 200000 );
#endif

    /* Create all modules */
    if ( CreateDriver() != TI_OK)
    {
        WLAN_OS_REPORT (("Error on CreateDriver() \n"));
        InitResponse (WHA::KFailed);        
    }

    /* Attach our client to HPA */
    ((WlanHpa*)iTwdCtrl.tOsContext.hHpa)->Attach(*iTiWlanHpaCb);
    /* Configure the context module */
    context_Init (iTwdCtrl.hContext, (TI_HANDLE)&iTwdCtrl.tOsContext,iTwdCtrl.hReport);

    /* Initialize timer module */
    tmr_Init (iTwdCtrl.hTimer, (TI_HANDLE)&iTwdCtrl.tOsContext, iTwdCtrl.hReport, iTwdCtrl.hContext);

    hInitializeTimer = os_timerCreate(&iTwdCtrl.tOsContext,(fTimerFunction)TIWhaAdaptCB::InitializeAfterTimer,this);

#ifndef WLAN_SDIO
    /* Wait some time for the HW to complete initialization */
    os_timerStart (&iTwdCtrl.tOsContext,hInitializeTimer,200);
#else
    InitializeAfterTimer();
#endif
}


 /** 
 * \fn     InitializeAfterTimer
 * \brief  start second part of initialization after timer expired
 *          
 * \note    
 * \param  
 * \return  
 * \sa      
 */ 
void TIWha::InitializeAfterTimer()
{
    TI_STATUS status;
    TUint8* data = (TUint8*)iData;
    TUint8* pRadio;

    TUint32 nvsLength = 0;
    TUint32 radioLength = 0;

    /* Create a connection timer that will be triggered after Join command */
    hConnectionTimer = os_timerCreate(&iTwdCtrl.tOsContext,(fTimerFunction)TIWhaAdaptCB::ConnectionTimeOut,this); 
    if (hConnectionTimer == NULL)
    {
 	WLAN_OS_REPORT(("Error Failed to create hConnectionTimer"));
    }

    /* Create a timer for losing context for Station ID Mib */
    readDot11StationIdMibTmr = os_timerCreate(&iTwdCtrl.tOsContext,(fTimerFunction)TIWhaAdaptCB::ReadDot11StationIdCb,this); 
    if (readDot11StationIdMibTmr == NULL)
    {
        WLAN_OS_REPORT(("Error Faild to create readDot11StationIdMibTmr"));
    }
    	
    /* Create a timer in case RequestForBuffer fails */
    hRxMemFailTimer = os_timerCreate(&iTwdCtrl.tOsContext,(fTimerFunction)TIWhaAdaptCB::RxMemFailTimerCb,this); 
    if (hRxMemFailTimer == NULL)
    {
        WLAN_OS_REPORT(("Error Faild to create hRxMemFailTimer"));
    }

    /* enable timer operation */    
    tmr_UpdateDriverState (iTwdCtrl.hTimer, TI_TRUE);
    
    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INIT,  "init TWD");
    /*init the txnq */
    
    txnQ_Init (iTwdCtrl.hTxnQ,
               (TI_HANDLE)&iTwdCtrl.tOsContext,
    	  iTwdCtrl.hReport,
               iTwdCtrl.hContext);

	/* Next section retrieves the pointer and Lenght of NVS & FW accordingly in th eaData Buf */

    /* 
    aData structue :
    ____________________________________________________________________________________________
    |         |               |            |              |          |                          |
    |NVS Len  | NVS File      |  Radio Len | Radio File   | FW Len   |  FW File                 |
    |4 Bytes  | WiLink6_nvs.h | 4 Bytes    | radio_ini.h  | 4 Bytes  |  WiLink6_firmware.h      |
    |_________|_______________|____________|______________|__________|__________________________|

    */
	
    #ifdef _SYMBIAN_  
    	nvsLength = *reinterpret_cast<const TUint32*>(&data[0]); 
    	radioLength = *reinterpret_cast<const TUint32*>(&data[sizeof(TUint32) + nvsLength]);            
    	TUint32 fwLength = *reinterpret_cast<const TUint32*>(&data[sizeof(TUint32) + nvsLength + sizeof(TUint32) + radioLength]);
        
        /* in case no fw in aData, use wilink6_firmware */
        if (fwLength == 0 )
        {
            WLAN_OS_REPORT(("TIWha::Initialize:: Taking default fw"));	
            iFwFile.pBuffer = (TI_UINT8*)wilink6_firmware;
        }
        else
        {
            WLAN_OS_REPORT(("TIWha::Initialize::received fw from LDD, fwLength : %d", fwLength ));  
            /* Update first FW Pointer to be the one residing in the firmware array from the fw1273_chip.h */            
            iFwFile.pBuffer = data + 3*sizeof(TUint32) + nvsLength + radioLength;
        }
    
    	/* Retrieve NVS pointer & Lenght */
        ipNVSbuf = data + sizeof(TUint32);
        iNVSlength = nvsLength;
    
        /* Copy the content of NVS Buffer for saving the MAC Address and Burst Read to be retrieved back when NVS 
        Array is received back from the FW */
        os_memoryCopy(iTwdCtrl.tOsContext.hOsa,iNvsStart,ipNVSbuf,24);
    
        /* FEM Auto Detection */
        /* Retrieve Radio pointer  */
        pRadio =  data + sizeof(TUint32) + nvsLength + sizeof(TUint32);
    
        /* save Radio params*/    
        os_memoryCopy(iTwdCtrl.tOsContext.hOsa,&iAutoRadioParams.tGeneralParams, pRadio, GENERAL_RADIO_PARAM_LEN);    
        os_memoryCopy(iTwdCtrl.tOsContext.hOsa,&iAutoRadioParams.tStatRadioParams, (TUint8*)(pRadio + STATIC_RADIO_PARAM_OFFSET), STATIC_RADIO_PARAM_LEN);
        os_memoryCopy(iTwdCtrl.tOsContext.hOsa,&iAutoRadioParams.tDynRadioParams[0], (TUint8*)(pRadio + FEM0_DYNAMIC_RADIO_PARAM_OFFSET), DYNAMIC_RADIO_PARAM_LEN);
        os_memoryCopy(iTwdCtrl.tOsContext.hOsa,&iAutoRadioParams.tDynRadioParams[1], (TUint8*)(pRadio + FEM1_DYNAMIC_RADIO_PARAM_OFFSET), DYNAMIC_RADIO_PARAM_LEN);
            
    #else   /* Other OS */
		/* Retrieve FW pointer  */
		iFwFile.pBuffer = (TI_UINT8*)wilink6_firmware;

        /*saving  NVS pointer */
        ipNVSbuf = (uint8*)aData;
        /* Set NVS length */
        iNVSlength = aLength;
  
      	/* Copy the content of NVS Buffer frir saving the MACF Address and Burst Read to be retrieved back when NVS 
        Array is receuved back from the FW */
        os_memoryCopy(iTwdCtrl.tOsContext.hOsa,iNvsStart,ipNVSbuf,24);

        /* FEM Auto Detection fill radio params */
        /*FillRadioData();*/

    #endif
    
    #if TI_DBG
        WLAN_OS_REPORT(("TIWha::Initialize::nvsLength : %d", nvsLength ));
        WLAN_OS_REPORT(("TIWha::Initialize::radioLength : %d", radioLength ));  
    #endif
   
    WLAN_OS_REPORT((" in Initialize calling txnQ_ConnectBus "));
    /* Configure the bus - in WSPI configuration is done internaly */
    /* This will download the FW image into part with DMA of 512 bytes each time */    
    BusDrvCfg.tSdioCfg.uBlkSizeShift = SDIO_BLK_SIZE_SHIFT_DEF;
    status = txnQ_ConnectBus (iTwdCtrl.hTxnQ, &BusDrvCfg,(TTxnDoneCb)TIWhaAdaptCB::ConnectBus,this,NULL,NULL);

    /* If the bus connect is sync, then we should lose the context and call the CB */
    if (status == TI_OK) 
    {        
        /* Register ConnectDfcClient to handle the connect bus CB from a different context */
        pConnectDfcClient->pConnectDfc->Enqueue(*pConnectDfcClient ,(TInt)this);        
    }    
    #ifdef OMAP3430_CPU_STALL
	    /* Add a stall to make sure that CPU is not going idle while initializing */
        os_StalluSec ((TI_HANDLE)&iTwdCtrl.tOsContext, STALL_ON_CPU );
    #endif
    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INIT,  "TIWha::Initialize --");
}

/** 
* \fn     Configure
* \brief  Configures the WLAN device after firmware download and basic 
*                 chip init has taken place
* \note   
* Method configures the WLAN device after the WHA layer has send the 
* EInitializeResponse event. Only after calling this method, 
* WLAN device is ready for use. 
* Note: the memory supplied by the command is valid to point the 
* corresponding command response event is send
*
* 
* /param aData firmware - data. The content is vendor specific.
* /param aWhaSettings - output data that holds the capabilities 
* of the WLAN vendor specific solution. 
* \return   
* \sa  
*/
void TIWha::Configure(  const WHA::SConfigureData&    aData,
							WHA::SSettings&               aWhaSettings) 
{                                             
    TFwInfo             *tChipVer;
    TI_UINT8            *pMac;  

    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INIT,  "WHA-Configure +");

    /* Enable interrupts on HPA */
    ((WlanHpa*)iTwdCtrl.tOsContext.hHpa)->EnableIrq();
    
    TWD_EnableInterrupts (iTwdCtrl.hTWD);

    /* Call the function to fill all the parameters in the NWSA Settings */
    FillNWSASettings (&aWhaSettings);

    /* Retrieve FW information from TWD */
    tChipVer= TWD_GetFWInfo (iTwdCtrl.hTWD);
    pMac = (TI_UINT8 *)tChipVer->macAddress;

    /* Update driver's MAC address */
    MAC_COPY (iTwdCtrl.pMacAddr, tChipVer->macAddress);

    /*
     *  Exit from init mode should be before smeSM starts. this enable us to send
     *  command to the MboxQueue(that store the command) while the interrupts are masked.
     *  the interrupt would be enable at the end of the init process.
     */
    TWD_ExitFromInitMode (iTwdCtrl.hTWD);

    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INIT, " TIWha::Configure : Before Config HW");
    if (TWD_ConfigFw (iTwdCtrl.hTWD) != TI_OK)
    {
        TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, " TIWha:: TWD_ConfigFw returned error");        
        ConfigFwCb( WHA::KFailed);
        return;
    }
    
    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INIT, "EXIT FROM INIT\n");

    /* Print the driver and firmware version and the mac address */
    WLAN_OS_REPORT(("\n"));
    WLAN_OS_REPORT(("--------------------------------------------------------------------\n"));
    WLAN_OS_REPORT(("Driver Version  : %s\n", SW_VERSION_STR));
    WLAN_OS_REPORT(("Firmware Version: %s\n", tChipVer->fwVer));
    WLAN_OS_REPORT(("Station ID      : %02X-%02X-%02X-%02X-%02X-%02X\n",
    				pMac[0], pMac[1], pMac[2], pMac[3], pMac[4], pMac[5]));
    WLAN_OS_REPORT(("--------------------------------------------------------------------\n"));
    WLAN_OS_REPORT(("\n"));

    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INIT, "WHA-Configure --");
}                                             


/** 
* \fn     Release
* \brief  Destroy TWD and utils
* \note   
*           Any symbian classes (sandbox) will be deleted in the destructor
* 
* /param aSynchronous - not used
* \return KSuccess  
* \sa  
*/
WHA::TStatus TIWha::Release( TBool aSynchronous )
{
    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INIT, "\n");
    
    /* Power off chip */
    ((WlanHpa*)iTwdCtrl.tOsContext.hHpa)->PowerOffDevice();
    
    /* Delete all modules */
    DestroyDriver();
    
    return WHA::KSuccess;
}

/**
* Note: The WLAN host driver does not issue this command while measure
* or 802.11 power management mode transition process is in progress.
* This method commands the WLAN device to start scanning in order to 
* determine the characteristics of the available BSSs and IBSSs to which 
* it may later to elect to join. 
* When the WLAN device receives this command, 
* it goes into a scanning mode. 
* The WHA layer sends the EScanCommandResponse event to inform the 
* WLAN host driver that the WLAN vendor specific 
* solution has accepted the command. 
* For each command there will be both EScanCommandResponse and 
* EScanComplete event regardless the fact if the scan did start or not.
* If the WLAN device is able to start the scanning process, all beacons
* and Probe response frames are sent to the WLAN host driver. 
* When the scanning has completed, the WHA layer sends the EScanComplete
* event to inform the WLAN host driver that the scan process has 
* completed in the WLAN device. 
* Note: while this command is in progress it disables the current Rx 
* frame filtering settings for the duration of the scan. 
* This means that only beacon and probe response frames are forwarded to
* the WLAN host driver (unless doing a split scan) in depended of the 
* current configuration related to Rx frame handling. 
* Note: scanning must be supported by the WLAN vendor specific solution 
* both in infrastructure and IBSS mode. 
* Although frames can be lost in IBSS mode due to scanning, 
* support for it is required.
*
* 
* /param aMaxTransmitRate - specifies the transmission rate of the 
* probe request in case of a active scan 
* Note: just a single rate is selected not multiple as rate fallback 
* is not to be used during the scan process
* /param aBand - selects the used frequency band. 
* Only 1 band is scanned at a time 
* and 1 bit is used to select the band to be scanned
* /param aNumOfChannels - number of channels provided in the command
* /param aChannels - specifies the scanned channels
* /param aScanType - specifies the scan type:
* 0 foreground scan
* 1 background scan
* 2 forced background scan
* /param aNumOfProbeRequests - number of probe requests (per SSID)
* sent to one (1) channel. 
* Zero (0) means that none is send, 
* which means that a passive scan is to be done.  
* Value greater than zero (0) means that an active scan is to be done
* /param aSplitScan - ETrue if a split scan method is to be used. 
* EFalse if not
* /param aNumOfSSID - number of SSID provided in the scan command 
* (this is zero (0) in broadcast scan)
* /param aSsid - Array of the SSID to be probed in scan
*/
#ifdef PLT_TESTER
TTestCmd tTest;
#endif /* PLT_TESTER */
void TIWha::Scan(   WHA::TRate                  aMaxTransmitRate, 
						WHA::TBand                  aBand,
						TUint8                      aNumOfChannels,
						const WHA::SChannels*   aChannels, 
						WHA::TScanType               aScanType,
						TUint8                      aNumOfProbeRequests,
						TBool                       aSplitScan,
						TUint8                      aNumOfSSID,
						const WHA::SSSID*       aSsid)
{
    E80211PsMode             PS_mode;
    TBool                    bEnterPS;				 /* whether to enter PS */
    TBool                    bScanOnDriverModeError; /* Forced background option */
    
    
#ifdef PLT_TESTER
//OpenAllReports();
WLAN_OS_REPORT(("**************************************\n"));
WLAN_OS_REPORT(("PLT_TESTER\n"));
WLAN_OS_REPORT(("**************************************\n"));

	iRealPlt = 0;

	os_printf("************ PLT Test os_printf working ***************\n");
	PltSm(NULL);


return;
#endif /* PLT_TESTER */




    TRACE4(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "TIWha::Scan: aMaxTransmitRate %d aBand %d aNumOfChannels %d aScanType 0x%x\n",        aMaxTransmitRate, aBand, aNumOfChannels, aScanType);

    TRACE4(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "TIWha::Scan: aNumOfProbeRequests %d aSplitScan %d aNumOfSSID %d aSsid.Len %d\n",        aNumOfProbeRequests, aSplitScan, aNumOfSSID, aSsid->iSSIDLength);

    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "[WLANPDD] WHA-Scan +");
    
    /* 
     * Configure Scan type:
     * (aNumOfProbeRequests > 0) ==> Active scan. (TI_TRUE == aSplitScan) ==> Triggered scan 
     */ 
    iTwdCtrl.iScanParams.scanType = 
    (aNumOfProbeRequests > 0) ? 
            ((TI_TRUE == aSplitScan) ? SCAN_TYPE_TRIGGERED_ACTIVE : SCAN_TYPE_NORMAL_ACTIVE) :
            ((TI_TRUE == aSplitScan) ? SCAN_TYPE_TRIGGERED_PASSIVE : SCAN_TYPE_NORMAL_PASSIVE);
            
    /* 
     * On "split scan" defines the access category for which the Fw waits before scanning the next channel
     * The default value is any AC. Note: ScanSrv configures TimeOut
     */ 
    iTwdCtrl.iScanParams.Tid  = AC_ANY_TID;

    /* Band */
    iTwdCtrl.iScanParams.band = 
    (aBand == WHA::KBand2dot4GHzMask) ? RADIO_BAND_2_4_GHZ : RADIO_BAND_5_0_GHZ;

    /* Probe requests */
#ifdef ENHANCE_PROB_SCAN
    if (aNumOfProbeRequests) {
        iTwdCtrl.iScanParams.probeReqNumber   = aNumOfProbeRequests * PB_FACTOR_FOR_SG;
    }
#else
    iTwdCtrl.iScanParams.probeReqNumber   = aNumOfProbeRequests; 
#endif
    iTwdCtrl.iScanParams.probeRequestRate = (ERateMask)TIWhaUtils::WhaToMaskRate (aMaxTransmitRate);

    /***************** Channels ******************/
    iTwdCtrl.iScanParams.numOfChannels    = aNumOfChannels;
                    
    for (int i = 0; i < aNumOfChannels; i++)
    {
    	for (int j = 0; j < MAC_ADDR_LEN; j++ )
    	{
    		(iTwdCtrl.iScanParams.channelEntry[i]).normalChannelEntry.bssId[j] = 0xFF;
    	}
    	(iTwdCtrl.iScanParams.channelEntry[i]).normalChannelEntry.channel = aChannels[i].iChannel;
#ifdef ENHANCE_PROB_SCAN
        if (aNumOfProbeRequests) {
            (iTwdCtrl.iScanParams.channelEntry[i]).normalChannelEntry.maxChannelDwellTime = CONVERT_TU_2_MICRO(aChannels[i].iMaxChannelTime * MAX_TIME_FACTOR_FOR_SG);
            (iTwdCtrl.iScanParams.channelEntry[i]).normalChannelEntry.minChannelDwellTime = CONVERT_TU_2_MICRO(aChannels[i].iMinChannelTime * MIN_TIME_FACTOR_FOR_SG);
        }
        else {
    	(iTwdCtrl.iScanParams.channelEntry[i]).normalChannelEntry.maxChannelDwellTime = CONVERT_TU_2_MICRO(aChannels[i].iMaxChannelTime);
    	(iTwdCtrl.iScanParams.channelEntry[i]).normalChannelEntry.minChannelDwellTime = CONVERT_TU_2_MICRO(aChannels[i].iMinChannelTime);
        }
#else
    	(iTwdCtrl.iScanParams.channelEntry[i]).normalChannelEntry.maxChannelDwellTime = CONVERT_TU_2_MICRO(aChannels[i].iMaxChannelTime);
    	(iTwdCtrl.iScanParams.channelEntry[i]).normalChannelEntry.minChannelDwellTime = CONVERT_TU_2_MICRO(aChannels[i].iMinChannelTime);
#endif
    	(iTwdCtrl.iScanParams.channelEntry[i]).normalChannelEntry.earlyTerminationEvent = SCAN_ET_COND_DISABLE;
    	(iTwdCtrl.iScanParams.channelEntry[i]).normalChannelEntry.ETMaxNumOfAPframes = 0;
    	(iTwdCtrl.iScanParams.channelEntry[i]).normalChannelEntry.txPowerDbm = DBM2DBMDIV10(aChannels[i].iTxPowerLevel);
      }

    /* Mix the channel to increase scan result and avoid collisions */
    if (aNumOfChannels > 4) {
        int Channelscan = 0;
        int Channel = 0;
        int k = 0;
        while (k < aNumOfChannels/2) {
            (iTwdCtrl.iScanParams.channelEntry[Channel]).normalChannelEntry.channel = aChannels[Channelscan].iChannel;
            Channel++;
            Channelscan += aNumOfChannels/2;
            if (aNumOfChannels - Channelscan <= 0) {
                k++;
                Channelscan = k;
            }
        }
    }

    /* Set SSID if present. This replaces the Template Frame configuration */
    if ( aNumOfSSID )
    {
        TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "TIWha::Scan: aSsid.Len %d\n", aSsid->iSSIDLength);

        iTwdCtrl.iScanParams.desiredSsid.len = aSsid->iSSIDLength;
        os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
                       (void *)iTwdCtrl.iScanParams.desiredSsid.str, 
                       (void *)aSsid->iSSID, 
                       aSsid->iSSIDLength);
    }      
    else
    {
        iTwdCtrl.iScanParams.desiredSsid.len = 0;  
    }

    /* PS on scan */
    switch (aScanType)
    {
    case  WHA::EFgScan:
    	bEnterPS = TI_FALSE;
    	/* Internal TI mode - means "don't care" for the Power save */
    	PS_mode = POWER_SAVE_KEEP_CURRENT;
    	bScanOnDriverModeError = TI_FALSE;
    	break;

    case  WHA::EBgScan:
    	bEnterPS = TI_TRUE;
    	PS_mode = POWER_SAVE_ON;
    	bScanOnDriverModeError = TI_FALSE;
    	break;

    case  WHA::EForcedBgScan:
    	bEnterPS = TI_TRUE;
    	PS_mode = POWER_SAVE_ON;
    	bScanOnDriverModeError = TI_TRUE;
    	break;

    default:
        TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, "TIWha::Scan aScanType 0x%x:\n",aScanType);
        /* Use some defaults */
    	bEnterPS = TI_FALSE;
    	PS_mode = POWER_SAVE_KEEP_CURRENT;
    	bScanOnDriverModeError = TI_FALSE;
    }

	/* Replace the Receive Packet while Scan to all scan resault sent to
	LDD befor scan complete event sent */
	TWD_RegisterCb (iTwdCtrl.hTWD,
					TWD_EVENT_RX_RECEIVE_PACKET, 
					(TTwdCB *)TIWhaAdaptCB::ReceivePacketWhileScan, 
					this);
	/* Set the Scan result count at the beginning of the scan to large value to continue sending resault
	till the scan complete arrive from FW and set the correct scan result */
	aScanResultCount = 0xFFFF;
	/* Initiate the Sent resault to Zero */
	SentScanResult = 0;

    /* Perform Scan */
    TI_STATUS status = TWD_Scan (iTwdCtrl.hTWD,
                                                                    &iTwdCtrl.iScanParams, 
                                                                    SCAN_RESULT_TAG_APPLICATION_ONE_SHOT, 
                                                                    TI_FALSE,
                                                                    bEnterPS,
                                                                    bScanOnDriverModeError, 
                                                                    PS_mode,
                                                                    TI_TRUE, 
                                                                    (TCmdResponseCb)TIWhaAdaptCB::ScanResponse,
                                                                    this);

    if (status != TI_OK)
    {
        TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, "TIWha::Scan returned status = 0x%x:\n", status);
    }
}

/** 
* \fn     StopScan
* \brief  stop scan 
* \note   
*           Any symbian classes (sandbox) will be deleted in the destructor
* 
* /param aSynchronous - not used
* \return KSuccess  
* \sa  
*/
void TIWha::StopScan()
{    
    TI_STATUS    status;

    status = TWD_StopScan (iTwdCtrl.hTWD,
                                                    SCAN_RESULT_TAG_APPLICATION_ONE_SHOT,
                                                    TI_FALSE,
                                                    (TCmdResponseCb)TIWhaAdaptCB::StopScanResponse,
                                                    this);

    if (status != TI_OK)
    {
        ASSERT_ERR (iTwdCtrl.hReport, "Stop scan failure, status=%d\n", status)
    }
}

/** 
* \fn     Join
* \brief  Join a BSS/IBSS
*          
* \note   
* \return   
* \sa     
*/ 
void TIWha::Join(
					WHA::TOperationMode aMode,
					const WHA::TMacAddress& aBSSID,
					const WHA::SSSID& aSSID, 
					WHA::TBand aBand,
					WHA::TChannelNumber aChannel,
					TUint32 aBeaconInterval,
					WHA::TRate aBasicRateSet,
					TUint16 aAtimWindow,
					WHA::TPreamble aPreambleType,
					TBool aProbeForJoin )
{    
    TJoinBss         joinBssParams;
    TI_STATUS        status;

#ifdef HT_SUPPORT
	/* Svae the join MAC address for BA use */
	os_memoryCopy(&iTwdCtrl.tOsContext,iJoinedMacAddress,(void*)(aBSSID.iMacAddress),aBSSID.KMacAddressLength);
    /* If re-join remove first all BA sessions */
    if (bJoined) {
        TWD_CloseAllBaSessions(iTwdCtrl.hTWD);
    }
#endif /* HT_SUPPORT */

    WLAN_OS_REPORT(("TWD_JoinBss : bssType = %d beaconInterval = %d channel = %d\n",aMode, (TUint16)aBeaconInterval, aChannel));
    WLAN_OS_REPORT(("BSSID = %x-%x-%x-%x-%x-%x\n",aBSSID.iMacAddress[0],aBSSID.iMacAddress[1],aBSSID.iMacAddress[2],aBSSID.iMacAddress[3],aBSSID.iMacAddress[4],aBSSID.iMacAddress[5]));
        
    /* If we are in PS mode, wake up from ELP and exit 802.11 PS mode */ 
    if ( TWD_GetPsStatus (iTwdCtrl.hTWD) )
    {
        /* Configure H/W to awake */ 
        SleepMode (WHA::KAwakeMode, TI_FALSE);

        /* Exit 802.11 PS mode with dummy callbacks */
        TWD_SetPsMode (iTwdCtrl.hTWD, 
                                        POWER_SAVE_OFF, 
                                        TI_FALSE, 
                                        this, 
                                        (TPowerSaveCompleteCb)TIWhaAdaptCB::SetPsModeCompleteDummy, 
                                        (TPowerSaveResponseCb)TIWhaAdaptCB::SetPsModeResponseDummy);
    }
         
    /*
     * Save BSS info parameters
     */
    joinBssParams.bssType = (ScanBssType_e)aMode;
    /* Save bss type for tx packets */
    iTwdCtrl.bssType = aMode;
    joinBssParams.pBSSID = (TUint8 *)aBSSID.iMacAddress;
    joinBssParams.pSSID = (TUint8 *)aSSID.iSSID;
    joinBssParams.ssidLength = aSSID.iSSIDLength;
    joinBssParams.beaconInterval = aBeaconInterval;
    joinBssParams.channel = aChannel;
#ifndef HT_SUPPORT
    joinBssParams.basicRateSet = TIWhaUtils::WhaToMaskRate (aBasicRateSet);
#else
	joinBssParams.basicRateSet = TIWhaUtils::HTWhaToMaskRate(aBasicRateSet,iMcsRates);
#endif /* HT_SUPPORT */
	
    joinBssParams.txSessionCount = 0; /* currently not used - we always use session '0' */
    /*
     * ATIM window of IBSS
     * Note that when ATIM window is zero the
     * initiated IBSS does not support power-save   
     * Note that PS on IBSS is not supported.
     */
        
    /*
     * DTIM - Temporary value till the Fw will find the right value from the first beacon 
     */
     joinBssParams.dtimInterval = TIWha_DEFAULT_DTIM_PERIOD;

    /* 
     * Band value is determined out of the channel
     */
    if (aBand == WHA::KBand2dot4GHzMask)
    {
        joinBssParams.radioBand = RADIO_BAND_2_4_GHZ;
    }   
    else if (aBand == WHA::KBand5GHzMask)
    {
        joinBssParams.radioBand = RADIO_BAND_5_0_GHZ;
    }   
    else
    {
        ASSERT_ERR (iTwdCtrl.hReport, "Invalid band=%d\n", aBand)
    }
            
   
    /* Set preamble MIB. The generic command response will be stopped in TIWhaAdaptCB::CommandResponse */
    if (aPreambleType == WHA::ELongPreamble)
    {
        TWD_CfgPreamble (iTwdCtrl.hTWD, PREAMBLE_LONG  );
    }
    else
    {
        TWD_CfgPreamble (iTwdCtrl.hTWD, PREAMBLE_SHORT  );
    }

     
	/* 
	 * Reset both group and pairwise keys to WEP in order to avoid 
	 * a scenario which only group or pairwise are actually configured in WEP
	 */
	iTwdCtrl.ePairwiseKeyMode = TWD_CIPHER_WEP;
	iTwdCtrl.eGroupKeyMode	= TWD_CIPHER_WEP;

    /*
     * Set new Tx fail low threshold so that FW fall back will start faster
     * The reason is that at first we start in 54Mbps and we need to drop faster if the AP is far away.
     * This will change after few seconds with the hConnectionTimer
     */
    SetTxFailLowThreshold (TIWHA_TX_FAIL_LOW_TH_FOR_CONNECTION);
    if (bConnectionTimerRunning)
    {
        /* Other join started and ended soon, so just stop the timer before setting it again */
        os_timerStop(&iTwdCtrl.tOsContext, hConnectionTimer);
    }
    bConnectionTimerRunning = TI_TRUE;
    os_timerStart (&iTwdCtrl.tOsContext, hConnectionTimer, uConnectionApproxTimeMs);

    
    status = TWD_CmdJoinBss (iTwdCtrl.hTWD, &joinBssParams);

    if (status != TI_OK)
    {
        ASSERT_ERR (iTwdCtrl.hReport, "Start join failure, status=%d\n", status)
    }

    /* set the join flag to true */
    bJoined = 1;

    #ifdef HT_SUPPORT	
	    /* Reset the BA Vectors */
	    iTxBlockAckUsageLast = 0;
        iRxBlockAckUsageLast = 0;
    #endif
}

/** 
* \fn     SetPsMode
* \brief  SetPsMode 
*          
* \note   
* \return   
* \sa     
*/ 
void TIWha::SetPsMode( WHA::TPsMode aPsMode )
{    
    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "\n");
    TI_STATUS   status;
    E80211PsMode ePsMode; /* TWD parameter for power save on/off */
    
    if (aPsMode == WHA::KPsDisable)
    {
        ePsMode = POWER_SAVE_OFF;

        /* Configure H/W to awake, don't wait for a response */ 
        SleepMode (WHA::KAwakeMode, FALSE);
    }
    else
    {
        ePsMode = POWER_SAVE_ON;
    }

    /* Set 802.11 PS mode */
    status = TWD_SetPsMode (iTwdCtrl.hTWD,
                                            ePsMode,
                                            TI_TRUE,
                                            this,
                                            (TPowerSaveCompleteCb)TIWhaAdaptCB::SetPsModeComplete,
                                            (TPowerSaveResponseCb)TIWhaAdaptCB::SetPsModeResponse);

    /* Convert status */
    switch (status)
    {
        case TI_OK:
        case POWER_SAVE_802_11_PENDING:
            TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION,  ": aPsMode 0x%x, status = %d (OK or PENDING)\n", aPsMode, status);
        break;
        
        case POWER_SAVE_802_11_SUCCESS:
        case POWER_SAVE_802_11_IS_CURRENT:
            TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, ": aPsMode 0x%x, status = %d (IS_CURRENT || SUCCESS)\n", aPsMode, status);
            
        /* This case will never return a response, that's why we don't 'break' this case */ 

        default:
            ASSERT_ERR (iTwdCtrl.hReport, "Set PS mode failure, mode=%d", aPsMode)
    }
}

/** 
* \fn     SetBssParameters
* \brief  Set AID 
*          
* \note   
* \return   
* \sa     
*/ 
void TIWha::SetBssParameters(
								TUint8 aDTIM, 
								TUint16 aAID )
{    
    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "\n");
    
    TTwdParamInfo    tTwdParam;
    TI_STATUS        status;

    /* Set AID */
    tTwdParam.paramType = TWD_AID_PARAM_ID;
    tTwdParam.content.halCtrlAid  = aAID;
    status = TWD_SetParam (iTwdCtrl.hTWD, &tTwdParam);

    /* DTIM is calculated in the Fw now, this is a change from WiLink 4.0.4 */
    
    if (status != TI_OK) 
    {
        ASSERT_ERR (iTwdCtrl.hReport, "Set BSS parameters failure, status=%d\n", status)
    }

    /* Send Station connect Signaling to FW for COEX and 11N purpose */
    TWD_CmdSetStaState(iTwdCtrl.hTWD,1,NULL,NULL);

}

/** 
* \fn     Measure
* \brief  Not supported in Symbian
*          
* \note   
* \return   
* \sa     
*/ 
void TIWha::Measure(
					   WHA::TPowerLevel aTxPowerLevel,
					   WHA::TBand aBand,
					   WHA::TChannelNumber aChannel,
					   TUint8 aActivationDelay,
					   TUint8 aMeasurementOffset,
					   TUint8 aNumberOfMeasurementTypes,
					   const WHA::SParameterSet* aParameterSet )
{    
        ASSERT_ERR (iTwdCtrl.hReport, "%s not supported\n", __FUNCTION__)
}

/** 
* \fn     StopMeasure
* \brief  Not supported in Symbian
*          
* \note   
* \return   
* \sa     
*/ 
void TIWha::StopMeasure()
{    
        ASSERT_ERR (iTwdCtrl.hReport, "%s not supported\n", __FUNCTION__)
}

/** 
* \fn     ReadMib
* \brief  ReadMib
*  
* 
* \note   
* \param aMib - ENUM according to wha_types.h in LDD  
* \return   
* \sa     
*/ 
void TIWha::ReadMib( WHA::TMib aMib ) 
{
    void       *pCb    = (void*)iTwdCtrl.readMIBBuf;
    TMib  *pMibBuf   = (TMib *)pCb;
    WHA::TMib        prevMib;
    TI_STATUS   status = TI_OK;

    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "ReadMIB: aMib %x:\n",aMib);

    prevMib = iTwdCtrl.currentReadMibID;
    iTwdCtrl.currentReadMibID = aMib;
   
    switch (aMib)
    {
    case WHA::KMibDot11StationId:
        /* 
         * open a timer since we can't send read mib results in one context
         */
        os_timerStart(&iTwdCtrl.tOsContext,readDot11StationIdMibTmr,0);
       
        return;
        
    case WHA::KMibStatisticsTable:
        /* 
         * Retrieve required information from the device 
         */
        pMibBuf->aMib = MIB_statisticsTable;

        status = TWD_ReadMib (iTwdCtrl.hTWD, 
                              this, 
                              (void *)TIWhaAdaptCB::ReadMIBResponse, 
                              pCb); 
        break; 

    case WHA::KMibCountersTable:
        /* 
         * Retrieve required information from the device.
         */
        pMibBuf->aMib = MIB_countersTable;
        status = TWD_ReadMib (iTwdCtrl.hTWD, 
                              this, 
                              (void *)TIWhaAdaptCB::ReadMIBResponse, 
                              pCb);
        break;
        
#if 0 /* Used for debug in WiLink 4.0.4 */ 
    case EMIBBtCoexistenceProfile:
        /* 
         * DEBUG ONLY:
         */
        paramInfo.paramType = TWD_SG_CONFIG_PARAM_ID;
        paramInfo.content.interogateCmdCBParams.pCb = (TI_UINT8*)pCb;
        paramInfo.content.interogateCmdCBParams.hCb = (TI_HANDLE)this;
       paramInfo.content.interogateCmdCBParams.fCb = (void *)TIWhaAdaptCB::ReadMIBResponse;
         
        status = TWD_GetParam (iTwdCtrl.hTWD, &paramInfo);
        break;

    case EMIBVersion:
        /* 
         * Retrieve firmware revision 
         */
        paramInfo.paramType = TWD_REVISION_PARAM_ID;
        paramInfo.content.interogateCmdCBParams.pCb = (TI_UINT8*)pCb;
        paramInfo.content.interogateCmdCBParams.hCb = (TI_HANDLE)this;
        paramInfo.content.interogateCmdCBParams.fCb = (void *)TIWhaAdaptCB::ReadMIBResponse;
         
        status = TWD_GetParam (iTwdCtrl.hTWD, &paramInfo);
        break;
#endif

    default:
        iTwdCtrl.currentReadMibID = prevMib;
        TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, ": aMib %x: Not supported\n",aMib);
    }

    if (status != TI_OK) 
    {
        TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, ": aMib %x: status = %d\n",aMib,status);
    }
}        

/** 
* \fn     WriteMib
* \brief  WriteMib
*  
* 
* \note   
* \param aMib - ENUM according to wha_types.h in LDD  
* \param aLength - size of aData
* \param aData - pointer to general data  
* \param aMore - not used 
* \return   
* \sa     
*/ 
WHA::TStatus TIWha::WriteMib(
								WHA::TMib aMib,
								TUint16 aLength,
								const void* aData,
								TBool aMore  )
{    
    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " Mib = 0x%x\n",aMib);

    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "WriteMIB - aMib = %d, aData\n", aMib);
    
    switch (aMib)
    {    
    case WHA::KMibDot11MaxReceiveLifetime:
        /* 
         * Converts the WriteMIB back to the correlated TWD command 
         * and activate the TWD set command
         */     
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
                                Dot11MaxReceiveLifeTime (aData))
        break;
        
    case WHA::KMibDot11SlotTime:
        /* 
         * Converts the WriteMIB back to the correlated whal command 
         * and activate the WHAL Set command
         */
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
                                SetSlotTime (aData))
        break;
        
    case WHA::KMibDot11GroupAddressesTable:
        /* 
         * Converts the WriteMIB back to the correlated TWD command 
         * and activate the TWD Set command
         */
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
            SetDot11GroupAddrTable (aData))
            
        break;

    case WHA::KMibDot11WepDefaultKeyId:
        /* 
         * Set the default Key Id
         */
 
        /*
         * NOTE: Setting default key will cause the securityMode to change into WEP
         */
        ASSERT_MIB(iTwdCtrl.hReport, aMib,
            SetDefaultKeyID (aData))
        break;
        
    case WHA::KMibDot11CurrentTxPowerLevel:
        
        /*
         * NOTE: no assert here. This function may return:
         *       KSuccess - if the same value is already configured
         *       KPending - if operation has succeeded
         *       KFailed - UnKnown error
         */
        return SetCurrentTxPowerLevel ( aData);

    case WHA::KMibDot11RTSThreshold:
        /* 
         * Converts the WriteMIB back to the correlated TWD command 
         * and activate the TWD Set command
         */
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
            Dot11RTSThreshold (aData))
        break;

    case WHA::KMibCtsToSelf:
        /* 
         * Converts the WriteMIB back to the correlated TWD command 
         * and activate the TWD Set command
         */     
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
            CtsToSelf (aData))
        break;

    case WHA::KMibArpIpAddressTable:
        /* 
         * Converts the WriteMIB back to the correlated TWD command 
         * and activate the TWD Set command
         */
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
            SetArpIpAddrTable (aData))
        break;

    case WHA::KMibTemplateFrame:
        /* 
         * Converts the WriteMIB back to the correlated TWD command 
         * and activate the TWD Set command
         */
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
                                SetTemplateFrame (aData))
        break;

    case WHA::KMibRxFilter:
        /* 
         * Converts the WriteMIB back to the correlated TWD command 
         * and activate the TWD Set command
         */
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
                                SetRxFilter (aData)) 
        break;

    case WHA::KMibBeaconFilterIeTable:
        /*      
         * Converts the beacon filter IE table configuration 
         * to TWD parameters and sets specific configuration
         */
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
            SetBeaconFilterIETable (aData))
        break;

    case WHA::KMibBeaconFilterEnable:
        /*      
         * Converts beacon filtering configuration 
         * to TWD parameters and sets specific configuration 
         */
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
            SetBeaconFilterEnable ( aData))
        break;

    case WHA::KMibSleepMode:
        /* 
         * Set user sleep mode
         */     
        iTwdCtrl.sleepMode = *(WHA::TSleepMode *)aData;

        /* If we are in 802.11 PS mode then we should enter new sleepMode (otherwise we just save it) */
        if (TWD_GetPsStatus (iTwdCtrl.hTWD))
        {
            /*
             * NOTE: no assert here. This function may return:
             *       KSuccess - if the same value is already configured
             *       KPending - if operation has succeeded
             */
            return SleepMode ( iTwdCtrl.sleepMode, TI_TRUE);
        }

        else
        {
            return WHA::KSuccess;
        }
                
    case WHA::KMibWlanWakeUpInterval:
        /* 
         * Converts the WriteMIB back to the correlated TWD command 
         * and activate the TWD Set command
         */     
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
            WakeUpConditions ( aData))
                                             
        break;
        
    case WHA::KMibBeaconLostCount:
        /* 
         * Configure the amount of consecutive beacons that can be lost 
         * before the WLAN device should report BSSLost indication to the 
         * WLAN host driver.
         */
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
            SetBeaconLostCount (aData))
        break;

    case WHA::KMibRcpiThreshold:
        /*  
         * Configure the threshold value for RCPI indication to the WLAN  
         * host driver. RcpiIndication event is triggered when the RCPI goes 
         * below or over the threshold.
         */
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
            SetRcpiThreshold (aData))
        break;
       
            
    case WHA::KMibTxRatePolicy:
        /* 
         * Converts the WriteMIB back to the correlated TWD command 
         * and activate the TWD Set command
         */
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
            TxRatePolicy ( aLength, aData))
        break;
        
/*  SG Mibs should be integrated to Symbian */
	case 0x110F: /* Should be KMibbtCoexistenceMode */

        /* Call the TWD with the specific mode of Bt */
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
            btCoexistenceMode ( aLength, aData))
        break;
        
	case 0x1110: /* should be KMibbtCoexistenceProfile */

        /* Call the TWD with the parameters of Bt */
        ASSERT_MIB (iTwdCtrl.hReport, aMib,
            btCoexistenceProfile ( aLength, aData))
        break;

#ifdef HT_SUPPORT
		case WHA::KMibHtCapabilities:
			/* send the HT Capabilties to FW */
            SetHTCapabilities ((WHA::ShtCapabilities*)aData);
			break;

		case WHA::KMibHtBssOperation:
			/* send the HT Information to FW */
            SetHTInformation ((WHA::ShtBssOperation*)aData);
			break;

		case WHA::KMibHtBlockAckConfigure:
			/* BA Initiator and Receiver to FW */
            if (ConfigureBA ((WHA::ShtBlockAckConfigure*)aData) == TI_NOK)
			{
				/* Return success for not stuck the LDD */
				/* This value return when ConfigureBA do nothing */
			     return WHA::KSuccess; 
			}
			break;		
#endif /* HT_SUPPORT */

        case WHA::KMibTxAutoRatePolicy:
			/* Set the supported rates */
			TxAutoRatePolicy((WHA::StxAutoRatePolicy*)aData);
			break;


	#ifdef TI_TEST
		case EMIBPltTest:

		    #ifdef PLT_TESTER
			    iRealPlt = 1;
		    #endif
            os_printf("EMIBPltTest");
			PltTester(aData);
			break;

        /*  FW statistics */ 
        case TWD_FW_PRINT_STATISTICS:
        	TWD_ItrStatistics (iTwdCtrl.hTWD, (void*)TIWhaAdaptCB::StatisticsReadResponse, this, &(((TTwd *)(iTwdCtrl.hTWD))->acxStatistic));           
            break;

        /*  Get AVR RSSI */ 
        case AVR_RSSI:
            TWD_ItrRSSI (iTwdCtrl.hTWD, (void*)TIWhaAdaptCB::AvrRssiReadResponse, this, &roamingStatistics);           
            break;    
        
    	case QOS_CHANGE_QEUEU: /* change output Queue */ 
            ChangeQueue(aData);
    		break;    
    
    	case SET_POWER_LEVEL: // KMibDot11PowerSavePowerLevel
    		SetPowerSavePowerLevel((void *)aData);
	#endif /* TI_TEST */

    default:
        TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, ": aMib %x: Not supported\n",aMib);
        return WHA::KSuccess; 

    } /* End switch */

    /* Indicate that the WriteMIB will return a Response in a different context */
    return WHA::KPending; 
}


/** 
* \fn     SetSlotTime
* 
* \param aData - TSlotTime
* \return KSuccess or KFailed  
* \sa     
*/ 
WHA::TStatus TIWha::SetSlotTime (const void *aData) 
{
    ESlotTime      slot  = PHY_SLOT_TIME_LONG;

    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "WRITE_MIB,TIWha::, slot time = 0x%x\n", *((WHA::TSlotTime *)aData));

    switch (*((WHA::TSlotTime *)aData))
    {   
        case WHA::KSlotTime9:
            slot = PHY_SLOT_TIME_SHORT;
            break;
        case WHA::KSlotTime20:
            slot = PHY_SLOT_TIME_LONG;
            break;
        default:
            TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, "TIWha:: recieving slot time = %d !!!\n",*((WHA::TSlotTime *)aData));
    }

    return (TWD_CfgSlotTime (iTwdCtrl.hTWD, slot) == TI_OK) ? WHA::KSuccess : WHA::KFailed;
}

/** 
* \fn     Dot11MaxReceiveLifeTime
* 
* \param aData - TUint32*
* \return KSuccess or KFailed  
* \sa     
*/ 
WHA::TStatus TIWha::Dot11MaxReceiveLifeTime (const void *aData) 
{
    TMib  Mib;

    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "TIWha::Dot11MaxReceiveLifeTime *aData= 0x%x\n",         *(TUint32 *)(aData));

    Mib.aMib = MIB_dot11MaxReceiveLifetime;
    Mib.aData.MaxReceiveLifeTime = *(TUint32 *)(aData);

    return (TWD_WriteMib (iTwdCtrl.hTWD, &Mib) == TI_OK) ? WHA::KSuccess : WHA::KFailed;
}

/** 
* \fn     SetRxFilter
*  
* 
* \note   - Always filter BSSID
* \param aData - not used
* \return   
* \sa     
*/ 
WHA::TStatus TIWha::SetRxFilter (const void *aData)
{
    TMib Mib;
    
    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "SetRxFilter,*aData= 0x%x\n",*(TUint8 *)aData);

    Mib.aMib = MIB_rxFilter;
    /* 
     * Note that we ignore the requested configuration, since any other configuration is 
     * causing a crash in the LDD. This is agreed with Palau
     */
    Mib.aData.RxFilter = MIB_RX_FILTER_BSSID_SET;
    
    return (TWD_WriteMib (iTwdCtrl.hTWD, &Mib) == TI_OK) ? WHA::KSuccess : WHA::KFailed;
}

/** 
* \fn     SetCurrentTxPowerLevel
*  
* 
* \note   - 
* \param aData -TPowerLevel (Power level in dbm)
* \return   
*           KSuccess - if the same value is already configured
*           KPending - if operation has succeeded
*           KFailed - UnKnown error
* \sa     
*/ 
WHA::TStatus TIWha::SetCurrentTxPowerLevel (const void *aData)
{
    TTwdParamInfo    param;

    /* Activates the TWD_SetParam function */
    param.paramType = TWD_TX_POWER_PARAM_ID;
    param.content.halCtrlTxPowerDbm = DBM2DBMDIV10((*((WHA::TPowerLevel *) aData)));
    
    TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "SetCurrentTxPowerLevel, Dbm = %d Power DBm*10 = %d \n",        *((WHA::TPowerLevel *) aData), param.content.halCtrlTxPowerDbm);

    TI_STATUS status = TWD_SetParam (iTwdCtrl.hTWD, &param);

    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION,  "SetCurrentTxPowerLevel, status = %d \n",status);
    
    switch (status)
    {
        case TI_OK:                             return WHA::KPending;
        case PARAM_ALREADY_CONFIGURED: return WHA::KSuccess;
        default: 
            TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR , "SetCurrentTxPowerLevel, returned status  = %d Power DBm*10 = %d \n",                status, param.content.halCtrlTxPowerDbm);
        return WHA::KFailed;

    }
}

/** 
* \fn     SetTemplateFrame
*  
* 
* \note   - 
* \param aData - StemplateFrame defined in wha_mib.h inside the LDD
* \return   
* \sa     
*/ 
WHA::TStatus TIWha::SetTemplateFrame (const void *aData)
{
    TMib    Mib;
    WHA::StemplateFrame* pMIBTemplateFrame = (WHA::StemplateFrame *)aData;

    TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "SetTemplateFrame aLength= %d,aRate= %d aType = %d\n",        pMIBTemplateFrame->iLength, pMIBTemplateFrame->iInitialTransmitRate, pMIBTemplateFrame->iFrameType);
    
    Mib.aMib = MIB_templateFrame;
    Mib.aData.TemplateFrame.FrameType = TIWhaUtils::WhaToTwdFrameType(pMIBTemplateFrame->iFrameType);
    Mib.aData.TemplateFrame.Rate = (TI_UINT32)TIWhaUtils::WhaToTwdRate (pMIBTemplateFrame->iInitialTransmitRate);
    Mib.aData.TemplateFrame.Length = pMIBTemplateFrame->iLength;

    os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
                   (void *)Mib.aData.TemplateFrame.Data, 
                   (void *)pMIBTemplateFrame->iTemplateData, 
                   pMIBTemplateFrame->iLength);
        
    return (TWD_WriteMib (iTwdCtrl.hTWD, &Mib) == TI_OK) ? WHA::KSuccess : WHA::KFailed;
}

/** 
* \fn     SetDot11GroupAddrTable
* 
* \param aData - *Sdot11GroupAddressesTable
* \return KSuccess or KFailed  
* \sa     
*/ 
WHA::TStatus TIWha::SetDot11GroupAddrTable (const void *aData)           
{
    WHA::Sdot11GroupAddressesTable *pTable = (WHA::Sdot11GroupAddressesTable *)aData;
    TMib Mib;
    int i;

    if (NULL == aData) 
    {
        return WHA::KFailed;
    } 


/*    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION,  TIWLANWHA_MODULE_LOG, MSG_2159,  Mib.aMib, *(TUint32 *)(aData));*/

    Mib.aMib = MIB_dot11GroupAddressesTable;
    Mib.aData.GroupAddressTable.bFilteringEnable = pTable->iEnable;
    Mib.aData.GroupAddressTable.nNumberOfAddresses = pTable->iNumOfAddrs;

    for (i = 0; i < Mib.aData.GroupAddressTable.nNumberOfAddresses; i++)
    {
        os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
                       (void *)Mib.aData.GroupAddressTable.aGroupTable[i], 
                       (void *)((TI_UINT8*)pTable->iAddrData + i*MAC_ADDR_LEN), 
                       MAC_ADDR_LEN);
    }

    TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION,  "SetDot11GroupAddrTable -Enabled=%d-num=%d\n" ,                            Mib.aData.GroupAddressTable.bFilteringEnable,                            Mib.aData.GroupAddressTable.nNumberOfAddresses);

    return (TWD_WriteMib (iTwdCtrl.hTWD, &Mib) == TI_OK) ? WHA::KSuccess : WHA::KFailed;
}

/** 
* \fn     SetDefaultKeyID
* 
* \param aData - *Sdot11WepDefaultKeyId
* \return KSuccess or KFailed  
* \sa     
*/ 
WHA::TStatus TIWha::SetDefaultKeyID (const void *aData) 
{
    TTwdParamInfo  param;

  //  TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION,  TIWLANWHA_MODULE_LOG, MSG_2161,((WHA::Sdot11WepDefaultKeyId*)aData)->iDot11WepDefaultKeyId);

    /* Set security mode to WEP */
    param.paramType = TWD_RSN_SECURITY_MODE_PARAM_ID;
    param.content.rsnEncryptionStatus = (ECipherSuite)TWD_CIPHER_WEP;
    TWD_SetParam (iTwdCtrl.hTWD, &param);

    param.paramType = TWD_RSN_DEFAULT_KEY_ID_PARAM_ID;
    param.content.configureCmdCBParams.pCb = (void*)aData;
    param.content.configureCmdCBParams.fCb = NULL;
    param.content.configureCmdCBParams.hCb = NULL;

    return (TWD_SetParam (iTwdCtrl.hTWD, &param) == TI_OK) ? WHA::KSuccess : WHA::KFailed;
}

/** 
* \fn     Dot11RTSThreshold
* 
* \param aData - *Sdot11RTSThreshold
* \return KSuccess or KFailed  
* \sa     
*/
WHA::TStatus TIWha::Dot11RTSThreshold (const void *aData)
{
    TTwdParamInfo  param;

    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "Dot11RTSThreshold ,*aData= 0x%x\n",((WHA::Sdot11RTSThreshold*)aData)->iDot11RTSThreshold);

    param.paramType = TWD_RTS_THRESHOLD_PARAM_ID;
    param.content.halCtrlRtsThreshold = ((WHA::Sdot11RTSThreshold*)aData)->iDot11RTSThreshold;

    return (TWD_SetParam (iTwdCtrl.hTWD, &param) == TI_OK) ? WHA::KSuccess : WHA::KFailed;
}

/** 
* \fn     CtsToSelf
* 
* \param aData - *SctsToSelf
* \return KSuccess or KFailed  
* \sa     
*/
WHA::TStatus TIWha::CtsToSelf (const void *aData)
{
    TMib Mib;

    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "CtsToSelf, *aData= 0x%x\n",        ((WHA::SctsToSelf *)aData)->iCtsToSelf);

    if (NULL == aData) 
    {
        return PARAM_VALUE_NOT_VALID;
    }
    
    Mib.aMib = MIB_ctsToSelf;
    Mib.aData.CTSToSelfEnable = (TI_UINT32)((WHA::SctsToSelf *)aData)->iCtsToSelf;

    return (TWD_WriteMib (iTwdCtrl.hTWD, &Mib) == TI_OK) ? WHA::KSuccess : WHA::KFailed;
}

/** 
* \fn     SetArpIpAddrTable
* 
* \param aData - *SarpIpAddressTable
* \return KSuccess or KFailed  
* \sa     
*/
WHA::TStatus TIWha::SetArpIpAddrTable (const void *aData)
{   
    TMib   Mib;
	WHA::SarpIpAddressTable *pTable = (WHA::SarpIpAddressTable *)aData;

    if (NULL == aData) 
    {
        return PARAM_VALUE_NOT_VALID;
    }

    Mib.aMib = MIB_arpIpAddressesTable;
    Mib.aData.ArpIpAddressesTable.FilteringEnable = pTable->iEnable;
    
    os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
        Mib.aData.ArpIpAddressesTable.addr, 
        (void*)&pTable->iIpV4Addr, 
        sizeof(pTable->iIpV4Addr));

    TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "SetarpIpAddressesTable,*aData= 0x%x addr = 0x%x\n",        pTable->iIpV4Addr, *(TI_UINT32*)Mib.aData.ArpIpAddressesTable.addr);

    return (TWD_WriteMib (iTwdCtrl.hTWD, &Mib) == TI_OK) ? WHA::KSuccess : WHA::KFailed;
}

/** 
* \fn     SetBeaconFilterIETable
* 
* \param aData - *SbeaconFilterIeTable
* \return KSuccess or KFailed  
* \sa     
*/
WHA::TStatus TIWha::SetBeaconFilterIETable (const void *aData)
{
	WHA::SbeaconFilterIeTable *pTable = (WHA::SbeaconFilterIeTable *)aData;
    TMib Mib;

    if (NULL == aData) 
    {
        return WHA::KFailed;
    }

    Mib.aMib = MIB_beaconFilterIETable;
    Mib.aData.BeaconFilter.iNumberOfIEs =  pTable->iNumofElems;

    os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
                   (void *)Mib.aData.BeaconFilter.iIETable, 
                   (void *)pTable->iIeTable, 
                   sizeof(Mib.aData.BeaconFilter.iIETable));
    
    return (TWD_WriteMib (iTwdCtrl.hTWD, &Mib) == TI_OK) ? WHA::KSuccess : WHA::KFailed;
}

/** 
* \fn     SetBeaconFilterEnable
* 
* \param aData - *SbeaconFilterEnable
* \return KSuccess or KFailed  
* \sa     
*/
WHA::TStatus TIWha::SetBeaconFilterEnable (const void *aData)
{
	WHA::SbeaconFilterEnable *pEnable = (WHA::SbeaconFilterEnable *)aData;

    TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "SetBeaconFilterEnable, *beaconFilterEnable Data= 0x%x 0x%x\n",        pEnable->iEnable, pEnable->iCount);

    return (TWD_CfgBeaconFilterOpt (iTwdCtrl.hTWD, pEnable->iEnable, 
                                    (TI_UINT8)pEnable->iCount) == TI_OK) ? WHA::KSuccess : WHA::KFailed; 
}

/** 
* \fn     SleepMode
* 
* \param aSleepMode - TSleepMode
* \return KSuccess or KFailed  
* \sa     
*/
WHA::TStatus TIWha::SleepMode (
							   const WHA::TSleepMode&       aSleepMode,        /* aSleepMode */
    TBool       bResponse)          /* whether Response required */   
{
    EPowerPolicy powerPolicy;

    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "SleepMode = %d\n",aSleepMode);

    /* Save response request flag in the context */
    iTwdCtrl.bResponse = bResponse;

    switch (aSleepMode)
    {
		case WHA::KAwakeMode:
            powerPolicy = POWERAUTHO_POLICY_AWAKE;
            break;
        case WHA::KPowerDownMode:
            powerPolicy = POWERAUTHO_POLICY_ELP;
            break;
        case WHA::KLowPowerMode:
            powerPolicy = POWERAUTHO_POLICY_ELP;
            break;
        default:
            TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, "Unknown SleepMode %d, setting to AWAKE\n",aSleepMode);
            powerPolicy = POWERAUTHO_POLICY_AWAKE;
    }

/* Currently No support for ELP -- Remove comment when ELP will be added */
    //#ifndef ELP_ENABLED
        /* Disabling Low Power (ELP) mode */
        powerPolicy = POWERAUTHO_POLICY_AWAKE;
    //#endif

    /* Call the power authentication module to set the new power policy */
    return (TWD_CfgSleepAuth (iTwdCtrl.hTWD, powerPolicy) == TI_OK) ? WHA::KPending : WHA::KSuccess;        
}

/** 
* \fn     WakeUpConditions
* 
* \param aData - *SwlanWakeUpInterval
* \return KSuccess or KFailed  
* \sa     
*/
WHA::TStatus TIWha::WakeUpConditions (const void *aData)
{
    WHA::SwlanWakeUpInterval *pWakeUp = (WHA::SwlanWakeUpInterval *)aData;
    TPowerMgmtConfig     PowerMgmtConfig;

    PowerMgmtConfig.listenInterval = pWakeUp->iListenInterval;
    PowerMgmtConfig.tnetWakeupOn = (ETnetWakeOn)pWakeUp->iMode;

    TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "WakeUpConditions ,listenInterval = 0x%x ,iWakeUpMode = 0x%x  \n", PowerMgmtConfig.listenInterval, PowerMgmtConfig.tnetWakeupOn);

    return (TWD_CfgWakeUpCondition (iTwdCtrl.hTWD, &PowerMgmtConfig) == TI_OK) ? WHA::KSuccess : WHA::KFailed;
}

/** 
* \fn     SetBeaconLostCount
* 
* \param aData - *SbeaconLostCount
* \return KSuccess or KFailed  
* \sa     
*/
WHA::TStatus TIWha::SetBeaconLostCount (const void  *aData)
{
    TRroamingTriggerParams roamingTriggerCmd;

    /* Configure TWD with 'No BSS' thresholds */
    roamingTriggerCmd.BssLossTimeout = NO_BEACON_DEFAULT_TIMEOUT;
    roamingTriggerCmd.TsfMissThreshold = (TI_UINT16)((WHA::SbeaconLostCount *)aData)->iLostCount;

    TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "SetBeaconLostCount  ,BssLossTimeout = 0x%x ,TsfMissThreshold = 0x%x  \n",        roamingTriggerCmd.BssLossTimeout, roamingTriggerCmd.TsfMissThreshold);

    return (TWD_CfgConnMonitParams (iTwdCtrl.hTWD, &roamingTriggerCmd) == TI_OK)
           ? WHA::KSuccess : WHA::KFailed;
}

/** 
* \fn     SetRcpiThreshold
* 
* \param aData - *SrcpiThreshold
* \description - we are supporting RCI
* \return KSuccess or KFailed  
* \sa     
*/

WHA::TStatus TIWha::SetRcpiThreshold (const void *aData)
{

    RssiSnrTriggerCfg_t tTriggerCfg;
    WHA::TRcpi iRCPI = *(WHA::TRcpi *)aData;
    
    tTriggerCfg.index  = TRIGGER_EVENT_LOW_RSSI;
    if (iRCPI<= TIWha_MIN_RCPI_VAL)
    {
        tTriggerCfg.threshold = TIWha_MIN_RSSI_VAL;
    }
    else
    {
        if (iRCPI >= TIWha_MAX_RCPI_VAL)
        {
            tTriggerCfg.threshold = TIWha_MAX_RSSI_VAL;
        }
        else
        {
            tTriggerCfg.threshold = (iRCPI / 2) + TIWha_MIN_RSSI_VAL;
        }

    }

    tTriggerCfg.pacing    = TRIGGER_LOW_RSSI_PACING;
    tTriggerCfg.metric    = METRIC_EVENT_RSSI_BEACON;
    tTriggerCfg.type      = RX_QUALITY_EVENT_EDGE;
    tTriggerCfg.direction = RSSI_EVENT_DIR_LOW;
    tTriggerCfg.hystersis = 3;
    tTriggerCfg.enable    = TI_TRUE;



    if (TI_OK == TWD_CfgRssiSnrTrigger (iTwdCtrl.hTWD, &tTriggerCfg))
    {
        return WHA::KSuccess;
    }
    else
    {
        return WHA::KFailed;
    }

 
}


/** 
* \fn     TxRatePolicy
* 
* \param aData - *StxRatePolicy
* \return KSuccess or KFailed  
* \sa     
*/
WHA::TStatus TIWha::TxRatePolicy (
    TUint32     aLength,            /* Specifies the length of the MIB */
    const void        *aData)             /* Pointer to the MIB data */
{
    TMib Mib;
    TUint32       i;
    WHA::StxRatePolicy *pPolicy = (WHA::StxRatePolicy*) aData;
    
    if (NULL == aData)
    {
        return PARAM_VALUE_NOT_VALID;
    }

    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "TxRatePolicy Num of Policies = %d \n",pPolicy->iNumOfPolicyObjects);

    if (pPolicy->iNumOfPolicyObjects > MAX_NUM_OF_TX_RATE_CLASS_POLICIES)
    {
        TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, "ERROR: Can't configure more than %d Classes",MAX_NUM_OF_TX_RATE_CLASS_POLICIES);
    }

    Mib.aMib = MIB_txRatePolicy;
    /* The length field is not in use */
    Mib.Length = 0;
    Mib.aData.txRatePolicy.numOfRateClasses = pPolicy->iNumOfPolicyObjects;

    for (i = 0; i < pPolicy->iNumOfPolicyObjects; i++)
    {
        
        Mib.aData.txRatePolicy.rateClass[i].txEnabledRates = 0;
        
        /* MCS rates not support in Symbian structure */
        (pPolicy->iTxRateClass[i].iTxPolicy54)  ? Mib.aData.txRatePolicy.rateClass[i].txEnabledRates |= HW_BIT_RATE_54MBPS  : NULL;
        (pPolicy->iTxRateClass[i].iTxPolicy48)  ? Mib.aData.txRatePolicy.rateClass[i].txEnabledRates |= HW_BIT_RATE_48MBPS  : NULL;
        (pPolicy->iTxRateClass[i].iTxPolicy36)  ? Mib.aData.txRatePolicy.rateClass[i].txEnabledRates |= HW_BIT_RATE_36MBPS  : NULL;
        /* 33Mbps not support in TWD */
        (pPolicy->iTxRateClass[i].iTxPolicy24)  ? Mib.aData.txRatePolicy.rateClass[i].txEnabledRates |= HW_BIT_RATE_24MBPS  : NULL;
        (pPolicy->iTxRateClass[i].iTxPolicy22)  ? Mib.aData.txRatePolicy.rateClass[i].txEnabledRates |= HW_BIT_RATE_22MBPS  : NULL;
        (pPolicy->iTxRateClass[i].iTxPolicy18)  ? Mib.aData.txRatePolicy.rateClass[i].txEnabledRates |= HW_BIT_RATE_18MBPS  : NULL;
        (pPolicy->iTxRateClass[i].iTxPolicy12)  ? Mib.aData.txRatePolicy.rateClass[i].txEnabledRates |= HW_BIT_RATE_12MBPS  : NULL;
        (pPolicy->iTxRateClass[i].iTxPolicy11)  ? Mib.aData.txRatePolicy.rateClass[i].txEnabledRates |= HW_BIT_RATE_11MBPS  : NULL;
        (pPolicy->iTxRateClass[i].iTxPolicy9)   ? Mib.aData.txRatePolicy.rateClass[i].txEnabledRates |= HW_BIT_RATE_9MBPS   : NULL;
        (pPolicy->iTxRateClass[i].iTxPolicy6)   ? Mib.aData.txRatePolicy.rateClass[i].txEnabledRates |= HW_BIT_RATE_6MBPS   : NULL;
        (pPolicy->iTxRateClass[i].iTxPolicy5_5) ? Mib.aData.txRatePolicy.rateClass[i].txEnabledRates |= HW_BIT_RATE_5_5MBPS : NULL;
        (pPolicy->iTxRateClass[i].iTxPolicy2)   ? Mib.aData.txRatePolicy.rateClass[i].txEnabledRates |= HW_BIT_RATE_2MBPS   : NULL;
        (pPolicy->iTxRateClass[i].iTxPolicy1)   ? Mib.aData.txRatePolicy.rateClass[i].txEnabledRates |= HW_BIT_RATE_1MBPS   : NULL;
        
        //os_printf("Policy[%d] is %x ",i,Mib.aData.txRatePolicy.rateClass[i]);
        Mib.aData.txRatePolicy.rateClass[i].longRetryLimit = pPolicy->iTxRateClass[i].iLongRetryLimit;
        Mib.aData.txRatePolicy.rateClass[i].shortRetryLimit = pPolicy->iTxRateClass[i].iShortRetryLimit;
        
    }
    
    return (TWD_WriteMib (iTwdCtrl.hTWD, &Mib) == TI_OK) ? WHA::KSuccess : WHA::KFailed;
    /*return WHA::KSuccess;*/
}


/** 
* \fn     btCoexistenceMode
* 
* \param aData - *TBTCoexMode
* \ indicats 3 operation modes
 *  0 - Disable
 *  1 - Enable
 *  2 - Auto	
* \return KSuccess or KFailed  
* \sa     
*/
WHA::TStatus TIWha::btCoexistenceMode(TUint32 aLength, const void *aData)
{
	ESoftGeminiEnableModes *pSGMode = (ESoftGeminiEnableModes*)aData;
	TTwdParamInfo			iParamInfo;

	iParamInfo.paramType = TWD_SG_ENABLE_PARAM_ID;
	iParamInfo.paramLength = aLength;
	switch (*pSGMode)
	{
		case SG_PROTECTIVE:
		case SG_OPPORTUNISTIC:
			iParamInfo.content.SoftGeminiEnable = SG_OPPORTUNISTIC;
            TWD_SetParam(iTwdCtrl.hTWD,&iParamInfo);
			break;

		case SG_DISABLE:
			iParamInfo.content.SoftGeminiEnable = SG_DISABLE;
			TWD_SetParam(iTwdCtrl.hTWD,&iParamInfo);
			break;

		default:
TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, "btCoexistenceMode - Value Invalide = %d",*pSGMode);
			return WHA::KFailed;
	}
	return WHA::KSuccess;
}

/** 
* \fn     btCoexistenceProfile
* 
* \param aData - *ESoftGeminiEnableProfile
* \ indicats 3 operation modes
 *  0 - Data-Data
 *  1 - VOIP-A2DP/Data
 *  2 - Data -A2DP	
* \return KSuccess or KFailed  
* \sa     
*/
WHA::TStatus TIWha::btCoexistenceProfile(TUint32 aLength, const void *aData)
{
	ESoftGeminiEnableProfile* pSGProfile = (ESoftGeminiEnableProfile*)aData;
	TTwdParamInfo			iParamInfo;

	iParamInfo.paramType = TWD_SG_CONFIG_PARAM_ID;
	iParamInfo.paramLength = aLength;
	

	switch (*pSGProfile)
	{
    case BtCoexProfData:
            iSgParams.coexParams[SOFT_GEMINI_BT_LOAD_RATIO] = 50;
			break;

		case BtCoexProfDataLowLatency:
			iSgParams.coexParams[SOFT_GEMINI_BT_LOAD_RATIO] = 50;
			break;

		case BtCoexProfA2DP:
			iSgParams.coexParams[SOFT_GEMINI_BT_LOAD_RATIO] = 71;
			break;

		default:
TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, "btCoexistenceProfile - Value Invalide = %d",*pSGProfile);
			return WHA::KFailed;
	}

	os_memoryCopy(iTwdCtrl.tOsContext.hOsa,&iParamInfo.content.SoftGeminiParam,&iSgParams,sizeof(TSoftGeminiParams));
	TWD_SetParam(iTwdCtrl.hTWD,&iParamInfo);
	return WHA::KSuccess;
}


/**
* \fn     TConvertTwdHtCapa2SHtCapa
* \brief  Convert the TTwdHtCapabilities structure to WHA::SHtCapabilities  structure
* 
* \param TTwdHtCapabilities
* \param WHA::SHtCapabilities
* 
* /note
* 
* /return none
*/
#ifdef HT_SUPPORT
void TIWha::TConvertTwdHtCapa2SHtCapa (TTwdHtCapabilities* pTwdHtCapabilities,WHA::SHtCapabilities* pHtCapabilities)
{
	os_memoryZero (&iTwdCtrl.tOsContext,pHtCapabilities,sizeof(WHA::SHtCapabilities));
	pHtCapabilities->iHTCapabilitiesBitMask = (
                                               #ifdef GREENFIELD_FRAME_SUPPORT
                                               ((pTwdHtCapabilities->uHTCapabilitiesBitMask & CAP_BIT_MASK_GREENFIELD_FRAME_FORMAT) << 2) |
                                               #endif
                                               #ifdef SHORT_GI_SUPPORT
											   ((pTwdHtCapabilities->uHTCapabilitiesBitMask & CAP_BIT_MASK_SHORT_GI_FOR_20MHZ_PACKETS) << 2) |
                                               #endif
											   ((pTwdHtCapabilities->uHTCapabilitiesBitMask & CAP_BIT_MASK_SHORT_GI_FOR_40MHZ_PACKETS) << 2) |
											   ((pTwdHtCapabilities->uHTCapabilitiesBitMask & CAP_BIT_MASK_SUPPORT_FOR_STBC_IN_TRANSMISSION) << 2) |
											   ((pTwdHtCapabilities->uHTCapabilitiesBitMask & CAP_BIT_MASK_DELAYED_BLOCK_ACK) << 2) |
											   ((pTwdHtCapabilities->uHTCapabilitiesBitMask & CAP_BIT_MASK_DSSS_CCK_IN_40_MHZ) << 2) |
											   ((pTwdHtCapabilities->uHTCapabilitiesBitMask & CAP_BIT_MASK_LSIG_TXOP_PROTECTION) << 3) |
											   ((pTwdHtCapabilities->uHTCapabilitiesBitMask & CAP_BIT_MASK_PCO) << 3) |
											   ((pTwdHtCapabilities->uHTCapabilitiesBitMask & CAP_BIT_MASK_LDPC_CODING) >> 8));

	pHtCapabilities->iRxMaxDataRate = pTwdHtCapabilities->uRxMaxDataRate;
	
	pHtCapabilities->iChannelWidth = pTwdHtCapabilities->uChannelWidth;

#ifdef RXSTBC_SUPPORT
	pHtCapabilities->iRxStbc = pTwdHtCapabilities->uRxSTBC;
#else
    pHtCapabilities->iRxStbc = RXSTBC_NOT_SUPPORTED;
#endif

	pHtCapabilities->iMaxAmsdu = pTwdHtCapabilities->uMaxAMSDU;

	pHtCapabilities->iMaxAmpdu = pTwdHtCapabilities->uMaxAMPDU;

	pHtCapabilities->iAmpduSpacing = pTwdHtCapabilities->uAMPDUSpacing;

	//pHtCapabilities->iRxMcs = pTwdHtCapabilities->aRxMCS;
	os_memoryCopy(&iTwdCtrl.tOsContext,pHtCapabilities->iRxMcs,pTwdHtCapabilities->aRxMCS,sizeof(WHA::THtMcsSet));

	//pHtCapabilities->iTxMcs = pTwdHtCapabilities->aTxMCS;

	os_memoryCopy(&iTwdCtrl.tOsContext,pHtCapabilities->iTxMcs,pTwdHtCapabilities->aTxMCS,sizeof(WHA::THtMcsSet));

	pHtCapabilities->iPcoTransTime = pTwdHtCapabilities->uPCOTransTime;

	pHtCapabilities->iMcsFeedback = pTwdHtCapabilities->uMCSFeedback;
}
#endif /* HT_SUPPORT */

/**
* \fn     SetHTCapabilities
* \brief  set the HT capabilities of  the AP coming from become/prob resp to FW
* 
* \param ShtCapabilities
* 
* /note
* 
* /return OK for sucsses NOK for failed
*/
#ifdef HT_SUPPORT
WHA::TStatus TIWha::SetHTCapabilities (WHA::ShtCapabilities* pApCapabilities)
{
	Tdot11HtCapabilitiesUnparse		aHtCapabilitiesUnparse;
	Tdot11HtCapabilitiesUnparse*	pHtCapabilitiesUnparse = &aHtCapabilitiesUnparse;

	os_memoryZero(&iTwdCtrl.tOsContext,pHtCapabilitiesUnparse, sizeof(Tdot11HtCapabilitiesUnparse));
	/* Etract the HT Capabilities for FW, at this poin we ignore the HT Control field bit
	   since no support in FW */
	pHtCapabilitiesUnparse->aHtCapabilitiesIe[1] |= (((pApCapabilities->iPeerFeatures & WHA::KGreenfieldFormat) ? HT_CAP_GREENFIELD_FRAME_FORMAT_BITMASK : 0) |
													 ((pApCapabilities->iPeerFeatures & WHA::KShortGiFor20Mhz) ? HT_CAP_SHORT_GI_FOR_20MHZ_BITMASK : 0));

	pHtCapabilitiesUnparse->aHtCapabilitiesIe[0] |= (((pApCapabilities->iPeerFeatures & WHA::KLsigTxopProtection) ? (HT_CAP_LSIG_TXOP_PROTECTION_BITMASK >> 8) : 0) |
													 ((pApCapabilities->iPeerFeatures & WHA::KReverseDirectionResp) ? (HT_EXT_RD_INITIATION_BITMASK >> 8) : 0));

	
	pHtCapabilitiesUnparse->aHtCapabilitiesIe[0] |= ((pApCapabilities->iPeerFeatures & WHA::KHtcField) ? (HT_EXT_HT_CONTROL_FIELDS_BITMASK >> 8) : 0);

	/* Extract the MAX MPDU and MPDU Spacing */
	pHtCapabilitiesUnparse->aHtCapabilitiesIe[HT_CAP_AMPDU_PARAMETERS_FIELD_OFFSET] = ((pApCapabilities->iMaxAmpduLength) |
																						(pApCapabilities->iAmpduSpacing << 2));


    return TWD_CfgSetFwHtCapabilities(iTwdCtrl.hTWD,pHtCapabilitiesUnparse,(pApCapabilities->iHtSupport > 0 ? TI_TRUE : TI_FALSE));
}
#endif /* HT_SUPPORT */


/**
* \fn     SetHTInformation
* \brief  set the HT Information to FW
* 
* \param ShtBssOperation
* 
* /note
* 
* /return OK for sucsses NOK for failed
*/
#ifdef HT_SUPPORT
WHA::TStatus TIWha::SetHTInformation (WHA::ShtBssOperation* pHtInformarion)
{
	Tdot11HtInformationUnparse	aHtInformationUnparse;
	Tdot11HtInformationUnparse*	pHtInformationUnparse = &aHtInformationUnparse;

	pHtInformationUnparse->aHtInformationIe[1] = ((pHtInformarion->iInfo & WHA::ShtBssOperation::KRifsPermitted) ? WHA::ShtBssOperation::KRifsPermitted : 0);
	pHtInformationUnparse->aHtInformationIe[2] = pHtInformarion->iOpMode;
	pHtInformationUnparse->aHtInformationIe[3] = (((pHtInformarion->iInfo & WHA::ShtBssOperation::KNonGreenfieldPresent) ? HT_INF_NON_GF_PRES_BITMASK : 0));
												  
	pHtInformationUnparse->aHtInformationIe[4] = ((pHtInformarion->iInfo & WHA::ShtBssOperation::KDualCtsProtReq) ? HT_INF_DUAL_BEACON_BITMASK : 0);

	/* Save the MCS rates for use in Join command at basic rates */
	os_memoryCopy(&iTwdCtrl.tOsContext,iMcsRates,pHtInformarion->iMcsSet,sizeof(WHA::THtMcsSet));

	return TWD_CfgSetFwHtInformation(iTwdCtrl.hTWD,pHtInformationUnparse);
}
#endif /* HT_SUPPORT */


/**
* \fn     ConfigureBA
* \brief  configure the BA initiator and BA receiver
* 
* \param ShtBssOperation
* 
* /note
* 
* /return OK for sucsses NOK for failed
*/
#ifdef HT_SUPPORT
WHA::TStatus TIWha::ConfigureBA (WHA::ShtBlockAckConfigure* pBABitMask)
{
	WHA::TStatus status = TI_NOK;
	iBACounertRespone = 0;
#ifndef __AMSDU_MODE__

// Check for which TID is enabled for BA session  

#ifdef TX_BA_SUPPORT
	/* Set all Transmit BA Support */
	for (int i = 0;i <= MAX_NUM_OF_802_1d_TAGS; i++)
	{
		if (((pBABitMask->iTxBlockAckUsage >> i) & 0x1) > ((iTxBlockAckUsageLast >> i) & 0x1))
		{
			status = TWD_CfgSetBaInitiator(iTwdCtrl.hTWD,i,TRUE,iJoinedMacAddress,RX_QUEUE_WIN_SIZE,HT_BA_INACTIVITY_TIMEOUT_DEF);
			iBACounertRespone++;
		}
		else {
			if (((pBABitMask->iTxBlockAckUsage >> i) & 0x1) < ((iTxBlockAckUsageLast >> i) & 0x1))
			{
				status = TWD_CfgSetBaInitiator(iTwdCtrl.hTWD,i,FALSE,iJoinedMacAddress,RX_QUEUE_WIN_SIZE,HT_BA_INACTIVITY_TIMEOUT_DEF);
				iBACounertRespone++;
			}
		}
	}
    iTxBlockAckUsageLast = pBABitMask->iTxBlockAckUsage;
#endif

	/* Set all Receive BA */
	
	for (int i = 0;i <= MAX_NUM_OF_802_1d_TAGS; i++)
	{
		if (((pBABitMask->iRxBlockAckUsage >> i) & 0x1) > ((iRxBlockAckUsageLast >> i) & 0x1))
		{
			status = TWD_CfgSetBaReceiver(iTwdCtrl.hTWD,i,TRUE,iJoinedMacAddress,RX_QUEUE_WIN_SIZE);
			iBACounertRespone++;
		}
		else {
			if (((pBABitMask->iRxBlockAckUsage >> i) & 0x1) < ((iRxBlockAckUsageLast >> i) & 0x1))
			{
				status = TWD_CfgSetBaReceiver(iTwdCtrl.hTWD,i,FALSE,iJoinedMacAddress,RX_QUEUE_WIN_SIZE);
				iBACounertRespone++;
	}
		}
	}
	

	/* Save the current BA vectors */
	iRxBlockAckUsageLast = pBABitMask->iRxBlockAckUsage;
#endif
	return status;
}
#endif /* HT_SUPPORT */

/**
* \fn     TxAutoRatePolicy
* \brief  Specifies the supported rates
* 
* \param StxAutoRatePolicy
* 
* /note
* 
* /return OK for sucsses NOK for failed
*/
WHA::TStatus TIWha::TxAutoRatePolicy (WHA::StxAutoRatePolicy* pTxAutoRatePolicy)
{
	iTxRatePolicy.aMib = MIB_txRatePolicy;
    /* The length field is not in use */
    iTxRatePolicy.Length = 0;
	/* we only have 1 policy since FW decide on which  rate to use */
    iTxRatePolicy.aData.txRatePolicy.numOfRateClasses = pTxAutoRatePolicy->iTxRateClassId;

    #ifdef HT_SUPPORT
        iTxRatePolicy.aData.txRatePolicy.rateClass[pTxAutoRatePolicy->iTxRateClassId - 1].txEnabledRates = TIWhaUtils::HTWhaRateToRatePolicy(pTxAutoRatePolicy);
    #else
        iTxRatePolicy.aData.txRatePolicy.rateClass[pTxAutoRatePolicy->iTxRateClassId - 1].txEnabledRates = TIWhaUtils::WhaRateToRatePolicy(pTxAutoRatePolicy);
    #endif

    #ifdef ALL_RATES_OPEN_DEBUG
        if ( iTxRatePolicy.aData.txRatePolicy.rateClass[pTxAutoRatePolicy->iTxRateClassId - 1].txEnabledRates & HW_BIT_RATE_54MBPS)
        {
             iTxRatePolicy.aData.txRatePolicy.rateClass[pTxAutoRatePolicy->iTxRateClassId - 1].txEnabledRates = 0x1DFF;    
        }
    #endif
    
	iTxRatePolicy.aData.txRatePolicy.rateClass[pTxAutoRatePolicy->iTxRateClassId - 1].shortRetryLimit = pTxAutoRatePolicy->iShortRetryLimit;
	iTxRatePolicy.aData.txRatePolicy.rateClass[pTxAutoRatePolicy->iTxRateClassId - 1].longRetryLimit = pTxAutoRatePolicy->iLongRetryLimit;

	return (TWD_WriteMib (iTwdCtrl.hTWD, &iTxRatePolicy) == TI_OK) ? WHA::KSuccess : WHA::KFailed;
}

/**
* \fn     SetPowerSavePowerLevel
* \brief  Set the Power save power level
*
* 
* /note
* 
* /return 
*/
void TIWha::SetPowerSavePowerLevel(void *aMib)
{
	//os_printf("SetPowerSavePowerLevel send EPowerPolicy = %d",*((EPowerPolicy*)aMib));
	TWD_CfgSleepAuth(iTwdCtrl.hTWD, *((EPowerPolicy*)aMib));
}


/**
* \fn     SoftGemini_SetParams
* \brief  initialization of Soft Gemini parameters
*
* used for init stage to initialize the SG parameters
* /note
* 
* /return 
*/

TI_STATUS TIWha::SoftGemini_SetParams (TI_HANDLE hTWD, TSoftGeminiParams *pSgParams,ESoftGeminiEnableModes aSgMode )
{
	TTwdParamInfo			iParamInfo;

	iParamInfo.paramType = TWD_SG_CONFIG_PARAM_ID;
	iParamInfo.paramLength = sizeof(TSoftGeminiParams);
	os_memoryCopy(iTwdCtrl.tOsContext.hOsa,&iParamInfo.content.SoftGeminiParam,&iSgParams,sizeof(TSoftGeminiParams));
	TWD_SetParam(iTwdCtrl.hTWD,&iParamInfo);

	iParamInfo.paramType = TWD_SG_ENABLE_PARAM_ID;
	iParamInfo.paramLength = sizeof(ESoftGeminiEnableModes);
	iParamInfo.content.SoftGeminiEnable = aSgMode;
	TWD_SetParam(iTwdCtrl.hTWD,&iParamInfo);
	return TI_OK;
}

/** 
* \fn     ConfigureBtCoex
* \brief  Set the Soft Gemini parameters and send it to FW 
*          
* \note   
* \return KSuccess or KFailed
* \sa     
*/ 
void TIWha::InitBtCoex()
{
    
	/* Set Static parameters */

    /* Setting the ratio for each profile */
    /* Both coexBtLoadRatio & coexBtLoadRatio are 50/50 profiles */
    /* The default profile is set to BtCoexProfData */
	switch (BTCOEX_DEFAULT_PROFILE)
	{
		case BtCoexProfData:
			iSgParams.coexParams[SOFT_GEMINI_BT_LOAD_RATIO] = 50;
			break;

		case BtCoexProfDataLowLatency:
			iSgParams.coexParams[SOFT_GEMINI_BT_LOAD_RATIO] = 50;
			break;

		case BtCoexProfA2DP:
			iSgParams.coexParams[SOFT_GEMINI_BT_LOAD_RATIO] = 71;
			break;

		default:
            TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, "btCoexistenceProfile - Value Invalide = %d",BTCOEX_DEFAULT_PROFILE);
			return;
	}

    iSgParams.coexParams[SOFT_GEMINI_BT_PER_THRESHOLD] = 7500;
    iSgParams.coexParams[SOFT_GEMINI_HV3_MAX_OVERRIDE] = 0;
    iSgParams.coexParams[SOFT_GEMINI_BT_NFS_SAMPLE_INTERVAL] = 400;
    iSgParams.coexParams[SOFT_GEMINI_AUTO_PS_MODE] = 0;	
    iSgParams.coexParams[SOFT_GEMINI_AUTO_SCAN_PROBE_REQ] = 170;
    iSgParams.coexParams[SOFT_GEMINI_ACTIVE_SCAN_DURATION_FACTOR_HV3]	= 50;   
    iSgParams.coexParams[SOFT_GEMINI_ANTENNA_CONFIGURATION] = 0;        
    iSgParams.coexParams[SOFT_GEMINI_BEACON_MISS_PERCENT] = 60;
    iSgParams.coexParams[SOFT_GEMINI_RATE_ADAPT_THRESH] = 21; /* Set to MCS_7 */
    #ifdef ENHANCED_OPPORTUNISTIC
        iSgParams.coexParams[SOFT_GEMINI_RATE_ADAPT_SNR] = 1;        
    #else
        iSgParams.coexParams[SOFT_GEMINI_RATE_ADAPT_SNR]= 0;
    #endif    


    iSgParams.coexParams[SOFT_GEMINI_WLAN_PS_BT_ACL_MASTER_MIN_BR] = 10;
    iSgParams.coexParams[SOFT_GEMINI_WLAN_PS_BT_ACL_MASTER_MAX_BR] = 30;
    iSgParams.coexParams[SOFT_GEMINI_WLAN_PS_MAX_BT_ACL_MASTER_BR] = 8;

    iSgParams.coexParams[SOFT_GEMINI_WLAN_PS_BT_ACL_SLAVE_MIN_BR] = 20;
    iSgParams.coexParams[SOFT_GEMINI_WLAN_PS_BT_ACL_SLAVE_MAX_BR] = 50;
    iSgParams.coexParams[SOFT_GEMINI_WLAN_PS_MAX_BT_ACL_SLAVE_BR] = 8;

    iSgParams.coexParams[SOFT_GEMINI_WLAN_PS_BT_ACL_MASTER_MIN_EDR] = 7;
    iSgParams.coexParams[SOFT_GEMINI_WLAN_PS_BT_ACL_MASTER_MAX_EDR] = 25;
    iSgParams.coexParams[SOFT_GEMINI_WLAN_PS_MAX_BT_ACL_MASTER_EDR] = 20;

    iSgParams.coexParams[SOFT_GEMINI_WLAN_PS_BT_ACL_SLAVE_MIN_EDR] = 8;
    iSgParams.coexParams[SOFT_GEMINI_WLAN_PS_BT_ACL_SLAVE_MAX_EDR] = 40;
    iSgParams.coexParams[SOFT_GEMINI_WLAN_PS_MAX_BT_ACL_SLAVE_EDR] = 12;    
    iSgParams.coexParams[SOFT_GEMINI_RXT] = 1200;
    iSgParams.coexParams[SOFT_GEMINI_TXT] = 1500;
    iSgParams.coexParams[SOFT_GEMINI_ADAPTIVE_RXT_TXT] = 1;
    iSgParams.coexParams[SOFT_GEMINI_PS_POLL_TIMEOUT] = 10;
    iSgParams.coexParams[SOFT_GEMINI_UPSD_TIMEOUT] = 10;
    iSgParams.coexParams[SOFT_GEMINI_UPSD_TIMEOUT] = 10;

    iSgParams.coexParams[SOFT_GEMINI_WLAN_ACTIVE_BT_ACL_MASTER_MIN_EDR] = 7;
    iSgParams.coexParams[SOFT_GEMINI_WLAN_ACTIVE_BT_ACL_MASTER_MAX_EDR] = 15;
    iSgParams.coexParams[SOFT_GEMINI_WLAN_ACTIVE_MAX_BT_ACL_MASTER_EDR] = 15;

    iSgParams.coexParams[SOFT_GEMINI_WLAN_ACTIVE_BT_ACL_SLAVE_MIN_EDR] = 8;
    iSgParams.coexParams[SOFT_GEMINI_WLAN_ACTIVE_BT_ACL_SLAVE_MAX_EDR] = 20;
    iSgParams.coexParams[SOFT_GEMINI_WLAN_ACTIVE_MAX_BT_ACL_SLAVE_EDR] = 15;

    iSgParams.coexParams[SOFT_GEMINI_WLAN_ACTIVE_BT_ACL_MIN_BR] = 20;
    iSgParams.coexParams[SOFT_GEMINI_WLAN_ACTIVE_BT_ACL_MAX_BR] = 50;
    iSgParams.coexParams[SOFT_GEMINI_WLAN_ACTIVE_MAX_BT_ACL_BR] = 5;

    iSgParams.coexParams[SOFT_GEMINI_PASSIVE_SCAN_DURATION_FACTOR_HV3] = 200;
    iSgParams.coexParams[SOFT_GEMINI_PASSIVE_SCAN_DURATION_FACTOR_A2DP] = 800;
    iSgParams.coexParams[SOFT_GEMINI_PASSIVE_SCAN_BT_TIME] = 75;
    iSgParams.coexParams[SOFT_GEMINI_PASSIVE_SCAN_WLAN_TIME] = 15;    
    iSgParams.coexParams[SOFT_GEMINI_HV3_MAX_SERVED] = 6;    

    iSgParams.coexParams[SOFT_GEMINI_DHCP_TIME] = 5000;    
    iSgParams.coexParams[SOFT_GEMINI_ACTIVE_SCAN_DURATION_FACTOR_A2DP] = 100;

    iSgParams.coexParams[SOFT_GEMINI_TEMP_PARAM_1] = 0;
    iSgParams.coexParams[SOFT_GEMINI_TEMP_PARAM_2] = 0;
    iSgParams.coexParams[SOFT_GEMINI_TEMP_PARAM_3] = 0;
    iSgParams.coexParams[SOFT_GEMINI_TEMP_PARAM_4] = 0;    
    iSgParams.coexParams[SOFT_GEMINI_TEMP_PARAM_5] = 0;    
}


/** 
* \fn     AddKey
* \brief  Add key according to the key type 
*          
* \note   In case we already used the given index, we remove the old key
* \return   
* \sa     
*/ 
void TIWha::AddKey(
					  WHA::TKeyType aType, 
					  const void* aKey,
					  TUint8 aEntryIndex )
{    
    TTwdParamInfo    param;
    TSecurityKeys    key;
    TI_STATUS        status;

    os_memoryZero ((TI_HANDLE)&iTwdCtrl.tOsContext, &key, sizeof(TSecurityKeys));

    if (aEntryIndex >= TIWha_MAX_PRIVACY_KEY_INDEX)
    {
        ASSERT_ERR (iTwdCtrl.hReport, "Invalid index=%d\n", aEntryIndex)
    }

    /* if the Key already exists, we remove it, and add the new one     */
    /* we check the encLen field beacuse if there is key in this entry  */
    /* the encLen field is always different than zero                   */
    if (iTwdCtrl.privacyKey[aEntryIndex].encLen !=0)
    {
        TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "Remove duplicate Key from AddKey, aEntryIndex= 0x%x\n", aEntryIndex);
       RemoveKey(aEntryIndex);
    }
    else
    {
         TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "AddKey, New key aEntryIndex= 0x%x, encLen= %d \n", aEntryIndex, iTwdCtrl.privacyKey[aEntryIndex].encLen);
    }

    /* Set the security mode */
    switch (aType)
    {
    case WHA::EWepGroupKey:
        param.content.rsnEncryptionStatus = (ECipherSuite)TWD_CIPHER_WEP;
        iTwdCtrl.eGroupKeyMode = TWD_CIPHER_WEP;
        break;

    case WHA::EWepPairWiseKey:
        param.content.rsnEncryptionStatus = (ECipherSuite)TWD_CIPHER_WEP;
        iTwdCtrl.ePairwiseKeyMode = TWD_CIPHER_WEP;
        break;

    case WHA::ETkipGroupKey:
        param.content.rsnEncryptionStatus = (ECipherSuite)TWD_CIPHER_TKIP;
        iTwdCtrl.eGroupKeyMode = TWD_CIPHER_TKIP;
        break;

    case WHA::ETkipPairWiseKey:
        param.content.rsnEncryptionStatus = (ECipherSuite)TWD_CIPHER_TKIP;
        iTwdCtrl.ePairwiseKeyMode = TWD_CIPHER_TKIP;
        break;

    case WHA::EAesGroupKey:
        param.content.rsnEncryptionStatus = (ECipherSuite)TWD_CIPHER_AES_CCMP;
        iTwdCtrl.eGroupKeyMode = TWD_CIPHER_AES_CCMP;
        break;

    case WHA::EAesPairWiseKey:
        param.content.rsnEncryptionStatus = (ECipherSuite)TWD_CIPHER_AES_CCMP;
        iTwdCtrl.ePairwiseKeyMode = TWD_CIPHER_AES_CCMP;
        break;

    #ifdef GEM_SUPPORT
    case GEMGroupKey:
            param.content.rsnEncryptionStatus = (ECipherSuite)TWD_CIPHER_GEM;
            iTwdCtrl.eGroupKeyMode = TWD_CIPHER_GEM;
            break;
    
    case GEMPairWiseKey:
            param.content.rsnEncryptionStatus = (ECipherSuite)TWD_CIPHER_GEM;
            iTwdCtrl.ePairwiseKeyMode = TWD_CIPHER_GEM;
            break;
    #endif

    default:
            ASSERT_ERR (iTwdCtrl.hReport, "Unknown key type=%d\n", aType)
    }   

    param.paramType = TWD_RSN_SECURITY_MODE_PARAM_ID;
    TWD_SetParam (iTwdCtrl.hTWD, &param);

    /*  Converts the AddKey back to the correlated TWD command   */
    if (ConstructAddKey (&key, 
                                        aType,
                                        aKey,
                                        aEntryIndex) != WHA::KSuccess)
    {
        ASSERT_ERR (iTwdCtrl.hReport, "Failed to construct key type=%d\n", aType)
    }

    os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
                   (void *)&iTwdCtrl.privacyKey[aEntryIndex], 
                   (void *)&key, 
                   sizeof(TSecurityKeys));
    
    /* Activates the whal TWD_SetParam function (with TWD_RSN_KEY_ADD_PARAM)  */
    param.paramType = TWD_RSN_KEY_ADD_PARAM_ID;
    param.content.configureCmdCBParams.pCb = (TUint8*)&key;
    param.content.configureCmdCBParams.fCb = (void *)TIWhaAdaptCB::AddKeyResponse;
    param.content.configureCmdCBParams.hCb = this;

    status = TWD_SetParam (iTwdCtrl.hTWD, &param);

    if (status != TI_OK) 
    {
        ASSERT_ERR (iTwdCtrl.hReport, "Add key failure, key index=%d\n", aEntryIndex)
    }
}

/** 
* \fn     ConstructAddKey
* \brief  utility function for AddKey
*          
* \note   
* \return   
* \sa     
*/ 
WHA::TStatus TIWha::ConstructAddKey (
    TSecurityKeys   *aSecurityKey,  /* structure to be filled               */
    WHA::TKeyType        aType,          /* Type of the key to be added          */
    const void      *aKey,          /* Pointer to buffer specifying the key */
                                    /* material, according to the key type  */
                                    /* (see specification for details).     */
    TUint8          aEntryIndex)    /* Key entry index. Valid range: 0-8.   */
{  
    TUint8      broadcast[MAC_ADDR_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

    TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "(%d) - ConstructAddKey, type=%d ïndex=%d\n",__LINE__,aType, aEntryIndex);
    
    /* Fill up the mutual fields */
    switch (aType) 
    {
        case WHA::EWepGroupKey:
        {
            /* 
             * WEP Group Key:
             * 1) Key length 
             * 2) Key
             * 3) Default key number
             */
            WHA::SWepGroupKey* pKey = (WHA::SWepGroupKey*)aKey;  
            
            aSecurityKey->keyType = KEY_WEP;
            aSecurityKey->encLen = pKey->iKeyLengthInBytes; 
            aSecurityKey->keyIndex = pKey->iKeyId;
            
            if (!bJoined) {
            /* Clear mac address to distinguish between Group and Pairwise */
            os_memoryZero ((TI_HANDLE)&iTwdCtrl.tOsContext,
                           (void*)aSecurityKey->macAddress,
                           sizeof(TMacAddr));
            }
            else
            {
                MAC_COPY (aSecurityKey->macAddress, broadcast);
            }


            os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
                           (void*)aSecurityKey->encKey, 
                           (void*)pKey->iKey, 
                           aSecurityKey->encLen);
                
            break;
        }       
        case WHA::EWepPairWiseKey:
        {
            /*
             * WEP Pairwise Key:
             * 1) MAC Address of the peer station
             * 2) Key length 
             * 3) Key
             */
            WHA::SWepPairwiseKey* pKey = (WHA::SWepPairwiseKey*)aKey;  
            
            aSecurityKey->keyType = KEY_WEP;
            aSecurityKey->keyIndex = 0;
            MAC_COPY (aSecurityKey->macAddress, (TI_UINT8*)&pKey->iMacAddr); 
            aSecurityKey->encLen = pKey->iKeyLengthInBytes; 
            os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
                           (void*)aSecurityKey->encKey, 
                           (void*)pKey->iKey, 
                           aSecurityKey->encLen);
        
                break;
        }       
                
        case WHA::ETkipGroupKey: 
        {
            /* TKIP Group Key:
             * 1) 128 encryption key
             * 2) 64 bit Rx MIC Key
             * 3) Key ID
             */
            WHA::STkipGroupKey* pKey = (WHA::STkipGroupKey*)aKey; 
            
            aSecurityKey->keyType = KEY_TKIP;
            aSecurityKey->encLen = WHA::KTKIPKeyLength;
            MAC_COPY (aSecurityKey->macAddress, broadcast);
            os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
                           (void*)aSecurityKey->micRxKey, 
                           (void*)pKey->iRxMicKey, 
                           WHA::KTKIPKeyLength);           
            os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
                           (void*)aSecurityKey->encKey, 
                           (void*)pKey->iTkipKey, 
                           WHA::KTKIPKeyLength);
            aSecurityKey->keyIndex = pKey->iKeyId;
            break;
        }       

        case WHA::ETkipPairWiseKey:
        {
            /* 
             * TKIP Pairwise Key:
             * 1) MAC Address of the peer station
             * 2) 64 bit Rx MIC Key
             * 3) 64 bit Tx MIC Key
             * 4) 128 encryption key
             * 5) Key ID
             */
            WHA::STkipPairwiseKey* pKey = (WHA::STkipPairwiseKey*)aKey;             

            aSecurityKey->keyType = KEY_TKIP;
            aSecurityKey->encLen = WHA::KTKIPKeyLength;
            MAC_COPY (aSecurityKey->macAddress, (TI_UINT8*)&pKey->iMacAddr); 
            os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
                           (void*)aSecurityKey->micRxKey, 
                           (void*)pKey->iRxMicKey, 
                           WHA::KTKIPKeyLength);            
            os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
                           (void*)aSecurityKey->micTxKey, 
                           (void*)pKey->iTxMicKey, 
                           WHA::KTKIPKeyLength);            
            os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
                           (void*)aSecurityKey->encKey, 
                           (void*)pKey->iTkipKey, 
                           WHA::KTKIPKeyLength);
            aSecurityKey->keyIndex = pKey->iKeyId;
            break;
        }       

        case WHA::EAesGroupKey:
        {
            /* AES Group Key:
             * 1) 128 encryption key 
             * 2) key ID
             */
            WHA::SAesGroupKey* pKey = (WHA::SAesGroupKey*)aKey;             

            /* Fill security key structure */
            aSecurityKey->keyType = KEY_AES;
            aSecurityKey->encLen = WHA::KAesKeyLength;
            MAC_COPY (aSecurityKey->macAddress, broadcast);
            os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
                           (void*)aSecurityKey->encKey, 
                           (void*)pKey->iAesKey, 
                           WHA::KAesKeyLength);
            aSecurityKey->keyIndex = pKey->iKeyId;
            break;
        }       

        case WHA::EAesPairWiseKey:
        {
            /* 
             * AES Pairwise Key:
             * 1) MAC Address of the peer station
             * 2) 128 encryption key 
             */
            WHA::SAesPairwiseKey* pKey = (WHA::SAesPairwiseKey*)aKey;             

            /* Fill security key structure */
            aSecurityKey->keyType = KEY_AES;
            aSecurityKey->encLen = WHA::KAesKeyLength;
            aSecurityKey->keyIndex = 0;
            MAC_COPY (aSecurityKey->macAddress, (TI_UINT8*)&pKey->iMacAddr); 
            os_memoryCopy ((TI_HANDLE)&iTwdCtrl.tOsContext, 
                           (void*)aSecurityKey->encKey, 
                           (void*)pKey->iAesKey, 
                           WHA::KAesKeyLength);                      
            break;
        }

        #ifdef GEM_SUPPORT
            case GEMGroupKey:
            {
                SGEMGroupKey* pKey = (SGEMGroupKey*)aKey;
    
                /* Fill security key structure */
                aSecurityKey->keyType = KEY_GEM;
                aSecurityKey->encLen = KGEMKeyLength;
                aSecurityKey->keyIndex = pKey->iKeyId;
                MAC_COPY (aSecurityKey->macAddress, broadcast);
                GROUP_KEY_COPY ((TI_HANDLE)&iTwdCtrl.tOsContext,aSecurityKey,pKey,KGEMKeyLength);
                GROUP_MIC_COPY ((TI_HANDLE)&iTwdCtrl.tOsContext,aSecurityKey,pKey,KGEMMicKeyLength);
                break;
            }
    
            case GEMPairWiseKey:
            {
                SGEMPairwiseKey* pKey = (SGEMPairwiseKey*)aKey;             
    
                /* Fill security key structure */
                aSecurityKey->keyType = KEY_GEM;
                aSecurityKey->encLen = KGEMKeyLength;
                aSecurityKey->keyIndex = pKey->iKeyId;
                MAC_COPY (aSecurityKey->macAddress, (TI_UINT8*)&pKey->iMacAddr); 
                PAIRWISE_KEY_COPY ((TI_HANDLE)&iTwdCtrl.tOsContext,aSecurityKey,pKey,KGEMKeyLength);
                PAIRWISE_MIC_COPY ((TI_HANDLE)&iTwdCtrl.tOsContext,aSecurityKey,pKey,KGEMMicKeyLength);
                break;
            }
        #endif /* GEM_SUPPORT */

        default:
            /*
             * NULL_KEY, XCC_KEY
             */
            TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, "(%d) - ConstructAddKey - ERROR - Key not supported, %d\n",__LINE__,aType);

            break;
    }

    return WHA::KSuccess;
}

/** 
* \fn     RemoveKey
* \brief  Remove key from our data base and from Fw
*          
* \note   This function is never called from LDD, therfore we use dummy response
* \return   
* \sa     
*/ 
void TIWha::RemoveKey( TUint8 aEntryIndex)
{        
    TTwdParamInfo    param;
    TI_STATUS        status;
	ECipherSuite	 eOldCipherSuite;
    TBool 			 bSecurityChanged = FALSE;


    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "RemoveKey, aEntryIndex= 0x%x\n", aEntryIndex);

    if (aEntryIndex >= TIWha_MAX_PRIVACY_KEY_INDEX)
    {
        ASSERT_ERR (iTwdCtrl.hReport, "Invalid key index=%d\n", aEntryIndex)
    }
    
	param.paramType = TWD_RSN_SECURITY_MODE_PARAM_ID;
	status = TWD_GetParam(iTwdCtrl.hTWD, &param);
	if (status != TI_OK)
    {
        ASSERT_ERR (iTwdCtrl.hReport, "RemoveKey : Remove key failure, key index=%d\n", aEntryIndex)
    }

	eOldCipherSuite = param.content.rsnEncryptionStatus;

	/* Synchronies the current security mode with the key security mode */ 
	switch (iTwdCtrl.privacyKey[aEntryIndex].keyType)
	{
		case KEY_WEP: param.content.rsnEncryptionStatus = TWD_CIPHER_WEP; break;
	    case KEY_TKIP: param.content.rsnEncryptionStatus = TWD_CIPHER_TKIP; break;
        case KEY_AES: param.content.rsnEncryptionStatus = TWD_CIPHER_AES_CCMP; break;
        #ifdef GEM_SUPPORT
            case KEY_GEM: param.content.rsnEncryptionStatus = TWD_CIPHER_GEM; break;
        #endif
	}

    TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "RemoveKey, keyType= 0x%x, rsnEncryptionStatus = 0x%x eOldCipherSuite = 0x%x\n",         iTwdCtrl.privacyKey[aEntryIndex].keyType, param.content.rsnEncryptionStatus, eOldCipherSuite);

    /* Check if we should update security mode */
    if (eOldCipherSuite != param.content.rsnEncryptionStatus)
    {
        bSecurityChanged = TRUE;
        /* Change security mode in whalSecurity  */
        status = TWD_SetParam (iTwdCtrl.hTWD, &param);
		if (status != TI_OK)
		{
			ASSERT_ERR (iTwdCtrl.hReport, "RemoveKey : Remove key failure, key index=%d\n", aEntryIndex)
		}
    }
	
    /* Activates the TWD_SetParam function (with TWD_RSN_KEY_REMOVE_PARAM) */
    param.paramType = TWD_RSN_KEY_REMOVE_PARAM_ID;        
    param.content.configureCmdCBParams.pCb = (TUint8*)&iTwdCtrl.privacyKey[aEntryIndex];
    /* Note that we never response a remove key since it is internal command. Symbian never issue this command */
    param.content.configureCmdCBParams.fCb = (void *)TIWhaAdaptCB::DummyResponse;
    param.content.configureCmdCBParams.hCb = this;

     status = TWD_SetParam (iTwdCtrl.hTWD, &param); 

    if (status != TI_OK)
    {
        ASSERT_ERR (iTwdCtrl.hReport, "Remove key failure, key index=%d\n", aEntryIndex)
    }
    else
    {
        /* remove the key from the keys table */
        os_memoryZero ((TI_HANDLE)&iTwdCtrl.tOsContext, &iTwdCtrl.privacyKey[aEntryIndex], sizeof(TSecurityKeys));
    }

    /* Check if we should update back security mode */
    if (bSecurityChanged)
    {
        param.content.rsnEncryptionStatus = eOldCipherSuite;
        param.paramType = TWD_RSN_SECURITY_MODE_PARAM_ID;
        status = TWD_SetParam (iTwdCtrl.hTWD, &param);
		if (status != TI_OK)
		{
			ASSERT_ERR (iTwdCtrl.hReport, "RemoveKey : Remove key failure, key index=%d\n", aEntryIndex)
		}
    }
}

/** 
* \fn     ConfigureQueue
* 
* \return
* \sa     
*/
void TIWha::ConfigureQueue(
							  WHA::TQueueId aQueueId,
							  TUint32 aMaxLifeTime,
							  WHA::TPsScheme aPsScheme,
							  const WHA::SSAPSDConfig& aSAPSDConfig,
							  WHA::TAckPolicy aAckPolicy,
							  	TUint16 aMediumTime )
{    
    TTwdParamInfo        param;
    TQueueTrafficParams  QueueTrafficParams;
    TI_STATUS            status;  

    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "\n");

    TRACE5(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " TIWha::ConfigureQueue: aQueueId = 0x%x, aPsScheme = 0x%x, APSDConf[0] = 0x%x, APSDConf[1] = 0x%x, aAckPolicy = 0x%x\n",                aQueueId,aPsScheme,aSAPSDConfig.iServiceStartTime,aSAPSDConfig.iServiceInterval,aAckPolicy);

    /* Set parameters */ 
    QueueTrafficParams.queueID = (TUint8)aQueueId;
    /* 0- 3 EDCA , 4 - HCCA */
    QueueTrafficParams.channelType = (aQueueId < 4) ? CHANNEL_TYPE_EDCF : CHANNEL_TYPE_HCCA;
    QueueTrafficParams.tsid        = (TUint8)aQueueId;   
    QueueTrafficParams.dot11EDCATableMSDULifeTime = 0;
    QueueTrafficParams.psScheme    = aPsScheme;
    QueueTrafficParams.APSDConf[0] = aSAPSDConfig.iServiceStartTime;
    QueueTrafficParams.APSDConf[1] = aSAPSDConfig.iServiceInterval;
    QueueTrafficParams.ackPolicy = aAckPolicy;

    param.paramType = (TUint32)TWD_QUEUES_PARAM_ID;
    param.content.pQueueTrafficParams = &QueueTrafficParams;

    status = TWD_SetParam (iTwdCtrl.hTWD, &param);

    if (status != TI_OK) 
    {
        ASSERT_ERR ( iTwdCtrl.hReport, "ERROR:Configure queue failure, queue ID=%d\n", aQueueId)
    }
}

/** 
* \fn     ConfigureAC
* 
* \return
* \sa     
*/
void TIWha::ConfigureAC(
						   TUint16 aCwMin[Wha::KNumOfEdcaQueues],
						   TUint16 aCwMax[Wha::KNumOfEdcaQueues],
						   TUint8 aAIFS[Wha::KNumOfEdcaQueues],
						   TUint16 aTxOplimit[Wha::KNumOfEdcaQueues],
						   TUint16 aMaxReceiveLifeTime[Wha::KNumOfEdcaQueues] )
{    
    TAcQosParams      AcQosParams;
    WHA::TStatus           status    = TI_OK;
    TUint8            i;

    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "\n");
      
    /* configure all AC's but the last one in a loop that returns as a dummy CB */
    for (i = 0; (i < Wha::KNumOfEdcaQueues - 1) && (TI_OK == status); i++)
    {       
        TRACE5(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " TIWha::ConfigureAC: Index = 0x%x, aCwMin = 0x%x, aCwMax = 0x%x, aAIFS = 0x%x, aTxOpLimit = 0x%x\n",            i,aCwMin[i],aCwMax[i],aAIFS[i],aTxOplimit[i]);

        AcQosParams.ac = i;
        AcQosParams.cwMin = (TUint8)aCwMin[i];
        AcQosParams.cwMax = aCwMax[i];
        AcQosParams.aifsn = aAIFS[i];
        AcQosParams.txopLimit = aTxOplimit[i];

        status = TWD_CfgAcParams (iTwdCtrl.hTWD, 
                                  &AcQosParams, 
                                  (void *)TIWhaAdaptCB::DummyResponse,
                                  this);
    }

    /* After the loop - the last AC returns as a real CB */
    if (TI_OK == status)
    {
        TRACE5(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " TIWha::ConfigureAC: Index = 0x%x, aCwMin = 0x%x, aCwMax = 0x%x, aAIFS = 0x%x, aTxOpLimit = 0x%x\n",            i,aCwMin[i],aCwMax[i],aAIFS[i],aTxOplimit[i]);
        
        AcQosParams.ac = i;
        AcQosParams.cwMin = (TUint8)aCwMin[i];
        AcQosParams.cwMax = aCwMax[i];
        AcQosParams.aifsn = aAIFS[i];
        AcQosParams.txopLimit = aTxOplimit[i];

        status = TWD_CfgAcParams (iTwdCtrl.hTWD, 
                                  &AcQosParams, 
                                  NULL, /* The generic CB will be called */
                                  NULL);
    }

    if (status != TI_OK) 
    {
        TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "ERROR: Configure AC failure, status=%d\n", status);
    }
}



/** 
* \fn     Reset
* \brief  disconnect from BSS/IBSS
* \return
* \sa     
*/
void TIWha::Reset() 
{    
    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "\n");
    int          status;

    /* Check if we are in 802.11 PS mode. If not we should update sleepMode */
    if (!(TWD_GetPsStatus (iTwdCtrl.hTWD)))
    {
        /* Configure H/W to user sleep mode */
        SleepMode (iTwdCtrl.sleepMode, FALSE);
    }


    /* If connection timer is running - stop it */
    if (bConnectionTimerRunning)
    {
        bConnectionTimerRunning = TI_FALSE;
        os_timerStop(&iTwdCtrl.tOsContext, hConnectionTimer);
    }

#ifdef HT_SUPPORT
	TWD_CloseAllBaSessions(iTwdCtrl.hTWD);
#endif /* HT_SUPPORT */
    
    status = TWD_CmdFwDisconnect ( iTwdCtrl.hTWD ,DISCONNECT_IMMEDIATE , STATUS_UNSPECIFIED);

    if (status != TI_OK) 
    {
        ASSERT_ERR ( iTwdCtrl.hReport, "Reset failure, status=%d\n", status)
    }

    /* We are not connected */
    bJoined = 0;
}

#ifdef TI_TEST
TTestCmd mTest;
/**
* \fn     PltTester
* \brief  Called when PLT commands sent externally
*
* /Param aData - parametrs of PLT command
* /return  
*/
void TIWha::PltTester(const void *aData)
{
	TTestCmd* pPltTester = (TTestCmd*)aData;
	os_memoryCopy(NULL,&mTest,(void *)aData,sizeof(TTestCmd));
	Plt(pPltTester->testCmdId,&mTest.testCmd_u);
}



/**
* \fn     ChangeQueue
* \brief  Change the output queue for all packets
*
* /Param aData - queue to be set
* /return  
*/
void TIWha::ChangeQueue(const void *aData)
{
	iQueueId = *(WHA::TQueueId*)aData;

    if( bErrorIndication == TI_FALSE)
    {    
        WhaCb()->CommandResponse(WHA::EWriteMIBResponse, WHA::KSuccess, iUCommandResponseParams);
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif
}
#endif /* TI_TEST */


/**
* \fn     Plt
* \brief  production line testing
*
* The host driver calls this method to perform production 
* line testing of the WLAN device
* /param eTestCmd - specific test enum
* /param pTestCmdParams - test specific parameters and place for return buffer
* /return  TStatus - KPending or KError
*/
WHA::TStatus TIWha::Plt (ETestCmdID eTestCmd, void *pTestCmdParams)
{
	TI_STATUS status;
    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " CmdID = 0x%x\n", eTestCmd);

    /* Save type of test to be retrieved later on the CB */
    iTwdCtrl.ePlt = eTestCmd;

	switch (eTestCmd)
	{
    case TEST_CMD_P2G_CAL:
        status =  TWDriverTest (iTwdCtrl.hTWD, 
                                  (TestCmdID_enum) eTestCmd, 
                                  pTestCmdParams,
                                  TIWhaAdaptCB::TxBipResponse, 
                                  (TI_HANDLE) this);
        break;
        
    case TEST_CMD_RX_PLT_CAL:
        status =  TWDriverTest (iTwdCtrl.hTWD, 
                                  (TestCmdID_enum) eTestCmd, 
                                  pTestCmdParams,
                                  TIWhaAdaptCB::RxBipResponse, 
                                  (TI_HANDLE) this);
        break;

    case TEST_CMD_RX_STAT_GET:
		status =  TWDriverTest (iTwdCtrl.hTWD, 
						   (TestCmdID_enum) eTestCmd, 
						   pTestCmdParams,
						   TIWhaAdaptCB::RxStatResponse, 
						   (TI_HANDLE) this);
		break;
    case TEST_CMD_FREE_RUN_RSSI:
        status =  TWDriverTest (iTwdCtrl.hTWD, 
						   (TestCmdID_enum) eTestCmd, 
						   pTestCmdParams,
						   TIWhaAdaptCB::RunRssiResponse, 
						   (TI_HANDLE) this);
		break;

	default:
		status =  TWDriverTest (iTwdCtrl.hTWD, 
    				   (TestCmdID_enum) eTestCmd, 
    				   pTestCmdParams, 
    				   TIWhaAdaptCB::PltResponse, 
    				   (TI_HANDLE) this);
    } /* switch */

	return (status == TI_OK) ? WHA::KPending : WHA::KFailed;
}
/**
* \fn     Plt 
* \brief  production line testing 
*
* The host driver calls this method to perform production 
* line testing of the WLAN device
* /param TPltType - specific test enum
* /param aParams - test specific parameters and place for return buffer
* /return  TStatus - KPending or KError
*/
void TIWha::Plt (WHA::TPltType aType, void *aParams)
{
    Plt((ETestCmdID) aType,aParams);
}


/** 
 * \fn     WriteMem
 * \brief  write buffer to HW address
 * 
 * \note    
 * \param  pMemoryAccess - address, length, buffer
 * \return  TStatus 
 */ 
WHA::TStatus TIWha::WriteMem (TMemoryAccess *pMemoryAccess)
{    
TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " addr = 0x%x length = %d p = %p\n", pMemoryAccess->addr, pMemoryAccess->length, pMemoryAccess);

    TI_STATUS status = TWD_writeMem (iTwdCtrl.hTWD, 
        (TFwDebugParams*) pMemoryAccess, (void*)TIWhaAdaptCB::WriteMemResponse, this);

    /* convert status to WHA::TStatus */
    return (status == TI_OK) ? WHA::KPending : WHA::KFailed;
}


/** 
 * \fn     ReadMem
 * \brief  read buffer from HW address
 * 
 * \note    
 * \param  pMemoryAccess - address, length, buffer
 * \return  TStatus 
 */ 
WHA::TStatus TIWha::ReadMem (TMemoryAccess  *pMemoryAccess)
{
TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " addr = 0x%x length = %d p = %p\n", pMemoryAccess->addr, pMemoryAccess->length, pMemoryAccess);

    TI_STATUS status = TWD_readMem (iTwdCtrl.hTWD, 
        (TFwDebugParams*) pMemoryAccess, (void*)TIWhaAdaptCB::ReadMemResponse, this);

    /* convert status to WHA::TStatus */
    return (status == TI_OK) ? WHA::KPending : WHA::KFailed;
}


/**********************************************************
TIWha Service functions
**********************************************************/

/**
* \fn     getMacAddress
* \brief  return the Mac address from the TWDCtrl
*
* 
* /note
* 
* /return 
*/
void * TIWha::getMacAddress()
{
	return (void *)iTwdCtrl.pMacAddr;

}

/**
* \fn     InitTwdParamTable
* \brief  initialization of TWD parameters
*
* since we have no ini file, we are initializing the 
* Modules wit hard coded values
* /note
* 
* /return 
*/
void TIWha::InitTwdParamTable()
{
    os_memoryZero (&iTwdCtrl.tOsContext, (TI_UINT8*)&iTwdCtrl.twdInitParams, sizeof(TTwdInitParams));

    iTwdCtrl.twdInitParams.tGeneral.packetDetectionThreshold = 0;
    iTwdCtrl.twdInitParams.tGeneral.qosNullDataTemplateSize = sizeof(QosNullDataTemplate_t);
    iTwdCtrl.twdInitParams.tGeneral.PsPollTemplateSize = sizeof(psPollTemplate_t);
    iTwdCtrl.twdInitParams.tGeneral.probeResponseTemplateSize = sizeof(probeRspTemplate_t);
    iTwdCtrl.twdInitParams.tGeneral.probeRequestTemplateSize = sizeof(probeReqTemplate_t);
    iTwdCtrl.twdInitParams.tGeneral.beaconTemplateSize = sizeof(probeRspTemplate_t);
    iTwdCtrl.twdInitParams.tGeneral.nullTemplateSize = sizeof(nullDataTemplate_t);
    iTwdCtrl.twdInitParams.tGeneral.disconnTemplateSize = sizeof(disconnTemplate_t);
    /* Beacon broadcast options */
    iTwdCtrl.twdInitParams.tGeneral.BeaconRxTimeout           = BCN_RX_TIMEOUT_DEF_VALUE;
    iTwdCtrl.twdInitParams.tGeneral.BroadcastRxTimeout        = BROADCAST_RX_TIMEOUT_DEF_VALUE;
    iTwdCtrl.twdInitParams.tGeneral.RxBroadcastInPs           = RX_BROADCAST_IN_PS_DEF_VALUE;
    iTwdCtrl.twdInitParams.tGeneral.ConsecutivePsPollDeliveryFailureThreshold = CONSECUTIVE_PS_POLL_FAILURE_DEF;

    iTwdCtrl.twdInitParams.tGeneral.halCtrlRxDisableBroadcast = TWD_RX_DISABLE_BROADCAST_DEF;
    iTwdCtrl.twdInitParams.tGeneral.halCtrlCalibrationChannel2_4 = TWD_CALIBRATION_CHANNEL_2_4_DEF;
    iTwdCtrl.twdInitParams.tGeneral.halCtrlCalibrationChannel5_0 = TWD_CALIBRATION_CHANNEL_5_0_DEF;


    iTwdCtrl.twdInitParams.tGeneral.halCtrlRtsThreshold       = TWD_RTS_THRESHOLD_DEF;


    iTwdCtrl.twdInitParams.tGeneral.WiFiWmmPS                 = WIFI_WMM_PS_DEF;

    iTwdCtrl.twdInitParams.tGeneral.halCtrlMaxTxMsduLifetime  = TWD_MAX_TX_MSDU_LIFETIME_DEF;  
    iTwdCtrl.twdInitParams.tGeneral.halCtrlMaxRxMsduLifetime  = TWD_MAX_RX_MSDU_LIFETIME_DEF;  

    iTwdCtrl.twdInitParams.tGeneral.rxTimeOut.psPoll          = QOS_RX_TIMEOUT_PS_POLL_DEF;  
    iTwdCtrl.twdInitParams.tGeneral.rxTimeOut.UPSD            =  QOS_RX_TIMEOUT_UPSD_DEF; 

    /* RSSI/SNR Weights for Average calculations */
    iTwdCtrl.twdInitParams.tGeneral.uRssiBeaconAverageWeight  = TWD_RSSI_WEIGHT_DEF;
    iTwdCtrl.twdInitParams.tGeneral.uRssiPacketAverageWeight  = TWD_RSSI_WEIGHT_DEF;
    iTwdCtrl.twdInitParams.tGeneral.uSnrBeaconAverageWeight   = TWD_RSSI_WEIGHT_DEF;
    iTwdCtrl.twdInitParams.tGeneral.uSnrPacketAverageWeight   = TWD_RSSI_WEIGHT_DEF;

    /* No used */
    iTwdCtrl.twdInitParams.tGeneral.halCtrlFragThreshold      = TWD_FRAG_THRESHOLD_DEF;
    iTwdCtrl.twdInitParams.tGeneral.halCtrlListenInterval = TWD_LISTEN_INTERVAL_DEF;
    iTwdCtrl.twdInitParams.tGeneral.halCtrlRateFallbackRetry  =  TWD_RATE_FB_RETRY_LIMIT_DEF;       
    iTwdCtrl.twdInitParams.tGeneral.halCtrlMacClock           = 80;     
    iTwdCtrl.twdInitParams.tGeneral.halCtrlArmClock           = 80;     

    iTwdCtrl.twdInitParams.tGeneral.TxCompletePacingThreshold = TWD_TX_CMPLT_THRESHOLD_DEF;
    iTwdCtrl.twdInitParams.tGeneral.TxCompletePacingTimeout   = TWD_TX_CMPLT_TIMEOUT_DEF;
    iTwdCtrl.twdInitParams.tGeneral.RxIntrPacingThreshold     = TWD_RX_INTR_THRESHOLD_DEF;
    iTwdCtrl.twdInitParams.tGeneral.RxIntrPacingTimeout       = TWD_RX_INTR_TIMEOUT_DEF;

	/* FM Coexistence params */
	iTwdCtrl.twdInitParams.tGeneral.tFmCoexParams.uEnable	  		= FM_COEX_ENABLE_DEF;
	iTwdCtrl.twdInitParams.tGeneral.tFmCoexParams.uSwallowPeriod 	= FM_COEX_SWALLOW_PERIOD_DEF;
	iTwdCtrl.twdInitParams.tGeneral.tFmCoexParams.uNDividerFrefSet1	= FM_COEX_N_DIVIDER_FREF_SET1_DEF;
	iTwdCtrl.twdInitParams.tGeneral.tFmCoexParams.uNDividerFrefSet2 = FM_COEX_N_DIVIDER_FREF_SET2_DEF;
	iTwdCtrl.twdInitParams.tGeneral.tFmCoexParams.uMDividerFrefSet1 = FM_COEX_M_DIVIDER_FREF_SET1_DEF;
	iTwdCtrl.twdInitParams.tGeneral.tFmCoexParams.uMDividerFrefSet2 = FM_COEX_M_DIVIDER_FREF_SET2_DEF;
	iTwdCtrl.twdInitParams.tGeneral.tFmCoexParams.uCoexPllStabilizationTime = FM_COEX_PLL_STABILIZATION_TIME_DEF;
	iTwdCtrl.twdInitParams.tGeneral.tFmCoexParams.uLdoStabilizationTime	  	= FM_COEX_LDO_STABILIZATION_TIME_DEF;
	iTwdCtrl.twdInitParams.tGeneral.tFmCoexParams.uFmDisturbedBandMargin	= FM_COEX_DISTURBED_BAND_MARGIN_DEF;
	iTwdCtrl.twdInitParams.tGeneral.tFmCoexParams.uSwallowClkDif	  		= FM_COEX_SWALLOW_CLK_DIF_DEF;


	/* Configure ARP IP */

    iTwdCtrl.twdInitParams.tArpIpFilter.isFilterEnabled       = DEF_FILTER_ENABLE_VALUE;
    // Compialtion problem     IP_COPY(iTwdCtrl.twdInitParams.tArpIpFilter.addr = "00 22 11 33 44 55");


    /* Configure address group */
    iTwdCtrl.twdInitParams.tMacAddrFilter.numOfMacAddresses   = NUM_GROUP_ADDRESS_VALUE_DEF; 
    iTwdCtrl.twdInitParams.tMacAddrFilter.isFilterEnabled     = DEF_FILTER_ENABLE_VALUE;
#if 0
    for (k = 0; k < pWlanParams->numGroupAddrs; k++)
    {
    	MAC_COPY (iTwdCtrl.twdInitParams.tMacAddrFilter.macAddrTable[k]); 
    }
#endif

    /* QoS configure queue */
    iTwdCtrl.twdInitParams.tGeneral.TxBlocksThresholdPerAc[0] = QOS_TX_BLKS_THRESHOLD_BE_DEF;
    iTwdCtrl.twdInitParams.tGeneral.TxBlocksThresholdPerAc[1] = QOS_TX_BLKS_THRESHOLD_BK_DEF;
    iTwdCtrl.twdInitParams.tGeneral.TxBlocksThresholdPerAc[2] = QOS_TX_BLKS_THRESHOLD_VI_DEF;
    iTwdCtrl.twdInitParams.tGeneral.TxBlocksThresholdPerAc[3] = QOS_TX_BLKS_THRESHOLD_VO_DEF;



    /* Configure the MAC services */ 

    /* Power server */
    iTwdCtrl.twdInitParams.tPowerSrv.hangOverPeriod           = HANGOVER_PERIOD_DEF_VALUE + 5; /* Increase the hang over time to 10ms */
    iTwdCtrl.twdInitParams.tPowerSrv.numNullPktRetries        = POWER_MGMNT_NUM_NULL_PACKET_RETRY_DEF_VALUE;
    /* Scan Server */
    iTwdCtrl.twdInitParams.tScanSrv.numberOfNoScanCompleteToRecovery = SCAN_SRV_NUMBER_OF_NO_SCAN_COMPLETE_TO_RECOVERY_DEF;
    iTwdCtrl.twdInitParams.tScanSrv.uTriggeredScanTimeOut     = SCAN_SRV_TRIGGERED_SCAN_TIME_OUT_DEF;
    iTwdCtrl.TxAlign.uCurrSend = 0;
    iTwdCtrl.TxAlign.uCurrHandle = 0;

    /* Configure the rate adaptaion */
    iTwdCtrl.twdInitParams.tRateMngParams.InverseCuriosityFactor[0] = RATE_MGMT_INVERSE_CURIOSITY_FACTOR_OTHER;
    iTwdCtrl.twdInitParams.tRateMngParams.InverseCuriosityFactor[1] = RATE_MGMT_INVERSE_CURIOSITY_FACTOR_VOICE;
    iTwdCtrl.twdInitParams.tRateMngParams.MaxPer = RATE_MGMT_MAX_PER;
    iTwdCtrl.twdInitParams.tRateMngParams.PerAdd = RATE_MGMT_PER_ADD;
    iTwdCtrl.twdInitParams.tRateMngParams.PerAddShift = RATE_MGMT_PER_ADD_SHIFT;
    iTwdCtrl.twdInitParams.tRateMngParams.PerAlphaShift = RATE_MGMT_PER_ALPHA_SHIFT;
    iTwdCtrl.twdInitParams.tRateMngParams.PerBeta1Shift = RATE_MGMT_PER_BETA1_SHIFT;
    iTwdCtrl.twdInitParams.tRateMngParams.PerBeta2Shift = RATE_MGMT_PER_BETA2_SHIFT;
    iTwdCtrl.twdInitParams.tRateMngParams.PerTh1 = RATE_MGMT_PER_TH1;
    iTwdCtrl.twdInitParams.tRateMngParams.PerTh2 = RATE_MGMT_PER_TH2;
    iTwdCtrl.twdInitParams.tRateMngParams.RateCheckDown = RATE_MGMT_RATE_CHECK_DOWN;
    iTwdCtrl.twdInitParams.tRateMngParams.RateCheckUp = RATE_MGMT_RATE_CHECK_UP;
    
    iTwdCtrl.twdInitParams.tRateMngParams.RateRetryPolicy[0] = 1;
    iTwdCtrl.twdInitParams.tRateMngParams.RateRetryPolicy[1] = 1;
    iTwdCtrl.twdInitParams.tRateMngParams.RateRetryPolicy[2] = 1;
    iTwdCtrl.twdInitParams.tRateMngParams.RateRetryPolicy[3] = 3;
    iTwdCtrl.twdInitParams.tRateMngParams.RateRetryPolicy[4] = 3;
    iTwdCtrl.twdInitParams.tRateMngParams.RateRetryPolicy[5] = 3;
    iTwdCtrl.twdInitParams.tRateMngParams.RateRetryPolicy[6] = 6;
    iTwdCtrl.twdInitParams.tRateMngParams.RateRetryPolicy[7] = 6;
    iTwdCtrl.twdInitParams.tRateMngParams.RateRetryPolicy[8] = 6;
    iTwdCtrl.twdInitParams.tRateMngParams.RateRetryPolicy[9] = 6;
    iTwdCtrl.twdInitParams.tRateMngParams.RateRetryPolicy[10] = 9;
    iTwdCtrl.twdInitParams.tRateMngParams.RateRetryPolicy[11] = 9;
    iTwdCtrl.twdInitParams.tRateMngParams.RateRetryPolicy[12] = 9;

    iTwdCtrl.twdInitParams.tRateMngParams.RateRetryScore = RATE_MGMT_RATE_RETRY_SCORE;
    iTwdCtrl.twdInitParams.tRateMngParams.TxFailLowTh = RATE_MGMT_TX_FAIL_LOW_TH;
    iTwdCtrl.twdInitParams.tRateMngParams.TxFailHighTh = RATE_MGMT_TX_FAIL_HIGH_TH;
    iTwdCtrl.twdInitParams.tRateMngParams.PerWeightShift[0] = RATE_MGMT_PER_WEIGHT_SHIFT_OTHER;
    iTwdCtrl.twdInitParams.tRateMngParams.PerWeightShift[1] = RATE_MGMT_PER_WEIGHT_SHIFT_VOICE;
    iTwdCtrl.twdInitParams.tRateMngParams.TpWeightShift[0] = RATE_MGMT_TP_WEIGHT_SHIFT_OTHER;
    iTwdCtrl.twdInitParams.tRateMngParams.TpWeightShift[1] = RATE_MGMT_TP_WEIGHT_SHIFT_VOICE;
    iTwdCtrl.twdInitParams.tRateMngParams.paramIndex = RATE_MGMT_ALL_PARAMS;

    iTwdCtrl.twdInitParams.tDcoItrimParams.enable = TWD_DCO_ITRIM_ENABLE_DEF;
    iTwdCtrl.twdInitParams.tDcoItrimParams.moderationTimeoutUsec = TWD_DCO_ITRIM_MODERATION_TIMEOUT_DEF;
}
/**
* \fn     InitTwdPlatformGenParam
* \brief  initialization Platform General parameters
*
* 
* /return 
*/

void  TIWha::InitTwdPlatformGenParam()
{
  os_memoryCopy(iTwdCtrl.tOsContext.hOsa,&iTwdCtrl.twdInitParams.tPlatformGenParams,&iAutoRadioParams.tGeneralParams,sizeof(IniFileGeneralParam));
}


/**
* \fn     InitTwdRadioParam
* \brief  initialization of Radio params
* * /return 
*/

void TIWha::InitTwdRadioParam()
{
  uint8 FemType;

  if (iAutoRadioParams.tGeneralParams.TXBiPFEMAutoDetect == FEM_MANUAL_DETECT_MODE_E)
      FemType = iAutoRadioParams.tGeneralParams.TXBiPFEMManufacturer;
  else
      FemType = TWD_GetFEMType(iTwdCtrl.hTWD);
  /* fill TWD init parms with relevant (RFMD or TriQuint) Radio parms before calling TWD_SetDefault*/
   os_memoryCopy(iTwdCtrl.tOsContext.hOsa,&iTwdCtrl.twdInitParams.tIniFileRadioParams.tDynRadioParams,&iAutoRadioParams.tDynRadioParams[FemType],sizeof(TDynRadioParams));
   os_memoryCopy(iTwdCtrl.tOsContext.hOsa,&iTwdCtrl.twdInitParams.tIniFileRadioParams.tStatRadioParams,&iAutoRadioParams.tStatRadioParams,sizeof(TStatRadioParams));
}


/**
* \fn     InitTwdRadioParam
* \brief  initialization of Radio params
* * /return 
*/

void TIWha::InitTwdRadioSmartReflexParam()
{
    /* fill TWD init parms with relevant (RFMD or TriQuint) Radio parms before calling TWD_SetDefault*/
   os_memoryCopy(iTwdCtrl.tOsContext.hOsa,&iTwdCtrl.twdInitParams.tSmartReflexParams.errorTable, &iSmartReflexParams.tErrorTable,sizeof(TSmartReflexErrTable));
   os_memoryZero(iTwdCtrl.tOsContext.hOsa, &iSmartReflexDebugParams, sizeof(TSmartReflexDebugParams));
   iTwdCtrl.twdInitParams.tSmartReflexState.enable = iSmartReflexParams.SmartReflexState;
}        


/**
* \fn     InitReportParamTable
* \brief  initialization of Report parameters
*
* since we ave no ini file, we are initializing the 
* Modules with hard coded values
* /note
* 
* /return 
*/
void TIWha::InitReportParamTable()
{
    /* Open All Modules and some severity */
    iTwdCtrl.report_init.aSeverityTable [REPORT_SEVERITY_INIT]           = '0';
    iTwdCtrl.report_init.aSeverityTable [REPORT_SEVERITY_INFORMATION]    = '0';	
    iTwdCtrl.report_init.aSeverityTable [REPORT_SEVERITY_WARNING]        = '0';
    iTwdCtrl.report_init.aSeverityTable [REPORT_SEVERITY_ERROR]          = '1';
    iTwdCtrl.report_init.aSeverityTable [REPORT_SEVERITY_FATAL_ERROR]    = '1';
    iTwdCtrl.report_init.aSeverityTable [REPORT_SEVERITY_SM]             = '0';
    iTwdCtrl.report_init.aSeverityTable [REPORT_SEVERITY_CONSOLE]        = '0';
    
    for (TUint8 index = 0; index < REPORT_FILES_NUM; index++)
    {
    	iTwdCtrl.report_init.aFileEnable [index] = '1';
    }
}

/**
* \fn     OpenAllReports
* \brief  open all reports in report module. 
*
* Use this function if you encounter some problem in a specific area.
*
* /note Used for debug only.
* 
* /return 
*/
void TIWha::OpenAllReports()
{
    for (TUint8 index = 0; index < SIZE_ARR(iTwdCtrl.report_init.aSeverityTable); index++)
    {
        iTwdCtrl.report_init.aSeverityTable [index] = '1';    
    }

    for (TUint8 index = 0; index < REPORT_FILES_NUM; index++)
    {
    	iTwdCtrl.report_init.aFileEnable [index] = '1';
    }

    report_SetDefaults (iTwdCtrl.hReport, &(iTwdCtrl.report_init));

    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "\n");
}


/** 
 * \fn     RegisterCb
 * \brief  registers the TIWha callbacks
 * 
 * Register callbacks To Twd level according to the action
 * that we would like to be triggered on 
 * 
 * \note    
 * \return  status whther the registration succeed or not.
 * \sa      
 */
WHA::TStatus TIWha::RegisterCb ()
{   
    /* Register the failure event callback */
    TWD_RegisterCb (iTwdCtrl.hTWD, 
                    TWD_EVENT_FAILURE, 
                (TTwdCB *)TIWhaAdaptCB::FailureIndication, 
                 this);

	/* Register the send packet callback */
	TWD_RegisterCb (iTwdCtrl.hTWD,
					TWD_EVENT_TX_XFER_SEND_PKT_TRANSFER, 
					(TTwdCB *)TIWhaAdaptCB::TxXfer, 
					this);

	/* Register the send complete packet callback */
	TWD_RegisterCb (iTwdCtrl.hTWD,
					TWD_EVENT_TX_RESULT_SEND_PKT_COMPLETE, 
					(TTwdCB *)TIWhaAdaptCB::TxComplete, 
					this);

	/* Register the receive packet callback */
	TWD_RegisterCb (iTwdCtrl.hTWD,
					TWD_EVENT_RX_RECEIVE_PACKET, 
					(TTwdCB *)TIWhaAdaptCB::ReceivePacket, 
					this);

	/* Register the request for buffer callback */
	TWD_RegisterCb (iTwdCtrl.hTWD,
					TWD_EVENT_RX_REQUEST_FOR_BUFFER, 
					(TTwdCB *)TIWhaAdaptCB::RequestForBuffer, 
					this);

    /* register scan SRV scan complete CB */
    TWD_RegisterScanCompleteCb (iTwdCtrl.hTWD, 
                                                (TScanSrvCompleteCb)TIWhaAdaptCB::ScanComplete, this);

    return WHA::KSuccess;
}

/** 
 * \fn     RegisterEvents
 * \brief  register the TIWha Events
 * 
 * Register callbacks To Twd level according to the event 
 * that we would like to be triggered on 
 * 
 * \note    
 * \return  status whther the registration succeed or not.
 * \sa      
 */
WHA::TStatus TIWha::RegisterEvents ()
{   
	/* Register the Regain BSS callback */
	TWD_RegisterEvent (iTwdCtrl.hTWD, 
					   TWD_OWN_EVENT_BSS_REGAIN, 
					   (void *)TIWhaAdaptCB::RegainBssIndication, 
					   this);

	/* Register the RSSI callback */
	TWD_RegisterEvent (iTwdCtrl.hTWD, 
					   TWD_OWN_EVENT_RSSI_SNR_TRIGGER_0, //adi - rssi - use TWD_EVENT_RSSI_SNR_TRIGGER_0
					   (void *)TIWhaAdaptCB::RcpiIndication, 
					   this);  
	TWD_EnableEvent (iTwdCtrl.hTWD, TWD_OWN_EVENT_RSSI_SNR_TRIGGER_0);
#if 0
	assaf: no support
	TWD_RegisterEvent (iTwdCtrl.hTWD, 
					   TWD_EVENT_RSSI_LEVEL_REGAIN,	//adi - rssi - use TWD_EVENT_RSSI_SNR_TRIGGER_0 
					   (void *)TIWhaAdaptCB::RcpiIndication, 
					   this);
#endif
	/* Register & enable the Join callback */
        TWD_RegisterEvent (iTwdCtrl.hTWD,  
                                        TWD_OWN_EVENT_JOIN_CMPLT, 
                                        (void *)TIWhaAdaptCB::JoinComplete, 
                                        this);

	TWD_EnableEvent (iTwdCtrl.hTWD, TWD_OWN_EVENT_JOIN_CMPLT);

#if 0    
	assaf: no BT support
	/* Register the BtCoexsitence indications */
	TWD_RegisterEvent (iTwdCtrl.hTWD, 
					   TWD_EVENT_BT_COEX_SENSE, 
					   (void *)TIWhaAdaptCB::btCoexSenseIndication, 
					   this);       

	TWD_RegisterEvent (iTwdCtrl.hTWD, 
					   TWD_EVENT_BT_COEX_PROTECTIVE_MODE, 
					   (void *)TIWhaAdaptCB::btCoexProtectiveIndication, 
					   this);        

	TWD_RegisterEvent (iTwdCtrl.hTWD, 
					   TWD_EVENT_BT_COEX_AVALANCHE, 
					   (void *)TIWhaAdaptCB::btCoexAvalancheIndication, 
					   this);       
#endif
	/* Roaming Triggers */
	TWD_RegisterEvent (iTwdCtrl.hTWD,
					   TWD_OWN_EVENT_BSS_LOSE,
					   (void *)TIWhaAdaptCB::LostBssIndication,
					   this);

	TWD_EnableEvent (iTwdCtrl.hTWD, TWD_OWN_EVENT_BSS_LOSE);



	return WHA::KSuccess;
}

 

 
/**
* \fn     FillNWSASettings
* \brief  Fill UMAC structure with correct settings
*
* since we ave no ini file, we are initializing the 
* Modules wit hard coded values
* \note
* 
* \param SNWSASettings - pointer pointer to the UMAC settings struct
* \return status
*/
WHA::TStatus TIWha::FillNWSASettings (WHA::SSettings *SNWSASettings)
{
#ifdef HT_SUPPORT
	TTwdHtCapabilities* pTwdHtCapabilities;
#endif /* HT_SUPPORT */
    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INIT, "FillNWSASettings +");
    
    /* Fill all the parameters in the NWSA Settings */
    SNWSASettings->iNumOfSSIDs              = TIWha_SCAN_MAX_SSID_NUM;        
    SNWSASettings->iRxBufAlignment          = TIWha_NO_BUFFER_ALIGNMENT_SETTING; /* no alignment is needed */
    SNWSASettings->iNumOfBytesForSsid       = TIWha_MAX_SSID_LEN;            
    SNWSASettings->iRates                   = TIWha_RATE_BIT_MASK_SUPPORT;               
    SNWSASettings->iBand                    = TIWha_BAND_BIT_MASK_SUPPORT;                                           
    SNWSASettings->iRxoffset                = WSPI_PAD_LEN_READ + sizeof(RxIfDescriptor_t);
    SNWSASettings->iNumOfGroupTableEntrys   = ADDRESS_GROUP_MAX;        
    SNWSASettings->iNumOfTxRateClasses      = MAX_NUM_OF_TX_RATE_CLASS_POLICIES;    
    SNWSASettings->iTxFrameTrailerSpace     = 0;   
    SNWSASettings->iTxFrameHeaderSpace      = TX_TOTAL_OFFSET_BEFORE_DATA;  
    SNWSASettings->iFlagsMask               = TIWha_RECEIVE_PACKET_BIT_MASK_SUPPORT; 
    SNWSASettings->iCapability              = TIWha_CAPABILITY_BIT_MASK_SUPPORT; 

#ifdef GEM_SUPPORT
	SNWSASettings->iCapability				|= TIWha_CAPABILITY_GEM_SUPPORT;
#endif /* GEM_SUPPORT */

#ifdef MB_ENABLE
	SNWSASettings->iCapability				|= TIWha_CAPABILITY_MB_SUPPORT;
#endif

#ifdef HT_SUPPORT
	SNWSASettings->iCapability |= TIWha_CAPABILITY_HT_OPERATION;	
#endif /* HT_SUPPORT */

    SNWSASettings->iCapability |= TIWha_CAPABILITY_AUTO_RATE;

    /* Add DS Parameters -- Allow only APs from the requested channel 
    to answer the probe increase scan result in crowded environment */
    
    //SNWSASettings->iCapability |= WHA::SSettings::KDsParamSetIeInProbe;

    /* Tx power level for each radio band  */
    TFwInfo *pFwInfo = TWD_GetFWInfo (iTwdCtrl.hTWD);

    /* 
     * 2.4 GHz
     * in txPowerTable[x][y]: x - 0 for 2.4GHz, y - power level starting from 0 to NUM_POWER_LEVELS - 1 
     */ 
    SNWSASettings->iTxPowerRange[0].iMaxPowerLevel = 
        pFwInfo->txPowerTable[RADIO_BAND_2_4_GHZ][0] / DBM_TO_TX_POWER_FACTOR;
    SNWSASettings->iTxPowerRange[0].iMinPowerLevel = 
        pFwInfo->txPowerTable[RADIO_BAND_2_4_GHZ][NUM_POWER_LEVELS - 1] / DBM_TO_TX_POWER_FACTOR;
    SNWSASettings->iTxPowerRange[0].iStepping = 0;           

    TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, ": 2.4 iMaxPowerLevel = %d iMinPowerLevel = %d stepping = %d\n",    						    SNWSASettings->iTxPowerRange[0].iMaxPowerLevel,                                          SNWSASettings->iTxPowerRange[0].iMinPowerLevel,                                          SNWSASettings->iTxPowerRange[0].iStepping);

    /* 4.9 GHZ temporary not supported for the power level */
    SNWSASettings->iTxPowerRange[1].iMaxPowerLevel = 0;
    SNWSASettings->iTxPowerRange[1].iMinPowerLevel = 0;
    SNWSASettings->iTxPowerRange[1].iStepping = 0;          

    /* 5.0 GHZ temporary not supported for the power level since there are 4 sub-bands */
    SNWSASettings->iTxPowerRange[2].iMaxPowerLevel  = 0;
    SNWSASettings->iTxPowerRange[2].iMinPowerLevel = 0;
    SNWSASettings->iTxPowerRange[2].iStepping = 0; 

#ifdef HT_SUPPORT
	TWD_GetTwdHtCapabilities (iTwdCtrl.hTWD,&pTwdHtCapabilities);
	TConvertTwdHtCapa2SHtCapa (pTwdHtCapabilities,&SNWSASettings->iHtCapabilities);
#endif /* HT_SUPPORT */

    return WHA::KSuccess;
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
void TIWha::FillRadioData()
{
   
   iAutoRadioParams.tGeneralParams.TXBiPFEMAutoDetect = FEM_AUTO_DETECT_MODE_E;
   iAutoRadioParams.tGeneralParams.TXBiPFEMManufacturer = FEM_TRIQUINT_TYPE_E;   

   iAutoRadioParams.tGeneralParams.RefClk = eREF_CLK_38_4_E;                                 
   iAutoRadioParams.tGeneralParams.SettlingTime = 5;                                                                 
   iAutoRadioParams.tGeneralParams.ClockValidOnWakeup = REF_CLK_NOT_VALID_E;                      
   iAutoRadioParams.tGeneralParams.DC2DCMode = BT_SPI_IS_NOT_USED_E;                               
   iAutoRadioParams.tGeneralParams.Single_Dual_Band_Solution = SINGLE_BAND_SOLUTION_E;  
      
   iAutoRadioParams.tStatRadioParams.RxTraceInsertionLoss_2_4G = 0;
   iAutoRadioParams.tStatRadioParams.TXTraceLoss_2_4G  = 0;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[0] = 0xEC;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[1] = 0xF6;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[2] = 0x00;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[3] = 0x0C;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[4] = 0x18;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[5] = 0xF8;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[6] = 0xFC;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[7] = 0x00;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[8] = 0x08;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[9] = 0x10;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[10] = 0xF0;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[11] = 0xF8;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[12] = 0x00;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[13] = 0x0A;
   iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_2_4G[14] = 0x14;
   memset(&iAutoRadioParams.tStatRadioParams.RxTraceInsertionLoss_5G[0],0,NUMBER_OF_SUB_BANDS_IN_5G_BAND_E);
   memset(&iAutoRadioParams.tStatRadioParams.TXTraceLoss_5G[0],0,NUMBER_OF_SUB_BANDS_IN_5G_BAND_E);
   memset(&iAutoRadioParams.tStatRadioParams.RxRssiAndProcessCompensation_5G[0],0,RSSI_AND_PROCESS_COMPENSATION_TABLE_SIZE); 

/* RMFD deafult value */
   if (iAutoRadioParams.tGeneralParams.Single_Dual_Band_Solution == DUAL_BAND_SOLUTION_E)
   {
   
    iAutoRadioParams.tDynRadioParams[0].TXBiPReferencePDvoltage_2_4G = 0 ;									
    iAutoRadioParams.tDynRadioParams[0].TxBiPReferencePower_2_4G = 0;																				
    iAutoRadioParams.tDynRadioParams[0].TxBiPOffsetdB_2_4G = 0;																							
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[0] = 0x1E;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[1] = 0x1F;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[2] = 0x22;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[3] = 0x24;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[4] = 0x28;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[5] = 0x29;							
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[0] = 0x1B;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[1] = 0x1C;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[2] = 0x1E;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[3] = 0x20;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[4] = 0x24;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[5] = 0x25;							
    for (int i=0; i<NUMBER_OF_2_4_G_CHANNELS;i++)
    {
        iAutoRadioParams.tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_11b[i] = 0x50;
        iAutoRadioParams.tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_OFDM[i] = 0x50;
    }
       
    iAutoRadioParams.tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_OFDM[0] = 0x20;
    iAutoRadioParams.tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_OFDM[10] = 0x20;
    memset(&iAutoRadioParams.tDynRadioParams[0].TxPDVsRateOffsets_2_4G[0],0,NUMBER_OF_RATE_GROUPS_E);												
     for (int i=0; i<NUMBER_OF_RATE_GROUPS_E;i++)
    {
        iAutoRadioParams.tDynRadioParams[0].TxIbiasTable_2_4G[i] = 0x0E;
    }
       iAutoRadioParams.tDynRadioParams[0].TxIbiasTable_2_4G[5] = 0x17;
    iAutoRadioParams.tDynRadioParams[0].RxFemInsertionLoss_2_4G = 0x0D;																			

       // SECTION 2: 5G parameters
    memset(&iAutoRadioParams.tDynRadioParams[0].TXBiPReferencePDvoltage_5G[0],0,NUMBER_OF_RATE_GROUPS_E);
    memset(&iAutoRadioParams.tDynRadioParams[0].TxBiPReferencePower_5G[0],0,NUMBER_OF_RATE_GROUPS_E);				
    memset(&iAutoRadioParams.tDynRadioParams[0].TxBiPOffsetdB_5G[0],0,NUMBER_OF_RATE_GROUPS_E);								
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_5G_Normal[0] = 0x1D;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_5G_Normal[1] = 0x1E;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_5G_Normal[2] = 0x21;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_5G_Normal[3] = 0x23;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_5G_Normal[4] = 0x27;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_5G_Normal[5] = 0;							
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_5G_Degraded[0]= 0x1A;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_5G_Degraded[1] = 0x1B;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_5G_Degraded[2] = 0x1D;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_5G_Degraded[3] = 0x1F;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_5G_Degraded[4] = 0x23;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_5G_Degraded[5] = 0x00;						
    memset(&iAutoRadioParams.tDynRadioParams[0].TxPerChannelPowerLimits_5G_OFDM[0],0,NUMBER_OF_5G_CHANNELS);                         
    memset(&iAutoRadioParams.tDynRadioParams[0].TxPDVsRateOffsets_5G[0],0,NUMBER_OF_RATE_GROUPS_E);										
    for (int i=0;i<NUMBER_OF_RATE_GROUPS_E;i++)
    {
        iAutoRadioParams.tDynRadioParams[0].TxIbiasTable_5G[i] = 0x27;
        iAutoRadioParams.tDynRadioParams[0].RxFemInsertionLoss_5G[i] = 0x12;
    }
    
    iAutoRadioParams.tDynRadioParams[0].TxIbiasTable_5G[5] = 0;
    								
   }
   else
   {
    iAutoRadioParams.tDynRadioParams[0].TXBiPReferencePDvoltage_2_4G =0x24E ;									
    iAutoRadioParams.tDynRadioParams[0].TxBiPReferencePower_2_4G = 0x78;																				
    iAutoRadioParams.tDynRadioParams[0].TxBiPOffsetdB_2_4G = 0;																							
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[0] = 0x1E;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[1] = 0x1F;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[2] = 0x22;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[3] = 0x24;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[4] = 0x28;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Normal[5] = 0x29;							
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[0] = 0x1B;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[1] = 0x1C;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[2] = 0x1E;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[3] = 0x20;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[4] = 0x24;
    iAutoRadioParams.tDynRadioParams[0].TxPerRatePowerLimits_2_4G_Degraded[5] = 0x25;							
    for (int i =0;i<NUMBER_OF_2_4_G_CHANNELS;i++)
    {
        iAutoRadioParams.tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_11b[i] = 0x50;
        iAutoRadioParams.tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_OFDM[i] = 0x50;
    }
    iAutoRadioParams.tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_OFDM[0] = 0x20;
    iAutoRadioParams.tDynRadioParams[0].TxPerChannelPowerLimits_2_4G_OFDM[10] = 0x20;   
    memset(&iAutoRadioParams.tDynRadioParams[0].TxPDVsRateOffsets_2_4G[0],0,NUMBER_OF_RATE_GROUPS_E);	
    for (int i = 0 ;i <NUMBER_OF_RATE_GROUPS_E;i++) 
    {
        iAutoRadioParams.tDynRadioParams[0].TxIbiasTable_2_4G[i] = 0x1A;
    }
   
    iAutoRadioParams.tDynRadioParams[0].TxIbiasTable_2_4G[5] = 0x2F;
    iAutoRadioParams.tDynRadioParams[0].RxFemInsertionLoss_2_4G = 0;	

/* TriQuint default value */
    iAutoRadioParams.tDynRadioParams[1].TXBiPReferencePDvoltage_2_4G= 0x168;									
    iAutoRadioParams.tDynRadioParams[1].TxBiPReferencePower_2_4G= 0x83;																				
    iAutoRadioParams.tDynRadioParams[1].TxBiPOffsetdB_2_4G=0;																							
    iAutoRadioParams.tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Normal[0] = 0x1E;
    iAutoRadioParams.tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Normal[1] = 0x1F;
    iAutoRadioParams.tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Normal[2] = 0x22;
    iAutoRadioParams.tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Normal[3] = 0x24;
    iAutoRadioParams.tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Normal[4] = 0x28;
    iAutoRadioParams.tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Normal[5] = 0x29;							
    iAutoRadioParams.tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Degraded[0] = 0x1B;
    iAutoRadioParams.tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Degraded[1] = 0x1C;
    iAutoRadioParams.tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Degraded[2] = 0x1E;
    iAutoRadioParams.tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Degraded[3] = 0x20;
    iAutoRadioParams.tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Degraded[4] = 0x24;
    iAutoRadioParams.tDynRadioParams[1].TxPerRatePowerLimits_2_4G_Degraded[5] = 0x25;	
    for (int i =0;i<NUMBER_OF_2_4_G_CHANNELS;i++)
    {
        iAutoRadioParams.tDynRadioParams[1].TxPerChannelPowerLimits_2_4G_11b[i] = 0x50;
        iAutoRadioParams.tDynRadioParams[1].TxPerChannelPowerLimits_2_4G_OFDM[i] = 0x50;
    }
    iAutoRadioParams.tDynRadioParams[1].TxPerChannelPowerLimits_2_4G_OFDM[0] = 0x20;
    iAutoRadioParams.tDynRadioParams[1].TxPerChannelPowerLimits_2_4G_OFDM[10] = 0x20;  
    memset(&iAutoRadioParams.tDynRadioParams[1].TxPDVsRateOffsets_2_4G[0],0,NUMBER_OF_RATE_GROUPS_E); 
     for (int i = 0 ;i <NUMBER_OF_RATE_GROUPS_E;i++) 
    {
        iAutoRadioParams.tDynRadioParams[1].TxIbiasTable_2_4G[i] = 0x11;
    }
  
    iAutoRadioParams.tDynRadioParams[1].TxIbiasTable_2_4G[5] = 0x12; 													
    iAutoRadioParams.tDynRadioParams[1].RxFemInsertionLoss_2_4G = 0x12;  
   }
}

void TIWha::FillSmartReflexData()
{
    iSmartReflexParams.SmartReflexState = 0;

    iSmartReflexParams.tErrorTable[0].len = 7;
    iSmartReflexParams.tErrorTable[0].upperLimit = 3;
    iSmartReflexParams.tErrorTable[0].values[0] = 24;
    iSmartReflexParams.tErrorTable[0].values[1] = 16;
    iSmartReflexParams.tErrorTable[0].values[2] = 5;
    iSmartReflexParams.tErrorTable[0].values[3] = -5;
    iSmartReflexParams.tErrorTable[0].values[4] = -16;
    iSmartReflexParams.tErrorTable[0].values[5] = -24;
    iSmartReflexParams.tErrorTable[1].len = 7;
    iSmartReflexParams.tErrorTable[1].upperLimit = 3;
    iSmartReflexParams.tErrorTable[1].values[0] = 24;
    iSmartReflexParams.tErrorTable[1].values[1] = 16;
    iSmartReflexParams.tErrorTable[1].values[2] = 5;
    iSmartReflexParams.tErrorTable[1].values[3] = -5;
    iSmartReflexParams.tErrorTable[1].values[4] = -16;
    iSmartReflexParams.tErrorTable[1].values[5] = -24;
    iSmartReflexParams.tErrorTable[2].len = 7;
    iSmartReflexParams.tErrorTable[2].upperLimit = 3;   
    iSmartReflexParams.tErrorTable[2].values[0] = 24;
    iSmartReflexParams.tErrorTable[2].values[1] = 16;
    iSmartReflexParams.tErrorTable[2].values[2] = 5;
    iSmartReflexParams.tErrorTable[2].values[3] = -5;
    iSmartReflexParams.tErrorTable[2].values[4] = -16;
    iSmartReflexParams.tErrorTable[2].values[5] = -24;
}

#if TI_DBG
/****************************************************************************
 *                     PrintRadioData
 ****************************************************************************
 * DESCRIPTION: print Radio params , used for debug
 * 
 * INPUTS:   
 * 
 * OUTPUT:  radio params
 * 
 * RETURNS: OK
 ****************************************************************************/
void TIWha::PrintRadioData()
{
    TI_UINT32 i,j;
    TI_UINT8* pRadio;

    pRadio = reinterpret_cast<TI_UINT8*>(&iAutoRadioParams); 

    i=0;
	os_printf("/**********************/ \n");
	os_printf("/* INI General params */ \n");
	os_printf("/**********************/ \n");

	os_printf("0x%x, /* RefClk */ \n",						pRadio[i++]);
	os_printf("0x%x, /* SettlingTime */ \n",				pRadio[i++]);
	os_printf("0x%x, /* ClockValidOnWakeup */ \n",			pRadio[i++]);
	os_printf("0x%x, /* DC2DCMode */ \n",					pRadio[i++]);
	os_printf("0x%x, /* Single_Dual_Band_Solution */ \n",	pRadio[i++]);
	os_printf("0x%x, /* TXBiPFEMAutoDetect */ \n",			pRadio[i++]);
	os_printf("0x%x, /* TXBiPFEMManufacturer */ \n",		pRadio[i++]);
	os_printf("0x%x, /* Padding */ \n",					pRadio[i++]);
	
	os_printf("/*************************/ \n");
	os_printf("/* INI Stat Radio params */ \n");
	os_printf("/*************************/ \n");
	
	os_printf("0x%x, /* RxTraceInsertionLoss_2_4G */ \n",	pRadio[i++]);
	os_printf("0x%x, /* TXTraceLoss_2_4G */ \n",			pRadio[i++]);
	for (j=0;j<15;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* RxRssiAndProcessCompensation_2_4G[15] */ \n");
	for (j=0;j<7;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* RxTraceInsertionLoss_5G[7] */ \n");
	for (j=0;j<7;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TXTraceLoss_5G[7] */ \n");
	for (j=0;j<15;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* RxRssiAndProcessCompensation_5G[15] */ \n");

	os_printf("/*******************************/ \n");
	os_printf("/* Dyn Radio Params. First FEM */ \n");
	os_printf("/*******************************/ \n");
	
	os_printf("0x%x, ", pRadio[i++]);
	os_printf("0x%x, /* TXBiPReferencePDvoltage_2_4G */ \n", pRadio[i++]);
	os_printf("0x%x, /* TxBiPReferencePower_2_4G */ \n",	pRadio[i++]);
	os_printf("0x%x, /* TxBiPOffsetdB_2_4G */ \n",			pRadio[i++]);
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPerRatePowerLimits_2_4G_Normal[6] */ \n");
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPerRatePowerLimits_2_4G_Degraded[6] */ \n");
	for (j=0;j<14;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPerChannelPowerLimits_2_4G_11b[14] */ \n");
	for (j=0;j<14;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPerChannelPowerLimits_2_4G_OFDM[14] */ \n");
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPDVsRateOffsets_2_4G[6] */ \n");
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxIbiasTable_2_4G[6] */ \n");
	os_printf("0x%x, /* RxFemInsertionLoss_2_4G */ \n",			pRadio[i++]);
	for (j=0;j<35;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPerChannelPowerLimits_5G_OFDM[35] */ \n");
	for (j=0;j<14;j++)
		os_printf("0x%x, ", pRadio[i++]);		
	os_printf("/* TXBiPReferencePDvoltage_5G[7*2] */ \n");
	for (j=0;j<7;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxBiPReferencePower_5G[7] */ \n");
	for (j=0;j<7;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxBiPOffsetdB_5G[7] */ \n");
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPerRatePowerLimits_5G_Normal[6] */ \n");
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPerRatePowerLimits_5G_Degraded[6] */ \n");	
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPDVsRateOffsets_5G[6] */ \n");
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxIbiasTable_5G[6] */ \n");
	for (j=0;j<7;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* RxFemInsertionLoss_5G[7] */ \n");
	os_printf("0x%x, /* Padding */ \n",					pRadio[i++]);

	os_printf("/************************************/ \n");
	os_printf("/* INI Dyn Radio Params. Second FEM */ \n");
	os_printf("/************************************/ \n");
	
	os_printf("0x%x, ", pRadio[i++]);	
	os_printf("0x%x, /* TXBiPReferencePDvoltage_2_4G */ \n", pRadio[i++]);	
	os_printf("0x%x, /* TxBiPReferencePower_2_4G */ \n",	pRadio[i++]);
	os_printf("0x%x, /* TxBiPOffsetdB_2_4G */ \n",			pRadio[i++]);
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPerRatePowerLimits_2_4G_Normal[6] */ \n");
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPerRatePowerLimits_2_4G_Degraded[6] */ \n");
	for (j=0;j<14;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPerChannelPowerLimits_2_4G_11b[14] */ \n");
	for (j=0;j<14;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPerChannelPowerLimits_2_4G_OFDM[14] */ \n");
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPDVsRateOffsets_2_4G[6] */ \n");
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxIbiasTable_2_4G[6] */ \n");
	os_printf("0x%x, /* RxFemInsertionLoss_2_4G */ \n",			pRadio[i++]);
	for (j=0;j<14;j++)
	{
		os_printf("0x%x, ", pRadio[i++]);		
	}
	os_printf("/* TXBiPReferencePDvoltage_5G[7*2] */ \n");
	for (j=0;j<7;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxBiPReferencePower_5G[7] */ \n");
	for (j=0;j<7;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxBiPOffsetdB_5G[7] */ \n");
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPerRatePowerLimits_5G_Normal[6] */ \n");
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPerRatePowerLimits_5G_Degraded[6] */ \n");
	for (j=0;j<35;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPerChannelPowerLimits_5G_OFDM[35] */ \n");
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxPDVsRateOffsets_5G[6] */ \n");
	for (j=0;j<6;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* TxIbiasTable_5G[6] */ \n");
	for (j=0;j<7;j++)
		os_printf("0x%x, ", pRadio[i++]);
	os_printf("/* RxFemInsertionLoss_5G[7] */ \n");
	os_printf("0x%x /* Padding */ \n",					pRadio[i++]);
}
#endif


/** 
* \fn     TIFailureDfcClient
* \brief  Constructor
* 
* This method is the default constructor for TIFailureDfcClient. 
* This class is in charge of creating a new DFC that will handle
* the failure handling & dispatch to the umac in case of a failure event.
* 
* \note   
* return   handle to TIFailureDfcClient class. 
* \sa     
*/ 
TIFailureDfcClient::TIFailureDfcClient(MWlanOsa& aOsa)
{
    pFailureDfc = aOsa.DfcCreate();
}


/**
* \fn     TIFailureDfcClient::OnDfc
* \brief  Call TIWhaAdaptCB::FailureIndicationDFC from TIFailureDfcClient context
* 
* \param handle to TIWha context
* 
* /note
* 
* /return void
*/
void TIFailureDfcClient::OnDfc(TInt aCtx)
{
    TIWhaAdaptCB::FailureIndicationDFC ((TI_HANDLE)aCtx);
}


/** 
* \fn     TIConnectDfcClient
* \brief  Constructor
* 
* This method is the default constructor for TIConnectDfcClient. 
* This class is in charge of creating a new DFC that will handle
* the end of the bus connection phase in case of a sync init bus transaction .
* 
* \note   
* return   handle to TIConnectDfcClient class. 
* \sa     
*/ 
TIConnectDfcClient::TIConnectDfcClient(MWlanOsa& aOsa)
{
    pConnectDfc = aOsa.DfcCreate();
}


/**
* \fn     TIConnectDfcClient::OnDfc
* \brief  Call TIWhaAdaptCB::ConnectBus from TIConnectDfcClient context
* 
* \param handle to TIWha context
* 
* /note
* 
* /return void
*/
void TIConnectDfcClient::OnDfc(TInt aCtx)
{    
    TIWhaAdaptCB::ConnectBus ((TI_HANDLE)aCtx);
}



/*****************************************************************************************************/
/************************************** CREATE DRIVER ************************************************/


DECLARE_STANDARD_PDD()
{
    TIWha* p = new TIWha;
    return p;
}


 TInt TIWha::Install()
 {
    TPtrC mDeviceName= DEVICE_NAME;

	return SetName(&mDeviceName);
 }

 void TIWha::GetCaps( TDes8& aDes ) const
 {
     aDes.FillZ(aDes.MaxLength());

     aDes.Copy( (TUint8*)&iVersion, Min(aDes.MaxLength(), sizeof(TVersion)) );
 }



 TInt TIWha::Create( DBase *&aChannel, TInt aUnit, const TDesC* anInfo, const TVersion& aVer)
 {
     if ( !Kern::QueryVersionSupported( iVersion, aVer ) ) 
        {
         return KErrNotSupported;
        }
     if (bDriverCreated) {
         aChannel = (MWlanPddIface*)this;
         return KErrNone;
     }
     else
     {
         return KErrNotReady;
     }
 }


 TInt TIWha::Validate( TInt aUnit, const TDesC8 *aInfo, const TVersion &aVer )
 {
     return KErrNone;
 }

 TBool TIWha::Attach( MWlanOsa& aWlanOsa, MWlanOsaExt& aWlanOsaExt )
 {
     iTwdCtrl.tOsContext.hOsa = &aWlanOsa;

     /* Create HPA */
     iTwdCtrl.tOsContext.hHpa = WlanHpa::Create( aWlanOsaExt, aWlanOsa);

     /* Create SPIA */
     iTwdCtrl.tOsContext.hSpia = WlanSpia::Create( aWlanOsaExt );

     if (iTwdCtrl.tOsContext.hHpa == NULL || iTwdCtrl.tOsContext.hSpia == NULL) {
         WLAN_OS_REPORT (("Failed to create Driver layers -- iHpa = 0x%x, , iSpia = 0x%x",iTwdCtrl.tOsContext.hHpa,iTwdCtrl.tOsContext.hSpia));
         return EFalse;
     }


     pFailureDfcClient = new TIFailureDfcClient(*iTwdCtrl.tOsContext.hOsa);
     if (pFailureDfcClient == NULL)
     {
         WLAN_OS_REPORT (("ERROR: CreateDriver TIFailureDfcClient failure\n"));        
     }

     pConnectDfcClient = new TIConnectDfcClient(*iTwdCtrl.tOsContext.hOsa);
     if (pConnectDfcClient == NULL)
     {
         WLAN_OS_REPORT (("ERROR: CreateDriver TIConnectDfcClient failure\n"));        
     }

     return ETrue;
 }


 WHA::Wha& TIWha::Extract()
 {
     return *this;
 }


 void TIWha::GetCapabilities( SCapabilities*& aCapabilities )
 {
    aCapability = (SCapabilities*)Kern::Alloc(sizeof(SCapabilities));
    aCapability->iCapabilities = SCapabilities::KCachedMemory;
    aCapability->iCacheLineLength = 32;

    aCapabilities = aCapability;
 }
