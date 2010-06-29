/*
 * TIWhaCb.cpp
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


/** \file TIWhaCB.c
 *  \brief TIWha Adaptation CB implementation
 *
 */


#include "TIWha.h"
#include "TIWhaAdaptCb.h"
#ifdef FPGA1273_STAGE_
#warning FPGA
#include "fw1273_fpga.h"
#endif
extern "C"
{
#include "timer.h"
#include "report.h"
#include "TWDriver.h"
#include "WlanDrvCommon.h"
#include "version.h"
#define __FILE_ID__								FILE_ID_146
}

 
/** 
 * \fn     prepareNextFwChunk
 * \brief  called for each Fw chunk. last chunk is marked with bLast
 * 
 * \note    
 * \return  
 * \sa      
 */ 
void TIWha:: prepareNextFwChunk ()
{
    /* Move to the next chunk (if this is the first chunk then uLength == 0)*/
    iFwFile.pBuffer 	+= iFwFile.uLength; 

    /* read Chunk's address */
    iFwFile.uAddress = BYTE_SWAP_LONG( *((TI_UINT32*)(iFwFile.pBuffer)) );
    iFwFile.pBuffer += DRV_ADDRESS_SIZE;
    /* read Portion's length */
    iFwFile.uLength = BYTE_SWAP_LONG( *((TI_UINT32*)(iFwFile.pBuffer)) );
    iFwFile.pBuffer += DRV_ADDRESS_SIZE;

    /* decrease number of chunks left, and check if this is the last one */
    if ( --iFwFile.uChunksLeft == 0 )  
    {
        /* Last chunk. Next time we should continue with our own sequence */
    	iFwFile.bLast = TI_TRUE;
    }     

    
    /*TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INIT,  TIWLANWHA_CB_MODULE_LOG, MSG_2195 , iFwFile.bLast, iFwFile.uChunksLeft, iFwFile.uLength, iFwFile.uAddress);*/
}


/* Thisis called whentxnQ_ConnectBus has finished */
void TIWha:: ConnectBusCb ( TI_STATUS status)
{ 
    /* This will download the FW image into part with DMA of 512 bytes each time */
    TWD_Init (iTwdCtrl.hTWD,
    	  iTwdCtrl.hReport,
    	  this,
    	  iTwdCtrl.hTimer,
    	  iTwdCtrl.hContext,
    	  iTwdCtrl.hTxnQ,
    	  (TTwdCallback)TIWhaAdaptCB::InitHw,
    	  (TTwdCallback)TIWhaAdaptCB::InitFw,
    	  (TTwdCallback)TIWhaAdaptCB::ConfigFw,
    	  NULL,//(TTwdCallback)drvMain_TwdStopCb,
    	  (TTwdCallback)TIWhaAdaptCB::InitFail);

    /* Set the default ELP mode to be in awake mode */
    TWD_CfgSleepAuth (iTwdCtrl.hTWD, POWERAUTHO_POLICY_AWAKE);

    
    /*TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INIT,  TIWLANWHA_MODULE_LOG, MSG_2196, "data = 0x%x len = %d\n",aData,aLength);*/

    /* Set Report default parameters in to the logger internal DB */
    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INIT, "Set Defaults");
     
    /* Setting the defaults values for the TWD and the Report modules
    note that all parameters are hardcoded and not from a configuration file */
    InitTwdParamTable ();
    InitTwdPlatformGenParam();
    
    /*  FEM Auto Detection Configure Radio Params only if Manual mode selected*/
    if (iAutoRadioParams.tGeneralParams.TXBiPFEMAutoDetect == FEM_MANUAL_DETECT_MODE_E)
    {     
      /* fill TWD init parms with relevant (RFMD or TriQuint) Radio parms before calling TWD_SetDefault*/
      InitTwdRadioParam();
    }

    
    /* Fill the default SmartReflex params */
    InitTwdRadioSmartReflexParam();

    /* Initializing the SG profile and params */
    InitBtCoex();

    TWD_SetDefaults (iTwdCtrl.hTWD, &(iTwdCtrl.twdInitParams));

    
    TWD_SetRateMngDebug(iTwdCtrl.hTWD, &(iTwdCtrl.twdInitParams.tRateMngParams));

    /* Set the default burst mode to false */
    TWD_CfgBurstMode (iTwdCtrl.hTWD, BURST_MODE_ENABLE_DEF);

    /* Set the SG params to the wlan internal DB */
    /* BTCOEX_DEFAULT_MODE is set to SG_OPPORTUNISTIC */
    SoftGemini_SetParams(iTwdCtrl.hTWD,&iSgParams,BTCOEX_DEFAULT_MODE);


    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INIT, "Register CB/Events");
    
    RegisterCb ();
    RegisterEvents ();

  /* sending the NVS buffer - it can be default or real nvs*/
    if ( TWD_InitHw (iTwdCtrl.hTWD,ipNVSbuf,iNVSlength) != TI_OK)
    {
        TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_FATAL_ERROR, "\n.....TWD_InitHw failure \n");
    }
}


/** 
 * \fn     InitHwCb
 * \brief  called after  init was done
 * 
 * kick the next stage of itit after all TWD modules wew initialaized. 
 * 
 * \note    
 * \param  hTIWha -  Handle to the TIWha object
 * \param  aStatus - status
 * \return  
 * \sa      
 */ 
void TIWha:: InitHwCb ( TI_STATUS status)
{ 
    uint8 FemType;
  
    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INIT, "\n");

   /*  FEM Auto Detection Configure Radio Params only if Manual mode selected*/
   if (iAutoRadioParams.tGeneralParams.TXBiPFEMAutoDetect == FEM_MANUAL_DETECT_MODE_E)
   {       
       FemType = TWD_GetFEMType(iTwdCtrl.hTWD);
    
       if (FemType != iAutoRadioParams.tGeneralParams.TXBiPFEMManufacturer)
       {
           WLAN_OS_REPORT(("TIWha:: InitHwCb  ERROR Manual FEM is %d  real FEM is %d \n",iAutoRadioParams.tGeneralParams.TXBiPFEMManufacturer,FemType));
       }       
   }
   
    /* update Radio params due to FEM type*/
    if (iAutoRadioParams.tGeneralParams.TXBiPFEMAutoDetect == FEM_AUTO_DETECT_MODE_E)
    {
     /* fill TWD init parms with relevant (RFMD or TriQuint) Radio parms before calling TWD_SetDefault*/
     InitTwdRadioParam();
      /* FEM Auto Detect, calling agian TWD_SetDefault, to store real Radio params in TWD DB due to FEM detection */
     TWD_SetDefaults (iTwdCtrl.hTWD, &(iTwdCtrl.twdInitParams));

    }
     
    /* 
     * prepare Fw file. 
     */
     
    /* First chunk points to begining of Fw */
    /*  This was retrieved in Iniatialize iFwFile.pBuffer = (TI_UINT8*)firmware;*/
    /* length of chunk will be updated later in prepareNextFwChunk() */
    iFwFile.uLength	= 0;
    /* First 4 bytes indicate the number of chunks */
    iFwFile.uChunksLeft = BYTE_SWAP_LONG( *((TI_UINT32*)(iFwFile.pBuffer)) );
    iFwFile.pBuffer += DRV_ADDRESS_SIZE;

    /* Mark if we are in the last chunk (i.e. only one chunk) */
    iFwFile.bLast = ( iFwFile.uChunksLeft == 1 ? TI_TRUE : TI_FALSE);

    /* Prepare next chunk for initFw */
    prepareNextFwChunk ();

    /* call TWD with first chunk */        
    TWD_InitFw ( iTwdCtrl.hTWD, &iFwFile);
}


/** 
 * \fn     InitFwCb
 * \brief  after FW download
 * 
 * giv back the context to the UMAC for configuration
 * 
 * \note    
 * \param  hTIWha -  Handle to the TIWha object
 * \param  aStatus - status
 * \return  
 * \sa      
 */ 
void TIWha:: InitFwCb (  TI_STATUS status)
{ 
    TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INIT, " last = %d ChunksLeft = %d\n", iFwFile.bLast, iFwFile.uChunksLeft);

    if (status != TI_OK )
    {
        TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, " status = %d \n", status);
    }
    
    /* check if another rounf of FW download is needed */
    if ( iFwFile.bLast == TI_FALSE )
    {
        /* download another chunk */
        prepareNextFwChunk ();
        TWD_InitFw ( iTwdCtrl.hTWD, &iFwFile);
    }
    else
    {
        /* Call Init Response with the given status */
        InitResponse ((WHA::TStatus)status);
    }
    /* next step will be configure from UMAC */       
}

/** 
 * \fn     ConfigFwCb
 * \brief  after FW configured
 * 
 * after FW configured using the set defauls stage.
 * 
 * \note    
 * \param  hTIWha -  Handle to the TIWha object
 * \param  aStatus - status
 * \return  
 * \sa      
 */ 
void TIWha:: ConfigFwCb (  TI_STATUS status)
{
 
    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INIT, " status = %d\n",status);

    if (status == TI_OK)
    {        
        /* Register the command complete generic callback. From now on we will get response for every command */
        TWD_RegisterCb (iTwdCtrl.hTWD, TWD_EVENT_COMMAND_COMPLETE, 
                                                (TTwdCB *)TIWhaAdaptCB::GenericCommandResponse, this);

        /* Enable external events from FW */
        TWD_EnableExternalEvents (iTwdCtrl.hTWD); 

        /* At this stage now send back the Command Response of the Configure stage to the UMAC */
        if( bErrorIndication == TI_FALSE)
        {    
            WhaCb()->CommandResponse(WHA::EConfigureResponse, WHA::KSuccess, iUCommandResponseParams);		
        }
        #if TI_DBG
            else
            {
                WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
            }
        #endif

        /* Indicate that UMAC will send the Release() command by itself, since EConfigureResponse was sent */
        bCallRelease  = TI_FALSE;

    }
    else    /* Configure Failure - Send error indication since it is the only thing that will unload the driver */ 
    {
        TIWhaAdaptCB::FailureIndication (this, HW_AWAKE_FAILURE);
    }
}


/** 
 * \fn     InitFailCb
 * \brief  general fail in init/configure stages
 * 
 * \note    
 * \param  hTIWha -  Handle to the TIWha object
 * \param  aStatus - status
 * \return  
 * \sa      
 */ 
void TIWha::InitFailCb ( TI_STATUS status)
{     
    InitResponse ((WHA::TStatus)WHA::KFailed);
}

/** 
 * \fn     ConnectionTimeOut
 * \brief  change rate management parameters after the connection phase
 * 
 * \note    
 * \return  
 * \sa      
 */ 
void TIWha::ConnectionTimeOut ()
{
    TRACE1( iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "%s\n",__FUNCTION__);

    bConnectionTimerRunning = TI_FALSE;
    SetTxFailLowThreshold(TIWHA_TX_FAIL_LOW_TH_AFTER_CONNECTION);
}

/** 
 * \fn     SetTxFailLowThreshold
 * \brief  Set new Tx Fail rate to FW
 * 
 * \note    
 * \return  
 * \sa      
 */    
void TIWha::SetTxFailLowThreshold( TI_UINT8 uTxFailLowTh)
{
    RateMangeParams_t tRM;

    TRACE2( iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "%s Th = %d\n",__FUNCTION__,uTxFailLowTh);
    /* Set the Tx fail Threshold again to the FW */
    tRM.paramIndex = RATE_MGMT_TX_FAIL_LOW_TH_PARAM;
    tRM.TxFailLowTh = uTxFailLowTh;
    /* CommandResponse would be ignored for this call (for ACX_SET_RATE_MAMAGEMENT_PARAMS) */
    TWD_SetRateMngDebug(iTwdCtrl.hTWD, &tRM);
}
/** 
 * \fn     FailureIndication
 * \brief  fail CB for the driver
 * 
 * propogats faliure indication
 * \note    
 * \param  failureEvent - failure Event
 * \return  
 * \sa      
 */ 
void TIWha::FailureIndicationCb ( EFailureEvent failureEvent)
{   
    bFailureIndication = TI_TRUE;

    /* 
     * Check if we should call Release() directly since UMAC calls release only after 
     * successful EConfigureResponse 
     */
    if (bCallRelease)
    {	

		/* in case we are before configure complete save the failure event to send it later */
		ifailureEvent = failureEvent;

        iConnectionCounter++;
        WLAN_OS_REPORT(("POR identified - Reconnecting #%d \n", iConnectionCounter));

        /* if we have consecutive failures then add delayes till next reconnect to increase success probability */
        if (iConnectionCounter > 2) 
        {
            /* if we fail more than 5 times notify the upper layer on failure to initailze */
            if (iConnectionCounter > 5) 
            {

                bCallRelease = TI_FALSE;
                 /* If we recognize failure in init time then release the driver and try to intialize again */
                 Release(TRUE);

                 WLAN_OS_REPORT((" ************** FailureIndicationCb == %d ****************** \n",ifailureEvent));

                 /* call to LDD with failure indication */                                      
                 if( bErrorIndication == TI_FALSE)
                 {    
                     WhaCb()->Indication (WHA::EError, iUIndicationParams);
                 }
                 #if TI_DBG
                     else
                     {
                        WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
                     }
                 #endif
                
                return;
            }
            /* 2 < iConnectionCounter <= 5 */
            else
            {
                /* If we recognize failure in init time then release the driver and try to intialize again */
                Release(TRUE);                
                /* If we have more than 2 conssecutive failures wait 400ms before turning on the device */
                os_StalluSec ((TI_HANDLE)&iTwdCtrl.tOsContext, STALL_ON_FAILURE );    
            }
        }
        /* iConnectionCounter <= 2 */
        else        
        {
            /* If we recognize failure in init time then release the driver and try to intialize again */
            Release(TRUE);
            /* Wait 1ms before turning on the device */
            os_StalluSec ((TI_HANDLE)&iTwdCtrl.tOsContext, STALL_ON_RECONNECT );
        }
        Initialize(iData, iLength);

        bFailureIndication = TI_FALSE;

        return;

	}
     

    TRACE1( iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, " ************** FailureIndicationCb == %d ****************** \n",failureEvent);

    #ifndef NO_ERROR_INDICATION
        /* Always return MacNotResponding in case of an EEROR */
        iUIndicationParams.iError.iStatus = WHA::KErrorMacNotResponding; 

        if( bErrorIndication == TI_FALSE)
        {    
            WhaCb()->Indication (WHA::EError, iUIndicationParams);
        }
        #if TI_DBG
            else
            {
                WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
            }
        #endif

    #endif

    /* Mark that from now on there will be no calls to LDD (this is done with a dummy WhaCb() ) */
    bErrorIndication = TI_TRUE;
}

/** 
 * \fn     RcpiIndication
 * \brief  rcpi CB for the driver
 * 
 * propogats rcpi indication
 * \note    
 * \param  buffer - RCPI value
 * \return  
 * \sa      
 */ 
void TIWha::RcpiIndicationCb ( TUint8* buffer, TUint32 len)
{
    TRACE0( iTwdCtrl.hReport, REPORT_SEVERITY_WARNING, " *********** RcpiIndication  ****************** \n");
    
	iUIndicationParams.iRcpi.iRcpi= *((WHA::TRcpi*)buffer);	
    if( bErrorIndication == TI_FALSE)
    {    
        WhaCb()->Indication (WHA::ERcpi, iUIndicationParams);
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif
}


/** 
 * \fn     TIWha_ScanResponseCb
 * \brief  Scan complete CB call directly to the host driver command response 
 * 
 * Function detailed description goes here 
 * 
 * \note    
 * \param  aStatus - status
 * \return  
 * \sa      
 */ 
void TIWha::ScanResponseCb (WHA::TStatus aStatus)
{
    
    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " aStatus = %d\n", aStatus);

    if( bErrorIndication == TI_FALSE)
    {    
        (WhaCb())->CommandResponse(WHA::EScanCommandResponse,  WHA::KSuccess, iUCommandResponseParams);
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif
}

/** 
 * \fn     ScanCompleteCb
 * \brief  Scan complete CB. 
 * 
 * \note    
 * \param  PSMode - not used
 * \param  returnStatus - status
 * \return  
 * \sa      
 */ 
void TIWha::ScanCompleteCb (TI_STATUS returnStatus , E80211PsStatus PSMode)
{
	WHA::TStatus status = ( returnStatus == TI_OK ) ? WHA::KSuccess : WHA::KFailed;

   /* return PS state by using PowerSaveSrv in TWD */
    iUCommandCompletionParams.iScanComplete.iDot11PowerManagementMode = TWD_GetPsStatus (iTwdCtrl.hTWD) ? WHA::KPsEnable : WHA::KPsDisable;

    TRACE2(iTwdCtrl.hTWD, REPORT_SEVERITY_INFORMATION , " - status = %d iDot11PowerManagementMode = %d\n", status, iUCommandCompletionParams.iScanComplete.iDot11PowerManagementMode );

    if( bErrorIndication == TI_FALSE)
    {    
        WhaCb()->CommandComplete(WHA::EScanComplete, status, iUCommandCompletionParams);
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif

	/* Set back the ReceivePacket CB to norman path */
    TWD_RegisterCb (iTwdCtrl.hTWD,
                    TWD_EVENT_RX_RECEIVE_PACKET, 
                    (TTwdCB *)TIWhaAdaptCB::ReceivePacket, 
                    this);
}


/** 
 * \fn     GenericCommandResponseCb
 * \brief  Whenever we don't have a specific callback we get into here
 *
 * \param           CmdType - Command Type
 * \param            CmdID   - ACX ID
 * \param            aStatus - Success or Failure    
 * \return
 * \sa     
 */
void TIWha::GenericCommandResponseCb ( TUint16 CmdType, TUint16 CmdID, TUint32 aStatus)
{
    WHA::TCommandId commandId = WHA::EWriteMIBResponse; /* By default we get here on WriteMib response */

    TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "GenericCommandResponseCb Command = 0x%x, ACX = 0x%x, status = %d\n",        CmdType, CmdID, aStatus);

    switch (CmdType)
    {
	/* Do not send Command Response in case of Ps Mode ==> this may happen when doing Scan in ForceBackgroundScan Type 
	that sets the PS to be entered and then a Command Response of Entering PS mode is received via this channel although the Scan command has been issued */
	case CMD_SET_PS_MODE:
		return;

    case CMD_CONFIGURE:
        switch (CmdID)
        {
			case ACX_TID_CFG:
			    commandId = WHA::EConfigureQueueResponse;
			    break;
			case ACX_AC_CFG:
			    commandId = WHA::EConfigureACResponse;
			    break;
			case ACX_AID:
			    commandId = WHA::ESetBssParametersResponse;
			    break;
			    
			case ACX_SLEEP_AUTH:
			    if ( ! iTwdCtrl.bResponse )
			    {   /* In this case we don't send response since it is an internal command */
			       return;
			    }
			    break;
#ifdef HT_SUPPORT
			case ACX_BA_SESSION_RESPONDER_POLICY:
			case ACX_BA_SESSION_INITIATOR_POLICY:
				iBACounertRespone--;
				if (iBACounertRespone > 0)
				{
					return;
				}
				commandId = WHA::EWriteMIBResponse;
				break;
#endif /* HT_SUPPORT */
			    
			/* In this case it is an internal MIB so do not send response to upper layer */
			case ACX_PREAMBLE_TYPE:
			    return;
            /* Internal command before join and after full connection */
            case ACX_SET_RATE_MAMAGEMENT_PARAMS:
                return;
        }
			    break;
    
		    case CMD_STOP_SCAN:
		        /* The regular StopScan command always goes through StopScanResponseCb(). This is a special
		         * error case where ScanSrv decided to send StopScan. Therefore we shouldn't forward the indication
		         * to UMAC 
		         */
		        return;
		        
		    case CMD_START_JOIN:
		        /* 
		         * In TWD we have 'join response' and 'join complete' while Symbian defines only 'join response'.
		         * Hence, here we ignore the response, and the join complete returns to Symbian as EJoinResponse
		         */
		        return;

		    case CMD_DISCONNECT:
		        commandId = WHA::EResetResponse;
		        break;

            case CMD_SET_STA_STATE:
				/* This is an internal command sent by WHA at Set AID method.
				   This is used to indicate the FW that the STA is connected for COEX and 11N BA */
                return;

	        default:
	            break;
        }

    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "GenericCommandResponseCb sending response to UMAC status = %d\n",aStatus);

    if( bErrorIndication == TI_FALSE)
    {    
        WhaCb()->CommandResponse(commandId, aStatus, iUCommandResponseParams);		
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif
}

/**
* \fn     InitResponse
* \brief ititializatin response function. In case of success - return command response
*								In case of error - send error indication (as requested from Palau)
*
* /note
* 
* /param aTwdInitStatus - Whether Init was successful
*   
* /return 
*/
void TIWha::InitResponse (WHA::TStatus aTwdInitStatus)
{       
    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INIT, " status = %d\n",aTwdInitStatus);
    
    if (aTwdInitStatus == WHA::KSuccess)
    {
            if( bErrorIndication == TI_FALSE)
            {    
                WhaCb()->CommandResponse(WHA::EInitializeResponse, WHA::KSuccess, iUCommandResponseParams);
            }
            #if TI_DBG
                else
                {
                    WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
                }
            #endif
            
    }
    else    /* Init Failure - Send error indication since it is the only thing that will unload the driver */ 
    {
        /* Call UMAC with error indication */        
        TIWhaAdaptCB::FailureIndication (this, HW_AWAKE_FAILURE);
    }
}


/****************************************************************************************
 *                     MeasureComplete()                                   *
 ****************************************************************************************
DESCRIPTION:    

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::MeasureCompleteCb ( TMeasurementReply* msrReply)
{
    return;
}


/****************************************************************************************
 *                     MeasureResponse()                                   *
 ****************************************************************************************
DESCRIPTION:    

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::MeasureResponseCb ( TUint16 aStatus)
{
    return;
}


/****************************************************************************************
 *                       StopMeasureResponse()                             *
 ****************************************************************************************
DESCRIPTION:    

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::StopMeasureResponseCb ( TUint16 aStatus)
{
    return;
}



/**
* \fn     SetPsModeResponseCb
*
* 
* /param aStatus 
*   
* /return 
*/
void TIWha::SetPsModeResponseCb (TUint8 aStatus)
{
	WHA::TStatus status = ( aStatus == TI_OK ) ? WHA::KSuccess : WHA::KFailed;

    TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION , " - aStatus: = %d status = %d\n", aStatus, status);

    if( bErrorIndication == TI_FALSE)
    {    
        WhaCb()->CommandResponse (WHA::ESetPsModeCommandResponse, status, iUCommandResponseParams);		
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif	
}


/**
* \fn     SetPsModeCompleteCb
*
* 
* /param aStatus 
*   
* /return 
*/
void TIWha::SetPsModeCompleteCb (TUint8 PSMode, TUint8 transStatus)
{
    /* By default the request was successful */
    WHA::TStatus        status = WHA::KSuccess;
    /* By default we are not in 802.11 ps mode */
    WHA::TPsMode psMode = WHA::KPsDisable;
    
    switch (transStatus)
    {
    case ENTER_POWER_SAVE_FAIL :  
        TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR , " - status: Enter PowerSaveFailure\n");

        status = WHA::KFailed;
        break;

    case EXIT_POWER_SAVE_FAIL :  
        TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR , " - status: Exit PowerSaveFailure (but we our awake anyway)\n");
        /* PS State is awake because even if we failed - we are not in PS - but the return is Failed */
        status = WHA::KFailed;
        break;

    case ENTER_POWER_SAVE_SUCCESS :  
        TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION , " - status: Enter PowerSave Success\n");
        psMode = WHA::KPsEnable;
        break;

    case EXIT_POWER_SAVE_SUCCESS :  
        TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION , " - status: Exit PowerSave Success\n");
        break;

    default :
        TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR , " unknown- status = %d: Failed\n", __FUNCTION__);
        status = WHA::KFailed;
    }

    /* 
     * On any case other then ENTER_POWER_SAVE_FAIL we change sleep mode
     * Note that on EXIT_POWER_SAVE_FAIL we are actually awake and therfore we change sleepMode
     */
    if ( transStatus != ENTER_POWER_SAVE_FAIL && transStatus != EXIT_POWER_SAVE_SUCCESS) 
    {
        /* Configure Awake if we are in PsDisable and user-defined sleepMode otherwise  */ 
        if ( psMode == WHA::KPsEnable )
        {
            /* Configure H/W to user sleep mode */
            SleepMode (iTwdCtrl.sleepMode, FALSE);
        }
        else 
        {
            /* Configure H/W to awake */
            SleepMode (WHA::KAwakeMode, FALSE);
        }
    }
    
    iUCommandCompletionParams.iSetPsModeComplete.iDot11PowerManagementMode = psMode;
    
    if( bErrorIndication == TI_FALSE)
    {    
        WhaCb()->CommandComplete(WHA::ESetPsModeComplete, status, iUCommandCompletionParams);
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif	
}


/****************************************************************************************
 *                        AddKeyResponse()                                 *
 ****************************************************************************************
DESCRIPTION:    This method is the adapt response to the AddKey command.

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::AddKeyResponseCb ( WHA::TStatus aStatus)
{
    WHA::TStatus status = ( aStatus == TI_OK ) ? WHA::KSuccess : WHA::KFailed;
    
    if( bErrorIndication == TI_FALSE)
    {    
        WhaCb()->CommandResponse (WHA::EAddKeyResponse, status, iUCommandResponseParams);		
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif	

    return;
}


/****************************************************************************************
 *                        RemoveKeyResponse()                              *
 ****************************************************************************************
DESCRIPTION:    This method is the adapt response to the RemoveKey command.

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::RemoveKeyResponseCb ( WHA::TStatus aStatus)
{
    WHA::TStatus status = ( aStatus == TI_OK ) ? WHA::KSuccess : WHA::KFailed;

    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "\n");

    if( bErrorIndication == TI_FALSE)
    {    
        WhaCb()->CommandResponse (WHA::ERemoveKeyResponse , status, iUCommandResponseParams);	
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif	

    return;
}


/** 
 * \fn     StopScanResponse
 * \brief  member function.
 * 
 * \note    
 * \param  aStatus - result of sending stop scan command to FW
 * \return  
 * \sa      
 */ 
void TIWha::StopScanResponseCb (WHA::TStatus aStatus)
{
    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION , " - status: %d\n", aStatus);

    if( bErrorIndication == TI_FALSE)
    {    
        WhaCb()->CommandResponse(WHA::EStopScanResponse, aStatus, iUCommandResponseParams);     
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif	
}

#ifdef PLT_TESTER
int state = 0;
TTestCmd test;
/** 
 * \fn     PltSm
 * \brief  debug function till we have full support for PLT.
 * 
 * \note    
 * \param  pBuf - contain PLT return buffer
 * \return  
 * \sa      
 */ 
void TIWha::PltSm(void *pBuf)
{
        TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION,  " state = %d ptr = %p plt = %d\n", state, pBuf, iTwdCtrl.ePlt);
	
	os_printf("*********** TIWha::PltSm state = %d ****************",state);


	switch (state)
	{

    case 0:
		{
           
            os_printf("*********** TEST_CMD_CHANNEL_TUNE  0 7 ****************");
            /* If we want to debug the plt sequence with no scan */
            /* state++; */
            state = 9; /* Start Tx Bip */
			test.testCmd_u.Channel.iChannel 	= 7;
			test.testCmd_u.Channel.iBand 		= 0;

			Plt ( TEST_CMD_CHANNEL_TUNE, (void *) &(test.testCmd_u));
			break;
		}

	case 1:
		{
            os_printf("*********** TEST_CMD_RX_STAT_RESET   ******************");
			state++;
			Plt ( TEST_CMD_RX_STAT_RESET, &(test.testCmd_u));
			break;
		}

    case 2:
		{
            os_printf("*********** TEST_CMD_RX_STAT_START   ******************");
			state++;
			Plt ( TEST_CMD_RX_STAT_START, &(test.testCmd_u));
			break;
		}
	case 3:
		{
            os_printf("*********** TEST_CMD_RX_STAT_GET     ******************");
			state++;
			os_memorySet(NULL , (void *) &test, 0,sizeof(test));
			Plt ( TEST_CMD_RX_STAT_GET, (void *) &(test.testCmd_u));
			break;
		}
    case 4:
		{
            os_printf("*********** TEST_CMD_RX_STAT_STOP    ******************");
			TTestCmd data;
			os_memoryCopy(NULL , (void *) &data.testCmd_u, 
				pBuf, sizeof(data.testCmd_u));

			WLAN_OS_REPORT(("Received Valid Packet no.: %d(0x%x)\n", data.testCmd_u.Statistics.oRxPathStatistics.ReceivedValidPacketsNumber,data.testCmd_u.Statistics.oRxPathStatistics.ReceivedValidPacketsNumber));
			WLAN_OS_REPORT(("Received FCS Error Packet no.: %d(0x%x)\n", data.testCmd_u.Statistics.oRxPathStatistics.ReceivedFcsErrorPacketsNumber,data.testCmd_u.Statistics.oRxPathStatistics.ReceivedFcsErrorPacketsNumber));
			WLAN_OS_REPORT(("Received Address missmatch Error Packet no.: %d(0x%x)\n", data.testCmd_u.Statistics.oRxPathStatistics.ReceivedPlcpErrorPacketsNumber,data.testCmd_u.Statistics.oRxPathStatistics.ReceivedPlcpErrorPacketsNumber));
			WLAN_OS_REPORT(("Sequence Number Missing Count: %d(0x%x)\n", data.testCmd_u.Statistics.oRxPathStatistics.SeqNumMissCount,data.testCmd_u.Statistics.oRxPathStatistics.SeqNumMissCount));
			WLAN_OS_REPORT(("Average SNR: %d(0x%x)\n", data.testCmd_u.Statistics.oRxPathStatistics.AverageSnr,data.testCmd_u.Statistics.oRxPathStatistics.AverageSnr));
			WLAN_OS_REPORT(("Average RSSI: %d(0x%x)\n", data.testCmd_u.Statistics.oRxPathStatistics.AverageRssi,data.testCmd_u.Statistics.oRxPathStatistics.AverageRssi));
			WLAN_OS_REPORT(("Base Packet ID: %d(0x%x)\n", data.testCmd_u.Statistics.oBasePacketId,data.testCmd_u.Statistics.oBasePacketId));
			WLAN_OS_REPORT(("Number of Packets: %d(0x%x)\n", data.testCmd_u.Statistics.ioNumberOfPackets,data.testCmd_u.Statistics.ioNumberOfPackets));
			WLAN_OS_REPORT(("Number of Missed Packets: %d(0x%x)\n", data.testCmd_u.Statistics.oNumberOfMissedPackets,data.testCmd_u.Statistics.oNumberOfMissedPackets));

			state++;
			Plt ( TEST_CMD_RX_STAT_STOP, pBuf);
			break;
		}
	case 5:
	   {
            os_printf("*********** TEST_CMD_FCC            ******************");
	 		state++;
            test.testCmdId = TEST_CMD_FCC;
			test.testCmd_u.TxPacketParams.iDelay = 5000;
			test.testCmd_u.TxPacketParams.iRate = 0x1;
			test.testCmd_u.TxPacketParams.iSize = 1000;
			test.testCmd_u.TxPacketParams.iAmount = 0;
			test.testCmd_u.TxPacketParams.iPower = 2000;
			test.testCmd_u.TxPacketParams.iSeed = 10000;
			test.testCmd_u.TxPacketParams.iPacketMode = 3;
			test.testCmd_u.TxPacketParams.iDcfOnOff = 0;
			test.testCmd_u.TxPacketParams.iGI = 204;
			test.testCmd_u.TxPacketParams.iPreamble = 0;
			test.testCmd_u.TxPacketParams.iType = 0;
			test.testCmd_u.TxPacketParams.iScrambler = 1;
			test.testCmd_u.TxPacketParams.iEnableCLPC = 204;
			test.testCmd_u.TxPacketParams.iSeqNumMode = 1;
			test.testCmd_u.TxPacketParams.iSrcMacAddr[0] = 0xDE;
			test.testCmd_u.TxPacketParams.iSrcMacAddr[1] = 0xAD;
			test.testCmd_u.TxPacketParams.iSrcMacAddr[2] = 0xBE;
			test.testCmd_u.TxPacketParams.iSrcMacAddr[3] = 0xFF;
			test.testCmd_u.TxPacketParams.iSrcMacAddr[4] = 0;
			test.testCmd_u.TxPacketParams.iSrcMacAddr[5] = 0;
			for (int i = 0 ; i < 6 ; i ++)
			{
				test.testCmd_u.TxPacketParams.iDstMacAddr[i] = 0x5A;
			}

			os_printf("Pad = 0x%x",&(test.testCmd_u.TxPacketParams.Pad));
			os_printf("iDelay = 0x%x",&(test.testCmd_u.TxPacketParams.iDelay));
			os_printf("iRate = 0x%x",&(test.testCmd_u.TxPacketParams.iRate));
			Plt( TEST_CMD_FCC, (void *) &(test.testCmd_u));
			break;
	}
    case 6:
		{
            os_printf("*********** TEST_CMD_STOP_TX         ******************");
			state++;
			Plt ( TEST_CMD_STOP_TX, pBuf);
          	break;
		}
	case 7:
		{
            os_printf("*********** TEST_CMD_TELEC           ******************");
			state++;
			test.testCmd_u.TxToneParams.iPower = 10000;

			Plt ( TEST_CMD_TELEC, (void *) &(test.testCmd_u));
			break;
		}
	case 8:
		{
            os_printf("*********** TEST_CMD_STOP_TX         ******************");
			state++;
			Plt ( TEST_CMD_STOP_TX, pBuf);
            
			break;
		}
	case 9:
		{
            os_printf("***********                  BIP                 ******************");
            os_printf("*********** TEST_CMD_UPDATE_PD_REFERENCE_POINT   ******************");
			state++;
			test.testCmd_u.PdBufferCalReferencePoint.iReferencePointDetectorValue = 375;
			test.testCmd_u.PdBufferCalReferencePoint.iReferencePointPower = 128; 
			test.testCmd_u.PdBufferCalReferencePoint.isubBand = 0;

			Plt ( TEST_CMD_UPDATE_PD_REFERENCE_POINT, (void *) &(test.testCmd_u));
			break;
		}
	case 10: 

		{
            os_printf("***********              TEST_CMD_P2G_CAL        ******************");
			state++;
			test.testCmd_u.P2GCal.iSubBandMask = 1;
			Plt ( TEST_CMD_P2G_CAL, (void *) &(test.testCmd_u));
			break;
		}
	case 11:
		{	
			state++;
			os_printf("**********               NVS                     *****************");

			/* Print the NVS as in the wilink6_nvs.h file */
			for (int i = 0; i < iNVSlength; i+=16)
			{
					os_printf("0x%02x ,0x%02x ,0x%02x ,0x%02x ,0x%02x ,0x%02x ,0x%02x ,0x%02x ,0x%02x ,0x%02x ,0x%02x ,0x%02x ,0x%02x ,0x%02x ,0x%02x ,0x%02x ,",
							  ((TUint8*)iBipNvsBuffer)[i+0],((TUint8*)iBipNvsBuffer)[i+1],((TUint8*)iBipNvsBuffer)[i+2],
							  ((TUint8*)iBipNvsBuffer)[i+3],((TUint8*)iBipNvsBuffer)[i+4],((TUint8*)iBipNvsBuffer)[i+5],
							  ((TUint8*)iBipNvsBuffer)[i+6],((TUint8*)iBipNvsBuffer)[i+7],((TUint8*)iBipNvsBuffer)[i+8],
							  ((TUint8*)iBipNvsBuffer)[i+9],((TUint8*)iBipNvsBuffer)[i+10],((TUint8*)iBipNvsBuffer)[i+11],
							  ((TUint8*)iBipNvsBuffer)[i+12],((TUint8*)iBipNvsBuffer)[i+13],((TUint8*)iBipNvsBuffer)[i+14],
							  ((TUint8*)iBipNvsBuffer)[i+15]);
			}
			
			//break;
		}

    case 12:

		{

		os_printf("**********          PltSm TEST END                *****************");

		state = 0;
		ScanResponseCb(WHA::KSuccess);
		ScanCompleteCb(TI_OK,POWER_SAVE_802_11_SUCCESS);
		return;
		}

	}
	
}
#endif /* PLT_TESTER */

/****************************************************************************************************/
/*  Function:		ConvertTxBip2Nvs																*/
/****************************************************************************************************/
/*  Description:	Recreate the NVS array from the result of the Tx BIP and from the				*/
/*                  NVS stored for MAC Address restoration	                                        */
/*                  iBipNvsBuffer - is used in order to keep the TxBiP part for future updates		*/	
/*  Parameters:		pBuf  - The original supplied BIP pointer										*/
/*  Return Value:	void																			*/
/****************************************************************************************************/
void TIWha::ConvertTxBip2Nvs(void* pBuf)
{
    /* The  P2GCAL Buffer coming from WHA (not containing the TesCmdID )*/
    TTestCmdP2GCal*	pTestCmdP2GCal = (TTestCmdP2GCal*)pBuf;

    #ifdef TI_TEST
        os_printf("Status = %d",pTestCmdP2GCal->oRadioStatus);
    #endif

    /* Restore back the MAC Address stored from the NVS into the NVS array */
    os_memoryCopy(NULL,iBipNvsBuffer,iNvsStart,MAC_ADD_SIZE);
    
    /* Copy the TxBIP data into the NVS */
    os_memoryCopy(NULL, &iBipNvsBuffer[TX_VALUE_INDEX], ((TTestCmdP2GCal*)pTestCmdP2GCal)->oNvsStruct.Buffer, ((TTestCmdP2GCal*)pTestCmdP2GCal)->oNvsStruct.Length);

    /* Update the Rx & Tx Params type */
    iBipNvsBuffer[TX_TYPE_INDEX] = TX_TYPE;    
    iBipNvsBuffer[TX_LEN_INDEX] = TX_LEN_LS;
    iBipNvsBuffer[TX_LEN_INDEX + 1] = TX_LEN_MS;

    iBipNvsBuffer[RX_TYPE_INDEX] = RX_TYPE;    
    iBipNvsBuffer[RX_LEN_INDEX] = RX_LEN;

    pTestCmdP2GCal->oNvsStruct.Length = NVS_LEN;

    #ifdef TI_TEST
        os_printf("NVS Length, After -- %d",pTestCmdP2GCal->oNvsStruct.Length);
    #endif
    
    iBipNvsBuffer[NVS_TYPE_INDEX] 		= NVS_TYPE;
    iBipNvsBuffer[NVS_TYPE_INDEX + 1]	= 0x03;
    iBipNvsBuffer[NVS_TYPE_INDEX + 2]	= 0x00;
    iBipNvsBuffer[NVS_TYPE_INDEX + 3]	= 0x00;
    iBipNvsBuffer[NVS_TYPE_INDEX + 4]	= 0x00;
    iBipNvsBuffer[NVS_TYPE_INDEX + 5]	= 0x02;
    iBipNvsBuffer[NVS_TYPE_INDEX + 6]	= 0xFF;
    iBipNvsBuffer[NVS_TYPE_INDEX + 7]	= 0xFF;
    iBipNvsBuffer[NVS_TYPE_INDEX + 8]	= 0x00;
    iBipNvsBuffer[NVS_TYPE_INDEX + 9]	= 0x00;

    /* Copy the TxBIP data into the pBuf */
    os_memoryCopy(NULL, pTestCmdP2GCal->oNvsStruct.Buffer, iBipNvsBuffer, pTestCmdP2GCal->oNvsStruct.Length);
    

    #ifdef TI_TEST
        os_printf("**********               NVS                     *****************");
        
        for (int i = 0; i < iNVSlength; i++)
        {
            os_printf("iBipNvsBuffer[%d] = 0x%x",i,((TUint8*)iBipNvsBuffer)[i]);
        }
    #endif /*TI_TEST*/
}


/****************************************************************************************************/
/*  Function:		ConvertRxBip2Nvs    															*/
/****************************************************************************************************/
/*  Description:	Recreate the NVS array from the result of the Rx BIP and from the				*/
/*                  NVS stored for MAC Address restoration											*/	
/*                  iBipNvsBuffer - is used in order to keep the RxBiP part for future updates		*/	
/*  Parameters:		pBuf  - The original supplied BIP pointer										*/
/*  Return Value:	void																			*/
/****************************************************************************************************/
void TIWha::ConvertRxBip2Nvs(void* pBuf)
{
    /* The  P2GCAL Buffer coming from WHA (not containing the TesCmdID )*/
    RadioRxPltCal*	pTestCmdRxPltCal = (RadioRxPltCal*)pBuf;

    #ifdef TI_TEST
        os_printf("Status = %d",pTestCmdRxPltCal->oRadioStatus);
    #endif

    /* Restore back the MAC Address stored from the NVS into the NVS array */
    os_memoryCopy(NULL,iBipNvsBuffer,iNvsStart,MAC_ADD_SIZE);

    /* Copy the TxBIP data into the NVS */
    os_memoryCopy(NULL, &iBipNvsBuffer[RX_VALUE_INDEX], ((RadioRxPltCal*)pTestCmdRxPltCal)->oNvsStruct.Buffer, ((RadioRxPltCal*)pTestCmdRxPltCal)->oNvsStruct.Length);

    /* Update the Rx & TX Params type */
    iBipNvsBuffer[TX_TYPE_INDEX] = TX_TYPE;    
    iBipNvsBuffer[TX_LEN_INDEX] = TX_LEN_LS;
    iBipNvsBuffer[TX_LEN_INDEX + 1] = TX_LEN_MS;

    iBipNvsBuffer[RX_TYPE_INDEX] = RX_TYPE;    
    iBipNvsBuffer[RX_LEN_INDEX] = RX_LEN;
        
    pTestCmdRxPltCal->oNvsStruct.Length = NVS_LEN;

    #ifdef TI_TEST
        os_printf("NVS Length, After -- %d",pTestCmdRxPltCal->oNvsStruct.Length);
    #endif
    
    
    iBipNvsBuffer[NVS_TYPE_INDEX] 		= NVS_TYPE;
    iBipNvsBuffer[NVS_TYPE_INDEX + 1]	= 0x03;
    iBipNvsBuffer[NVS_TYPE_INDEX + 2]	= 0x00;
    iBipNvsBuffer[NVS_TYPE_INDEX + 3]	= 0x00;
    iBipNvsBuffer[NVS_TYPE_INDEX + 4]	= 0x00;
    iBipNvsBuffer[NVS_TYPE_INDEX + 5]	= 0x02;
    iBipNvsBuffer[NVS_TYPE_INDEX + 6]	= 0xFF;
    iBipNvsBuffer[NVS_TYPE_INDEX + 7]	= 0xFF;
    iBipNvsBuffer[NVS_TYPE_INDEX + 8]	= 0x00;
    iBipNvsBuffer[NVS_TYPE_INDEX + 9]	= 0x00;

    /* Copy the RxBIP data into the pBuf */
    os_memoryCopy(NULL, pBuf, pTestCmdRxPltCal->oNvsStruct.Buffer, pTestCmdRxPltCal->oNvsStruct.Length);

    #ifdef  TI_TEST
        os_printf("**********               NVS                     *****************");
        
        for (int i = 0; i < iNVSlength; i++)
        {
            os_printf("iBipNvsBuffer[%d] = 0x%x",i,((TUint8*)iBipNvsBuffer)[i]);
        }
    #endif
}

/****************************************************************************************************/
/*  Function:		UpdateNVSFile																	*/
/****************************************************************************************************/
/*  Description:	Create NVS file																	*/ 
/*  Parameters:		updatedProtocol - protocol sending BIP if == NVS_FILE_WRONG_UPDATE				*/
/*									  will get the previous NVS file								*/
/*					nvsPtr - tx PTL NVS structure													*/
/*  Return Value:	True - file created, False - otherwise											*/
/****************************************************************************************************/
/*	25.05.2008		Efil	Function Created														*/
/****************************************************************************************************/

#if 0
bool TIWha::UpdateNVSFile(const void	*nvsPtr)
{
	
	// read previous NVS
	nvsFileValid = true;//this->ReadNVSFile(currentNVSbuffer, &lengthOfCurrentNVSBufer);
	lengthOfCurrentNVSBufer = iNVSlength;
	currentNVSbuffer = ipNVSbuf;

	txTypeIndexValue = currentNVSbuffer[NVS_PRE_PARAMETERS_LENGTH + NVS_TX_TYPE_INDEX];
	rxTypeIndexValue = currentNVSbuffer[NVS_PRE_PARAMETERS_LENGTH + NVS_RX_TYPE_INDEX];

	// if read all the parameter (needed) from the previous NVS
	if (!nvsFileValid									|| 
		(lengthOfCurrentNVSBufer >= NVS_TOTAL_LENGTH)	||
		(txTypeIndexValue != eNVS_RADIO_TX_PARAMETERS)	||
		(rxTypeIndexValue != eNVS_RADIO_RX_PARAMETERS))
	{
		os_printf("lengthOfCurrentNVSBufer = %d txTypeIndexValue = %d rxTypeIndexValue = %d",lengthOfCurrentNVSBufer,txTypeIndexValue,rxTypeIndexValue);
		nvsFileValid = false;
	}
	
	//nvsBinFile = fopen(NVS_FILE_NAME, "wb");

   /* if (NULL == nvsBinFile)
	{
		return false;
	}*/

	// fill MAC Address
	//FillMACAddressToNVS(nvsBinFile);
	
	// fill end burst transaction zeros
	/*for (index = 0; index < NVS_END_BURST_TRANSACTION_LENGTH; index++)
	{
		fwrite(&valueToSet, sizeof(uint8), 1, nvsBinFile);
	}*/

	// fill zeros to Align TLV start address
	/*for (index = 0; index < NVS_ALING_TLV_START_ADDRESS_LENGTH; index++)
	{
		fwrite(&valueToSet, sizeof(uint8), 1, nvsBinFile);
	}*/

	// Getting from TX BiP Command
//	if(updatedProtocol)
//	{
		// Fill new TX BiP values
		FillTXParameterToNVS(currentNVSbuffer,
			                 nvsPtr);
		if (nvsFileValid)
		{
			// set Parameters of RX from the previous file
			FillOldRXParameterToNVS(nvsBinFile,
				                    currentNVSbuffer,
									lengthOfCurrentNVSBufer);
		}
		else
		{
			FillDefaultRXParameterToNVS(nvsBinFile);
		}

//	}
	// Getting from RX BiP Command
	else if (NVS_FILE_RX_PARAMETERS_UPDATE == updatedProtocol)
	{
		if (nvsFileValid)
		{
			// set Parameters of TX from the previous file
			FillOldTXParameterToNVS(nvsBinFile,
				                    currentNVSbuffer,
									lengthOfCurrentNVSBufer);
		}
		else
		{
			FillDefaultTXParameterToNVS(nvsBinFile);
		}

		// Fill new RX BiP values 
		FillRXParameterToNVS(nvsBinFile,
			                 nvsPtr);
	
	}
	// Getting error from Sending aider TX or RX BiP
	else  /* NVS_FILE_WRONG_UPDATE == updatedProtocol */
	{
		// Fill new TX BiP values
		FillTXParameterToNVS(nvsBinFile,
			                 nvsPtr);

		// Fill new RX BiP values 
		FillRXParameterToNVS(nvsBinFile,
			                 nvsPtr);

	}

	// Fill the NVS version to the NVS
	this->FillVersionToNVS(nvsBinFile,
						   versionStr,
						   nvsFileValid,
						   currentNVSbuffer,
						   lengthOfCurrentNVSBufer);

	// End of NVS
	this->WriteEndNVSParam(nvsBinFile);
	

	// close the file
	fclose(nvsBinFile);

	return true;
}
#endif
/*
BipRespons(TI_STATUS aStatus, void *pItrBuf)
Call ConvertBip2Nvs(pItrBuf)
Call PltResponse
*/

/** 
 * \fn     PltResponseCb
 * \brief  member function.
 * 
 * \note    
 * \param  aStatus - result of sending stop scan command to FW
 * \param  pItrBuf -  buffer for results
 * \return  
 * \sa          TIWHA::Plt
 */ 
void TIWha::PltResponseCb ( TI_STATUS aStatus, void *pItrBuf)
{
	WHA::TStatus status = ( aStatus == TI_OK ) ? WHA::KSuccess : WHA::KFailed;
	os_printf("*** PltResponseCb status = %d  ***",status);
    TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " aStatus = %d ptr = %p plt = %d\n", aStatus, pItrBuf, iTwdCtrl.ePlt);

    #ifdef PLT_TESTER
        os_printf("*** PltResponseCb  iRealPlt %d ***",iRealPlt);
        if(iRealPlt)
        {		
            if( bErrorIndication == TI_FALSE)
            {    
                WhaCb()->CommandResponse(WHA::EWriteMIBResponse,status,iUCommandResponseParams);
            }
            #if TI_DBG
                else
                {
                    WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
                }
            #endif	
        }
        else
        {
            PltSm (pItrBuf);
        }
    #else 
        /* Note that the results are located in the original given buffer in TIWha::Plt() */
        #ifdef TI_TEST            
            if( bErrorIndication == TI_FALSE)
            {    
                WhaCb()->CommandResponse(WHA::EWriteMIBResponse,status,iUCommandResponseParams);
            }
            #if TI_DBG
                else
                {
                    WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
                }
            #endif	
    	#else
            /* in case of using an external tester we have to return response */            
            if( bErrorIndication == TI_FALSE)
            {    
                WhaCb()->CommandResponse(WHA::EPLtResponse, status, iUCommandResponseParams);
            }
            #if TI_DBG
                else
                {
                    WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
                }
            #endif	
	    #endif
    #endif /* PLT_TESTER */

	return;
}

/** 
 * \fn     ReadMemResponseCb
 * \brief  member function.
 * 
 * \note    
 * \param  params - struct of the retrieved results
 * \return  
 * \sa          TIWha::WriteMem
 */ 
 void TIWha::ReadMemResponseCb (TFwDebugParams* params)
{
     TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " p = %p\n",params);
    
    #ifdef WRITE_READ_MEM_NEW_API  
        /* Note that the results are located in the original given buffer in TIWha::ReadMem() */	
        if( bErrorIndication == TI_FALSE)
        {    
            WhaCb()->CommandResponse(WHA::EReadMemoryResponse, WHA::KSuccess, iUCommandResponseParams);		
        }
        #if TI_DBG
            else
            {
                WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
            }
        #endif	
    #endif
}

/** 
 * \fn     WriteMemResponseCb
 * \brief  member function.
 * 
 * \note    
 * \param  params - struct of the retrieved results
 * \return  
 * \sa          TIWha::WriteMem
 */ 
void TIWha::WriteMemResponseCb (TFwDebugParams* params)
{
    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " p = %p\n", params);

    #ifdef WRITE_READ_MEM_API  
        /* Note that the results are located in the original given buffer in TIWha::WriteMem() */
        if( bErrorIndication == TI_FALSE)
        {    
            WhaCb()->CommandResponse(WHA::EWriteMemoryResponse, WHA::KSuccess, iUCommandResponseParams);		
        }
        #if TI_DBG
            else
            {
                WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
            }
        #endif	
    #endif
}



/****************************************************************************************
 *                        PltPerStartResponse()                            
 ****************************************************************************************
DESCRIPTION:    

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::PltPerStartResponseCb ( TUint16 aStatus, void *pItrBuf)
{   

    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "\n");

    return;
}

/****************************************************************************************
 *                        PltPerStopResponse()                                 
 ****************************************************************************************
DESCRIPTION:    

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::PltPerStopResponseCb ( TUint16 aStatus, void *pItrBuf)
{

    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "\n");

    return;
}


/****************************************************************************************
 *                        PltPerClearResponse()                                 
 ****************************************************************************************
DESCRIPTION:    

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::PltPerClearResponseCb ( TUint16 aStatus, void *pItrBuf)
{

    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "\n");

    return;
}


/****************************************************************************************
 *                        PltPerGetResultResponse()                                 
 ****************************************************************************************
DESCRIPTION:    

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::PltPerGetResultResponseCb ( TUint16 aStatus, void *pItrBuf)
{
    return;
}


/****************************************************************************************
 *                        PltGainGetResponse()                                 
 ****************************************************************************************
DESCRIPTION:    

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::PltGainGetResponseCb ( TUint16 aStatus, void *pItrBuf)
{
    return;
}


/****************************************************************************************
 *                        PltGetNVSUpdateBufferResponse()                                 
 ****************************************************************************************
DESCRIPTION:    

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::PltGetNVSUpdateBufferResponseCb ( TUint16 aStatus, void *InterrogateParamsBuf)
{
    return;
}


/** 
 * \fn     JoinCompleteCb
 * \brief  Send Join response with the current power level table
 * 
 * \note    
 * \return  
 * \sa      
 */ 
void TIWha::JoinCompleteCb ()
{    
    /* Tx power level for each radio band  */
    TFwInfo *pFwInfo = TWD_GetFWInfo (iTwdCtrl.hTWD);

    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "JoinCompleteCb \n");

    /*
     * Note: Only 2.4 band is supported for power level feedback.
     */
    iUCommandResponseParams.iJoinResponse.iMinPowerLevel = 
		pFwInfo->txPowerTable[RADIO_BAND_2_4_GHZ][MIN_POWER_LEVEL-1] / DBM_TO_TX_POWER_FACTOR;
    iUCommandResponseParams.iJoinResponse.iMaxPowerLevel= 
		pFwInfo->txPowerTable[RADIO_BAND_2_4_GHZ][MAX_POWER_LEVEL] / DBM_TO_TX_POWER_FACTOR;

    if( bErrorIndication == TI_FALSE)
    {    
        WhaCb()->CommandResponse (WHA::EJoinResponse, WHA::KSuccess, iUCommandResponseParams);		
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif	
}

/** 
 * \fn     ReadMIBResponseCb
 * \brief  member function.
 * 
 * \note    
 * \return  
 * \sa      
 */ 
void TIWha::ReadMIBResponseCb ( TUint16 aStatus, void *pItrBuf)
{
    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " Mib = %d \n", iTwdCtrl.currentReadMibID);
    
    if (aStatus != TI_OK)
    {
        TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, "GWSI_AdaptCB_ReadMIBResponse: error received from Mail Box on MIB read");
    }

    /* 
     * iData points to the pre-allocated ReadMib buffer. This buffer will be filled with the results
     * in the next ReadMIBXXX functions 
     */
    iUCommandResponseParams.iReadMibResponse.iData = (const void*)iReabMibMem;
    
    /* Fill in iUCommandResponseParams according to the current MIB */
    switch (iTwdCtrl.currentReadMibID)
    {
    case WHA::KMibDot11StationId:
        ReadMIBStationIdResponse(aStatus, pItrBuf);
        break;
    
    case WHA::KMibStatisticsTable:
        ReadMIBstatisticsTableResponse (aStatus, pItrBuf);
        break;
    
    case WHA::KMibCountersTable:
        ReadMIBcountersTableResponse (aStatus, pItrBuf);
        break;
    default:
        TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, "ReadMIBResponse: error unknown MIB = %d", iTwdCtrl.currentReadMibID);
      
    }

    /* Send response to upper layer */    
    if( bErrorIndication == TI_FALSE)
    {    
        WhaCb()->CommandResponse(WHA::EReadMIBResponse, (WHA::TStatus)aStatus, iUCommandResponseParams);		
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif	
}

/** 
 * \fn     ReadMIBStationIdResponse 
 * \brief  Copy station ID to iReabMibMem buffer
 * 
 * \note    
 * \return  
 * \sa      
 */ 
void TIWha::ReadMIBStationIdResponse (TUint16 aStatus, void *pItrBuf)
{    
    iUCommandResponseParams.iReadMibResponse.iLength = sizeof(WHA::Sdot11StationId);
    iUCommandResponseParams.iReadMibResponse.iMib = WHA::KMibDot11StationId;
    MAC_COPY (iReabMibMem, ((TI_UINT8*)pItrBuf));
    
    TRACE6(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",                iReabMibMem[0],iReabMibMem[1],iReabMibMem[2],                iReabMibMem[3],iReabMibMem[4],iReabMibMem[5]);
}


/** 
 * \fn     ReadMIBstatisticsTableResponse
 * \brief  Copy statistics table to iReabMibMem buffer
 * 
 * \note    
 * \return  
 * \sa      
 */ 
void TIWha::ReadMIBstatisticsTableResponse (TUint16 aStatus, void *pItrBuf)
{
    WHA::SstatisticsTable *pStatisticsTable = (WHA::SstatisticsTable* )iReabMibMem;
    ACXRoamingStatisticsTable_t *pRetreivedResult = (ACXRoamingStatisticsTable_t *)pItrBuf;

    pStatisticsTable->iSnr = pRetreivedResult->snrData;
    /* 
     * Temporary, use RSSI value instead of RCPI. Make it positive. 
     * In case the RSSI is positive already (odd but theoretically 
     * possible result), set it to 0 
     */
    if ((pRetreivedResult->rssiData < TIWha_MAX_RSSI_VAL) && (pRetreivedResult->rssiData > TIWha_MIN_RSSI_VAL))
    {
        pStatisticsTable->iRcpi = (pRetreivedResult->rssiBeacon - TIWha_MIN_RSSI_VAL) * 2;
    }
    else
    {
        /* incase that the RSSI value shifted we fixed it to -45db (130 incase of RCPI) */
        if (pRetreivedResult->rssiData < 0) {
            pStatisticsTable->iRcpi = 130;
        }
        pStatisticsTable->iRcpi = 0;
    }
    
    TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "ReadMIBstatisticsTableResponse:  status = %d SNR=0x%x, RCPI=0x%x\n",          aStatus, pStatisticsTable->iSnr, pStatisticsTable->iRcpi);
    
    iUCommandResponseParams.iReadMibResponse.iLength = sizeof(WHA::SstatisticsTable);
    iUCommandResponseParams.iReadMibResponse.iMib = WHA::KMibStatisticsTable;
}


/** 
 * \fn     ReadMIBcountersTableResponse
 * \brief  Copy counters table to iReabMibMem buffer
 * 
 * \note    
 * \return  
 * \sa      
 */ 
void TIWha::ReadMIBcountersTableResponse (TUint16 aStatus, void *pItrBuf)
{
    WHA::ScountersTable       *pCountersTable = (WHA::ScountersTable*)iReabMibMem;
    ACXErrorCounters_t     *pRetreivedResult = (ACXErrorCounters_t *)pItrBuf;

    pCountersTable->iPlcpError = pRetreivedResult->PLCPErrorCount;
    pCountersTable->iFcsError = pRetreivedResult->FCSErrorCount;

    TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "ReadMIBcountersTableResponse:  FCSErrorCount = 0x%x, PLCPErrorCount = 0x%x \n",         pCountersTable->iFcsError, pCountersTable->iPlcpError);

    iUCommandResponseParams.iReadMibResponse.iLength = sizeof(WHA::SstatisticsTable);
    iUCommandResponseParams.iReadMibResponse.iMib = WHA::KMibCountersTable;
}


/****************************************************************************************
 *                        ReadMIBBtCoexParamsResponse()                    *
 ****************************************************************************************
DESCRIPTION:    result of Read counters Table

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::ReadMIBBtCoexParamsResponse (TUint16 aStatus, void *pItrBuf)
{
    return;
}


/****************************************************************************************
 *                        ReadMIBVersion()                                 *
 ****************************************************************************************
DESCRIPTION:    result of Read counters Table

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::ReadMIBVersion (TUint16 aStatus, void *pItrBuf)
{
    return;
}


/****************************************************************************************
 *                        LostBssIndication()                              *
 ****************************************************************************************
DESCRIPTION:    LostBss Event Callback

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::LostBssIndicationCb ()
{
	WHA::UIndicationParams	*pParams = NULL;
    
    if( bErrorIndication == TI_FALSE)
    {    
        WhaCb()->Indication(WHA::EBssLost,*pParams);
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif	

    #ifdef TI_DBG
        WLAN_OS_REPORT(("TIWha::LostBssIndicationCb () --"));
    #endif
}


/****************************************************************************************
 *                        RegainBssIndication()                            *
 ****************************************************************************************
DESCRIPTION:    RegainBss Event Callback

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::RegainBssIndicationCb ()
{
    return;
}


/****************************************************************************************
 *                        btCoexSenseIndication()                          *
 ****************************************************************************************
DESCRIPTION:    Bt Coexistence Sense CB (In auto mode: whether we are active or not) 

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::btCoexSenseIndicationCb ( TUint8* buffer, TUint32 len)
{
    return;
}


/****************************************************************************************
 *                        btCoexProtectiveIndication()                     *
 ****************************************************************************************
DESCRIPTION:    Bt Coexistence Protective mode CB (whether BT is in protective mode) 

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::btCoexProtectiveIndicationCb ( TUint8* buffer, TUint32 len)
{
    return;
}


/****************************************************************************************
 *                        btCoexAvalancheIndication()                      *
 ****************************************************************************************
DESCRIPTION:    Bt Coexistence avalanche CB (Indicate that the quality of connection with the AP is 
                reducing and we should reconnect) 
INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void TIWha::btCoexAvalancheIndicationCb ( TUint8* buffer, TUint32 len)
{
    return;
}


/****************************************************************************
 *                      TWD_StatisticsReadCB ()
 ****************************************************************************
 * DESCRIPTION: Interrogate Statistics from the wlan hardware
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: TI_OK or TI_NOK
 ****************************************************************************/
void TIWha::TWD_AvrRssiReadCB (TI_HANDLE hCb, TI_UINT16 MboxStatus, ACXRoamingStatisticsTable_t* roamingStatistics)
{
	/* Call MIB Response befor printing to release system while traffic */	
    if( bErrorIndication == TI_FALSE)
    {    
        ((TIWha*)(hCb))->WhaCb()->CommandResponse(WHA::EWriteMIBResponse,TI_OK,((TIWha*)(hCb))->iUCommandResponseParams);    
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif      

    #if TI_DBG
        WLAN_OS_REPORT(("Avr beacon Rssi = %d\n", roamingStatistics->rssiBeacon));
        WLAN_OS_REPORT(("Avr data Rssi = %d\n", roamingStatistics->rssiData));
    #endif	
}



/****************************************************************************
 *                      TWD_StatisticsReadCB ()
 ****************************************************************************
 * DESCRIPTION: Interrogate Statistics from the wlan hardware
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: TI_OK or TI_NOK
 ****************************************************************************/
void TIWha::TWD_StatisticsReadCB (TI_HANDLE hCb, TI_UINT16 MboxStatus, ACXStatistics_t* pElem)
{
	/* Call MIB Response befor printing to release system while traffic */	
    if( bErrorIndication == TI_FALSE)
    {    
        ((TIWha*)(hCb))->WhaCb()->CommandResponse(WHA::EWriteMIBResponse,TI_OK,((TIWha*)(hCb))->iUCommandResponseParams);
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif      
            
    /* 
     *  Handle FW statistics endianess
     *  ==============================
     */

    /* Ring */
    pElem->ringStat.numOfTxProcs       = ENDIAN_HANDLE_LONG(pElem->ringStat.numOfTxProcs);
    pElem->ringStat.numOfPreparedDescs = ENDIAN_HANDLE_LONG(pElem->ringStat.numOfPreparedDescs);
    pElem->ringStat.numOfTxXfr         = ENDIAN_HANDLE_LONG(pElem->ringStat.numOfTxXfr);
    pElem->ringStat.numOfTxDma         = ENDIAN_HANDLE_LONG(pElem->ringStat.numOfTxDma);
    pElem->ringStat.numOfTxCmplt       = ENDIAN_HANDLE_LONG(pElem->ringStat.numOfTxCmplt);
    pElem->ringStat.numOfRxProcs       = ENDIAN_HANDLE_LONG(pElem->ringStat.numOfRxProcs);
    pElem->ringStat.numOfRxData        = ENDIAN_HANDLE_LONG(pElem->ringStat.numOfRxData);

    /* Debug */
    pElem->debug.debug1                = ENDIAN_HANDLE_LONG(pElem->debug.debug1);
    pElem->debug.debug2                = ENDIAN_HANDLE_LONG(pElem->debug.debug2);
    pElem->debug.debug3                = ENDIAN_HANDLE_LONG(pElem->debug.debug3);
    pElem->debug.debug4                = ENDIAN_HANDLE_LONG(pElem->debug.debug4);
    pElem->debug.debug5                = ENDIAN_HANDLE_LONG(pElem->debug.debug5);
    pElem->debug.debug6                = ENDIAN_HANDLE_LONG(pElem->debug.debug6);

    /* Isr */
    pElem->isr.IRQs                    = ENDIAN_HANDLE_LONG(pElem->isr.IRQs);

    /* Rx */
    pElem->rx.RxDroppedFrame           = ENDIAN_HANDLE_LONG(pElem->rx.RxDroppedFrame);
	pElem->rx.RxCompleteDroppedFrame   = ENDIAN_HANDLE_LONG(pElem->rx.RxCompleteDroppedFrame);
    
    pElem->rx.RxHdrOverflow            = ENDIAN_HANDLE_LONG(pElem->rx.RxHdrOverflow);
    pElem->rx.RxHWStuck                = ENDIAN_HANDLE_LONG(pElem->rx.RxHWStuck);
    pElem->rx.RxOutOfMem               = ENDIAN_HANDLE_LONG(pElem->rx.RxOutOfMem);
    pElem->rx.RxAllocFrame             = ENDIAN_HANDLE_LONG(pElem->rx.RxAllocFrame);

    /* Tx */
    pElem->tx.numOfTxTemplatePrepared  	= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxTemplatePrepared);
	pElem->tx.numOfTxDataPrepared  		= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxDataPrepared);
	pElem->tx.numOfTxTemplateProgrammed = ENDIAN_HANDLE_LONG(pElem->tx.numOfTxTemplateProgrammed);
	pElem->tx.numOfTxDataProgrammed  	= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxDataProgrammed);
	pElem->tx.numOfTxBurstProgrammed  	= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxBurstProgrammed);
	pElem->tx.numOfTxStarts  			= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxStarts);
	pElem->tx.numOfTxImmResp  			= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxImmResp);
	pElem->tx.numOfTxStartTempaltes  	= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxStartTempaltes);
	pElem->tx.numOfTxStartIntTemplate  	= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxStartIntTemplate);
	pElem->tx.numOfTxStartFwGen  		= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxStartFwGen);
	pElem->tx.numOfTxStartData  		= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxStartData);
	pElem->tx.numOfTxStartNullFrame  	= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxStartNullFrame);
	pElem->tx.numOfTxExch  				= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxExch);
	pElem->tx.numOfTxRetryTemplate  	= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxRetryTemplate);
	pElem->tx.numOfTxRetryData  		= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxRetryData);
	pElem->tx.numOfTxExchPending  		= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxExchPending);
	pElem->tx.numOfTxExchExpiry  		= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxExchExpiry);
	pElem->tx.numOfTxExchMismatch  		= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxExchMismatch);
	pElem->tx.numOfTxDoneTemplate  		= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxDoneTemplate);
	pElem->tx.numOfTxDoneData  			= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxDoneData);
	pElem->tx.numOfTxDoneIntTemplate  	= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxDoneIntTemplate);
	pElem->tx.numOfTxPreXfr  			= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxPreXfr);
	pElem->tx.numOfTxXfr  				= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxXfr);
	pElem->tx.numOfTxXfrOutOfMem  		= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxXfrOutOfMem);
	pElem->tx.numOfTxDmaProgrammed  	= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxDmaProgrammed);
	pElem->tx.numOfTxDmaDone  			= ENDIAN_HANDLE_LONG(pElem->tx.numOfTxDmaDone);

    /* Dma */
    pElem->dma.RxDMAErrors             = ENDIAN_HANDLE_LONG(pElem->dma.RxDMAErrors);
    pElem->dma.TxDMAErrors             = ENDIAN_HANDLE_LONG(pElem->dma.TxDMAErrors);

    /* Wep */
    pElem->wep.WepAddrKeyCount         = ENDIAN_HANDLE_LONG(pElem->wep.WepAddrKeyCount);
    pElem->wep.WepDefaultKeyCount      = ENDIAN_HANDLE_LONG(pElem->wep.WepDefaultKeyCount);
    pElem->wep.WepKeyNotFound          = ENDIAN_HANDLE_LONG(pElem->wep.WepKeyNotFound);
    pElem->wep.WepDecryptFail          = ENDIAN_HANDLE_LONG(pElem->wep.WepDecryptFail);
    
    /* AES */
    pElem->aes.AesEncryptFail          = ENDIAN_HANDLE_LONG(pElem->aes.AesEncryptFail);     
    pElem->aes.AesDecryptFail          = ENDIAN_HANDLE_LONG(pElem->aes.AesDecryptFail);     
    pElem->aes.AesEncryptPackets       = ENDIAN_HANDLE_LONG(pElem->aes.AesEncryptPackets);  
    pElem->aes.AesDecryptPackets       = ENDIAN_HANDLE_LONG(pElem->aes.AesDecryptPackets);  
    pElem->aes.AesEncryptInterrupt     = ENDIAN_HANDLE_LONG(pElem->aes.AesEncryptInterrupt);
    pElem->aes.AesDecryptInterrupt     = ENDIAN_HANDLE_LONG(pElem->aes.AesDecryptInterrupt);

    /* Events */
    pElem->event.calibration           = ENDIAN_HANDLE_LONG(pElem->event.calibration);
    pElem->event.rxMismatch            = ENDIAN_HANDLE_LONG(pElem->event.rxMismatch); 
    pElem->event.rxMemEmpty            = ENDIAN_HANDLE_LONG(pElem->event.rxMemEmpty); 

    /* PS */
    pElem->pwr.MissingBcnsCnt          	= ENDIAN_HANDLE_LONG(pElem->pwr.MissingBcnsCnt);
    pElem->pwr.RcvdBeaconsCnt          	= ENDIAN_HANDLE_LONG(pElem->pwr.RcvdBeaconsCnt);
    pElem->pwr.ConnectionOutOfSync     	= ENDIAN_HANDLE_LONG(pElem->pwr.ConnectionOutOfSync);
    pElem->pwr.ContMissBcnsSpread[0]   	= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[0]);
    pElem->pwr.ContMissBcnsSpread[1]   	= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[1]);
    pElem->pwr.ContMissBcnsSpread[2]   	= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[2]);
    pElem->pwr.ContMissBcnsSpread[3]   	= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[3]);
    pElem->pwr.ContMissBcnsSpread[4]   	= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[4]);
    pElem->pwr.ContMissBcnsSpread[5]   	= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[5]);
    pElem->pwr.ContMissBcnsSpread[6]   	= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[6]);
    pElem->pwr.ContMissBcnsSpread[7]   	= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[7]);
    pElem->pwr.ContMissBcnsSpread[8]   	= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[8]);
    pElem->pwr.ContMissBcnsSpread[9]   	= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[9]);

    pElem->ps.psPollTimeOuts           	= ENDIAN_HANDLE_LONG(pElem->ps.psPollTimeOuts);
    pElem->ps.upsdTimeOuts             	= ENDIAN_HANDLE_LONG(pElem->ps.upsdTimeOuts);
    pElem->ps.upsdMaxAPturn            	= ENDIAN_HANDLE_LONG(pElem->ps.upsdMaxAPturn); 
    pElem->ps.psPollMaxAPturn          	= ENDIAN_HANDLE_LONG(pElem->ps.psPollMaxAPturn);
    pElem->ps.psPollUtilization        	= ENDIAN_HANDLE_LONG(pElem->ps.psPollUtilization);
    pElem->ps.upsdUtilization          	= ENDIAN_HANDLE_LONG(pElem->ps.upsdUtilization);

	pElem->rxFilter.arpFilter		   	= ENDIAN_HANDLE_LONG(pElem->rxFilter.arpFilter);
	pElem->rxFilter.beaconFilter	   	= ENDIAN_HANDLE_LONG(pElem->rxFilter.beaconFilter);
	pElem->rxFilter.dataFilter		   	= ENDIAN_HANDLE_LONG(pElem->rxFilter.dataFilter);
	pElem->rxFilter.dupFilter		   	= ENDIAN_HANDLE_LONG(pElem->rxFilter.dupFilter);
	pElem->rxFilter.MCFilter		   	= ENDIAN_HANDLE_LONG(pElem->rxFilter.MCFilter);
	pElem->rxFilter.ibssFilter		   	= ENDIAN_HANDLE_LONG(pElem->rxFilter.ibssFilter);

	pElem->radioCal.calStateFail	   	= ENDIAN_HANDLE_LONG(pElem->radioCal.calStateFail);
	pElem->radioCal.initCalTotal	   	= ENDIAN_HANDLE_LONG(pElem->radioCal.initCalTotal);
	pElem->radioCal.initRadioBandsFail 	= ENDIAN_HANDLE_LONG(pElem->radioCal.initRadioBandsFail);
	pElem->radioCal.initRxIqMmFail		= ENDIAN_HANDLE_LONG(pElem->radioCal.initRxIqMmFail);
	pElem->radioCal.initSetParams		= ENDIAN_HANDLE_LONG(pElem->radioCal.initSetParams);
	pElem->radioCal.initTxClpcFail		= ENDIAN_HANDLE_LONG(pElem->radioCal.initTxClpcFail);
	pElem->radioCal.tuneCalTotal		= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneCalTotal);
	pElem->radioCal.tuneDrpwChanTune	= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneDrpwChanTune);
	pElem->radioCal.tuneDrpwLnaTank		= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneDrpwLnaTank);
	pElem->radioCal.tuneDrpwPdBufFail	= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneDrpwPdBufFail);
	pElem->radioCal.tuneDrpwRTrimFail	= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneDrpwRTrimFail);
	pElem->radioCal.tuneDrpwRxDac		= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneDrpwRxDac);
	pElem->radioCal.tuneDrpwRxIf2Gain	= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneDrpwRxIf2Gain);
	pElem->radioCal.tuneDrpwRxTxLpf		= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneDrpwRxTxLpf);
	pElem->radioCal.tuneDrpwTaCal		= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneDrpwTaCal);
	pElem->radioCal.tuneDrpwTxMixFreqFail = ENDIAN_HANDLE_LONG(pElem->radioCal.tuneDrpwTxMixFreqFail);
	pElem->radioCal.tuneRxAnaDcFail		= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneRxAnaDcFail);
	pElem->radioCal.tuneRxIqMmFail		= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneRxIqMmFail);
	pElem->radioCal.tuneTxClpcFail		= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneTxClpcFail);
	pElem->radioCal.tuneTxIqMmFail		= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneTxIqMmFail);
	pElem->radioCal.tuneTxLOLeakFail	= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneTxLOLeakFail);
	pElem->radioCal.tuneTxPdetFail		= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneTxPdetFail);
	pElem->radioCal.tuneTxPPAFail		= ENDIAN_HANDLE_LONG(pElem->radioCal.tuneTxPPAFail);

    #if TI_DBG
        /* 
         *  Print FW statistics 
         *  ===================
         */
    
        /* Ring */
        WLAN_OS_REPORT(("------  Ring statistics  -------------------\n"));
        WLAN_OS_REPORT(("numOfTxProcs       = %d\n", pElem->ringStat.numOfTxProcs));
        WLAN_OS_REPORT(("numOfPreparedDescs = %d\n", pElem->ringStat.numOfPreparedDescs));
        WLAN_OS_REPORT(("numOfTxXfr         = %d\n", pElem->ringStat.numOfTxXfr));
        WLAN_OS_REPORT(("numOfTxDma         = %d\n", pElem->ringStat.numOfTxDma));
        WLAN_OS_REPORT(("numOfTxCmplt       = %d\n", pElem->ringStat.numOfTxCmplt));
        WLAN_OS_REPORT(("numOfRxProcs       = %d\n", pElem->ringStat.numOfRxProcs));
        WLAN_OS_REPORT(("numOfRxData        = %d\n", pElem->ringStat.numOfRxData));
    
        /* Debug */
        WLAN_OS_REPORT(("------  Debug statistics  -------------------\n"));
        WLAN_OS_REPORT(("debug1 = %d\n", pElem->debug.debug1));
        WLAN_OS_REPORT(("debug2 = %d\n", pElem->debug.debug2));
        WLAN_OS_REPORT(("debug3 = %d\n", pElem->debug.debug3));
        WLAN_OS_REPORT(("debug4 = %d\n", pElem->debug.debug4));
        WLAN_OS_REPORT(("debug5 = %d\n", pElem->debug.debug5));
        WLAN_OS_REPORT(("debug6 = %d\n", pElem->debug.debug6));
    
        /* Isr */
        WLAN_OS_REPORT(("------  Isr statistics  -------------------\n"));
        WLAN_OS_REPORT(("IRQs = %d\n", pElem->isr.IRQs));
    
        /* Rx */
        WLAN_OS_REPORT(("------  Rx  statistics  -------------------\n"));
        WLAN_OS_REPORT(("RxDroppedFrame    = %d\n", pElem->rx.RxDroppedFrame));
        WLAN_OS_REPORT(("RxHdrOverflow     = %d\n", pElem->rx.RxHdrOverflow));
        WLAN_OS_REPORT(("RxHWStuck         = %d\n", pElem->rx.RxHWStuck));
        WLAN_OS_REPORT(("RxOutOfMem        = %d\n", pElem->rx.RxOutOfMem));
        WLAN_OS_REPORT(("RxAllocFrame      = %d\n", pElem->rx.RxAllocFrame));
    	WLAN_OS_REPORT(("pElem->rx.RxCompleteDroppedFrame = %d\n", pElem->rx.RxCompleteDroppedFrame)); 
    
        WLAN_OS_REPORT(("------  RxFilters statistics  --------------\n"));
    	WLAN_OS_REPORT(("arpFilter    = %d\n", pElem->rxFilter.arpFilter));
    	WLAN_OS_REPORT(("beaconFilter = %d\n", pElem->rxFilter.beaconFilter));
    	WLAN_OS_REPORT(("dataFilter   = %d\n", pElem->rxFilter.dataFilter));
    	WLAN_OS_REPORT(("dupFilter    = %d\n", pElem->rxFilter.dupFilter));
    	WLAN_OS_REPORT(("MCFilter     = %d\n", pElem->rxFilter.MCFilter));
    	WLAN_OS_REPORT(("ibssFilter   = %d\n", pElem->rxFilter.ibssFilter));
    
        /* Tx */
        WLAN_OS_REPORT(("------  Tx  statistics  -------------------\n"));
    	WLAN_OS_REPORT(("numOfTxTemplatePrepared    = %d\n", pElem->tx.numOfTxTemplatePrepared));
    	WLAN_OS_REPORT(("numOfTxDataPrepared        = %d\n", pElem->tx.numOfTxDataPrepared));
    	WLAN_OS_REPORT(("numOfTxTemplateProgrammed  = %d\n", pElem->tx.numOfTxTemplateProgrammed));
    	WLAN_OS_REPORT(("numOfTxDataProgrammed      = %d\n", pElem->tx.numOfTxDataProgrammed));
    	WLAN_OS_REPORT(("numOfTxBurstProgrammed     = %d\n", pElem->tx.numOfTxBurstProgrammed));
    	WLAN_OS_REPORT(("numOfTxStarts              = %d\n", pElem->tx.numOfTxStarts));
    	WLAN_OS_REPORT(("numOfTxImmResp             = %d\n", pElem->tx.numOfTxImmResp));
    	WLAN_OS_REPORT(("numOfTxStartTempaltes      = %d\n", pElem->tx.numOfTxStartTempaltes));
    	WLAN_OS_REPORT(("numOfTxStartIntTemplate    = %d\n", pElem->tx.numOfTxStartIntTemplate));
    	WLAN_OS_REPORT(("numOfTxStartFwGen          = %d\n", pElem->tx.numOfTxStartFwGen));
    	WLAN_OS_REPORT(("numOfTxStartData           = %d\n", pElem->tx.numOfTxStartData));
    	WLAN_OS_REPORT(("numOfTxStartNullFrame      = %d\n", pElem->tx.numOfTxStartNullFrame));
    	WLAN_OS_REPORT(("numOfTxExch                = %d\n", pElem->tx.numOfTxExch));
    	WLAN_OS_REPORT(("numOfTxRetryTemplate       = %d\n", pElem->tx.numOfTxRetryTemplate));
    	WLAN_OS_REPORT(("numOfTxRetryData           = %d\n", pElem->tx.numOfTxRetryData));
    	WLAN_OS_REPORT(("numOfTxExchPending         = %d\n", pElem->tx.numOfTxExchPending));
    	WLAN_OS_REPORT(("numOfTxExchExpiry          = %d\n", pElem->tx.numOfTxExchExpiry));
    	WLAN_OS_REPORT(("numOfTxExchMismatch        = %d\n", pElem->tx.numOfTxExchMismatch));
    	WLAN_OS_REPORT(("numOfTxDoneTemplate        = %d\n", pElem->tx.numOfTxDoneTemplate));
    	WLAN_OS_REPORT(("numOfTxDoneData            = %d\n", pElem->tx.numOfTxDoneData));
    	WLAN_OS_REPORT(("numOfTxDoneIntTemplate     = %d\n", pElem->tx.numOfTxDoneIntTemplate));
    	WLAN_OS_REPORT(("numOfTxPreXfr              = %d\n", pElem->tx.numOfTxPreXfr));
    	WLAN_OS_REPORT(("numOfTxXfr                 = %d\n", pElem->tx.numOfTxXfr));
    	WLAN_OS_REPORT(("numOfTxXfrOutOfMem         = %d\n", pElem->tx.numOfTxXfrOutOfMem));
    	WLAN_OS_REPORT(("numOfTxDmaProgrammed       = %d\n", pElem->tx.numOfTxDmaProgrammed));
    	WLAN_OS_REPORT(("numOfTxDmaDone             = %d\n", pElem->tx.numOfTxDmaDone));
    
        /* Dma */
        WLAN_OS_REPORT(("------  Dma  statistics  -------------------\n"));
        WLAN_OS_REPORT(("RxDMAErrors  = %d\n", pElem->dma.RxDMAErrors));
        WLAN_OS_REPORT(("TxDMAErrors  = %d\n", pElem->dma.TxDMAErrors));
    
        /* Wep */
        WLAN_OS_REPORT(("------  Wep statistics  -------------------\n"));
        WLAN_OS_REPORT(("WepAddrKeyCount   = %d\n", pElem->wep.WepAddrKeyCount));
        WLAN_OS_REPORT(("WepDefaultKeyCount= %d\n", pElem->wep.WepDefaultKeyCount));
        WLAN_OS_REPORT(("WepKeyNotFound    = %d\n", pElem->wep.WepKeyNotFound));
        WLAN_OS_REPORT(("WepDecryptFail    = %d\n", pElem->wep.WepDecryptFail));
    
        /* AES */
        WLAN_OS_REPORT(("------------  AES Statistics --------------\n"));
        WLAN_OS_REPORT(("AesEncryptFail      = %d\n", pElem->aes.AesEncryptFail));
        WLAN_OS_REPORT(("AesDecryptFail      = %d\n", pElem->aes.AesDecryptFail));
        WLAN_OS_REPORT(("AesEncryptPackets   = %d\n", pElem->aes.AesEncryptPackets));
        WLAN_OS_REPORT(("AesDecryptPackets   = %d\n", pElem->aes.AesDecryptPackets));
        WLAN_OS_REPORT(("AesEncryptInterrupt = %d\n", pElem->aes.AesEncryptInterrupt));
        WLAN_OS_REPORT(("AesDecryptInterrupt = %d\n", pElem->aes.AesDecryptInterrupt));
    
        /* Events */
        WLAN_OS_REPORT(("------  Events  -------------------\n"));
        WLAN_OS_REPORT(("Calibration   = %d\n", pElem->event.calibration));
        WLAN_OS_REPORT(("rxMismatch    = %d\n", pElem->event.rxMismatch));
        WLAN_OS_REPORT(("rxMemEmpty    = %d\n", pElem->event.rxMemEmpty));
    
       /* PsPoll/Upsd */ 
        WLAN_OS_REPORT(("----------- PsPoll / Upsd -----------\n"));
        WLAN_OS_REPORT(("psPollTimeOuts     = %d\n",pElem->ps.psPollTimeOuts));
        WLAN_OS_REPORT(("upsdTimeOuts       = %d\n",pElem->ps.upsdTimeOuts));
        WLAN_OS_REPORT(("upsdMaxAPturn      = %d\n",pElem->ps.upsdMaxAPturn));
        WLAN_OS_REPORT(("psPollMaxAPturn    = %d\n",pElem->ps.psPollMaxAPturn));
        WLAN_OS_REPORT(("psPollUtilization  = %d\n",pElem->ps.psPollUtilization));
        WLAN_OS_REPORT(("upsdUtilization    = %d\n",pElem->ps.upsdUtilization));
    
    
    
    	/* Calibration */
    	WLAN_OS_REPORT(("----------- Calibrations -------------\n"));
    	WLAN_OS_REPORT(("calStateFail         	= %d\n", pElem->radioCal.calStateFail));
    	WLAN_OS_REPORT(("initCalTotal   		= %d\n", pElem->radioCal.initCalTotal));   	 
    	WLAN_OS_REPORT(("initRadioBandsFail   	= %d\n", pElem->radioCal.initRadioBandsFail));
    	WLAN_OS_REPORT(("initRxIqMmFail   		= %d\n", pElem->radioCal.initRxIqMmFail));
    	WLAN_OS_REPORT(("initSetParams   		= %d\n", pElem->radioCal.initSetParams));
    	WLAN_OS_REPORT(("initTxClpcFail   		= %d\n", pElem->radioCal.initTxClpcFail));	
    	WLAN_OS_REPORT(("tuneCalTotal   		= %d\n", pElem->radioCal.tuneCalTotal));	
    	WLAN_OS_REPORT(("tuneDrpwChanTune		= %d\n", pElem->radioCal.tuneDrpwChanTune));
    	WLAN_OS_REPORT(("tuneDrpwLnaTank		= %d\n", pElem->radioCal.tuneDrpwLnaTank));
    	WLAN_OS_REPORT(("tuneDrpwPdBufFail		= %d\n", pElem->radioCal.tuneDrpwPdBufFail));
    	WLAN_OS_REPORT(("tuneDrpwRTrimFail		= %d\n", pElem->radioCal.tuneDrpwRTrimFail));
    	WLAN_OS_REPORT(("tuneDrpwRxDac			= %d\n", pElem->radioCal.tuneDrpwRxDac));
    	WLAN_OS_REPORT(("tuneDrpwRxIf2Gain		= %d\n", pElem->radioCal.tuneDrpwRxIf2Gain));
    	WLAN_OS_REPORT(("tuneDrpwRxTxLpf		= %d\n", pElem->radioCal.tuneDrpwRxTxLpf));
    	WLAN_OS_REPORT(("tuneDrpwTaCal			= %d\n", pElem->radioCal.tuneDrpwTaCal));
    	WLAN_OS_REPORT(("tuneDrpwTxMixFreqFail	= %d\n", pElem->radioCal.tuneDrpwTxMixFreqFail));
    	WLAN_OS_REPORT(("tuneRxAnaDcFail   		= %d\n", pElem->radioCal.tuneRxAnaDcFail));		
    	WLAN_OS_REPORT(("tuneRxIqMmFail   		= %d\n", pElem->radioCal.tuneRxIqMmFail));
    	WLAN_OS_REPORT(("tuneTxClpcFail   		= %d\n", pElem->radioCal.tuneTxClpcFail));
    	WLAN_OS_REPORT(("tuneTxIqMmFail   		= %d\n", pElem->radioCal.tuneTxIqMmFail));
    	WLAN_OS_REPORT(("tuneTxLOLeakFail   	= %d\n", pElem->radioCal.tuneTxLOLeakFail));
    	WLAN_OS_REPORT(("tuneTxPdetFail   		= %d\n", pElem->radioCal.tuneTxPdetFail));
    	WLAN_OS_REPORT(("tuneTxPPAFail   		= %d\n", pElem->radioCal.tuneTxPPAFail)); 
    
    
    
    	/* Power Save Counters */
        WLAN_OS_REPORT(("------  Power management  ----------\n"));
        if(pElem->pwr.RcvdBeaconsCnt != 0)
        {
            WLAN_OS_REPORT(("MissingBcnsCnt    = %d (percentage <= %d) \n", 
                    pElem->pwr.MissingBcnsCnt,
                    ((pElem->pwr.MissingBcnsCnt * 100) / (pElem->pwr.RcvdBeaconsCnt + pElem->pwr.MissingBcnsCnt)) ));
        }
        else
        {
            WLAN_OS_REPORT(("MissingBcnsCnt    = %d (percentage = 0) \n", pElem->pwr.MissingBcnsCnt));
        }
        WLAN_OS_REPORT(("RcvdBeaconsCnt    = %d\n", pElem->pwr.RcvdBeaconsCnt));
        WLAN_OS_REPORT(("ConnectionOutOfSync    = %d\n\n", pElem->pwr.ConnectionOutOfSync));
        WLAN_OS_REPORT(("Single Missed Beacon           = %d\n", (pElem->pwr.ContMissBcnsSpread[0] & 0xFFFF)));
        WLAN_OS_REPORT(("2 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[1] & 0xFFFF)));
        WLAN_OS_REPORT(("3 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[2] & 0xFFFF)));
        WLAN_OS_REPORT(("4 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[3] & 0xFFFF)));
        WLAN_OS_REPORT(("5 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[4] & 0xFFFF)));
        WLAN_OS_REPORT(("6 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[5] & 0xFFFF)));
        WLAN_OS_REPORT(("7 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[6] & 0xFFFF)));
        WLAN_OS_REPORT(("8 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[7] & 0xFFFF)));
        WLAN_OS_REPORT(("9 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[8] & 0xFFFF)));
        WLAN_OS_REPORT((">=10 Continuous Missed Beacons = %d\n\n", (pElem->pwr.ContMissBcnsSpread[9] & 0xFFFF)));
    
        WLAN_OS_REPORT(("RcvdAwakeBeaconsCnt    = %d\n", pElem->pwr.RcvdAwakeBeaconsCnt));
        WLAN_OS_REPORT(("Single Missed Beacon        [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[0] >> 16)));
        WLAN_OS_REPORT(("2 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[1] >> 16)));
        WLAN_OS_REPORT(("3 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[2] >> 16)));
        WLAN_OS_REPORT(("4 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[3] >> 16)));
        WLAN_OS_REPORT(("5 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[4] >> 16)));
        WLAN_OS_REPORT(("6 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[5] >> 16)));
        WLAN_OS_REPORT(("7 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[6] >> 16)));
        WLAN_OS_REPORT(("8 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[7] >> 16)));
        WLAN_OS_REPORT(("9 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[8] >> 16)));
        WLAN_OS_REPORT((">=10 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[9] >> 16)));
    #endif    
}



