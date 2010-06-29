/*
 * TIWhaAdaptCb.cpp
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



/** \file  WhaAdaptCB.cpp 
 *  \brief  Adapt CB between C code and C++ code (i.e TWD to TIWha functions)
 *
 *  \see   
 */
#include "external_inc.h"
#include "TIWhaAdaptCb.h"

#define __FILE_ID__								FILE_ID_145

#define pTIWha (reinterpret_cast<TIWha*>(hTIWha))



/** 
 * \fn     ConnectBus
 * \brief  
 *          ConnectBus Callback called in two contexts:  
 *          1. From Bus driver context on an A-sync bus init transaction
 *          2. From MWlanDfcClient context, on a Sync bus init transaction
 * \note
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::ConnectBus (TI_HANDLE hTIWha)
{
    pTIWha->ConnectBusCb ((WHA::TStatus)0); 
}

void TIWhaAdaptCB::ConnectBusDFC (TI_HANDLE hTIWha)
{    
    pTIWha->pConnectDfcClient->pConnectDfc->Enqueue(*(pTIWha->pConnectDfcClient) ,(TInt)hTIWha);
}


/** 
 * \fn     InitHw
 * \brief  
 *              Init HW Callback called after all software init is done  
 *              and NVS configuration was saved as defaults, will download the FW
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::InitHw (TI_HANDLE hTIWha, TI_STATUS status)
{
    pTIWha->InitHwCb ((WHA::TStatus)status);    
}

/** 
 * \fn     InitFw
 * \brief  
 *             Init FW Callback called after Fw was downloaded will configure the FW
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::InitFw (TI_HANDLE hTIWha, TI_STATUS status)
{
    pTIWha->InitFwCb ((WHA::TStatus)status);    
}

/** 
 * \fn     InitFw
 * \brief  
 *              Config  FW Callback will set the defaults
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::ConfigFw (TI_HANDLE hTIWha, TI_STATUS status)
{
    pTIWha->ConfigFwCb ((WHA::TStatus)status);    
}

/** 
 * \fn     InitFail
 * \brief  
 *              
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::InitFail (TI_HANDLE hTIWha, TI_STATUS status)
{    
    pTIWha->InitFailCb ((WHA::TStatus)status);    
}

/** 
 * \fn     ScanComplete
 * \brief  
 *              Scan Complete Callback
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::ScanComplete (TI_HANDLE hTIWha, EScanResultTag eTag,TI_UINT32 uResultCount, 
					                                    TI_UINT16 SPSStatus, TI_BOOL TSFError, TI_STATUS ScanStatus,
					                                    TI_STATUS PSMode)
{
	/* save the Scan result and PS for furder use */
	pTIWha->aScanResultCount = uResultCount;
	pTIWha->aPsMode = PSMode;
	pTIWha->aScanStatus = ScanStatus;

#ifdef TI_DBG
	WLAN_OS_REPORT(("ScanComplete -- uResultCount = %d",uResultCount));
    WLAN_OS_REPORT(("ScanComplete -- aScanResultCount = %d, SentScanResult= %d",pTIWha->aScanResultCount, pTIWha->SentScanResult));
    WLAN_OS_REPORT(("ScanComplete -- ScanStatus = %d, PSMode= %d",ScanStatus, PSMode));
#endif

    /* Pass to TIWha only relevant parameters */
	if (pTIWha->aScanResultCount <= pTIWha->SentScanResult)
	{
		//os_printf("TIWhaAdaptCB::ScanComplete Call ScanCompleteCb");
		pTIWha->ScanCompleteCb ( (WHA::TStatus)ScanStatus,  (E80211PsStatus)PSMode);
	}
}

/** 
 * \fn     SetPsModeComplete
 * \brief  
 *              Power Save Complete Callback
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::SetPsModeComplete             (TI_HANDLE hTIWha, TI_UINT8 PSMode, TI_UINT8 transStatus)
{
    pTIWha->SetPsModeCompleteCb (PSMode, transStatus);
}

/** 
 * \fn     SetPsModeCompleteDummy
 * \brief  
 *              Power Save Complete Dummy Callback
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::SetPsModeCompleteDummy        (TI_HANDLE hTIWha, TI_UINT8 PSMode, TI_UINT8 transStatus)
{
}

/** 
 * \fn     FailureIndication
 * \brief  
 *         Failure Indication Callback
 * \note   call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::FailureIndication             (TI_HANDLE hTIWha, EFailureEvent failureEvent)
{    
    /* Save the failure event reason, so we will have after we context swith to iFailureDfc */
    pTIWha->iFailureEvent = failureEvent;
    /* Register FailureDfcClient to handle the failure from a different context */
    pTIWha->pFailureDfcClient->pFailureDfc->Enqueue(*(pTIWha->pFailureDfcClient) ,(TInt)hTIWha);            
}


/** 
 * \fn     FailureIndicationDFC
 * \brief  
 *         Failure Indication DFC Callback - Called from MWlanDfcClient context
 * \note
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::FailureIndicationDFC (TI_HANDLE hTIWha)
{    
    pTIWha->FailureIndicationCb(pTIWha->iFailureEvent);
}

/** 
 * \fn     ConnectionTimeOut
 * \brief  
 *              End of connection phase
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::ConnectionTimeOut             (TI_HANDLE hTIWha)
{
    pTIWha->ConnectionTimeOut ();
}

/** 
 * \fn     InitializeAfterTimer
 * \brief  call second part of initialization after timer expired
 *          
 * \note    
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::InitializeAfterTimer           (TI_HANDLE hTIWha)
{
    pTIWha->InitializeAfterTimer();
}

/** 
 * \fn     RcpiIndication
 * \brief  
 *              RCPI Indication Callback
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::RcpiIndication                (TI_HANDLE hTIWha, TI_UINT8* buffer, TI_UINT32 len)
{
    pTIWha->RcpiIndicationCb ( buffer, len);  
}

/** 
 * \fn     LostBssIndication
 * \brief  
 *              Lost Beacon Indication Callback 
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::LostBssIndication             (TI_HANDLE hTIWha)
{
    pTIWha->LostBssIndicationCb ( );  
}

/** 
 * \fn     RegainBssIndication
 * \brief  
 *              Regain Bss Indication Callback 
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::RegainBssIndication           (TI_HANDLE hTIWha)
{
    pTIWha->RegainBssIndicationCb ( );  
}

/** 
 * \fn     btCoexSenseIndication
 * \brief  
 *              
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::btCoexSenseIndication         (TI_HANDLE hTIWha, TI_UINT8* buffer, TI_UINT32 len)
{
    pTIWha->btCoexSenseIndicationCb ( buffer, len);  
}

/** 
 * \fn     btCoexProtectiveIndication
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::btCoexProtectiveIndication    (TI_HANDLE hTIWha, TI_UINT8* buffer, TI_UINT32 len)
{
    pTIWha->btCoexProtectiveIndicationCb ( buffer, len);  
}

/** 
 * \fn     btCoexAvalancheIndication
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::btCoexAvalancheIndication     (TI_HANDLE hTIWha, TI_UINT8* buffer, TI_UINT32 len)
{
    pTIWha->btCoexAvalancheIndicationCb ( buffer, len);  
}

/** 
 * \fn     GenericCommandResponse
 * \brief  
 *              CB for every CMD without a specific CB
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::GenericCommandResponse (TI_HANDLE hTIWha, TI_UINT16 CmdType, TI_UINT16 CmdID, TI_UINT32 aStatus)
{
    pTIWha->GenericCommandResponseCb ( CmdType, CmdID, aStatus);
}

/** 
 * \fn     DummyResponse
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::DummyResponse (TI_HANDLE Dummy, TI_STATUS aStatus)
{
}

/** 
 * \fn     ScanResponse
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::ScanResponse(TI_HANDLE hTIWha,TI_STATUS aStatus)
{
    pTIWha->ScanResponseCb ( (WHA::TStatus)aStatus);
}

/** 
 * \fn     JoinComplete
 * \brief  
 *              Actually join complete...
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::JoinComplete                  (TI_HANDLE hTIWha)
{
    pTIWha->JoinCompleteCb ();
}

/** 
 * \fn     StopScanResponse
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::StopScanResponse              (TI_HANDLE hTIWha, TI_STATUS aStatus)
{
    pTIWha->StopScanResponseCb ( (WHA::TStatus)aStatus);
}

/** 
 * \fn     SetPsModeResponse
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::SetPsModeResponse             (TI_HANDLE hTIWha, TI_UINT8 aStatus)
{
    pTIWha->SetPsModeResponseCb (aStatus);
}

/** 
 * \fn     SetPsModeResponseDummy
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::SetPsModeResponseDummy        (TI_HANDLE hTIWha, TI_UINT8 aStatus)
{
}

/** 
 * \fn     ReadMIBResponse
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::ReadMIBResponse               (TI_HANDLE hTIWha, TI_UINT16 aStatus,  void *InterrogateParamsBuf)
{
    pTIWha->ReadMIBResponseCb ( aStatus, InterrogateParamsBuf);
}

/** 
 * \fn     MeasureResponse
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::MeasureResponse               (TI_HANDLE hTIWha, TI_UINT16 aStatus)
{ 
}

/** 
 * \fn     StopMeasureResponse
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::StopMeasureResponse           (TI_HANDLE hTIWha, TI_UINT16 aStatus)
{ 
}

/** 
 * \fn     StopMeasureResponse
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::AddKeyResponse			(TI_HANDLE hTIWha, TI_STATUS aStatus)
{
    pTIWha->AddKeyResponseCb (aStatus);
    
}

/* RX CallBacks */
/** 
 * \fn     RequestForBuffer
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
ERxBufferStatus TIWhaAdaptCB::RequestForBuffer (TI_HANDLE hTIWha, void **pWbuf, TI_UINT16 aLength, TI_UINT32 uEncryptionFlag)
{
    return pTIWha->RequestForBufferCb (pWbuf, aLength, uEncryptionFlag);
}

/** 
 * \fn     ReceivePacket
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::ReceivePacket (TI_HANDLE        hTIWha,
                             const void    *aFrame)
{
	pTIWha->ReceivePacketCb (aFrame);
}


/** 
 * \fn     ReceivePacketWhileScan
 * \brief  
 * \note    Use while Scan only to count the scan resault
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::ReceivePacketWhileScan(TI_HANDLE hTIWha, const void *aFrame)
{
        RxIfDescriptor_t*  pRxParams = (RxIfDescriptor_t*)aFrame;

	pTIWha->ReceivePacketCb (aFrame);

	if (pRxParams->packet_class_tag == TAG_CLASS_BCN_PRBRSP)
	{
		pTIWha->SentScanResult++;
        if (pTIWha->aScanResultCount <= pTIWha->SentScanResult)
		{
			/* Send the scan complete CB */
			#ifdef TI_DBG
				WLAN_OS_REPORT(("ReceivePacketWhileScan Call Scan Complete"));
			#endif
			pTIWha->ScanCompleteCb (pTIWha->aScanStatus, (E80211PsStatus)pTIWha->aPsMode);			
		}
	}
}

/** 
 * \fn     ReceivePacket
 * \brief  Frame was written to FW 
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::TxXfer ( TI_HANDLE hTIWha, TTxCtrlBlk *pPktCtrlBlk)
{
    pTIWha->TxXferCb (pPktCtrlBlk);
}

/** 
 * \fn     TxComplete
 * \brief  Frame was transmitted over the air (or failed to be transmitted)
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::TxComplete ( TI_HANDLE hTIWha, TxResultDescriptor_t *pTxResultInfo, TI_UINT32 backpressure)
{
    pTIWha->TxCompleteCb ( pTxResultInfo, backpressure);
}


/** 
 * \fn     TxBipResponse
 * \brief   TxBIP Response Callback 
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::TxBipResponse (TI_HANDLE hTIWha, TI_STATUS aStatus,  void *InterrogateParamsBuf)
{
	// Convert from FW result to NVS file All manipulation is done on buffer
	// supplied by Symbian
	os_printf("BipResponse call ConvertBip2Nvs");
    /* ConvertRxBip2Nvs() will update only the Tx part of the NVS */
	pTIWha->ConvertTxBip2Nvs(InterrogateParamsBuf); 

	// After convertion finished calling Regular PLT response to indicate that NVS file is ready
	PltResponse(hTIWha,aStatus,InterrogateParamsBuf);
}


/** 
 * \fn     RxBipResponse
 * \brief   RxBIP Response Callback 
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::RxBipResponse (TI_HANDLE hTIWha, TI_STATUS aStatus,  void *InterrogateParamsBuf)
{
	// Convert from FW result to NVS file All manipulation is done on buffer
	// supplied by Symbian
    /* ConvertRxBip2Nvs() will update only the Rx part of the NVS */
	os_printf("BipResponse call ConvertBip2Nvs");
	pTIWha->ConvertRxBip2Nvs(InterrogateParamsBuf); 

	// After convertion finished calling Regular PLT response to indicate that NVS file is ready
	PltResponse(hTIWha,aStatus,InterrogateParamsBuf);
}

/** 
 * \fn     RxStatResponse
 * \brief   Get Statistic Response Callback 
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::RxStatResponse				  (TI_HANDLE hTIWha, TI_STATUS aStatus,  void *InterrogateParamsBuf)
{

	//normalize the RSSI to RCPI in 0.5DB scale

	/*valus are in db/8
	so if RCPI = (db +110) * 2 then to have it in valus of db/2 -> RCPI = (db/4 +220) */

	if ((*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.AverageRssi > 0)
	{
		(*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.AverageRssi = 220;
	}
	else
	{
		if ((*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.AverageRssi/8 < -110)
		{
			(*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.AverageRssi = 0;
		}
		else
		{
			(*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.AverageRssi=
			(*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.AverageRssi/4 +220;
		}
	}


	/* On RX Statistic we move thru this function to print the RX statistic */

	WLAN_OS_REPORT(("Received Valid Packet no.: %d(0x%x)\n", (*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.ReceivedValidPacketsNumber,(*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.ReceivedValidPacketsNumber));
	WLAN_OS_REPORT(("Received FCS Error Packet no.: %d(0x%x)\n", (*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.ReceivedFcsErrorPacketsNumber,(*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.ReceivedFcsErrorPacketsNumber));
	WLAN_OS_REPORT(("Received Address missmatch Error Packet no.: %d(0x%x)\n", (*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.ReceivedPlcpErrorPacketsNumber,(*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.ReceivedPlcpErrorPacketsNumber));
	WLAN_OS_REPORT(("Sequence Number Missing Count: %d(0x%x)\n", (*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.SeqNumMissCount,(*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.SeqNumMissCount));
	WLAN_OS_REPORT(("Average SNR: %d(0x%x)\n", (*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.AverageSnr,(*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.AverageSnr));
	WLAN_OS_REPORT(("Average RSSI: %d(0x%x)\n", (*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.AverageRssi,(*((RadioRxStatistics*)InterrogateParamsBuf)).oRxPathStatistics.AverageRssi));
	WLAN_OS_REPORT(("Base Packet ID: %d(0x%x)\n", (*((RadioRxStatistics*)InterrogateParamsBuf)).oBasePacketId,(*((RadioRxStatistics*)InterrogateParamsBuf)).oBasePacketId));
	WLAN_OS_REPORT(("Number of Packets: %d(0x%x)\n", (*((RadioRxStatistics*)InterrogateParamsBuf)).ioNumberOfPackets,(*((RadioRxStatistics*)InterrogateParamsBuf)).ioNumberOfPackets));
	WLAN_OS_REPORT(("Number of Missed Packets: %d(0x%x)\n", (*((RadioRxStatistics*)InterrogateParamsBuf)).oNumberOfMissedPackets,(*((RadioRxStatistics*)InterrogateParamsBuf)).oNumberOfMissedPackets));

	PltResponse(hTIWha,aStatus,InterrogateParamsBuf);
}


/** 
 * \fn     RunRssiResponse
 * \brief   Print the RSSI value 
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::RunRssiResponse (TI_HANDLE hTIWha, TI_STATUS aStatus,  void *InterrogateParamsBuf)
{
    /* Print the Rssi value, the value is RSSI * 8 */
    WLAN_OS_REPORT (("PltResponseCb : FLALI Command Respons -- Status = %d",((TTestCmdFreeRSSI*)InterrogateParamsBuf)->oRadioStatus));
    WLAN_OS_REPORT (("PltResponseCb : FLALI Command Respons -- RSSI = %d",((TTestCmdFreeRSSI*)InterrogateParamsBuf)->RSSIVal));

	PltResponse(hTIWha,aStatus,InterrogateParamsBuf);
}

/** 
 * \fn     PltResponse
 * \brief   PLT Response Callback 
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::PltResponse (TI_HANDLE hTIWha, TI_STATUS aStatus,  void *InterrogateParamsBuf)
{
    pTIWha->PltResponseCb ( aStatus, InterrogateParamsBuf);
}


/** 
 * \fn     ReadMemResponse
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::ReadMemResponse          (TI_HANDLE hTIWha, TFwDebugParams* params)
{
    pTIWha->ReadMemResponseCb (params);
	
}


/** 
 * \fn     WriteMemResponse
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::WriteMemResponse         (TI_HANDLE hTIWha, TFwDebugParams* params)
{
    pTIWha->WriteMemResponseCb (params);
}


/** 
 * \fn     ReadDot11StationIdCb
 * \brief  
 * \note    call  ReadMIBResponseCb after timer context change
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::ReadDot11StationIdCb         (TI_HANDLE hTIWha)
{
    /* Call ReadMIBResponse from the ReadDot11StationIdCb since we have the CHIP ID */
    pTIWha->ReadMIBResponseCb ( WHA::KSuccess, pTIWha->getMacAddress());
}

/** 
 * \fn     RxMemFailTimerCb
 * \brief  
 * \note    call  TIWha::RxMemFailTimerCb
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::RxMemFailTimerCb         (TI_HANDLE hTIWha)
{
    pTIWha->RxMemFailTimerCb ();
}

/** 
 * \fn     AvrRssiReadResponse
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::AvrRssiReadResponse (TI_HANDLE hTIWha, TI_UINT16 MboxStatus, ACXRoamingStatisticsTable_t* roamingStatistics)
{
    pTIWha->TWD_AvrRssiReadCB (hTIWha, MboxStatus, roamingStatistics);
}


/** 
 * \fn     StatisticsReadResponse
 * \brief  
 * \note    call member function of TIWha class
 * \param  hTIWha - handle to TIWha class
 * \return  
 * \sa      
 */ 
void TIWhaAdaptCB::StatisticsReadResponse (TI_HANDLE hTIWha, TI_UINT16 MboxStatus, ACXStatistics_t* pElem)
{
    pTIWha->TWD_StatisticsReadCB (hTIWha, MboxStatus, pElem);
}
