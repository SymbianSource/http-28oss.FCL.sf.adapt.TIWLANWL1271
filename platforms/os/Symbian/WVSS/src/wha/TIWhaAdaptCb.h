/*
 * TIWhaAdaptCb.h
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



/** \file  TIWhaAdaptCB.h 
 *  \brief  Adapt CB between C code and C++ code (i.e TWD to TIWha functions)
 *
 *  \see   
 */

#ifndef TI_WHA_ADAPT_CB_H
#define TI_WHA_ADAPT_CB_H

#include "TIWha.h"

class TIWhaAdaptCB
{
public:

    /* Connect CB called after that TxnQ_Connect is done */
    static void  ConnectBus (TI_HANDLE hTIWha);    
    static void ConnectBusDFC (TI_HANDLE hTIWha);
    
	/* Init HW Callback called after all software init is done  and NVS configuration was saved as defaults, will download the FW*/
	static void  InitHw (TI_HANDLE hTIWha, TI_STATUS status);
	/* Init FW Callback called after Fw was downloaded will configure the FW*/
	static void InitFw (TI_HANDLE hTIWha, TI_STATUS status);
	/* Config  FW Callback will set the defaults*/
	static void ConfigFw (TI_HANDLE hTIWha, TI_STATUS status);
	
	static void InitFail (TI_HANDLE hTIWha, TI_STATUS status);

	/* Scan Complete Callback */
	static void ScanComplete (TI_HANDLE hTIWha, EScanResultTag eTag,TI_UINT32 uResultCount, 
					                                    TI_UINT16 SPSStatus, TI_BOOL TSFError, TI_STATUS ScanStatus,
					                                    TI_STATUS PSMode);

	/* Power Save Complete Callback */
	static void SetPsModeComplete             (TI_HANDLE hTIWha, TI_UINT8 PSMode, TI_UINT8 transStatus);

	/* Power Save Complete Dummy Callback */
	static void SetPsModeCompleteDummy        (TI_HANDLE hTIWha, TI_UINT8 PSMode, TI_UINT8 transStatus);

	/* Failure Indication Callback */
	static void FailureIndication             (TI_HANDLE hTIWha, EFailureEvent failureEvent);    

        /* Failure Indication DFC Callback */
        static void FailureIndicationDFC             (TI_HANDLE hTIWha);    

        /* Connection time out CB, used to indicate that the connection proccess is over */
        static void ConnectionTimeOut             (TI_HANDLE hTIWha);

        /* Start Initialize */
        static void InitializeAfterTimer          (TI_HANDLE hTIWha);

	/* RCPI Indication Callback */
	static void RcpiIndication                (TI_HANDLE hTIWha, TI_UINT8* buffer, TI_UINT32 len);

	/* Lost Beacon Indication Callback */        
	static void LostBssIndication             (TI_HANDLE hTIWha);

	/* Regain Bss Indication Callback */        
	static void RegainBssIndication           (TI_HANDLE hTIWha);

	/* BtCoexistence Indication CallBacks */
	static void btCoexSenseIndication         (TI_HANDLE hTIWha, TI_UINT8* buffer, TI_UINT32 len);
	static void btCoexProtectiveIndication    (TI_HANDLE hTIWha, TI_UINT8* buffer, TI_UINT32 len);
	static void btCoexAvalancheIndication     (TI_HANDLE hTIWha, TI_UINT8* buffer, TI_UINT32 len);

	/* Init Response Callback */
	static void GenericCommandResponse (TI_HANDLE hTIWha, TI_UINT16 CmdType, TI_UINT16 CmdID, TI_UINT32 aStatus);
	/* Dummy Response Callback - needed in cases that Mib was done but no indication in neede in the UMAC */
	static void DummyResponse (TI_HANDLE Dummy, TI_STATUS aStatus);
	/* Scan Response Callback */
	static void ScanResponse(TI_HANDLE hTIWha,TI_STATUS aStatus);
	/* JoinComplete Callback */
	static void JoinComplete                  (TI_HANDLE hTIWha);
	/* Stop Scan Response Callback */
	static void StopScanResponse              (TI_HANDLE hTIWha, TI_STATUS aStatus);
	/* Power Save Response Callback */
	static void SetPsModeResponse             (TI_HANDLE hTIWha, TI_UINT8 aStatus);
	/* Power Save Response Dummy Callback */
	static void SetPsModeResponseDummy        (TI_HANDLE hTIWha, TI_UINT8 aStatus);
	/* Read MIB Response Callback */
	static void ReadMIBResponse               (TI_HANDLE hTIWha, TI_UINT16 aStatus,  void *InterrogateParamsBuf);
	/* Measure Response Callback */
	static void MeasureResponse               (TI_HANDLE hTIWha, TI_UINT16 aStatus);
	/* Stop Measure Response Callback */
	static void StopMeasureResponse           (TI_HANDLE hTIWha, TI_UINT16 aStatus);
	/* Add key response callback */
	static void AddKeyResponse			( TI_HANDLE hTIWha, TI_STATUS aStatus);

	/* PLT */
	/* PLT Response Callback */
	static void PltResponse                   (TI_HANDLE hTIWha, TI_STATUS aStatus,  void *InterrogateParamsBuf);
	/* Tx BIP Response Callbake */
	static void TxBipResponse                   (TI_HANDLE hTIWha, TI_STATUS aStatus,  void *InterrogateParamsBuf);
        /* Rx BIP Response Callbake */
        static void RxBipResponse                   (TI_HANDLE hTIWha, TI_STATUS aStatus,  void *InterrogateParamsBuf);
	/* RX Statistic Parameters */
	static void RxStatResponse				  (TI_HANDLE hTIWha, TI_STATUS aStatus,  void *InterrogateParamsBuf);

    /* Free Run RSSI Response */
    static void RunRssiResponse (TI_HANDLE hTIWha, TI_STATUS aStatus,  void *InterrogateParamsBuf);

	/* PLT RX PER callbacks */
	static void PltPerGetResultResponse       (TI_HANDLE hTIWha, TI_UINT16 aStatus,  void *InterrogateParamsBuf);
	static void PltPerStartResponse           (TI_HANDLE hTIWha, TI_UINT16 aStatus,  void *InterrogateParamsBuf);
	static void PltPerStopResponse            (TI_HANDLE hTIWha, TI_UINT16 aStatus,  void *InterrogateParamsBuf);
	static void PltPerClearResponse           (TI_HANDLE hTIWha, TI_UINT16 aStatus, void *InterrogateParamsBuf);
	/* Read Memory/Registers Response Callback */
	static void ReadMemResponse          (TI_HANDLE hTIWha, TFwDebugParams* params);
	/* Write Registers Response Callback */
	static void WriteMemResponse         (TI_HANDLE hTIWha, TFwDebugParams* params);
	/* PLT TX calibration */
	static void PltGainGetResponse            (TI_HANDLE hTIWha, TI_UINT16 aStatus,  void *InterrogateParamsBuf);
	static void PltGetNVSUpdateBufferResponse (TI_HANDLE hTIWha, TI_UINT16 aStatus,  void *InterrogateParamsBuf);

	/* RX CallBacks */
    static ERxBufferStatus RequestForBuffer (TI_HANDLE hTIWha, void **pWbuf, TI_UINT16 aLength, TI_UINT32 uEncryptionflag);
    static void ReceivePacket (TI_HANDLE hTIWha, const void *aFrame);
    static void ReceivePacketWhileScan (TI_HANDLE hTIWha, const void *aFrame);

	/* Frame was written to FW */
	static void TxXfer ( TI_HANDLE hTIWha, TTxCtrlBlk *pPktCtrlBlk);

	/* Frame was transmitted over the air (or failed to be transmitted) */
	static void TxComplete ( TI_HANDLE hTIWha, TxResultDescriptor_t *pTxResultInfo, TI_UINT32 backpressure);
	
    static void ReadDot11StationIdCb        (TI_HANDLE hTIWha);

	static void RxMemFailTimerCb         (TI_HANDLE hTIWha);

    static void AvrRssiReadResponse (TI_HANDLE hTIWha, TI_UINT16 MboxStatus, ACXRoamingStatisticsTable_t* roamingStatistics);

    static void StatisticsReadResponse (TI_HANDLE hTIWha, TI_UINT16 MboxStatus, ACXStatistics_t* pElem);                
};



#endif /* TI_WHA_ADAPT_CB_H */
