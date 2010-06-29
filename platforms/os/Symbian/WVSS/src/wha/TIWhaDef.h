/*
 * TIWhaDef.h
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


/*--------------------------------------------------------------------------*/
/* Module:      TIWhaDef.H*/
/**/
/* Purpose:     This module defines unified interface to use by the PhysicalChannel*/
/**/
/*--------------------------------------------------------------------------*/

#ifndef _TIWhaDef_
#define _TIWhaDef_

#include "external_inc.h"
#include "gendebug.h"

#include <wlanwha.h>
#include <wlanwha_types.h>
#include <wlanspia.h>
#include <wlanhpa.h>
#include <wlanosa.h>

/*******************************************************************************
 * stuff for C file linking.                                                   *
 *******************************************************************************/
extern "C" 
{
#include "report.h"
#include "tidef.h"
#include "osApi.h"
#include "TWDriverInternal.h"
#include "TWDriverRate.h"
#include "TWDriver.h"
#include "802_11Defs.h" 
#include "coreDefaultParams.h"
}

/* Device Name */

#define DEVICE_NAME _L("wlan.phys")


#include "TIWhaPLTDef.h"

#ifdef GEM_SUPPORT
    #include "TIWhaGemDef.h"
#endif /* GEM_SUPPORT */


/****************************** DEBUG MIBS *****************************************/
#ifdef TI_TEST
    #define SET_POWER_LEVEL             0x2003
    #define TWD_FW_PRINT_STATISTICS     0x3100
    #define QOS_CHANGE_QEUEU            0x4001
    #define EMIBPltTest                 0x3001
    #define AVR_RSSI                    0x3101
#endif /* TI_TEST */
/***********************************************************************************/


/****************************************************************************
*       Constants                                                           *
*****************************************************************************/

#define MAX_XFER_WAITING_PACKETS    50

/* Stalling the CPU for 400Ms in case of a failure */
#define STALL_ON_FAILURE 400000

#define STALL_ON_RECONNECT 1000

#ifdef ENHANCE_PROB_SCAN
    #define PB_FACTOR_FOR_SG    3
    #define MIN_TIME_FACTOR_FOR_SG  2
    #define MAX_TIME_FACTOR_FOR_SG  2
#endif

/* Maximum num of ssid in scan*/
#define TIWha_SCAN_MAX_SSID_NUM                                  1

/* Max size of ReadMibResponse according to the structures that are used for the Mib results */
#define TIWha_MAX_READ_MIB_BUFFER \
    TI_MAX(sizeof(WHA::SstatisticsTable),TI_MAX (sizeof(WHA::Sdot11StationId),sizeof(WHA::SstatisticsTable)))

/* Maximum driver version length */
#define TIWha_MAX_DRV_VER_LEN                                    64

/* Maximum firmware version length */
#define TIWha_MAX_FRM_VER_LEN                                    32

#define TIWha_NO_BUFFER_ALIGNMENT_SETTING                        1

/* TIWha max table size of the security keys */
#define TIWha_MAX_PRIVACY_KEY_INDEX                              9

#define TIWha_MAX_SSID_LEN                                       32

#define TIWha_MAX_RCPI_VAL                                       220

#define TIWha_MIN_RCPI_VAL                                       0

#define TIWha_MIN_RSSI_VAL                                       -110

#define TIWha_MAX_RSSI_VAL                                       -45

#define TRIGGER_LOW_RSSI_PACING                                  1000

#define TIWha_DEFAULT_DTIM_PERIOD                   0x3

/* An extra 4 bytes for alginment */
#define PAYLOAD_ALIGN_PAD_BYTES     4

/* 
 * TIWha supported capabilities bit mask definition. 
 * The bit mask is held in the SSettings table                                                      
 */
#define TIWha_RECEIVE_PACKET_BIT_MASK_SUPPORT                    0

/* Multi Buffer */
#ifdef MB_ENABLE
    #define MAX_WHA_RX_BUFFERS                                       14
#else
    #define MAX_WHA_RX_BUFFERS                                       1
#endif

/* Used in order to signal the correct security bit of aFlags, that is sent to upper layer ReceivePacket() */
#define WHA_WEP                                                 0x10000
#define WHA_TKIP                                                0x20000
#define WHA_AES                                                 0x30000
#ifdef GEM_SUPPORT
    #define WHA_GEM                                                 0x8000
#endif

#define TIWha_RATE_BIT_MASK_SUPPORT   ((WHA::KRate1Mbits)         | \
                                                            (WHA::KRate2Mbits)         | \
                                                            (WHA::KRate5_5Mbits)     | \
                                                            (WHA::KRate6Mbits)         | \
                                                            (WHA::KRate9Mbits)         | \
                                                            (WHA::KRate11Mbits)        | \
                                                            (WHA::KRate12Mbits)        | \
                                                            (WHA::KRate18Mbits)        | \
                                                            (WHA::KRate24Mbits)        | \
                                                            (WHA::KRate36Mbits)        | \
                                                            (WHA::KRate48Mbits)        | \
                                                            (WHA::KRate54Mbits))       


#define TIWha_CAPABILITY_BIT_MASK_SUPPORT   ((WHA::SSettings::KExpiryTimeParam)   | \
                                                                    (WHA::SSettings::KDot11SlotTime)           | \
                                                                    (WHA::SSettings::KRadioMeasurements)     | \
                                                                    (WHA::SSettings::KWep16ByteKey)         | \
                                                                    (WHA::SSettings::KScanChannelTimes))

#ifdef GEM_SUPPORT
    #define TIWha_CAPABILITY_GEM_SUPPORT        GemBit
#endif /* GEM_SUPPORT */

#ifdef MB_ENABLE
    #define TIWha_CAPABILITY_MB_SUPPORT     WHA::SSettings::KMultipleRxBuffers
#endif /* MB_ENABLE */


#ifdef HT_SUPPORT
    #define TIWha_CAPABILITY_HT_OPERATION       WHA::SSettings::KHtOperation
    #define RX_QUEUE_WIN_SIZE               8
    #define HT_CONTROL_FILED_SIZE           4

    #define TID0    BIT_0
    #define TID1    BIT_1
    #define TID2    BIT_2
    #define TID3    BIT_3
    #define TID4    BIT_4
    #define TID5    BIT_5
    #define TID6    BIT_6
    #define TID7    BIT_7

#endif /* HT_SUPPORT */

#define TIWha_CAPABILITY_AUTO_RATE          WHA::SSettings::KAutonomousRateAdapt

#define TIWha_BAND_BIT_MASK_SUPPORT      ((WHA::KBand2dot4GHzMask) | (WHA::KBand5GHzMask))          

/* max table size of the security keys */
#define TIWLANWHA_MAX_PRIVACY_KEY_INDEX                              9


#define     MAX_INFO_ELEMENT_LEN    (32)

#ifndef BTCOEX_DEFAULT_PROFILE
    #define BTCOEX_DEFAULT_PROFILE BtCoexProfData
#endif

#ifndef BTCOEX_DEFAULT_MODE
    #define BTCOEX_DEFAULT_MODE SG_OPPORTUNISTIC
#endif

/* For connection phase we will use this parameter to make sure we can drop down the rates till 1Mbps */
#define TIWHA_TX_FAIL_LOW_TH_FOR_CONNECTION 2

/* After connection we use the default definition */
#define TIWHA_TX_FAIL_LOW_TH_AFTER_CONNECTION RATE_MGMT_TX_FAIL_LOW_TH


/********* Typedefs ********/

/* Rx quality triggers indexes */
typedef enum
{
    TRIGGER_EVENT_LOW_RSSI   = 0,
    TRIGGER_EVENT_LOW_SNR    = 1,
    TRIGGER_EVENT_HIGH_TX_PW = 2,
    TRIGGER_EVENT_LOW_TX_PW  = 3,
    TRIGGER_EVENT_BG_SCAN    = 4,
    TRIGGER_EVENT_USER_0     = 5,
    TRIGGER_EVENT_USER_1     = 6,
    TRIGGER_EVENT_MAX        = 7

}ETriggerEventIndex;

typedef enum
{
    vendorSpecificConfig_idle,
    vendorSpecificConfig_btCoexMode,
    vendorSpecificConfig_btCoexEnabled,
} vendorSpecificConfigEnum;


typedef struct  
{
    void*   pFrame[MAX_XFER_WAITING_PACKETS];              /* Hold the last sent frame to TnetDrv                    */
    TUint32  uCurrSend;              /* Hold the frame index that is going to be sent    (0/1) */
    TUint32  uCurrHandle;            /* Hold the frame index that is going to be handled (0/1) */
    TUint32  uHeaderToDataOffset[MAX_XFER_WAITING_PACKETS]; /* Gap between Header and Data due to security alignment  */
    TBool    bIsQosHeader[MAX_XFER_WAITING_PACKETS];        /* determine whether we have 24 or 26 bytes of header     */
} TTxAlign;


struct TIFirmwareStructure
{
    const TUint8*   pFw;
    TUint32             fwLength;
    const TUint8*   pNvs;
    TUint32             nvsLength;
};


typedef struct
{
    TUint8  eleId;
    TUint8  eleLen;
} TDot11InfoElemHdr; 


typedef struct
{
    TDot11InfoElemHdr   hdr;    
    TUint8              info[MAX_INFO_ELEMENT_LEN];             

} TInformationElement; 


struct TOsContext
{
    MWlanOsa* hOsa;
    WlanSpia* hSpia;
    WlanHpa* hHpa;
};

/* Use the next macros to get MWlanOsa /WlanSpia / WlanHpa from hOS */
#define GET_OSA(hOS)  (((TOsContext*)(hOS))->hOsa)
#define GET_SPIA(hOS) (((TOsContext*)(hOS))->hSpia)
#define GET_HPA(hOS) (((TOsContext*)(hOS))->hHpa)


struct TwdCtrl
{
    /* OS Module (contains both OSA and SPIA)*/
    TOsContext         tOsContext;

    /* HAL handles */
    TI_HANDLE         hApp;
    TI_HANDLE         hReport;
    TI_HANDLE         hTWD;
    TI_HANDLE         hContext;
    TI_HANDLE         hTimer;
    TI_HANDLE         hTxnQ;

    /* The read MIB buffer to be copied to from HAL Cmd MBOX */
    TUint8          readMIBBuf [MAX_CMD_PARAMS];
    /* Current read MIB ID */
    WHA::TMib       currentReadMibID;
    /* Scan parameters */
    TScanParams     scanParams;
    /* TX power level table */
    TInt8           txPowerLevelTable [RADIO_BAND_NUM_OF_BANDS][NUM_POWER_LEVELS];
    /* Current radio band */
    WHA::TBand      radioBand;
    /* PLT specific test that is running right now */
    ETestCmdID      ePlt;
    /* Array of configured privacy keys */
    TSecurityKeys   privacyKey [TIWLANWHA_MAX_PRIVACY_KEY_INDEX];
    /* User configured sleep mode */
    WHA::TSleepMode sleepMode;
    /* Send response flag */
    TBool           bResponse;
    /* Tx frame parameters. used for security alignment */
    TTxAlign        TxAlign;
    /* Twd init parameters . used for setting the defaults for TWD */
    TTwdInitParams  twdInitParams;
    /* Report init parameters */
    TReportInitParams  report_init;
    /* mac address */
    TMacAddr    	pMacAddr;
    /* Add Scan parameters */
    TScanParams     iScanParams; 

    /* Save the last security mode configuration */
    ECipherSuite ePairwiseKeyMode;
    ECipherSuite eGroupKeyMode;

    /* Save BSS type for Tx packets */
    WHA::TOperationMode bssType;

/***************************************************************************/


} ;  


/***************************** HT Definitions ******************************/

#define MCS_TABLE   10

/* MCS Rate for WHA */

#define WHA_MCS_0   BIT_0
#define WHA_MCS_1   BIT_1
#define WHA_MCS_2   BIT_2
#define WHA_MCS_3   BIT_3
#define WHA_MCS_4   BIT_4
#define WHA_MCS_5   BIT_5
#define WHA_MCS_6   BIT_6
#define WHA_MCS_7   BIT_7

struct TApCapabilities
{
    TBool       iHTSupport;       /* 0 - HT disable, 1 - HT Enable */

    TMacAddr    iMacAddr;        /* The MAC address of the peer station. This field is valid only in IBSS mode; 
                                    in BSS mode, the host driver sets this field to 0xFFFFFFFFFFFF */

    TUint8      iRxStbc;        /* STBC reception support: 0 - Not Supported, 1 - Supported for one spatial stream,
                                   2 – Supported for one and two spatial streams, 3 – Supported for one, two and three spatial streams */

    TUint8      iHtMaxAmpdu;    /* maximum A-MPDU size: 0 – 8191 octets. 1 – 16383 octets. 2 – 32767 octets. 3 – 65535 octets. */

    TUint32     iHtCapabilities; /* bit mask for HT capabilities: */

    TUint8      iHtMcsSet[MCS_TABLE]; /* A bit mask that defines the supported MCSs.
                                        The least significant bit of the byte[0] corresponds to MCS 0,
                                        the least significant bit of byte[1] corresponds to MCS 8 and so on */

    TUint8      iHtAmpduSpacing;        /* the A-MPDU spacing: 0 – no restriction.
                                        • 1 – ¼ microseconds.
                                        • 2 – ½ microseconds.
                                        • 3 – 1 microsecond.
                                        • 4 – 2 microseconds.
                                        • 5 – 4 microseconds.
                                        • 6 – 8 microseconds.
                                        • 7 – 16 microseconds. */

    TUint8      iHtMcsFeedback;         /* the MCS feedback capability: 0 – no MCS feedback. 1 – reserved. 
                                           2 – unsolicited MCS feedback only. 
                                           3 – both solicited and unsolicted MCS feedback.*/

    TUint32     iBeamFormingCapabilities; /* Transmit Beam Forming capabilities. */

    TUint8      iAntennaSelection;        /* Antenna selection capabilities. */

    TUint8      Padding[3];
};


struct THtInformation 
{
    TUint32     iHTInformation;     /* HT information bit mask.
                                    Bit 0 Non-Greenfield HT STAs present in the BSS.
                                    Bit 1 Transmit burst limited.
                                    Bit 2 PCO active.
                                    Bit 3 RIFS Permitted.
                                    Bit 4 Dual CTS protection required.
                                    Bit 5 Secondary beacon transmitted
                                    Bit 6-31 Reserved. */


    TUint8      iHtMcsSet[MCS_TABLE]; /* A bit mask that defines the supported MCSs.
                                        The least significant bit of the byte[0] corresponds to MCS 0,
                                        the least significant bit of byte[1] corresponds to MCS 8 and so on */

    TUint8      iHtProtection;      /* the HT protection mode:
                                    0 – operation mode 0. 1 – operation mode 1 (non-member protection mode).
                                    2 – operation mode 2. 3 – operation mode 3 (mixed mode). */

    TUint8      iHtSecChannelOffset; /* the secondary channel offset:
                                     0 – no secondary channel. 1 – secondary channel is above the primary channel.
                                     3 - secondary channel is below the primary channel. */

    TUint8      iHtChannelWidth;    /* the channel width:
                                    0 – only 20 MHz channel width supported or used.
                                    1 – both 40 MHz and 20 MHz channel width supported or used. */

    TUint8      Padding[3];
};



struct TBABitMask
{
    TUint8      iTxTidBASupport;    /* A bit-map containing the block ACK usage status for the tx direction */

    TUint8      iRxTidBASupport;    /* A bit-map containing the block ACK usage status the rx direction */

    TUint8      Padding[2];
};


#define GENERAL_RADIO_PARAM_LEN         58

#define STATIC_RADIO_PARAM_LEN          17
#define STATIC_RADIO_PARAM_OFFSET       58

#define DYNAMIC_RADIO_PARAM_LEN         65
#define FEM0_DYNAMIC_RADIO_PARAM_OFFSET 76
#define FEM1_DYNAMIC_RADIO_PARAM_OFFSET 142



typedef struct 
{
    IniFileGeneralParam   tGeneralParams;
    TStatRadioParams      tStatRadioParams;
    TDynRadioParams       tDynRadioParams[NUMBER_OF_FEM_TYPES_E];

}TAutoRadioIniParams;  


typedef struct 
{
	TUint8 len; //maximum length is 14
	TInt8  upperLimit;
	TInt8  values[14]; //this is the maximum length (in rows) of the error table
}TSmartReflexErrTable;

typedef struct
{
    TUint8  SmartReflexState;
	TSmartReflexErrTable tErrorTable[3]; 
}TSmartReflexConfigParams;

typedef struct
{	
	TSmartReflexErrTable errorTable; 
	unsigned short senN_P;
	unsigned short senNRN;
	unsigned short senPRN;
	unsigned short senN_P_Gain;
}TSmartReflexDebugParams;


#endif /* _TIWhaDef_ */
