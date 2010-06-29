/*
 * osdot11XCC.h
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
/* Module:		osDot11XCC.h */
/**/
/* Purpose:		*/
/**/
/*--------------------------------------------------------------------------*/
#ifndef __OSDOT11XCC_H__
#define __OSDOT11XCC_H__

#ifdef XCC_MODULE_INCLUDED

#include  "osDot11.h"

/************************************************************************
 * XCC types  - taken from Funk inc file: fswXCC.h                      *
 ************************************************************************/

/* Rogue AP structure */

typedef struct _OS_XCC_ROGUE_AP_DETECTED
{
	TI_UINT16             FailureReason;
	TMacAddr              RogueAPMacAddress;
	char                  RogueAPName[16];
} OS_XCC_ROGUE_AP_DETECTED, *POS_XCC_ROGUE_AP_DETECTED;

/**/
/*  Auth Success structure*/
/**/

typedef struct _OS_XCC_AUTH_SUCCESS
{
  	OS_802_11_SSID          Ssid;
	TMacAddr                BSSID;
} OS_XCC_AUTH_SUCCESS, *POS_XCC_AUTH_SUCCESS;


/**/
/*  CCKM Request structure*/
/**/

/* RequestCode values*/
typedef enum _OS_XCC_CCKM_REQUEST_CODE
{
	XCC_CckmFirstTime = 0,
	XCC_CckmFastHandoff
} OS_XCC_CCKM_REQUEST_CODE;

typedef struct _OS_XCC_CCKM_REQUEST
{
	OS_XCC_CCKM_REQUEST_CODE    RequestCode;
	TI_UINT32                    AssociationRequestIELength;
	TI_UINT8                     AssociationRequestIE[1];
} OS_XCC_CCKM_REQUEST;

typedef struct _XCC_radioManagmentCapability_IE_t
{
	TI_UINT8			eleID;
	TI_UINT8			len;
	TI_UINT8			ciscoAironetOUI[3];
	TI_UINT8			version;
	TI_UINT16  		rmState;
} XCC_radioManagmentCapability_IE_t;

#define     OS_XCC_CONFIGURATION_ENABLE_CKIP        0x0001
#define     OS_XCC_CONFIGURATION_ENABLE_ROGUE_AP    0x0002
#define     OS_XCC_CONFIGURATION_ENABLE_CCKM        0x0004

#define     OS_XCC_CONFIGURATION_ENABLE_ALL         0x0007 



typedef enum _OS_XCC_NETWORK_EAP
{
    OS_XCC_NETWORK_EAP_OFF  = 0,
    OS_XCC_NETWORK_EAP_ON,
    OS_XCC_NETWORK_EAP_ALLOWED,
    OS_XCC_NETWORK_EAP_PREFERRED
} OS_XCC_NETWORK_EAP;

/**/
/*  CCKM Result structure*/
/**/

/* ResultCode values*/
typedef enum _OS_XCC_CCKM_RESULT_CODE
{
	osXCC_CckmSuccess = 0,
	osXCC_CckmFailure,
	osXCC_CckmNotInUse
} OS_XCC_CCKM_RESULT_CODE;

typedef struct _OS_XCC_CCKM_RESULT
{
	OS_XCC_CCKM_RESULT_CODE ResultCode;
} OS_XCC_CCKM_RESULT;


/**/
/*  CCKM Start structure*/
/**/

typedef struct _OS_XCC_CCKM_START
{
	TI_UINT8 Timestamp[8];
	TMacAddr BSSID;
} OS_XCC_CCKM_START;

/*
	GUIDs for custom OIDs in #define form
	-------------------------------------

	The same GUIDs constructed using DEFINE_GUID (above) are also 
	available in #define form. This allows a GUID structure to be 
	constructed directly. For example, an NDIS_GUID structure could be 
	constructed as follows:

		NDIS_GUID ng = {CGUID_FSW_XCC_CONFIGURATION, OID_FSW_XCC_CONFIGURATION, 4, fNDIS_GUID_TO_OID};
 */

/* oids*/
#define CGUID_FSW_XCC_CONFIGURATION				{0x21190696, 0x118d, 0x4654, {0x9e, 0x9a, 0xc6, 0x9c, 0xa7, 0xc7, 0x95, 0xb8}}
#define CGUID_FSW_XCC_NETWORK_EAP				{0x0725e492, 0x3025, 0x477c, {0x91, 0xdc, 0xd5, 0xc1, 0x2a, 0x4e, 0xec, 0x1f}}
#define CGUID_FSW_XCC_ROGUE_AP_DETECTED			{0x5858fa82, 0x0dfd, 0x4a4a, {0xbb, 0xc9, 0xdc, 0xc7, 0x8f, 0x63, 0x01, 0x70}}
#define CGUID_FSW_XCC_REPORT_ROGUE_APS			{0x6e72993a, 0x59a7, 0x4a3e, {0xb1, 0x65, 0x0c, 0xec, 0xb3, 0xc5, 0x0c, 0xdc}}
#define CGUID_FSW_XCC_AUTH_SUCCESS				{0x55019653, 0x0454, 0x4309, {0xb8, 0xca, 0xd2, 0xe9, 0xf4, 0xd0, 0xaf, 0x83}}
#define CGUID_FSW_XCC_CCKM_REQUEST				{0xf5190942, 0x6d90, 0x4858, {0x8a, 0xdf, 0x08, 0x6a, 0x2f, 0xa5, 0xb7, 0xeb}}
#define CGUID_FSW_XCC_CCKM_RESULT				{0x1163fca7, 0x9c1a, 0x4e39, {0xa8, 0x79, 0x9f, 0x93, 0xad, 0x1b, 0x84, 0x07}}

/* status code*/
#define CGUID_FSW_XCC_CCKM_START				{0x8c389e47, 0xe511, 0x4d96, {0xae, 0xfe, 0x2f, 0xb7, 0x31, 0xd8, 0x0c, 0x05}}

#define OID_FSW_XCC_CONFIGURATION				0xFF010201
#define OID_FSW_XCC_NETWORK_EAP					0xFF010202
#define OID_FSW_XCC_ROGUE_AP_DETECTED			0xFF010203
#define OID_FSW_XCC_REPORT_ROGUE_APS			0xFF010204
#define OID_FSW_XCC_AUTH_SUCCESS				0xFF010205
#define OID_FSW_XCC_CCKM_REQUEST				0xFF010206
#define OID_FSW_XCC_CCKM_RESULT					0xFF010207
/* status code*/
#define NDIS_STATUS_FSW_XCC_CCKM_START			0x60010001



#endif /*XCC_MODULE_INCLUDED*/

#endif /*__OSDOT11XCC_H__*/
