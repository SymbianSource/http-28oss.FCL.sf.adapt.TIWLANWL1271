/*
 * wlantrace.cpp
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


/** \file  wlantrace.cpp 
 *  \brief Handle any event interrupt from the FW
 *
 *  \see   
 */


#include <kernel/kernel.h>

#ifdef __cplusplus
	extern "C" 
	{
		#include "CDebugOutput.h"
	}
#endif

#include "gendebug.h"
#define __FILE_ID__								FILE_ID_143

#ifndef LOCAL
	#define 	LOCAL 	static
#endif

#if WSTH /* Work station Test Harness */
	#define OUTPUT(msg) printf msg; printf("\n");
#else
	#define OUTPUT(msg) Kern::Printf msg;
#endif

#define 	TRACE_BUFFER_SIZE 	250
#define 	MAX_FORMAT_WIDTH 		10 	/* must not be less than 10, which represents a 32 bit value in decimal */


TUint gModuleId;

/* order must be the same as that defined in wlanTraceConfig.h */
const char* const wlanModuleNameStringArray[] = 	{
																		"wlanResourceManager:",
																		"wlanSpiManager:",
																		"wlanPhysicalChannelOmap:",
																		"wlanIf:",
																		"wlanControl:",
																	};

typedef enum
{
	uToAFormatDec 	= 10,
	uToAFormatHex 	= 16
} utoaFormats;

void Assert( const TInt8* aFile, TInt aLine, TBool aExpression )
    {
#ifndef NDEBUG

    if ( !aExpression )
        {
        TraceDump( CRIT_LEVEL, (reinterpret_cast<const char*>(aFile)) );
        TraceDump( CRIT_LEVEL, (("Line: %d"), aLine));

        Kern::Fault(("[WLANPDD] Subsystem programmed fault"), 0);
        }
    else
        {
        // left intentionally empty
        }

#endif // NDEBUG
    }


LOCAL TUint32 wlanStrlen(const char* pBuffer)
{
	TUint32 	length 	= 0;

	if(pBuffer)
	{
		while(*pBuffer++)
		{
			length++;
		}
	}

	return length;
}

LOCAL TUint32 wlanStrcpy(char* pDestination, const char* pSource)
{
	TUint32 	charactersCopied 	= 0;

	if(pSource && pDestination)
	{
		while(*pSource)
		{
			*pDestination++ = *pSource++;
			charactersCopied++;
		}
	}

	return charactersCopied;
}

LOCAL void wlanReverse(char* pStr)
{
	TUint32 	start;
	TUint32 	end;
	char 		c;

	for(start = 0, end = wlanStrlen(pStr)-1 ; start < end ; start++, end --)
	{
		c = *(pStr + start);
		*(pStr + start) = *(pStr + end);
		*(pStr + end) = c;
	}
}

LOCAL void wlanUtoa(TUint32 n, char* pStr, utoaFormats base, char paddingCharacter, TUint32 width)
{
	const char 	uppercaseHexValues[] 	= "0123456789ABCDEF";
/*	const char 	lowercaseHexValues[] 	= "0123456789abcdef"; never implemented - what is the point.. */
	TUint32 		i 					= 0;

	do
	{
		*(pStr + i++) = uppercaseHexValues[(n % base)];
		width--; /* may wrap round - but handled by check below */
	} while ((n /= base) > 0);

	if(width < MAX_FORMAT_WIDTH)
	{
		while(width--)
			*(pStr + i++) = paddingCharacter;
	}
		
	*(pStr + i) = '\0';
	wlanReverse(pStr);
}


LOCAL bool wlanIsDigit(char character)
{
	if(character >= '0' && character <= '9')
	{
		return true;
	}
	else
	{
		return false;
	}
}


void wlanPrintOutBuffer(char* const pBuffer, const char* const pPrefix, TUint32* bufferLength)
{
	*(pBuffer + (*bufferLength)++) = '\0';
		
	if(wlanStrlen(pBuffer) > TRACE_BUFFER_SIZE)
	{
		OUTPUT(("ERROR:INCREASE TRACE_BUFFER_SIZE - needed %d", *bufferLength))
	}

#ifdef WSTH
	printf(pBuffer);
	printf("\n"); /* STI adds \n normally - but here we have to add it manually */
#else
	Kern::Printf(pBuffer);
#endif

	*bufferLength = 0;
	*bufferLength += wlanStrcpy((pBuffer + *bufferLength), pPrefix); /* add trace identifier to new line */
}


void wlanPrintf(const char* const pPrefix, const char* pText, VA_LIST args)
{
	char 				traceBuffer[TRACE_BUFFER_SIZE];
	const char* 	pNullString 							= "<NULL>";
	const char* 	pUnknown 								= "<UPF>"; /* Unknown Print Format */
	char* 			pString;
	TUint32 			value;
	TUint32 			bufferPos 								= 0;
	TUint32 			error 									= 0;
	TUint32 			formatWidth;
	char 				valueBuffer[MAX_FORMAT_WIDTH + 1]; /* for the terminator */
	char 				paddingChar;
	
	if(pText)
	{
		bufferPos += wlanStrcpy((traceBuffer + bufferPos), pPrefix); /* add trace identifier to first line */
		for( ; *pText != '\0' && error == 0 ; pText++)
		{
			if (*pText != '%')
			{
				if(*pText == '\n')
				{
					wlanPrintOutBuffer(traceBuffer, pPrefix, &bufferPos);
				}
				else
				{
					traceBuffer[bufferPos++] = *pText;
				}
				continue;
			}
			else
			{
				pText++; /* skip % */
				formatWidth = 0;
				memset(valueBuffer, 0, MAX_FORMAT_WIDTH); /* reset the width buffer */
				if(*pText == '0')
					paddingChar = '0';
				else
					paddingChar = ' ';

				while( wlanIsDigit(*pText) || *pText == '.')
				{
					if(*pText == '.')
					{
						while( wlanIsDigit(*pText) || *pText == '.')
							pText++;
						break; /* only deal with leading width - therefore not decimal places - however %f is not supported at time of writting */
					}
					else
					{
						formatWidth = (formatWidth * 10) + (*pText - '0');
						pText++;
					}
				}
				if(formatWidth > MAX_FORMAT_WIDTH)
				{
					OUTPUT(("ERROR:formatWidth too large - %d", formatWidth))
					formatWidth = MAX_FORMAT_WIDTH;
				}
			}
			
			if(*pText == '#')
			{
				pText++; /* skip the hash */
				
				/* Now look at the formatting character */
				switch(*pText)
				{
					case 'x':
					case 'X':
					case 'p':
						traceBuffer[bufferPos++] = '0';
						if(*pText == 'p')
							traceBuffer[bufferPos++] = 'x';
						else
							traceBuffer[bufferPos++] = *pText;
						formatWidth -= 2;
					break;

					case 'd':
						/* do nothing - there is no formating needed */
					break;
					
					default:
						OUTPUT(("ERROR:unknown hash formatting - %c", *pText))
					break;
				}
			}
			
			switch (*pText)
			{
				case 'd': /* signed */
					value = va_arg(args, TUint32);
					if( value & 0x8000000 ) /* MSB 0 = Not signed. MSB 1 = signed*/
					{
						/* Signed value */
						traceBuffer[bufferPos++] = '-'; 
						/* Ignore MSB which is the sign. Negate and inc by 1 to get the absolute value to be printed. */
						value = ((~(value & 0x7FFFFFFF) & 0x7FFFFFFF) + 1);
					}
					wlanUtoa(value, valueBuffer, uToAFormatDec, paddingChar, formatWidth);
					bufferPos += wlanStrcpy((traceBuffer + bufferPos), valueBuffer);
				break;
				
				case 'u': /* unsigned */
					value = va_arg(args, TUint32);
					wlanUtoa(value, valueBuffer, uToAFormatDec, paddingChar, formatWidth);
					bufferPos += wlanStrcpy((traceBuffer + bufferPos), valueBuffer);
				break;
				
				case 'X': /* Fall through */
				case 'x':
				case 'p':
					value = va_arg(args, TUint32);
					wlanUtoa(value, valueBuffer, uToAFormatHex, paddingChar, formatWidth);
					bufferPos += wlanStrcpy((traceBuffer + bufferPos), valueBuffer);
				break;
				
				case 'c':
					traceBuffer[bufferPos++] = va_arg(args, int); /* all params are 32 bit - hence int */
				break;
				
				case 's':
					pString = va_arg(args, char *);
					if(pString == NULL)
					{
						pString = (char*)pNullString;
					}
					bufferPos += wlanStrcpy((traceBuffer + bufferPos), pString);
				break;
				
				default:
					OUTPUT(("ERROR:unknown formating char found - %c", *pText))
					bufferPos += wlanStrcpy((traceBuffer + bufferPos), pUnknown);
					/* Break the loop: If we dont know the size of a given argument, we
						cant know when subsequent arguments start so well have to bail out */
					error = 1;
				break;
			}
		}

		if( bufferPos > wlanStrlen(pPrefix) )
		{
			wlanPrintOutBuffer(traceBuffer, pPrefix, &bufferPos);
		}
		
	}
}


#define WLAN_BYTES_PER_LINE 	32
void wlanTraceData(TUint moduleId, const char* const pString, const TUint8* const pData, TUint dataLength)
{

	const char* 	pPrefix = wlanModuleNameStringArray[moduleId];
	TUint 			writeCounter 	= 0;
	TUint 			readCounter 	= 0;
	TUint 			numberOfLines 	= 0;
	char 				line[WLAN_BYTES_PER_LINE*3+2]; /* 'XX ' = 3 bytes per input byte +2 for safety */
	char 				character;

	Kern::Printf("%s%s - length 0x%X (%d)", pPrefix, pString, dataLength, dataLength);

	if(dataLength == 0 || !pData)
	{
		Kern::Printf("%s:zero length", pPrefix);
	}
	else
	{
		while(readCounter < dataLength)
		{
			character = (char)((pData[readCounter] >> 4) & 0x0f);
			character = (char)(character + ((character < 0x0a) ? 0x30 : 0x37) );
			line[writeCounter++] = character;

			character = (char)(pData[readCounter] & 0x0f);
			character = (char)(character + ( (character < 0x0a) ? 0x30 : 0x37) );
			line[writeCounter++] = character;
			line[writeCounter++] = ' ';

			if(writeCounter == WLAN_BYTES_PER_LINE*3 /*&& readCounter < dataLength*/)
			{
				line[--writeCounter] = '\0';
				Kern::Printf("%s[%3d]%s", pPrefix, WLAN_BYTES_PER_LINE*numberOfLines, line);
				numberOfLines++;
				writeCounter = 0;
			}
			readCounter++;
		}

		if(writeCounter)
		{
			line[--writeCounter] = '\0';
			Kern::Printf("%s[%3d]%s", pPrefix, WLAN_BYTES_PER_LINE*numberOfLines, line);
		}
	}
}

void wlanTraceAscii(TUint moduleId, const char* const pString, const TUint8* pBuffer, TUint length)
{
	char* pAscii = (char*)Kern::Alloc(length + 1);
	TUint i = 0;
	while(i < length)
	{
		pAscii[i++] = *pBuffer++;
	}
	pAscii[length] = '\0';
	Kern::Printf("%s%s - '%s'", wlanModuleNameStringArray[moduleId], pString, pAscii);
	Kern::Free(pAscii);
}

void wlanTrace(const char* const pFormat, ...)
{
	VA_LIST args;
	VA_START(args, pFormat);
	wlanPrintf(wlanModuleNameStringArray[gModuleId], pFormat, args);
	VA_END(args);
}

void wlanTraceLine(TUint moduleId, ... /* pFunction, line */)
{
	VA_LIST args;
	VA_START(args, moduleId);
	wlanPrintf(wlanModuleNameStringArray[moduleId], "%s - line %d", args);
	VA_END(args);
}

void wlanTraceEnterFn(TUint moduleId, ... /* pFunction */)
{
	VA_LIST args;
	VA_START(args, moduleId);
	wlanPrintf(wlanModuleNameStringArray[moduleId], ":%s+", args);
	VA_END(args);
}

void wlanTraceExitFn(TUint moduleId, ... /* pFunction */)
{
	VA_LIST args;
	VA_START(args, moduleId);
	wlanPrintf(wlanModuleNameStringArray[moduleId], ":%s-", args);
	VA_END(args);
}

void wlanTraceProcess(TUint moduleId, ... /* pFunction, thread */)
{
	VA_LIST args;
	VA_START(args, moduleId);
	wlanPrintf(wlanModuleNameStringArray[moduleId], "%s - Thread = 0x%08x", args);
	VA_END(args);
}



/*void Assert( const signed char* aFile, int aLine, int aExpression )
    {

    }

#endif */ /*_DEBUG */



