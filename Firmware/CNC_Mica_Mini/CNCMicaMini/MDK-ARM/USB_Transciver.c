#include "USB_Transciver.h"
#include <stdint.h>

void USBprintf(const char *pcString, va_list vaArgP)
{
	uint32_t ui32Idx, ui32Value, ui32Pos, ui32Count, ui32Base, ui32Neg;
	char *pcStr, pfBuf[16], cFill;
	
	while(*pcString)
	{
		for( ui32Idx = 0; (pcString[ui32Idx] != '%') && (pcString[ui32Idx] != '/0'); ui32Idx++);
		{
		}
		
	}
}