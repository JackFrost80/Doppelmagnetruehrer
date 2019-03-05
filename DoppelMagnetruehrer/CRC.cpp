/*
 * CRC.cpp
 *
 * Created: 05.03.2019 21:51:42
 *  Author: JackFrost
 */ 


// CRC-16(CRC-CCITT) Polynom x^16 + x^12 + x^5 + 1 --> 0x1021

#include <avr/io.h>
#include <avr/interrupt.h>

uint32_t calculate_crc32_checksum(unsigned char *data, unsigned char count)
{
	unsigned char i = 0;
	uint32_t crc32_checksum = 0;
	
	// Die 4 Checksum Register auf 0x00 zurücksetzen
	CRC.CTRL = (CRC.CTRL & (~CRC_RESET_gm)) | CRC_RESET_RESET0_gc;
	
	// Den CRC-Kern anhalten, indem keine Quellec angegeben wird
	CRC.CTRL = (CRC.CTRL & (~CRC_SOURCE_gm)) | CRC_SOURCE_DISABLE_gc;
	
	// Den CRC-Kern auf CRC32 konfigurieren
	CRC.CTRL |= CRC_CRC32_bm;

	// IO als Quelle für den CRC-Kern auswhählen
	CRC.CTRL = (CRC.CTRL & (~CRC_SOURCE_gm)) | CRC_SOURCE_IO_gc;
	
	//ACHTUNG IST RESET IMMER NOCH 1?
	CRC.CTRL &= ~CRC_RESET_gm;
	
	for(i=0; i<count; i++)
	{
		CRC.DATAIN = data[i];
	}

	// Das Busy Flag muss im CRC IO-Modus gelöscht werden, indem eine 1 in das Register geschrieben wird
	CRC.STATUS |= CRC_BUSY_bm;
	
	while((CRC.STATUS & CRC_BUSY_bm) == CRC_BUSY_bm)
	{
		// Nichts machen solange der CRC-Kern noch beschäftigt ist
	}
	
	crc32_checksum = ((uint32_t) CRC.CHECKSUM0 & 0x000000FF);
	crc32_checksum |= (((uint32_t) CRC.CHECKSUM1 << 8) & 0x0000FF00);
	crc32_checksum |= (((uint32_t) CRC.CHECKSUM2 << 16) & 0x00FF0000);
	crc32_checksum |= (((uint32_t) CRC.CHECKSUM3 << 24) & 0xFF000000);
		
	// Den CRC-Kern anhalten, indem keine Quellec angegeben wird
	CRC.CTRL = (CRC.CTRL & (~CRC_SOURCE_gm)) | CRC_SOURCE_DISABLE_gc;
	
	return crc32_checksum;
}