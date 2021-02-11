/**********************************************************************
 *      Author: tstern
 *
 *	Misfit Tech invests time and resources providing this open source code,
 *	please support Misfit Tech and open-source hardware by purchasing
 *	products from Misfit Tech, www.misifittech.net!
 *
 *	Written by Trampas Stern  for Misfit Tech.
 *	BSD license, check license.txt for more information
 *	All text above, must be included in any redistribution
 *********************************************************************/
#include "Flash.h"
#include "syslog.h"

//SAMD51 has ECC feature?
//Power reduction mode can be disabled? (CTRLA.PRM)

bool flashInit(void)
{
	if (NVMCTRL->PARAM.bit.PSZ != 3)
	{
		ERROR("FLASH PAGE SIZE is not 64 bytes");
		return false;
	}
	return true;
}

//Flash erase row
//6ms for row erase
//2.5ms for page programming
static void erase(const volatile void *flash_ptr)
{
	NVMCTRL->ADDR.reg = ((uint32_t)flash_ptr) / 2;
	
#ifdef _SAMD21_
	NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;		//ER = erase row
#else	//SAMD51
	NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EB; 		//EB = erase block, EP = erase page
	//Each region has dedicated lock bit preventing writing and erasing pages in the region
#endif // !
	
	flashWaitForReady();
}

void flashWaitForReady(void)
{
	int i = 0;
	
#ifdef _SAMD21_
	while (!NVMCTRL->INTFLAG.bit.READY) 
#else
	while(!NVMCTRL->STATUS.bit.READY)
#endif
	{
		i++;
	}
}

bool flashErase(const volatile void *flash_ptr, uint32_t size)
{
	const uint8_t *ptr = (const uint8_t *)flash_ptr;
	
	//if(flash_ptr < NVM_ADDRESS) ERROR
	
	while (size > FLASH_ROW_SIZE)
	{
		erase(ptr);
		ptr += FLASH_ROW_SIZE;
		size -= FLASH_ROW_SIZE;
	}

	if (size > 0)
	{
		erase(ptr);
	}
	
	//Check last page erase successful (all 0xFF)
	for (int i = FLASH_ROW_SIZE; i > 0; i--)
	{
		if (ptr[i-1] != FLASH_ERASE_VALUE)
		{
			ERROR("Flash erase failed");
			return false;
		}			
	}
			
	return true;
}

static inline uint32_t read_unaligned_uint32(const void *data)
{
	union {
		uint32_t u32;
		uint8_t u8[4];
	} res;
	const uint8_t *d = (const uint8_t *)data;
	res.u8[0] = d[0];
	res.u8[1] = d[1];
	res.u8[2] = d[2];
	res.u8[3] = d[3];
	return res.u32;
}

void flashWrite(const volatile void *flash_ptr, const void *data, uint32_t size)
{
	uint32_t *ptrPage;
	uint8_t *srcPtr, *destPtr;
	uint32_t bytesInBlock, offset;
	__attribute__((__aligned__(4))) uint8_t buffer[FLASH_ROW_SIZE];

	destPtr = (uint8_t *)flash_ptr;
	srcPtr = (uint8_t *)data;

	//LOG("flash write called");
	while (size > 0)
	{
		uint32_t i, j;

		//calculate the maximum number of bytes we can write in page
		offset = ((uint32_t)destPtr) % (FLASH_ROW_SIZE); //offset into page
		bytesInBlock = FLASH_ROW_SIZE - offset;			 //this is how many bytes we need to overwrite in this page

//		LOG("offset %d, bytesInBlock %d size %d", offset, bytesInBlock, size);
		//get pointer to start of page
		ptrPage = (uint32_t *)((((uint32_t)destPtr) / (FLASH_ROW_SIZE)) * FLASH_ROW_SIZE);
		//LOG("pointer to page %d(0x%08x) dest:%d", (uint32_t)ptrPage, (uint32_t)ptrPage, destPtr);

		//fill page buffer with data from flash
		memcpy(buffer, ptrPage, FLASH_ROW_SIZE);

		//now fill buffer with new data that needs changing
		i = bytesInBlock;
		if (size < i)
		{
			i = size;
		}
		memcpy(&buffer[offset], srcPtr, i);

		//LOG("changing %d bytes (%0x %0x)", i, buffer[offset], buffer[offset+1]);
		flashErase(ptrPage, FLASH_ROW_SIZE);				//erase page
		flashWritePage(ptrPage, buffer, FLASH_ROW_SIZE);	//write new data to flash

		uint32_t *ptr = (uint32_t *)buffer;
		for (j = 0; j < FLASH_ROW_SIZE / 4; j++)
		{
			if (*ptrPage != *ptr)
			{
				ERROR("write failed on byte %d %x %x", j, *ptrPage, *ptr);
			}
			ptrPage++;
			ptr++;
		}

		size = size - i; //decrease number of bytes to write
		srcPtr += i;	 //increase pointer to next bytes to read
		destPtr += i;	//increment destination pointer
	}
}

void flashWritePage(const volatile void *flash_ptr, const void *data, uint32_t size)
{
	// Calculate data boundaries
	size = (size + 3) / 4; //convert bytes to words with rounding

	volatile uint32_t *dst_addr = (volatile uint32_t *)flash_ptr;
	const uint8_t *src_addr = (uint8_t *)data;

	if (0 != ((uint32_t)flash_ptr) % (FLASH_PAGE_SIZE))
	{
		ERROR("Flash page write must be on boundry");
		return;
	}

	
#ifdef _SAMD21_
	// Disable automatic page write
	NVMCTRL->CTRLB.bit.MANW = 1;
	
	// Do writes in pages
while(size)
	{
		// Execute "PBC" Page Buffer Clear
		NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
		flashWaitForReady();

		// Fill page buffer
		uint32_t i;
		for (i = 0; i < (FLASH_PAGE_SIZE / 4) && size; i++) //we write 4 bytes at a time
			{
				*dst_addr = read_unaligned_uint32(src_addr);
				src_addr += 4;
				dst_addr++;
				size--;  //size is set to number of 32bit words in first line above
			}

		// Execute "WP" Write Page
		NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
		flashWaitForReady();
	}
	
#else	//SAMD51
	
	//Configure manual write mode for NVM using WMODE
	NVMCTRL->CTRLA.bit.WMODE = NVMCTRL_CTRLA_WMODE_MAN_Val;	//
	flashWaitForReady();			//Make sure NVM ready to accept a new command
			
// Do writes in pages
while(size)
	{
		NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_PBC;	//Clear page buffer
		flashWaitForReady();
		NVMCTRL->INTFLAG.bit.DONE = 1; 		//Clear the done flag
	
		// Fill page buffer
		uint32_t i;
		for (i = 0; i < (FLASH_PAGE_SIZE / 4) && size; i++) //we write 4 bytes at a time
			{
				*dst_addr = read_unaligned_uint32(src_addr);
				src_addr += 4;
				dst_addr++;
				size--;   //size is set to number of 32bit words in first line above
			}

		NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WP;	// Execute "WP" Write Page
		flashWaitForReady();
		NVMCTRL->INTFLAG.bit.DONE = 1;  		//Clear the done flag
	}
#endif // !_SAMD21
}
