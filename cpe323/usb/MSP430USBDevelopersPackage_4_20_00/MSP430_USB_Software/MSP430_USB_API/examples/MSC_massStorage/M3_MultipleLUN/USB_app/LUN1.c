/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*
 * ======== msc.c ========
 *
 *
 */
#include <string.h>
#include <stdint.h>
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_MSC_API/UsbMsc.h"
#include "USB_config/descriptors.h"

#include "FatFs/diskio.h"
#include "FatFs/mmc.h"
#include "USB_app/LUN1.h"

extern const uint16_t BYTES_PER_BLOCK;
extern struct config_struct USBMSC_config;
extern uint8_t RWbuf[];
extern USBMSC_RWbuf_Info *RWbuf_info;
extern struct USBMSC_mediaInfoStr mediaInfo;
extern uint8_t bDetectCard;

/*
 * This function initializes the MSC data variables
 */
void LUN1_init(void)
{
    //SD-cards must go through a setup sequence after powerup.
    //This FatFs call does this.
    disk_initialize(0);

    //The API maintains an instance of the USBMSC_RWbuf_Info structure.This is
    // a shared resource between the API and application; the application must
    //request the pointer.

    RWbuf_info = USBMSC_fetchInfoStruct();


    //USBMSC_updateMediaInfo() must be called prior to USB connection.  We
    //check if the card is present, and if so, pull its size and bytes per
    //block.
    if (detectCard()){
        mediaInfo.mediaPresent = kUSBMSC_MEDIA_PRESENT;
    }
	else {
	    mediaInfo.mediaPresent = kUSBMSC_MEDIA_NOT_PRESENT;
	}
    mediaInfo.mediaChanged = 0x00;
    mediaInfo.writeProtected = 0x00;
    //Returns the number of blocks (sectors) in the media.
    disk_ioctl(0,GET_SECTOR_COUNT,&mediaInfo.lastBlockLba);
    //Block size will always be 512
    mediaInfo.bytesPerBlock = BYTES_PER_BLOCK;
    USBMSC_updateMediaInfo(1, &mediaInfo);

    //The data interchange buffer (used when handling SCSI READ/WRITE) is
    //declared by the application, and registered with the API using this
    //function.  This allows it to be assigned dynamically, giving
    //the application more control over memory management.
    USBMSC_registerBufInfo(1, &RWbuf[0], NULL, 512);
}

void LUN1_processBuffer(void)
{

	//Verify SD card before attempting to read.  If this condition is not there Windows dis	plays
	//a message about 'Reformatting the disk' when SD card is not in the drive.
	if(detectCard()){          //Fix for Bug 6754
		bDetectCard = 0x01;   //Fix for bug 6754.  Update the global flag.  There seems to be a timing issue 
                             //between when the main 
                             //application calls mscProcessBuffer() and when the card is detected.  Setting
                             //the global flag here prevents the Windows message 'Reformat Disk' from 
                             //showing up when an SD card is inserted into an empty drive.
	
		//If the API needs the application to process a buffer, it will keep the
		//CPU awake by returning kUSBMSC_processBuffer
		//from USBMSC_poll().  The application should then check the 'operation'
		//field of all defined USBMSC_RWbuf_Info
		//structure instances.  If any of them is non-null, then an operation
		//needs to be processed.  A value of
		//kUSBMSC_READ indicates the API is waiting for the application to fetch
		//data from the storage volume, in response
		//to a SCSI READ command from the USB host.  After the application does
		//this, it must indicate whether the
		//operation succeeded, and then close the buffer operation by calling
		//USBMSC_bufferProcessed().

		while (RWbuf_info->operation == kUSBMSC_READ)
		{
			//A READ operation is underway, and the app has been requested to access
			//the medium.  So, call file system to read
			//to do so.  Note this is a low level FatFs call -- we are not
			//attempting to open a file ourselves.  The host is
			//in control of this access, we're just carrying it out.
			DRESULT dresult = disk_read(0, //Physical drive number (0)
				RWbuf_info->bufferAddr,    //Pointer to the user buffer
				RWbuf_info->lba,           //First LBA of this buffer operation
				RWbuf_info->lbCount);      //The number of blocks being requested
											//as part of this operation

			//The result of the file system call needs to be communicated to the
			//host.  Different file system software uses
			//different return codes, but they all communicate the same types of
			//results.  This code ultimately gets passed to the
			//host app that issued the command to read (or if the user did it the
			//host OS, perhaps in a dialog box).
			switch (dresult)
			{
				case RES_OK:
					RWbuf_info->returnCode = kUSBMSC_RWSuccess;
					break;
				//In FatFs, this result suggests the medium may have been removed
				//recently.
				case RES_ERROR:
					//This application function checks for the SD-card,
					//and if missing,calls USBMSC_updateMediaInfo() to inform
					//the API
					if (!LUN1_checkInsertionRemoval()){
						RWbuf_info->returnCode =
							kUSBMSC_RWMedNotPresent;
					}
					break;
				case RES_NOTRDY:
					RWbuf_info->returnCode = kUSBMSC_RWNotReady;
					break;
				case RES_PARERR:
					RWbuf_info->returnCode = kUSBMSC_RWLbaOutOfRange;
					break;
			}
			USBMSC_bufferProcessed();
		}

		//Everything in this section is analogous to READs.  Reference the
		//comments above.
		while (RWbuf_info->operation == kUSBMSC_WRITE)
		{
			DRESULT dresult = disk_write(0, //Physical drive number (0)
				RWbuf_info->bufferAddr,     //Pointer to the user buffer
				RWbuf_info->lba,            //First LBA of this buffer operation
				RWbuf_info->lbCount);       //The number of blocks being requested
										//as part of this operation
			switch (dresult)
			{
				case RES_OK:
					RWbuf_info->returnCode = kUSBMSC_RWSuccess;
					break;
				case RES_ERROR:
					if (!LUN1_checkInsertionRemoval()){
						RWbuf_info->returnCode =
							kUSBMSC_RWMedNotPresent;
					}
					break;
				case RES_NOTRDY:
					RWbuf_info->returnCode = kUSBMSC_RWNotReady;
					break;
				case RES_PARERR:
					RWbuf_info->returnCode = kUSBMSC_RWLbaOutOfRange;
					break;
				default:
					RWbuf_info->returnCode = kUSBMSC_RWNotReady;
					break;
			}
			USBMSC_bufferProcessed();
		}
	}
}

/*
 * ======== LUN1_checkInsertionRemoval ========
 *
 * This function checks for insertion/removal of the card.  If either is
 * detected, it informs the API by calling USBMSC_updateMediaInfo().  Whether
 * it detects it or not, it returns non-zero if the card is present, or zero if
 * not present
 */
uint8_t LUN1_checkInsertionRemoval (void)
{
    //Check card status -- there or not?
    uint8_t newCardStatus = detectCard();

    if ((newCardStatus) &&
        (mediaInfo.mediaPresent == kUSBMSC_MEDIA_NOT_PRESENT)){
        //An insertion has been detected -- inform the API
        mediaInfo.mediaPresent = kUSBMSC_MEDIA_PRESENT;
        mediaInfo.mediaChanged = 0x01;
        //Get the size of this new medium
        DRESULT SDCard_result = disk_ioctl(0,
            GET_SECTOR_COUNT,
            &mediaInfo.lastBlockLba);
        USBMSC_updateMediaInfo(1, &mediaInfo);   //Fix for bug 6773
    }

    if ((!newCardStatus) && (mediaInfo.mediaPresent == kUSBMSC_MEDIA_PRESENT)){
        //A removal has been detected -- inform the API
        mediaInfo.mediaPresent = kUSBMSC_MEDIA_NOT_PRESENT;
        mediaInfo.mediaChanged = 0x01;
        USBMSC_updateMediaInfo(1, &mediaInfo);   //Fix for bug 6773
    }

    return ( newCardStatus) ;
}
//Released_Version_4_20_00
