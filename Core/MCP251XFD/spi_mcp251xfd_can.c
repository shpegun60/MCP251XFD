/*
 * spi_mcp251xfd_can.c
 *
 *  Created on: Apr 30, 2024
 *      Author: admin
 */

#include "spi_mcp251xfd_can.h"
#include <string.h>

static void mcp_receiveCallback_default(mcp_spi_fdCAN_t* const self, const uint32_t ID, const uint8_t len, uint8_t RxData[MCP_FDCAN_DATA_SIZE])
{
	(void)self;
	(void)ID;
	(void)len;
	(void)RxData;
}

eERRORRESULT mcp_fdCan_init(mcp_spi_fdCAN_t * const self, MCP_FDCAN_drv_t* const drv, SPI_HandleTypeDef * const hspi, mcp_spi_fdCAN_filterInit user_filt_init)
{
	if(self == NULL || hspi == NULL || drv == NULL) {
		return ERR__INVALID_DATA;
	}

	memset(self, 0, sizeof(mcp_spi_fdCAN_t));
	self->can_drv 	= drv;
	self->SPI 		= hspi;

	MCP251XFD* const mcp_can 			= &drv->MCP251XFD;
	MCP251XFD_Config* const mcp_conf 	= &drv->MCP251XFD_conf;

	//--- Configure module on Ext1 ---
	eERRORRESULT ErrorExt1 = ERR__NO_DEVICE_DETECTED;
	ErrorExt1 = Init_MCP251XFD(mcp_can, mcp_conf);
	if (ErrorExt1 != ERR_OK) return ErrorExt1;

	if(user_filt_init) {
		ErrorExt1 = user_filt_init(self);
		if (ErrorExt1 != ERR_OK) return ErrorExt1;
	}

	ErrorExt1 = MCP251XFD_StartCANFD(mcp_can);
	//ErrorExt1 = MCP251XFD_RequestOperationMode(mcp_can, MCP251XFD_EXTERNAL_LOOPBACK_MODE, false);
	//ErrorExt1 = MCP251XFD_RequestOperationMode(mcp_can, MCP251XFD_NORMAL_CANFD_MODE, true);

	self->m_rxCallback = mcp_receiveCallback_default;
	return ErrorExt1;
}

eERRORRESULT mcp_fdCan_setCallback(mcp_spi_fdCAN_t* const self, mcp_spi_fdCAN_receiveCallback m_rxCallback)
{
	if(self != NULL || m_rxCallback != NULL) {
		self->m_rxCallback = m_rxCallback;
		return ERR_OK;
	}

	return ERR__INVALID_DATA;
}




//--------------------------------------------------------------------------------------------------------------------------

static inline const uint8_t dlc_to_byte(const uint8_t DLC)
{
	/* declare dlc lookup table */
	const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
	/* get length */
	const uint8_t len = DLCtoBytes[DLC];
	return len;
}

static inline const uint8_t len_to_dlc(const uint8_t len)
{
	/* Define a lookup table for DLC codes based on data length for CAN FD
			0..8 -- 0..8
			9  -- 12
			10 -- 16
			11 -- 20
			12 -- 24
			13 -- 32
			14 -- 48
			15 -- 64
	 */

	const uint8_t dlc_lookup[65] = {
			// |0--1--2--3--4--5--6--7--8|
			0, 1, 2, 3, 4, 5, 6, 7, 8,											// Lengths 0-8: DLC matches length
			// |9--10--11--12|
			9,  9,  9,  9,														// Lengths 9-12: DLC is 9
			// |13--14--15--16|
			10, 10, 10, 10,														// Lengths 13-16: DLC is 10
			// |17--18--19--20|
			11, 11, 11, 11,														// Lengths 17-20: DLC is 11
			// |21--22--23--24|
			12, 12, 12, 12,														// Lengths 21-24: DLC is 12
			// |25--26--27--28--29--30--31--32|
			13, 13, 13, 13, 13, 13, 13, 13,										// Lengths 25-32: DLC is 13
			// |33--34--35--36--37--38--39--40--41--42--43--44--45--46--47--48|
			14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,		// Lengths 33-48: DLC is 14
			// |49--50--51--52--53--54--55--56--57--58--59--60--61--62--63--64|
			15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15		// Lengths 49-64: DLC is 15
	};

	/* get DLC */
	const uint8_t DLC = (len > 64) ? 15 : dlc_lookup[len];

	return DLC;
}


uint32_t fdCan_send(mcp_spi_fdCAN_t* const self, const uint32_t ID, const uint32_t len, uint8_t TxData[MCP_FDCAN_DATA_SIZE])
{
	eERRORRESULT ErrorExt1 = ERR_OK;
	eMCP251XFD_FIFOstatus FIFOstatus = 0;
	MCP251XFD* const mcp_can 			= &self->can_drv->MCP251XFD;
	//MCP251XFD_Config* const mcp_conf 	= &self->can_drv->MCP251XFD_conf;

	ErrorExt1 = MCP251XFD_GetFIFOStatus(mcp_can, MCP251XFD_FIFO2, &FIFOstatus); // First get FIFO2 status
	if (ErrorExt1 != ERR_OK) return 0;


	if ((FIFOstatus & MCP251XFD_TX_FIFO_NOT_FULL) > 0) { // Second check FIFO not full

		const uint8_t DLC = len_to_dlc(len);

		//***** Fill the message as you want *****
		MCP251XFD_CANMessage TansmitMessage = {
				.MessageID = ID,                         		//!< Contain the message ID to send
				.MessageSEQ = 0,                        		//!< This is the context of the CAN message. This sequence will be copied in the TEF to trace the message sent
				.ControlFlags = MCP251XFD_CANFD_FRAME |
				MCP251XFD_SWITCH_BITRATE |
				MCP251XFD_EXTENDED_MESSAGE_ID |
				MCP251XFD_TRANSMIT_ERROR_PASSIVE, 				//!< Contain the CAN controls flags

				.DLC = (eMCP251XFD_DataLength)DLC,				//!< Indicate how many bytes in the payload data will be sent or how many bytes in the payload data is received
				.PayloadData = &TxData[0]               		//!< Pointer to the payload data that will be sent. PayloadData array should be at least the same size as indicate by the DLC
		};

		ErrorExt1 = MCP251XFD_TransmitMessageToFIFO(mcp_can, &TansmitMessage, MCP251XFD_FIFO2, true);
		if (ErrorExt1 != ERR_OK) return 0;

		return dlc_to_byte(DLC);
	}

	return 0;
}

void fdCan_proceed(mcp_spi_fdCAN_t* const self)
{
	eERRORRESULT ErrorExt1 				= ERR_OK;
	eMCP251XFD_FIFOstatus FIFOstatus 	= 0;
	MCP251XFD* const mcp_can			= &self->can_drv->MCP251XFD;

	ErrorExt1 = MCP251XFD_GetFIFOStatus(mcp_can, MCP251XFD_FIFO1, &FIFOstatus); // First get FIFO1 status
	if (ErrorExt1 != ERR_OK) return;

	if ((FIFOstatus & MCP251XFD_RX_FIFO_NOT_EMPTY) > 0) { // Second check FIFO not empty

		MCP251XFD_CANMessage* const head = &self->RX.head;
		uint8_t* const data = &self->RX.RxData[0];
		//uint32_t MessageTimeStamp = 0;

		head->PayloadData = data; // Add receive payload data pointer to the message structure

		// that will be received
		ErrorExt1 = MCP251XFD_ReceiveMessageFromFIFO(mcp_can, head, MCP251XFD_PAYLOAD_64BYTE, NULL, MCP251XFD_FIFO1);
		if (ErrorExt1 == ERR_OK) {
			//***** Do what you want with the message *****
			const uint8_t len = dlc_to_byte(head->DLC);
			self->m_rxCallback(self, head->MessageID, len, data);
		}
	}
}


void fdCan_proceed_from_pin(mcp_spi_fdCAN_t* const self, const bool pin_state)
{
	if (pin_state == false) { // Check INT1 pin status of the MCP251XFD (Active low state)

		eERRORRESULT ErrorExt1 = ERR_OK;
		eMCP251XFD_FIFO FIFOname;
		eMCP251XFD_FIFOstatus FIFOstatus = 0;
		MCP251XFD* const mcp_can			= &self->can_drv->MCP251XFD;

		ErrorExt1 = MCP251XFD_GetCurrentReceiveFIFONameAndStatusInterrupt(mcp_can, &FIFOname, &FIFOstatus);
		if (ErrorExt1 != ERR_OK) return; // First get which FIFO set interrupt and its status

		// Second check FIFO not empty
		if (((FIFOstatus & MCP251XFD_RX_FIFO_NOT_EMPTY) > 0) && (FIFOname != MCP251XFD_NO_FIFO)) {
			MCP251XFD_CANMessage* const head = &self->RX.head;
			uint8_t* const data = &self->RX.RxData[0];

			head->PayloadData = data; // Add receive payload data pointer to the message structure
			// that will be received
			ErrorExt1 = MCP251XFD_ReceiveMessageFromFIFO(mcp_can, head, MCP251XFD_PAYLOAD_64BYTE, NULL, FIFOname);
			if (ErrorExt1 == ERR_OK) {
				//***** Do what you want with the message *****
				const uint8_t len = dlc_to_byte(head->DLC);
				self->m_rxCallback(self, head->MessageID, len, data);
			}
		}
	}
}
