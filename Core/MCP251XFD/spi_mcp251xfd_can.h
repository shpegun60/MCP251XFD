/*
 * spi_mcp251xfd_can.h
 *
 *  Created on: Apr 30, 2024
 *      Author: admin
 */

#ifndef _SPI_MCP251XFD_CAN_H_
#define _SPI_MCP251XFD_CAN_H_

#include "main.h"
#include "MCP251XFD.h"
#include "Conf_MCP251XFD.h"
#include "ErrorsDef.h"

#define MCP_FDCAN_DATA_SIZE 64

// typedefs --------------------------------------------------------------------------------------------------
typedef struct mcp_spi_fdCAN mcp_spi_fdCAN_t;
typedef eERRORRESULT (*const mcp_spi_fdCAN_filterInit)(mcp_spi_fdCAN_t* const self);
typedef void (* mcp_spi_fdCAN_receiveCallback)(mcp_spi_fdCAN_t* const self, const uint32_t ID, const uint8_t len, uint8_t RxData[MCP_FDCAN_DATA_SIZE]);
//------------------------------------------------------------------------------------------------------------
typedef struct {
	MCP251XFD MCP251XFD;
	MCP251XFD_Config MCP251XFD_conf;
} MCP_FDCAN_drv_t;

struct mcp_spi_fdCAN {
	MCP_FDCAN_drv_t* can_drv;

	// periphery
	SPI_HandleTypeDef* SPI;

	struct {
		MCP251XFD_CANMessage	head;
		uint8_t 				RxData[MCP_FDCAN_DATA_SIZE];
	} RX;

	mcp_spi_fdCAN_receiveCallback m_rxCallback;
};

eERRORRESULT mcp_fdCan_init(mcp_spi_fdCAN_t * const self, MCP_FDCAN_drv_t* const drv, SPI_HandleTypeDef * const hspi, mcp_spi_fdCAN_filterInit user_filt_init);
eERRORRESULT mcp_fdCan_setCallback(mcp_spi_fdCAN_t* const self, mcp_spi_fdCAN_receiveCallback m_rxCallback);


uint32_t fdCan_send(mcp_spi_fdCAN_t* const self, const uint32_t ID, const uint32_t len, uint8_t TxData[MCP_FDCAN_DATA_SIZE]);
void fdCan_proceed(mcp_spi_fdCAN_t* const self);
void fdCan_proceed_from_pin(mcp_spi_fdCAN_t* const self, const bool pin_state);



#endif /* _SPI_MCP251XFD_CAN_H_ */
