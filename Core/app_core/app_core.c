/*
 * app_core.c
 *
 *  Created on: Apr 23, 2024
 *      Author: admin
 */

#include "app_core.h"
#include "main.h"

#include "CRC16_CMS.h"
#include "spi_mcp251xfd_can.h"

extern SPI_HandleTypeDef hspi1;

eERRORRESULT mcp_spi_spiInit(void *pIntDev, uint8_t chipSelect, const uint32_t sckFreq)
{
	return ERR_OK;
}

eERRORRESULT mcp_spi_transfer(void *pIntDev, uint8_t chipSelect, uint8_t *txData, uint8_t *rxData, size_t size)
{
	SPI_HandleTypeDef* const hspi = pIntDev;
	if(txData && rxData) {
		HAL_SPI_TransmitReceive(hspi, txData, rxData, size, HAL_MAX_DELAY);
	} else if(txData) {
		HAL_SPI_Transmit(hspi, txData, size, HAL_MAX_DELAY);
	} else if(rxData) {
		HAL_SPI_Receive(hspi, rxData, size, HAL_MAX_DELAY);
	}
	return ERR_OK;
}

eERRORRESULT  mcp_spi_filterInit(mcp_spi_fdCAN_t* const self)
{
	eERRORRESULT ErrorExt1 = ERR__NO_DEVICE_DETECTED;
	MCP251XFD* const mcp_drv = &self->can_drv->MCP251XFD;

	MCP251XFD_FIFO MCP2517FD_Ext1_FIFOlist[2] =
	{
			{
					.Name = MCP251XFD_FIFO1,
					.Size = MCP251XFD_FIFO_14_MESSAGE_DEEP,
					.Payload = MCP251XFD_PAYLOAD_64BYTE,
					.Direction = MCP251XFD_RECEIVE_FIFO,
					.ControlFlags = MCP251XFD_FIFO_NO_CONTROL_FLAGS,
					.InterruptFlags = MCP251XFD_FIFO_OVERFLOW_INT + MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT,
					.RAMInfos = NULL,
			},

			{
					.Name = MCP251XFD_FIFO2,
					.Size = MCP251XFD_FIFO_14_MESSAGE_DEEP,
					.Payload = MCP251XFD_PAYLOAD_64BYTE,
					.Direction = MCP251XFD_TRANSMIT_FIFO,
					.Attempts = MCP251XFD_UNLIMITED_ATTEMPTS,//MCP251XFD_THREE_ATTEMPTS,
					.Priority = MCP251XFD_MESSAGE_TX_PRIORITY16,
					.ControlFlags = MCP251XFD_FIFO_NO_RTR_RESPONSE,
					.InterruptFlags = MCP251XFD_FIFO_TX_ATTEMPTS_EXHAUSTED_INT + MCP251XFD_FIFO_TRANSMIT_FIFO_NOT_FULL_INT,
					.RAMInfos = NULL,
			}
	};

	MCP251XFD_Filter MCP2517FD_Ext1_FilterList[1] =
	{
			{
					.Filter = MCP251XFD_FILTER0,
					.EnableFilter = true,
					.Match = MCP251XFD_MATCH_ONLY_EID,
					.PointTo = MCP251XFD_FIFO1,
					.AcceptanceID = MCP251XFD_ACCEPT_ALL_MESSAGES,
					.AcceptanceMask = MCP251XFD_ACCEPT_ALL_MESSAGES,
					.ExtendedID  = true
			}
	};



	ErrorExt1 = MCP251XFD_ConfigureFIFOList(mcp_drv, &MCP2517FD_Ext1_FIFOlist[0], 2);
	if (ErrorExt1 != ERR_OK) return ErrorExt1;

	ErrorExt1 = MCP251XFD_ConfigureFilterList(mcp_drv, MCP251XFD_D_NET_FILTER_DISABLE, &MCP2517FD_Ext1_FilterList[0], 1);
	return ErrorExt1;
}

void mcp_spi_receiveCallback(mcp_spi_fdCAN_t* const self, const uint32_t ID, const uint8_t len, uint8_t RxData[MCP_FDCAN_DATA_SIZE])
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

void app_main(void)
{
	MCP251XFD_BitTimeStats MCP2517FD_Ext1_BTStats;
	uint32_t SYSCLK_Ext1;
	MCP_FDCAN_drv_t Mcp_drv = {
			.MCP251XFD = {
					.UserDriverData = NULL,
					//--- Driver configuration ---
					.DriverConfig = MCP251XFD_DRIVER_USE_READ_WRITE_CRC
					//| MCP251XFD_DRIVER_USE_SAFE_WRITE
					| MCP251XFD_DRIVER_ENABLE_ECC
					| MCP251XFD_DRIVER_INIT_SET_RAM_AT_0
					| MCP251XFD_DRIVER_CLEAR_BUFFER_BEFORE_READ,
					//--- IO configuration ---
					.GPIOsOutLevel = MCP251XFD_GPIO0_LOW | MCP251XFD_GPIO1_HIGH,
					//--- Interface driver call functions ---
					.SPI_ChipSelect = 0, 		// Here the chip select of the EXT1 interface is 1
					.InterfaceDevice = &hspi1, 	// Here this point to the address memory of the peripheral SPI0
					.fnSPI_Init = mcp_spi_spiInit,
					.fnSPI_Transfer = mcp_spi_transfer,
					//--- Time call function ---
					.fnGetCurrentms = HAL_GetTick,
					//--- CRC16-CMS call function ---
					.fnComputeCRC16 = ComputeCRC16CMS,
					//--- Interface clocks ---
					.SPIClockSpeed = 8125000, // 8.125MHz
			},

			.MCP251XFD_conf = {
					//--- Controller clocks ---
					.XtalFreq = 20000000, 			// CLKIN is a 40MHz crystal
					.OscFreq = 0, 	// CLKIN is not a oscillator
					.SysclkConfig = MCP251XFD_SYSCLK_IS_CLKIN,
					.ClkoPinConfig = MCP251XFD_CLKO_SOF,
					.SYSCLK_Result = &SYSCLK_Ext1,
					//--- CAN configuration ---
					.NominalBitrate = 500000, // Nominal Bitrate to 1Mbps
					.DataBitrate = 1000000, // Data Bitrate to 2Mbps
					.BitTimeStats = &MCP2517FD_Ext1_BTStats,
					.Bandwidth = MCP251XFD_DELAY_8BIT_TIMES,
					.ControlFlags = MCP251XFD_CAN_RESTRICTED_MODE_ON_ERROR
					| MCP251XFD_CAN_ESI_REFLECTS_ERROR_STATUS
					| MCP251XFD_CAN_RESTRICTED_RETRANS_ATTEMPTS
					| MCP251XFD_CANFD_BITRATE_SWITCHING_ENABLE
					| MCP251XFD_CAN_PROTOCOL_EXCEPT_AS_FORM_ERROR
					| MCP251XFD_CANFD_USE_ISO_CRC
					| MCP251XFD_CANFD_DONT_USE_RRS_BIT_AS_SID11,
					//--- GPIOs and Interrupts pins ---
					.GPIO0PinMode = MCP251XFD_PIN_AS_GPIO0_OUT,
					.GPIO1PinMode = MCP251XFD_PIN_AS_INT1_RX,
					.INTsOutMode = MCP251XFD_PINS_PUSHPULL_OUT,
					.TXCANOutMode = MCP251XFD_PINS_PUSHPULL_OUT,
					//--- Interrupts ---
					.SysInterruptFlags = MCP251XFD_INT_ENABLE_ALL_EVENTS
			}
	};

	mcp_spi_fdCAN_t Mcp_can;
	mcp_fdCan_init(&Mcp_can, &Mcp_drv, &hspi1, mcp_spi_filterInit);
	mcp_fdCan_setCallback(&Mcp_can, mcp_spi_receiveCallback);

	uint8_t txData[64];
	txData[0] = 0x11;
	txData[1] = 0x22;
	txData[2] = 0xAA;
	txData[3] = 0x55;


	uint32_t last_time = 0;
	while(1) {
		const uint32_t current_time = HAL_GetTick();

		if((current_time - last_time) > 100) {
			last_time = current_time;
			fdCan_send(&Mcp_can, 0x1234567F, 4, txData);
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		}

		//fdCan_proceed(&Mcp_can);
		fdCan_proceed_from_pin(&Mcp_can, HAL_GPIO_ReadPin(RX_INT_GPIO_Port, RX_INT_Pin) == GPIO_PIN_SET);
	}
}
