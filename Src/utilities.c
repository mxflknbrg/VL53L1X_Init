/*
 * utilities.c
 *
 *  Created on: 08.11.2018
 *      Author: mfalkenberg
 */


#include "utilities.h"

/*
 * ****************************************************************************
 * Global Variables
 * ****************************************************************************
 */
// FreeRTOS
extern osThreadId 		RangingTaskHandle;

// Watchdog
uint8_t 				wwdg_flag 							= 0;
static uint8_t 			activetasks;

// VL53L1X
bool 					vl53l1x_initdone_flag				= false;

// CAN - MCP2515
CANInterruptFlags_t canintflags = {false, false, false, false, false, false, false, false, false};

/*
 * ****************************************************************************
 * Watchdog
 * ****************************************************************************
 */
void WWDGAlive(TaskNumber task){
	portENTER_CRITICAL();
	wwdg_flag |= task;
	portEXIT_CRITICAL();
}

void WWDGSetTaskActive(TaskNumber task){
	portENTER_CRITICAL();
	activetasks |= task;
	portEXIT_CRITICAL();
}

void WWDGSetTaskInactive(TaskNumber task){
	portENTER_CRITICAL();
	activetasks &= ~task;
	portEXIT_CRITICAL();
}

uint8_t WWDGGetActiveTasks(void){
	return activetasks;
}


/*
 * ****************************************************************************
 * VL53L1X
 * ****************************************************************************
 */
static VL53L1_Error VL53L1XSetupROI(VL53L1_DEV dev){
	VL53L1_UserRoi_t 		roiconfig;
	VL53L1_Error			status 				= 0;

#if ROI_SIZE == 4
	roiconfig.TopLeftX 	= 6;
	roiconfig.TopLeftY 	= 9;
	roiconfig.BotRightX = 9;
	roiconfig.BotRightY = 6;
#elif ROI_SIZE == 8
	roiconfig.TopLeftX 	= 4;
	roiconfig.TopLeftY 	= 11;
	roiconfig.BotRightX = 11;
	roiconfig.BotRightY = 4;
#elif ROI_SIZE == 16
	roiconfig.TopLeftX 	= 0;
	roiconfig.TopLeftY 	= 15;
	roiconfig.BotRightX = 15;
	roiconfig.BotRightY = 0;
#elif ROI_SIZE == 416															/* ROI Size 4 * 16 */
	roiconfig.TopLeftX 	= 0;
	roiconfig.TopLeftY 	= 9;
	roiconfig.BotRightX = 15;
	roiconfig.BotRightY = 6;
#elif ROI_SIZE == 816															/* ROI Size 8 * 16 */
	roiconfig.TopLeftX 	= 0;
	roiconfig.TopLeftY 	= 11;
	roiconfig.BotRightX = 15;
	roiconfig.BotRightY = 4;
#endif
	status = VL53L1_SetUserROI(dev, &roiconfig);

	return status;
}

VL53L1_Error VL53L1XInitModules(VL53L1_Dev_t* pmodules){
	VL53L1_DEV 					dev;
	VL53L1_Error 				status			= 0;
	uint8_t 					i2caddress;
	extern I2C_HandleTypeDef 	hi2c1;

	SEGGER_RTT_printf(0, "Initialize Lidar Modules\n\r");

	HAL_GPIO_WritePin(VL53L1X_XSDN0_GPIO_Port, VL53L1X_XSDN0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(VL53L1X_XSDN1_GPIO_Port, VL53L1X_XSDN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(VL53L1X_XSDN2_GPIO_Port, VL53L1X_XSDN2_Pin, GPIO_PIN_RESET);

	for(uint8_t tofsensor = 0; tofsensor < NUMBER_OF_MODULES; tofsensor++){
		status = 0;

		i2caddress = (0x54 + (tofsensor*2));

		osDelay(10);

		switch (tofsensor) {
		case 0:
			dev = &pmodules[tofsensor];
			HAL_GPIO_WritePin(VL53L1X_XSDN0_GPIO_Port, VL53L1X_XSDN0_Pin, GPIO_PIN_SET);
			break;
		case 1:
			dev = &pmodules[tofsensor];
			HAL_GPIO_WritePin(VL53L1X_XSDN1_GPIO_Port, VL53L1X_XSDN1_Pin, GPIO_PIN_SET);
			break;
		case 2:
			dev = &pmodules[tofsensor];
			HAL_GPIO_WritePin(VL53L1X_XSDN2_GPIO_Port, VL53L1X_XSDN2_Pin, GPIO_PIN_SET);
			break;
		default:
			break;
		}

		dev->I2cDevAddr = 0x52;													// Default Address
		dev->I2cHandle = &hi2c1;
		dev->comms_speed_khz = 400;
		dev->comms_type = 1;

		osDelay(10);

		status |= VL53L1_SetDeviceAddress(dev, i2caddress);

		dev->I2cDevAddr = i2caddress;

		status |= VL53L1_WaitDeviceBooted(dev);
		status |= VL53L1_DataInit(dev);
		status |= VL53L1_StaticInit(dev);
		status |= VL53L1_SetDistanceMode(dev, VL53L1_DISTANCEMODE_SHORT);
		status |= VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, 40000);		// The minimum and maximum timing budgets are [20 ms, 1000 ms]
		status |= VL53L1_SetInterMeasurementPeriodMilliSeconds(dev, 44);			// must be >= timing budget + 4 ms

#ifdef ROI_SIZE
		status |= VL53L1XSetupROI(dev);
#endif		/* ROI_SIZE */

		if(status == 0)
			SEGGER_RTT_printf(0, "Modul %d successfully initialized\n\r", tofsensor);
	}

	HAL_GPIO_WritePin(VL53L1X_XSDN0_GPIO_Port, VL53L1X_XSDN0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(VL53L1X_XSDN1_GPIO_Port, VL53L1X_XSDN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(VL53L1X_XSDN2_GPIO_Port, VL53L1X_XSDN2_Pin, GPIO_PIN_SET);

#ifdef DEBUG
	/* Checks whether the communication of the Lidar modules with the new addresses is possible */
	HAL_StatusTypeDef i2cstat = 0;
	uint8_t tempdata[3] = {0xAA, 0xAA, 0xAA};
	uint8_t tempaddress = 0x52;

	for(uint8_t k = 16; k < 127; k++){
		tempaddress = k;
		i2cstat = HAL_I2C_Master_Transmit(&hi2c1, tempaddress, tempdata, 3, 10000);
		SEGGER_RTT_printf(0, "%x\t%d\n\r",k, i2cstat);
	}
#endif

	SEGGER_RTT_printf(0, "Initialization Done\n\r");

	vl53l1x_initdone_flag = true;

	return status;
}

VL53L1_Error VL53L1XSequentialRanging(VL53L1_Dev_t* pmodules, VL53L1_RangingMeasurementData_t* prangingdata){
	VL53L1_DEV 		dev;
	VL53L1_Error 	status;

	if((HAL_GPIO_ReadPin(VL53L1X_XSDN0_GPIO_Port, VL53L1X_XSDN0_Pin) &&
			HAL_GPIO_ReadPin(VL53L1X_XSDN1_GPIO_Port, VL53L1X_XSDN1_Pin) &&
			HAL_GPIO_ReadPin(VL53L1X_XSDN2_GPIO_Port, VL53L1X_XSDN2_Pin)) == 0){
		HAL_GPIO_WritePin(VL53L1X_XSDN0_GPIO_Port, VL53L1X_XSDN0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(VL53L1X_XSDN1_GPIO_Port, VL53L1X_XSDN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(VL53L1X_XSDN2_GPIO_Port, VL53L1X_XSDN2_Pin, GPIO_PIN_SET);
	}

	for(uint8_t tofsensor = 0; tofsensor < NUMBER_OF_MODULES; tofsensor++){
		dev = pmodules + tofsensor;
		status = VL53L1_StartMeasurement(dev);

		WWDGSetTaskInactive(kRangingTask);

		switch (tofsensor) {
		case 0:
			osSignalWait(kVL53L1XInterrupt0, osWaitForever);
			break;
		case 1:
			osSignalWait(kVL53L1XInterrupt1, osWaitForever);
			break;
		case 2:
			osSignalWait(kVL53L1XInterrupt2, osWaitForever);
			break;
		default:
			for(;;)
				__asm("NOP");															// WWDG reset	TODO: asm instruction to halt the MCU
			break;
		}

		WWDGSetTaskActive(kRangingTask);
		WWDGAlive(kRangingTask);

		status = VL53L1_GetRangingMeasurementData(dev, (prangingdata + tofsensor));
		status = VL53L1_ClearInterruptAndStartMeasurement(dev);
		status = VL53L1_StopMeasurement(dev);

		//SEGGER_RTT_printf(0, "%d, %d, %d \n\r", tofsensor, prangingdata[tofsensor].RangeStatus, prangingdata[tofsensor].RangeMilliMeter);

	}
	SEGGER_RTT_printf(0, "%d, %d, %d \n\r", prangingdata[0].RangeMilliMeter, prangingdata[1].RangeMilliMeter, prangingdata[2].RangeMilliMeter);

	return status;
}

/*
 * ****************************************************************************
 * CAN - MCP2515
 * ****************************************************************************
 */
void MCP2515Init(void){
	MCP2515Reset();

	MCP2515WriteByte(0x2A, 0x00);												// CNF1						// 250 kbit/s -> 0x00
	MCP2515WriteByte(0x29, 0x90);												// CNF2						// 250 kbit/s -> 0xB1
	MCP2515WriteByte(0x28, 0x02);												// CNF3						// 250 kbit/s -> 0x05

	MCP2515WriteByte(0x0F, 0x07);												// Leave Configuration Mode
}
void MCP2515Reset(void){
	uint8_t 					txbuf[] 		= {kCANReset};

	SPITransmit(txbuf, 1);

	MCP2515WriteByte(0x2C, 0x0);												// Reset all interrupt flags
}

void MCP2515RTS(uint8_t t2, uint8_t t1, uint8_t t0){
	uint8_t						txbuf[1];
	uint8_t						instruction 	= 0x80;

	instruction |= ((t2 << 2) | (t1 << 1) | t0);
	txbuf[0] = instruction;

	SPITransmit(txbuf, 1);
}

void MCP2515ReadRxBuffer(uint8_t n, uint8_t m, uint8_t* rxbuf, uint16_t size){
	uint8_t						txbuf[1];
	uint8_t						instruction 	= 0x90;

	instruction |= ((n << 2) | (m << 1));
	txbuf[0] = instruction;

	SPIReceiveTransmit(rxbuf, txbuf, size);
}

uint8_t MCP2515ReadRegister(uint8_t address){
	uint8_t 					txbuf[2] 		= {kCANRead, address};
	uint8_t						rxbuf[2]		= {0};

	SPIReceiveTransmit(rxbuf, txbuf, 2);

	return rxbuf[0];
}

void MCP2515LoadTxBuffer(uint8_t a, uint8_t b, uint8_t c, uint8_t* txbuf, uint16_t size){
	uint8_t						_txbuf[size + 1];
	uint8_t						instruction		= 0x40;

	instruction |= ((a << 2) | (b << 1) | c);
	_txbuf[0] = instruction;

	for(uint16_t i = 1; i <= size + 1; i++)
		_txbuf[i] = txbuf[i - 1];

	SPITransmit(_txbuf, size + 1);
}

uint8_t MCP2515ReadStatus(void){
	uint8_t 					txbuf[] 		= {kCANReadStatus};
	uint8_t						rxbuf[2];

	SPIReceiveTransmit(rxbuf, txbuf, 2);

	return rxbuf[0];
}

void MCP2515WriteByte(uint8_t address, uint8_t byte){
	uint8_t 					txbuf[] 		= {kCANWrite, address, byte};

	SPITransmit(txbuf, 3);
}

void MCP2515EnableInterrupt(uint8_t mask){
	MCP2515WriteByte(0x2B, mask);
}

void MCP2515SetMessagePriority(uint8_t numbuffer, uint8_t priority){
	uint8_t 					address 		= 0;
	switch (numbuffer) {
	case 0:
		address = 0x30;
		break;
	case 1:
		address = 0x40;
		break;
	case 2:
		address = 0x50;
		break;
	default:
		break;
	}

	MCP2515WriteByte(address, priority);
}

void MCP2515SendMessage(uint8_t numbuffer, uint16_t identifier, uint8_t priority, uint8_t* message, uint8_t size){
	uint8_t						txbuf[13]		= {0};

	txbuf[0] = ((identifier >> 3) & 0xFF);										// TXBnSIDH
	txbuf[1] = ((identifier & 0x7) << 5);										// TXBnSIDL
	txbuf[4] = (size & 0xF);													// TXBnDLC

	for(uint8_t i = 0; i <= size; i++)
		txbuf[i + 5] = message[i];

	MCP2515EnableInterrupt(kCAN_RX0 | kCAN_RX1 | kCAN_TX0 | kCAN_TX1 | kCAN_TX2);

	MCP2515SetMessagePriority(numbuffer, priority);

	switch (numbuffer) {
	case 0:
		MCP2515LoadTxBuffer(0, 0, 0, txbuf, 13);
		MCP2515RTS(0, 0, 1);
		break;
	case 1:
		MCP2515LoadTxBuffer(0, 1, 0, txbuf, 13);
		MCP2515RTS(0, 1, 0);
		break;
	case 2:
		MCP2515LoadTxBuffer(1, 0, 0, txbuf, 13);
		MCP2515RTS(1, 0, 0);
		break;
	default:
		break;
	}
}

void MCP2515HandleInterrupt(){
	uint8_t						interrupt 		= 0;

	interrupt = MCP2515ReadRegister(0x2C);

	switch (interrupt) {
	case kCAN_RX0:
		canintflags.intrx0if = true;
		SEGGER_RTT_printf(0, "RX0\n\r");
		break;
	case kCAN_RX1:
		canintflags.intrx1if = true;
		SEGGER_RTT_printf(0, "RX1\n\r");
		break;
	case kCAN_TX0:
		canintflags.inttx0if = true;
		SEGGER_RTT_printf(0, "TX0\n\r");
		break;
	case kCAN_TX1:
		canintflags.inttx1if = true;
		SEGGER_RTT_printf(0, "TX1\n\r");
		break;
	case kCAN_TX2:
		canintflags.inttx2if = true;
		SEGGER_RTT_printf(0, "TX2\n\r");
		break;
	case kCAN_ERR:
		canintflags.interrif = true;
		SEGGER_RTT_printf(0, "ERR\n\r");
		break;
	case kCAN_WAK:
		canintflags.intwakif = true;
		SEGGER_RTT_printf(0, "WAK\n\r");
		break;
	case kCAN_MERR:
		canintflags.intmerrf = true;
		SEGGER_RTT_printf(0, "MERR\n\r");
		break;
	default:
		break;
	}

	MCP2515BitModify(0x2C, interrupt, 0x00);									// Reset the interrupt flag(s)

	canintflags.intoccurred = false;
}

void MCP2515BitModify(uint8_t address, uint8_t mask, uint8_t value){
	uint8_t 					txbuf[4]		= {kCANBitModify, address, mask, value};

	SPITransmit(txbuf, 4);
}

/*
 * ****************************************************************************
 * SPI Custom
 * ****************************************************************************
 */
void SPITransmit(uint8_t* txbuf, uint8_t size){
	extern SPI_HandleTypeDef 	hspi1;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, txbuf, size, SPI_TIMEOUT_TICKS);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void SPIReceiveTransmit(uint8_t* rxbuf, uint8_t* txbuf, uint8_t size){
	extern SPI_HandleTypeDef 	hspi1;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(&hspi1, txbuf, rxbuf, size, SPI_TIMEOUT_TICKS);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

/*
 * ****************************************************************************
 * Interrupt Callbacks
 * ****************************************************************************
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch (GPIO_Pin) {
	case VL53L1X_INT0_Pin:
		if(vl53l1x_initdone_flag)
			osSignalSet(RangingTaskHandle, kVL53L1XInterrupt0);
		break;
	case VL53L1X_INT1_Pin:
		if(vl53l1x_initdone_flag)
			osSignalSet(RangingTaskHandle, kVL53L1XInterrupt1);
		break;
	case VL53L1X_INT2_Pin:
		if(vl53l1x_initdone_flag)
			osSignalSet(RangingTaskHandle, kVL53L1XInterrupt2);
		break;
	case CAN_INT_Pin:
		__asm("NOP");
		canintflags.intoccurred = true;
		break;
	default:
		break;
	}
}
