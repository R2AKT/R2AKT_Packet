 /******************************************************************************
 *
 *    FILE NAME : r2akt_packet.cpp
 *       AUTHOR : Sergey Dorozhkin (R2AKT)
 *         DATE : 15-november-2024
 *      VERSION : 0.0.2
 * MODIFICATION : 2
 *      PURPOSE : Arduino library for packet data exchange
 *          URL : https://github.com/R2AKT/r2akt_packet
 *
 ******************************************************************************/
#include <r2akt_packet.h>

 /******************************************************************************/
Packet::Packet (Stream *Port, uint8_t SrcAddr, uint16_t BuffSize, bool COBS_KISS, uint8_t ToglePin, bool Blocking, uint16_t TimeOut) {
	_Port = Port;
	_SrcAddr = SrcAddr;
	_BuffSize = BuffSize;
	if (_BuffSize > 250) {
		_COBS = false;
	} else {
		_COBS = COBS_KISS;
	}
	_ToglePin = ToglePin;
	_Blocking = Blocking;
	_TimeOut = TimeOut;
	//
	uint16_t PacketBuffSize;
	if (_COBS) { // COBS
		PacketBuffSize = _BuffSize+5;
	} else { // KISS (SLIP)
		PacketBuffSize = (_BuffSize+4)*2;
	}
	_PacketBuffSize = PacketBuffSize;
	PHY_Exchange_Rx = new uint8_t [_PacketBuffSize];
	//
	PHY_Error_Num = error_num_no_error;
	MAC_Error_Num = error_num_no_error;
	PACKET_Error_Num = error_num_no_error;
}
//
void Packet::begin (const uint8_t SrcAddr) {
	_SrcAddr = SrcAddr;
	if (_ToglePin != 0) {
		pinMode (_ToglePin, OUTPUT);
	}
	setRXmode();
	//
	PHY_Error_Num = error_num_no_error;
	MAC_Error_Num = error_num_no_error;
	PACKET_Error_Num = error_num_no_error;
	//
	//return void;
}
//
void Packet::setTXmode(void) {
	if (_ToglePin != 0) {
		digitalWrite (_ToglePin, packetTransmit);
	}
	delay (TxDelay);
	DEBUG_PRINTLN ("Togle pin - Tx.");
}
//
void Packet::setRXmode(void) {
	delay (TxTail);
	if (_ToglePin != 0) {
		digitalWrite (_ToglePin, packetReceive);
	}
	DEBUG_PRINTLN ("Togle pin - Rx.");
}
//
int Packet::available() {
	return _Port->available();
}
//
int Packet::read() {
  return _Port->read();
}
//
int Packet::peek() {
	return _Port->peek();
}
//
void Packet::flush() {
	_Port->flush();
}
//
int Packet::availableForWrite() {
	PHY_Error_Num = error_num_no_error;
	return _Port->availableForWrite();
}
//
size_t Packet::write (const uint8_t c) {
	size_t n = _Port->write (c);
	_Port->flush();
	if (n > 0) {
		PHY_Error_Num = error_num_no_error;
	} else {
		PHY_Error_Num = error_num_write_error;
	}
	return n;
}
//
size_t Packet::write (const char *array, size_t size) {
	int16_t n = write ((uint8_t *)array, size);
	if (n > 0) {
		PHY_Error_Num = error_num_no_error;
	} else {
		PHY_Error_Num = error_num_write_error;
	}
	return n;
}
//
size_t Packet::write (const uint8_t *array, size_t size) {
	size_t n = _Port->write (array, size);
	_Port->flush();
	if (n > 0) {
		PHY_Error_Num = error_num_no_error;
	} else {
		PHY_Error_Num = error_num_write_error;
	}
	return n;
}

 /******************************************************************************/
int16_t Packet::send_phy (const uint8_t *Buff, const size_t size) {
	DEBUG_PRINT (F ("Send PHY data. Data size "));
	DEBUG_PRINT (size);

	int16_t RAW_Len;
	int16_t RAW_BuffSize;
	
	DEBUG_PRINTLN (F ("Send 'Start' byte..."));
	setTXmode();//digitalWrite (_ToglePin, packetTransmit); // Enable RS-485 Tx Data

	if (_COBS) { // COBS
		if (size > _BuffSize+5) {
			DEBUG_PRINTLN (F ("PHY packet data len overflow!"));
			PHY_Error_Num = error_num_oversize;
			return -1;
		}
		RAW_BuffSize = size + 5;
	} else { // KISS (SLIP)
		if (size > (_BuffSize + 4)*2) {
			DEBUG_PRINTLN (F ("PHY packet data len overflow!"));
			PHY_Error_Num = error_num_oversize;
			return -1;
		}
		RAW_BuffSize = (size + 4) * 2;
	}

	uint8_t *PHY_Exchange_Tx = new uint8_t [RAW_BuffSize];
	if (_COBS) { // COBS
		RAW_Len = StuffData (PHY_Exchange_Tx, Buff, size);
	} else { // KISS (SLIP)
		RAW_Len = ESCData (PHY_Exchange_Tx, Buff, size);
	}

	DEBUG_PRINT (F (", RAW data size "));
	DEBUG_PRINTLN (RAW_Len+2);

	#ifdef DEBUG_ENABLE
		if (_COBS) { // COBS
			DEBUG_PRINT_HEX (_COBEND); // Frame 'Start'
		} else { // KISS (SLIP)
			DEBUG_PRINT_HEX (_FEND); // Frame 'Start'
		}
		for (uint16_t i = 0; i < RAW_Len; i++) {
			DEBUG_PRINT_HEX (PHY_Exchange_Tx[i]);
		}
		if (_COBS) { // COBS
			DEBUG_PRINT_HEX (_COBEND); // Frame 'End'
		} else { // KISS (SLIP)
			DEBUG_PRINT_HEX (_FEND); // Frame 'End'
		}
		DEBUG_PRINTLN (F (""));
	#endif

	if (_COBS) { // COBS
		_Port->write ((uint8_t)_COBEND); // Frame 'Start'
	} else { // KISS (SLIP)
		_Port->write ((uint8_t)_FEND); // Frame 'Start'
	}

	DEBUG_PRINTLN (F ("Send data byte..."));
	_Port->write (PHY_Exchange_Tx, RAW_Len);

	DEBUG_PRINTLN (F ("Send 'Stop' byte..."));

	if (_COBS) { // COBS
		_Port->write ((uint8_t)_COBEND); // Frame 'End'
	} else { // KISS (SLIP)
		_Port->write ((uint8_t)_FEND); // Frame 'End'
	}

	_Port->flush (); // Waits for the transmission to complete.
	//
	delete [] PHY_Exchange_Tx;
	//
	PHY_Error_Num = error_num_no_error;
	//
	setRXmode();//digitalWrite (_ToglePin, packetReceive); // Enable RS-485 Rx Data
	//
	return RAW_Len;
}
 /******************************************************************************/
int16_t Packet::receive_phy (uint8_t *Buff, bool Blocking, uint16_t TimeOut) {
	setRXmode();//digitalWrite (BusTxToglePin, RS485Receive); // Enable RS-485 Rx Data
	//
	uint16_t RxBuffLen;
	uint16_t WaitTime;
	unsigned long StartTime;
	bool Block;
	//
	if (Blocking || _Blocking) {
		Block = true;
		if (TimeOut == 0) {
			WaitTime = _TimeOut;
		} else {
			WaitTime = TimeOut;
		}
	} else {
		Block = false;
	}
	//
	StartTime = millis();
	//
	do {
		if (_Port->available() > 0) {
			int16_t incomingByte = _Port->read();
			if (incomingByte != -1) { // Data present
				if (((_COBS) && ((uint8_t) incomingByte == _COBEND)) || ((!_COBS) && ((uint8_t) incomingByte == _FEND))) { // COBS or KISS (SLIP)
					if (PHY_Packet_Status.packet_Rx_Sync) { // ReSync packet or End packet
						if (PHY_Packet_Status.packet_Rx_Len > 0) { // End packet
							DEBUG_PRINTLN (F ("Packet end byte!"));
							PHY_Packet_Status.packet_Rx_Sync = false;
							////
							if (_COBS) { // COBS
								RxBuffLen = DeStuffData (Buff, PHY_Exchange_Rx, PHY_Packet_Status.packet_Rx_Len);
							} else { // KISS (SLIP)
								RxBuffLen = DeESCData (Buff, PHY_Exchange_Rx, PHY_Packet_Status.packet_Rx_Len);
								if (RxBuffLen > _BuffSize) {
									DEBUG_PRINTLN (F ("DeESC overflow..."));
									PHY_Packet_Status.packet_Rx_Sync = false;
									PHY_Packet_Status.packet_Rx_Len = 0;
									PHY_Error_Num = error_num_oversize;
									return -1;
								}
							}
							PHY_Packet_Status.packet_Rx_Len = 0;
							PHY_Error_Num = error_num_no_error;
							return RxBuffLen;
						} else { // ReSync
							DEBUG_PRINTLN (F ("ReSync..."));
							PHY_Packet_Status.packet_Rx_Len = 0;
							if (!Block) { 
								PHY_Error_Num = error_num_no_error;
								return 0;
							}
						}
					} else {
						DEBUG_PRINTLN (F ("Packet start byte!"));
						PHY_Packet_Status.packet_Rx_Len = 0;
						PHY_Packet_Status.packet_Rx_Sync = true;
						if (!Block) { 
							PHY_Error_Num = error_num_no_error;
							return 0;
						}
					}
				} else {
					if (PHY_Packet_Status.packet_Rx_Sync) {
						DEBUG_PRINTLN (F ("Packet data byte..."));
						if (PHY_Packet_Status.packet_Rx_Len > _PacketBuffSize) { // Rx buffer overflow
							DEBUG_PRINTLN (F ("Rx buffer overflow..."));
							PHY_Packet_Status.packet_Rx_Sync = false;
							PHY_Packet_Status.packet_Rx_Len = 0;
							PHY_Error_Num = error_num_oversize;
							return -1;
						} else {
							PHY_Exchange_Rx[PHY_Packet_Status.packet_Rx_Len++] = (uint8_t)(incomingByte&UCHAR_MAX);
							if (!Block) { 
								PHY_Error_Num = error_num_no_error;
								return 0;
							}
						}
					} else {
							DEBUG_PRINTLN (F ("'Noise' data byte..."));
						if (!Block) { 
							PHY_Error_Num = error_num_error_data;
							return -1;
						}
					}
				}
			} else {
					DEBUG_PRINTLN (F ("No data is available..."));
				if (!Block) { 
					PHY_Error_Num = error_num_no_data;
					return 0;
				}
			}
		} else {
				DEBUG_PRINTLN (F ("No Rx data..."));
			if (!Block) { 
				PHY_Error_Num = error_num_no_data;
				return 0;
			}
		}
		//
		if ((millis() - StartTime) > WaitTime) {
			PHY_Error_Num = error_num_timeout;
			return -1;
		}
	} while (Block);
	PHY_Error_Num = error_num_unkw_error;
	return -1;
}
 /******************************************************************************/
int16_t Packet::send_mac (const uint8_t DstAddr, const uint8_t *Buff, const size_t size) {
	if (size > _BuffSize) {
		DEBUG_PRINTLN (F ("MAC packet data len overflow!"));
		MAC_Error_Num = error_num_oversize;
		return -1;
	}
	uint8_t *MAC_Packet_Tx = new uint8_t [size + 2];

	///
	DEBUG_PRINT (F ("MAC Destination address "));
	DEBUG_PRINTLN_HEX (DstAddr);
	//
	DEBUG_PRINT (F ("MAC Source address "));
	DEBUG_PRINTLN_HEX (_SrcAddr);
	//
	DEBUG_PRINT (F ("Packet size "));
	DEBUG_PRINTLN (size);
	//
	MAC_Packet_Tx[0] = DstAddr;
	MAC_Packet_Tx[1] = _SrcAddr;
	//
	memcpy (MAC_Packet_Tx+2, Buff, size);
	//
	#ifdef DEBUG_ENABLE
		DEBUG_PRINTLN (F ("RAW MAC packet"));
		for (uint16_t i = 0; i < size + 2; i++) {
			DEBUG_PRINT_HEX (MAC_Packet_Tx[i]);
		}
		DEBUG_PRINTLN (F (""));
	#endif

	DEBUG_PRINTLN (F ("Send MAC Packet"));

	int16_t n = send_phy (MAC_Packet_Tx, size + 2);
	delete [] MAC_Packet_Tx;

	if (n > 0) {
		MAC_Error_Num = error_num_no_error;
		return size;
	} else {
		MAC_Error_Num = error_num_write_error;
		return -1;
	}
}
 /******************************************************************************/
int16_t Packet::receive_mac (uint8_t *Buff, uint8_t *SrcAddr, bool Blocking, uint16_t TimeOut) {
	uint8_t *MAC_Packet_Rx = new uint8_t [_BuffSize + 2];
	int16_t phy_rx_len;
	uint16_t WaitTime;
	unsigned long StartTime;
	bool Block;
	//
	if (Blocking || _Blocking) {
		Block = true;
		if (TimeOut == 0) {
			WaitTime = _TimeOut;
		} else {
			WaitTime = TimeOut;
		}
	} else {
		Block = false;
	}
	//
	StartTime = millis();
	//
	do {
		phy_rx_len = receive_phy (MAC_Packet_Rx, Block, WaitTime);
		if (phy_rx_len > 0) {
			if ((MAC_Packet_Rx[0] == _SrcAddr) || (MAC_Packet_Rx[0] == 0xFF)) { // Our or Broadcast address
				*SrcAddr = MAC_Packet_Rx[1];
				memcpy (Buff, MAC_Packet_Rx+2, phy_rx_len - 2);
				delete [] MAC_Packet_Rx;
				if (*SrcAddr == _SrcAddr) { // Source our address ! Loop ?!
					MAC_Error_Num = error_num_read_error_souce_self;
				} else if (*SrcAddr == 0xFF) { // Source boadcast address ! Error ?!
					MAC_Error_Num = error_num_read_error_souce_broadcast;
				}
				return phy_rx_len - 2;
			} else { // Not Our or Broadcast address
				delete [] MAC_Packet_Rx;
				MAC_Error_Num = error_num_no_error;
				return 0;
			}
		} else {
			if (!Block) { 
				delete [] MAC_Packet_Rx;
				MAC_Error_Num = error_num_read_error;
				return -1;
			}
		}
		//
		if ((millis() - StartTime) > WaitTime) {
			delete [] MAC_Packet_Rx;
			MAC_Error_Num = error_num_timeout;
			return -1;
		}
	} while (Block);
	delete [] MAC_Packet_Rx;
	MAC_Error_Num = error_num_unkw_error;
	return -1;
}
 /******************************************************************************/
int16_t Packet::packet_send_to (const uint8_t DstAddr, const uint8_t *Buff, const uint16_t size) {
	if (size > _BuffSize) {
		DEBUG_PRINTLN (F ("APP packet data len overflow!"));
		PACKET_Error_Num = error_num_oversize;
		return -1;
	}
	uint8_t *MAC_Data = new uint8_t [size + 2];

	DEBUG_PRINT (F ("Send packet to address "));
	DEBUG_PRINTLN_HEX (DstAddr);

	memcpy (MAC_Data, Buff, size);
	//
	uint16_t CRC16 = USHRT_MAX;
	CRC16 = crc16_calc_poly (CRC16, MAC_Data, size);
	MAC_Data[size] = (CRC16>>CHAR_BIT)&UCHAR_MAX;
	MAC_Data[size+1] = CRC16&UCHAR_MAX;

	DEBUG_PRINT (F ("Packet size "));
	DEBUG_PRINTLN (size+2); // Data+CRC16

	DEBUG_PRINT (F ("Packet CRC "));
	DEBUG_PRINTLN_HEX (CRC16);

	int16_t n = send_mac (DstAddr, MAC_Data, size+2); // Data+CRC16
	delete [] MAC_Data;

	if (n > 0) {
		PACKET_Error_Num = error_num_no_error;
		return n;
	} else {
		PACKET_Error_Num = error_num_write_error;
		return -1;
	}
}
 /******************************************************************************/
int16_t Packet::packet_receive_from (uint8_t *Buff, const uint8_t SrcAddr, bool Blocking, uint16_t TimeOut) {
	uint8_t RxSrcAddr;
	int16_t mac_rx_len;
	uint8_t *MAC_Data = new uint8_t [_BuffSize + 2];
	uint16_t Rx_CRC;
	uint16_t WaitTime;
	unsigned long StartTime;
	bool Block;
	//
	if (Blocking || _Blocking) {
		Block = true;
		if (TimeOut == 0) {
			WaitTime = _TimeOut;
		} else {
			WaitTime = TimeOut;
		}
	} else {
		Block = false;
	}
	//
	StartTime = millis();
	//
	do {
		mac_rx_len = receive_mac (MAC_Data, &RxSrcAddr, Block, WaitTime);

		if (mac_rx_len > 0) {
			uint16_t CRC16 = USHRT_MAX;
			CRC16 = crc16_calc_poly (CRC16, MAC_Data, mac_rx_len-2);

			DEBUG_PRINT (F ("Calculated packet CRC "));
			DEBUG_PRINT_HEX (CRC16);
			DEBUG_PRINT (F (", Receive packet CRC "));

			Rx_CRC = (((MAC_Data[mac_rx_len-2])&UCHAR_MAX)<<CHAR_BIT) + (MAC_Data[mac_rx_len-1]&UCHAR_MAX);
			DEBUG_PRINTLN_HEX (Rx_CRC);

			if (Rx_CRC == CRC16) { // CRC OK
				if (RxSrcAddr == SrcAddr) {
					memcpy (Buff, MAC_Data, mac_rx_len-2);
					delete [] MAC_Data;
					PACKET_Error_Num = error_num_no_error;
					return mac_rx_len - 2;
				} else {
					if (!Block) { 
						delete [] MAC_Data;
						PACKET_Error_Num = error_num_no_data;
						return 0;
					}
				}
			} else { // CRC Error!
				if (!Block) { 
					delete [] MAC_Data;
					PACKET_Error_Num = error_num_crc_error;
					return -1;
				}
			}
		} else { // No data
			if (!Block) { 
				delete [] MAC_Data;
				PACKET_Error_Num = error_num_no_data;
				return 0;
			}
		}
		//
		if ((millis() - StartTime) > WaitTime) {
			delete [] MAC_Data;
			PACKET_Error_Num = error_num_timeout;
			return -1;
		}
	} while (Block);
	PACKET_Error_Num = error_num_unkw_error;
	delete [] MAC_Data;
	return -1;
}
 /******************************************************************************/
int16_t Packet::packet_receive (uint8_t *Buff, uint8_t *SrcAddr, bool Blocking, uint16_t TimeOut) {
	uint8_t RxSrcAddr;
	int16_t mac_rx_len;
	uint8_t *MAC_Data = new uint8_t [_BuffSize + 2];
	uint16_t Rx_CRC;
	uint16_t WaitTime;
	unsigned long StartTime;
	bool Block;
	//
	if (Blocking || _Blocking) {
		Block = true;
		if (TimeOut == 0) {
			WaitTime = _TimeOut;
		} else {
			WaitTime = TimeOut;
		}
	} else {
		Block = false;
	}
	//
	StartTime = millis();
	//
	do {
		mac_rx_len = receive_mac (MAC_Data, &RxSrcAddr, Block, WaitTime);

		if (mac_rx_len > 0) {
			uint16_t CRC16 = USHRT_MAX;
			CRC16 = crc16_calc_poly (CRC16, MAC_Data, mac_rx_len-2);

			DEBUG_PRINT (F ("Calculated packet CRC "));
			DEBUG_PRINT_HEX (CRC16);
			DEBUG_PRINT (F (", Receive packet CRC "));

			Rx_CRC = (((MAC_Data[mac_rx_len-2])&UCHAR_MAX)<<CHAR_BIT) + (MAC_Data[mac_rx_len-1]&UCHAR_MAX);
			DEBUG_PRINTLN_HEX (Rx_CRC);

			if (Rx_CRC == CRC16) { // CRC OK
				*SrcAddr = RxSrcAddr;
				memcpy (Buff, MAC_Data, mac_rx_len-2);
				delete [] MAC_Data;
				PACKET_Error_Num = error_num_no_error;
				return mac_rx_len - 2;
			} else { // CRC Error!
				delete [] MAC_Data;
				PACKET_Error_Num = error_num_crc_error;
				return -1;
			}
		} else { // No data
			if (!Block) { 
				delete [] MAC_Data;
				PACKET_Error_Num = error_num_no_data;
				return 0;
			}
		}
		//
		if ((millis() - StartTime) > WaitTime) {
			delete [] MAC_Data;
			PACKET_Error_Num = error_num_timeout;
			return -1;
		}
	} while (Block);
	delete [] MAC_Data;
	PACKET_Error_Num = error_num_unkw_error;
	return -1;
}
 /************************************************************** END OF FILE ***/