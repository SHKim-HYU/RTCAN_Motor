/*! 
 *  @file MotorCiA402.cpp
 *  @brief MotorCiA402
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Nov. 20. 2023
 *  @Comm
 */

#include "MotorCiA402.h"

Motor_CiA402::Motor_CiA402() {
	cob = 0x000;
	rx_frame.id = cob; 
	motor_id = 0;
	res = 0;
	n_Nodes = 0;
}

Motor_CiA402::~Motor_CiA402() {
}

void Motor_CiA402::SDO_CONTROLWORD(int NodeID, int RW, unsigned char data)
{
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));

	switch(RW)
	{
	case OBJ_READ:

		s_obj.uint16Value[0] = OBJ_STATUSWORD;

		memset(&s_packet, 0, sizeof(SDO_PACKET));
		s_packet.info.type = READ_REQUEST;
		s_packet.info.index_low = s_obj.uint8Value[0];
		s_packet.info.index_high = s_obj.uint8Value[1];
		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
		cob = COB_RxSDO+NodeID;
		SDO_SEND(cob, s_packet.value, 4);
		Print_CAN_FRAME(OBJ_WRITE);
		res = SDO_RECEIVE();
		// if (!res) return false;
		Print_CAN_FRAME(OBJ_READ);
		break;
	case OBJ_WRITE:

		s_obj.uint16Value[0] = OBJ_CONTROLWORD;

		if( data == CONTROL_COMMAND_SHUTDOWN
				|| data == CONTROL_COMMAND_SWITCH_ON
				|| data == CONTROL_COMMAND_ENABLE_OPERATION
				|| data == CONTROL_COMMAND_QUICK_STOP )
		{
			s_packet.info.type = WRITE_REQUEST_2BYTE;
			s_packet.info.index_low = s_obj.uint8Value[0];
			s_packet.info.index_high = s_obj.uint8Value[1];
			s_packet.info.subindex = OBJ_SUBINDEX_NULL;
			s_packet.info.data[0] = data;
			cob = COB_RxSDO+NodeID;
			SDO_SEND(cob, s_packet.value, 5);
			Print_CAN_FRAME(OBJ_WRITE);
			usleep(10000);
			res = SDO_RECEIVE();
			// if (!res) return false;
			Print_CAN_FRAME(OBJ_READ);

		}
		break;
	default:
		break;

	}
}

void Motor_CiA402::SDO_MODES_OPERTAION(unsigned char NodeID, int RW, unsigned char data)
{
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));

	switch(RW)
	{
	case OBJ_READ:

		s_obj.uint16Value[0] = OBJ_OPERATIONMODE_MONITOR;

		s_packet.info.type = READ_REQUEST;
		s_packet.info.index_low = s_obj.uint8Value[0];
		s_packet.info.index_high = s_obj.uint8Value[1];
		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
		cob = COB_RxSDO+NodeID;
		SDO_SEND(cob, s_packet.value, 4);
		Print_CAN_FRAME(OBJ_WRITE);
		res = SDO_RECEIVE();
		// if (!res) return false;
		Print_CAN_FRAME(OBJ_READ);
		break;
	case OBJ_WRITE:

		s_obj.uint16Value[0] = OBJ_OPERATIONMODE;
		if(data == OP_MODE_NO_MODE || data == OP_MODE_PROFILE_POSITION || data == OP_MODE_CYCLIC_SYNC_TORQUE)
		{
			s_packet.info.type = WRITE_REQUEST_1BYTE;
			s_packet.info.index_low = s_obj.uint8Value[0];
			s_packet.info.index_high = s_obj.uint8Value[1];
			s_packet.info.subindex = OBJ_SUBINDEX_NULL;
			s_packet.info.data[0] = data;
			cob = COB_RxSDO+NodeID;
			SDO_SEND(cob, s_packet.value, 5);
			if (DEBUG_PRINT) printf("SDO: mode of operation\n");
			Print_CAN_FRAME(OBJ_WRITE);
			res = SDO_RECEIVE();
			// if (!res) return false;
			Print_CAN_FRAME(OBJ_READ);
		}

		break;
	default:
		break;

	}
}

void Motor_CiA402::SDO_TARGET_TORQUE(unsigned char NodeID, int val)
{
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));

	cob = COB_RxSDO+NodeID;

	s_obj.uint16Value[0] = 0x6074;

	s_packet.info.type = WRITE_REQUEST_2BYTE;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = OBJ_SUBINDEX_NULL;

	unsigned short tmp = (unsigned short)val;

	s_packet.info.data[0] = tmp & 0x00FF;
	s_packet.info.data[1] = (tmp & 0xFF00) >> 8;

	SDO_SEND(cob, s_packet.value, 6);
	res = SDO_RECEIVE();
	// if (!res) return false;
}

int Motor_CiA402::SDO_RATE_CURRENT(unsigned char NodeID)
{

	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));

	s_obj.uint16Value[0] = OBJ_RATE_CURRENT;
	s_packet.info.type = READ_REQUEST;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = OBJ_SUBINDEX_NULL;
	cob = COB_RxSDO+NodeID;
	SDO_SEND(cob, s_packet.value, 6);
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);

	int res = (int)(tx_frame.data[4] + (tx_frame.data[5]<<8) 
					+ (tx_frame.data[6]<<16) + (tx_frame.data[7]<<24));
	return res;
}

int Motor_CiA402::SDO_ENCODER_RESOLUTION(unsigned char NodeID)
{
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));

	s_obj.uint16Value[0] = OBJ_ENCODER_RESOLUTION;
	s_packet.info.type = READ_REQUEST;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = 0x01;
	cob = COB_RxSDO+NodeID;
	SDO_SEND(cob, s_packet.value, 6);
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);

	int res = (int)(tx_frame.data[4] + (tx_frame.data[5]<<8) 
					+ (tx_frame.data[6]<<16) + (tx_frame.data[7]<<24));
	return res;
}

int Motor_CiA402::SDO_TORQUE_CONSTANT(unsigned char NodeID)
{
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));

	s_obj.uint16Value[0] = OBJ_TORQUE_CONSTANT;
	s_packet.info.type = READ_REQUEST;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = OBJ_SUBINDEX_NULL;
	cob = COB_RxSDO+NodeID;
	SDO_SEND(cob, s_packet.value, 6);
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);

	int res = (int)(tx_frame.data[4] + (tx_frame.data[5]<<8) 
					+ (tx_frame.data[6]<<16) + (tx_frame.data[7]<<24));
	return res;
}

int Motor_CiA402::SDO_MOTOR_DIRECTION(unsigned char NodeID)
{
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));

	s_obj.uint16Value[0] = OBJ_MOTOR_DIRECTION;
	s_packet.info.type = READ_REQUEST;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = OBJ_SUBINDEX_NULL;
	cob = COB_RxSDO+NodeID;
	SDO_SEND(cob, s_packet.value, 6);
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);

	int res = (int)(tx_frame.data[4]);
	if (res == 0) res=1;	// 0 - nomal  -->  1 - nomal
	else res=-1;			// 1 - invert --> -1 - invert
	return res;
}

void Motor_CiA402::SDO_READ_COB_ID(unsigned char PDO_VAL)
{
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));

	switch(PDO_VAL)
	{
	case 1:
		s_obj.uint16Value[0] = TxPDO1_COMM;
		s_packet.info.type = READ_REQUEST;
		s_packet.info.index_low = s_obj.uint8Value[0];
		s_packet.info.index_high = s_obj.uint8Value[1];
		s_packet.info.subindex = 0x01;

		break;
	case 2:
		s_obj.uint16Value[0] = TxPDO2_COMM;
		s_packet.info.type = READ_REQUEST;
		s_packet.info.index_low = s_obj.uint8Value[0];
		s_packet.info.index_high = s_obj.uint8Value[1];
		s_packet.info.subindex = 0x01;

		break;
	case 3:
		s_obj.uint16Value[0] = TxPDO3_COMM;
		s_packet.info.type = READ_REQUEST;
		s_packet.info.index_low = s_obj.uint8Value[0];
		s_packet.info.index_high = s_obj.uint8Value[1];
		s_packet.info.subindex = 0x01;

		break;
	case 4:
		s_obj.uint16Value[0] = TxPDO4_COMM;
		s_packet.info.type = READ_REQUEST;
		s_packet.info.index_low = s_obj.uint8Value[0];
		s_packet.info.index_high = s_obj.uint8Value[1];
		s_packet.info.subindex = 0x01;

		break;
	default:
		break;
	}
	SDO_SEND(cob, s_packet.value, 4);
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);
}

int Motor_CiA402::SDO_RECEIVE(void)
{
	memset(&can_motor, 0, sizeof(CAN_msg_t));
	memset(&tx_frame, 0, sizeof(CAN_msg_t));
	
	// Loop until Receive returns true or timeout occurs
    const int maxAttempts = 1000; // Example limit, adjust as needed
    int attempts = 0;
    bool received = false;
    while (attempts < maxAttempts && !received) {
        received = Receive(tx_frame);
		usleep(1000);
        attempts++;
    }
    if (!received) {
        // Handle the case where Receive never returned true
        return false; // or some error code
    }

	return true;
}

void Motor_CiA402::SDO_SEND(uint32_t _cob_id, uint8_t *_s_packet, uint16_t _length)
{
	memset(&rx_frame, 0, sizeof(CAN_msg_t));
	rx_frame.id = _cob_id;
	rx_frame.length = _length;
	memcpy(rx_frame.data.data(), _s_packet, _length);
	Send(_cob_id, _s_packet, _length);
}

void Motor_CiA402::NMT_STATE(unsigned char NodeID, unsigned char data)
{
	memset(&s_packet, 0, sizeof(SDO_PACKET));

	cob = COB_NMT;
	s_packet.info.type = data;
	s_packet.info.index_low = NodeID;
	if (DEBUG_PRINT) printf("NMT State: 0x%02x, NodeID: 0x%02x\n",data, NodeID);
	SDO_SEND(cob, s_packet.value, 2);
	Print_CAN_FRAME(OBJ_WRITE);
	if (data == NMT_PREOP_MODE)
	{
		res = SDO_RECEIVE();
		// if (!res) return false;
		Print_CAN_FRAME(OBJ_READ);
	}
}

void Motor_CiA402::PDO_STOP(unsigned char NodeID, unsigned char PDO_VAL)
{
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));

	cob = COB_RxSDO+NodeID;
	if (DEBUG_PRINT) printf("TxPDO COB-ID\n");
	SDO_READ_COB_ID(PDO_VAL);

	// TxPDO Deleting
	if (DEBUG_PRINT) printf("TxPDO Deleting\n");
	switch(PDO_VAL)
	{
	case 1:
		s_obj.uint16Value[0] = TxPDO1_MAP;
		s_packet.info.type = WRITE_REQUEST_1BYTE;
		s_packet.info.index_low = s_obj.uint8Value[0];
		s_packet.info.index_high = s_obj.uint8Value[1];
		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
		s_packet.info.data[0] = 0x00;
		break;
	case 2:
		s_obj.uint16Value[0] = TxPDO2_MAP;
		s_packet.info.type = WRITE_REQUEST_1BYTE;
		s_packet.info.index_low = s_obj.uint8Value[0];
		s_packet.info.index_high = s_obj.uint8Value[1];
		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
		s_packet.info.data[0] = 0x00;
		break;
	case 3:
		s_obj.uint16Value[0] = TxPDO3_MAP;
		s_packet.info.type = WRITE_REQUEST_1BYTE;
		s_packet.info.index_low = s_obj.uint8Value[0];
		s_packet.info.index_high = s_obj.uint8Value[1];
		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
		s_packet.info.data[0] = 0x00;
		break;
	case 4:
		s_obj.uint16Value[0] = TxPDO4_MAP;
		s_packet.info.type = WRITE_REQUEST_1BYTE;
		s_packet.info.index_low = s_obj.uint8Value[0];
		s_packet.info.index_high = s_obj.uint8Value[1];
		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
		s_packet.info.data[0] = 0x00;
		break;
	default:
		break;
	}
	SDO_SEND(cob, s_packet.value, 5);
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);
	
	// RxPDO Stop
	if (DEBUG_PRINT) printf("RxPDO Stopping\n");
	switch(PDO_VAL)
	{
	case 1:
		s_obj.uint16Value[0] = RxPDO1_MAP;
		s_packet.info.type = WRITE_REQUEST_1BYTE;
		s_packet.info.index_low = s_obj.uint8Value[0];
		s_packet.info.index_high = s_obj.uint8Value[1];
		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
		s_packet.info.data[0] = 0x00;
		break;
	case 2:
		s_obj.uint16Value[0] = RxPDO2_MAP;
		s_packet.info.type = WRITE_REQUEST_1BYTE;
		s_packet.info.index_low = s_obj.uint8Value[0];
		s_packet.info.index_high = s_obj.uint8Value[1];
		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
		s_packet.info.data[0] = 0x00;
		break;
	case 3:
		s_obj.uint16Value[0] = RxPDO3_MAP;
		s_packet.info.type = WRITE_REQUEST_1BYTE;
		s_packet.info.index_low = s_obj.uint8Value[0];
		s_packet.info.index_high = s_obj.uint8Value[1];
		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
		s_packet.info.data[0] = 0x00;
		break;
	case 4:
		s_obj.uint16Value[0] = RxPDO4_MAP;
		s_packet.info.type = WRITE_REQUEST_1BYTE;
		s_packet.info.index_low = s_obj.uint8Value[0];
		s_packet.info.index_high = s_obj.uint8Value[1];
		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
		s_packet.info.data[0] = 0x00;
		break;
	default:
		break;
	}
	SDO_SEND(cob, s_packet.value, 5);
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);
	if (DEBUG_PRINT) printf("\n");
}


void Motor_CiA402::TxPDO1_MAPPING(unsigned char NodeID)
{
	// mapping objects
	// 4byte actual position, 4byte actual velocity
	cob = COB_RxSDO+NodeID;

	if (DEBUG_PRINT) printf("TxPDO1 Mapping\n");

	//pdo - actual position
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));
	s_obj.uint16Value[0] = TxPDO1_MAP;
	s_obj.uint16Value[1] = OBJ_POSITION_ACTUAL;

	s_packet.info.type = WRITE_REQUEST_4BYTE;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = 0x01;

	s_packet.info.data[0] = PDO_4BYTE;
	s_packet.info.data[1] = 0x00;
	s_packet.info.data[2] = s_obj.uint8Value[2];
	s_packet.info.data[3] = s_obj.uint8Value[3];

	SDO_SEND(cob, s_packet.value, 8);
	if (DEBUG_PRINT) printf("TxPDO1-01: actual position (0x6064)\n");
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);
	
	//pdo - actual velocity
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));
	s_obj.uint16Value[0] = TxPDO1_MAP;
	s_obj.uint16Value[1] = OBJ_VELOCITY_ACTUAL;

	s_packet.info.type = WRITE_REQUEST_4BYTE;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = 0x02;

	s_packet.info.data[0] = PDO_4BYTE;
	s_packet.info.data[1] = 0x00;
	s_packet.info.data[2] = s_obj.uint8Value[2];
	s_packet.info.data[3] = s_obj.uint8Value[3];

	SDO_SEND(cob, s_packet.value, 8);
	if (DEBUG_PRINT) printf("TxPDO1-02: actual velocity (0x606c)\n");
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);

	// regist the object
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));
	s_obj.uint16Value[0] = TxPDO1_MAP;
	s_packet.info.type = WRITE_REQUEST_1BYTE;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = OBJ_SUBINDEX_NULL;

	s_packet.info.data[0] = 0x02;
	SDO_SEND(cob, s_packet.value, 5);
	if (DEBUG_PRINT) printf("TxPDO1: register object\n");
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);

	// set sync(transmission type) subindex:0x02, UINT8
	// Acyclic synchronous: 0x00, Cyclic synchronous: 0x01~0xF0, Cyclic update: 0xFE, Acyclic synchronous: 0xFF
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));
	s_obj.uint16Value[0] = TxPDO1_COMM;

	s_packet.info.type = WRITE_REQUEST_1BYTE;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = 0x02;

	s_packet.info.data[0] = 0x01;
	SDO_SEND(cob, s_packet.value, 5);
	if (DEBUG_PRINT) printf("TxPDO1-Comm: set sync(transmission type)\n");
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);

	if (DEBUG_PRINT) printf("\n");
}

void Motor_CiA402::TxPDO2_MAPPING(unsigned char NodeID)
{
	// mapping objects
	// 2byte actual current, 2byte status word, 1byte mode of operation
	cob = COB_RxSDO+NodeID;

	if (DEBUG_PRINT) printf("TxPDO2 Mapping\n");

	//pdo - actual current
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));
	s_obj.uint16Value[0] = TxPDO2_MAP;
	s_obj.uint16Value[1] = OBJ_CURRENT_ACTUAL;

	s_packet.info.type = WRITE_REQUEST_4BYTE;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = 0x01;

	s_packet.info.data[0] = PDO_2BYTE;
	s_packet.info.data[1] = 0x00;
	s_packet.info.data[2] = s_obj.uint8Value[2];
	s_packet.info.data[3] = s_obj.uint8Value[3];

	SDO_SEND(cob, s_packet.value, 8);
	if (DEBUG_PRINT) printf("TxPDO2-01: actual current (0x6078)\n");
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);

	//pdo - statusword
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));
	s_obj.uint16Value[0] = TxPDO2_MAP;
	s_obj.uint16Value[1] = OBJ_STATUSWORD;

	s_packet.info.type = WRITE_REQUEST_4BYTE;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = 0x02;

	s_packet.info.data[0] = PDO_2BYTE;
	s_packet.info.data[1] = 0x00;
	s_packet.info.data[2] = s_obj.uint8Value[2];
	s_packet.info.data[3] = s_obj.uint8Value[3];

	SDO_SEND(cob, s_packet.value, 8);
	if (DEBUG_PRINT) printf("TxPDO2-02: statusword (0x6041)\n");
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);

	//pdo - mode of operation display
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));
	s_obj.uint16Value[0] = TxPDO2_MAP;
	s_obj.uint16Value[1] = OBJ_OPERATIONMODE_MONITOR;

	s_packet.info.type = WRITE_REQUEST_4BYTE;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = 0x03;

	s_packet.info.data[0] = PDO_1BYTE;
	s_packet.info.data[1] = 0x00;
	s_packet.info.data[2] = s_obj.uint8Value[2];
	s_packet.info.data[3] = s_obj.uint8Value[3];

	SDO_SEND(cob, s_packet.value, 8);
	if (DEBUG_PRINT) printf("TxPDO2-03: mode of operation display (0x6061)\n");
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);

	// regist the object
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));
	s_obj.uint16Value[0] = TxPDO2_MAP;
	s_packet.info.type = WRITE_REQUEST_1BYTE;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = OBJ_SUBINDEX_NULL;

	s_packet.info.data[0] = 0x03;
	SDO_SEND(cob, s_packet.value, 5);
	if (DEBUG_PRINT) printf("TxPDO2: register object\n");
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);

	// set sync(transmission type) subindex:0x02 UINT8
	// Acyclic synchronous: 0x00, Cyclic synchronous: 0x01~0xF0, Cyclic update: 0xFE, Acyclic synchronous: 0xFF
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));
	s_obj.uint16Value[0] = TxPDO2_COMM;

	s_packet.info.type = WRITE_REQUEST_1BYTE;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = 0x02;

	s_packet.info.data[0] = 0x01;
	SDO_SEND(cob, s_packet.value, 5);
	if (DEBUG_PRINT) printf("TxPDO2-Comm: set sync(transmission type)\n");
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);

	if (DEBUG_PRINT) printf("\n");
}

void Motor_CiA402::RxPDO1_MAPPING(unsigned char NodeID)
{
	// mapping objects
	// 2byte target-torque, 2byte controlword, 1byte mode of operation
	cob = COB_RxSDO + NodeID;
	
	if (DEBUG_PRINT) printf("RxPDO1 Mapping\n");

	//rpdo1-01 - target-torque
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));
	s_obj.uint16Value[0] = RxPDO1_MAP;
	s_obj.uint16Value[1] = OBJ_TARGET_TORQUE;

	s_packet.info.type = WRITE_REQUEST_4BYTE;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = 0x01;

	s_packet.info.data[0] = PDO_2BYTE;
	s_packet.info.data[1] = 0x00;
	s_packet.info.data[2] = s_obj.uint8Value[2];
	s_packet.info.data[3] = s_obj.uint8Value[3];
	SDO_SEND(cob, s_packet.value, 8);
	if (DEBUG_PRINT) printf("RxPDO1-01: target-torque\n");
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);
	usleep(10000);

	//rpdo1-02 - controlword
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));
	s_obj.uint16Value[0] = RxPDO1_MAP;
	s_obj.uint16Value[1] = OBJ_CONTROLWORD;

	s_packet.info.type = WRITE_REQUEST_4BYTE;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = 0x02;

	s_packet.info.data[0] = PDO_2BYTE;
	s_packet.info.data[1] = 0x00;
	s_packet.info.data[2] = s_obj.uint8Value[2];
	s_packet.info.data[3] = s_obj.uint8Value[3];
	SDO_SEND(cob, s_packet.value, 8);
	if (DEBUG_PRINT) printf("RxPDO1-02: controlword\n");
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);
	usleep(10000);

	//rpdo1-02 - 1 mapping object
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));
	s_obj.uint16Value[0] = RxPDO1_MAP;
	s_packet.info.type = WRITE_REQUEST_1BYTE;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = OBJ_SUBINDEX_NULL;
	s_packet.info.data[0] = 0x02;
	SDO_SEND(cob, s_packet.value, 5);
	if (DEBUG_PRINT) printf("RxPDO1: mapping object\n");
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);
	usleep(10000);

	// set sync(transmission type) subindex:0x02 UINT8
	// Synchronous: 0x00, Asynchronous: 0xFF
	memset(&s_obj, 0, sizeof(DATA_OBJECT));
	memset(&s_packet, 0, sizeof(SDO_PACKET));
	s_obj.uint16Value[0] = RxPDO1_COMM;

	s_packet.info.type = WRITE_REQUEST_1BYTE;
	s_packet.info.index_low = s_obj.uint8Value[0];
	s_packet.info.index_high = s_obj.uint8Value[1];
	s_packet.info.subindex = 0x02;

	s_packet.info.data[0] = 0xff;
	SDO_SEND(cob, s_packet.value, 5);
	if (DEBUG_PRINT) printf("RxPDO1-Comm: set sync(transmission type)\n");
	Print_CAN_FRAME(OBJ_WRITE);
	res = SDO_RECEIVE();
	// if (!res) return false;
	Print_CAN_FRAME(OBJ_READ);
	if (DEBUG_PRINT) printf("\n");
}

void Motor_CiA402::RxPDO1_SEND(unsigned char NodeID, short RPDO_VAL)
{
	cob = COB_RxPDO1 + NodeID;

	unsigned short tmp = (unsigned short)RPDO_VAL;

	s_packet.value[0] = tmp & 0x00FF;
	s_packet.value[1] = (tmp & 0xFF00) >> 8;

	s_packet.value[2] = 0x0f;
	s_packet.value[3] = 0x00;

	res = Send(cob, s_packet.value, 4);
	memset(&rx_frame, 0, sizeof(CAN_msg_t));
	rx_frame.id = cob; 
	memcpy(rx_frame.data.data(), s_packet.value, 4);
	rx_frame.length = 4;
	// usleep(50);
	// Print_CAN_FRAME(OBJ_WRITE);
}

void Motor_CiA402::SYNC(void)
{
	cob = COB_SYNC;
	Send(cob, 0, 0);
}

void Motor_CiA402::Motor_STATE(int *d1, int *d2, int *d3, int *d4, int *d5)
{
	SYNC();

	TxPDO_READ(d1,d2,d3,d4,d5);
}


void Motor_CiA402::Print_CAN_FRAME(int type)
{
#if DEBUG_PRINT
	switch(type)
	{
	case OBJ_WRITE:
		printf("Rx 0x%03x, %d\t", rx_frame.id, rx_frame.length);
		printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
				rx_frame.data[0],
				rx_frame.data[1],
				rx_frame.data[2],
				rx_frame.data[3],
				rx_frame.data[4],
				rx_frame.data[5],
				rx_frame.data[6],
				rx_frame.data[7]);
		break;
	case OBJ_READ:
		printf("Tx 0x%03x, %d\t", tx_frame.id, tx_frame.length);
		printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
				tx_frame.data[0],
				tx_frame.data[1],
				tx_frame.data[2],
				tx_frame.data[3],
				tx_frame.data[4],
				tx_frame.data[5],
				tx_frame.data[6],
				tx_frame.data[7]);
		break;
	case PDO_READ:
		printf("Tx 0x%03x, %d\t", can_motor.id, can_motor.length);
		printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
				can_motor.data[0],
				can_motor.data[1],
				can_motor.data[2],
				can_motor.data[3],
				can_motor.data[4],
				can_motor.data[5],
				can_motor.data[6],
				can_motor.data[7]);
		break;
	default:
		printf("Print error\n");
		break;
	}
#endif
}

void Motor_CiA402::TxPDO_READ(int *d1, int *d2, int *d3, int *d4, int *d5)
{
	while (Receive(can_motor))
	{
		motor_id = can_motor.id & 0x00F;
		cob = can_motor.id & 0xFF0;
		
		if(cob == COB_TxPDO1)
		{
			// printf("[TxPDO1] motor id: 0x%03x, 0x%03x\n", can_motor.id, cob);
			//TxPDO1-01 - actual position (4Byte)
			d1[motor_id-1] = (int)(can_motor.data[0] + (can_motor.data[1]<<8)
					+ (can_motor.data[2]<<16) + (can_motor.data[3]<<24));
			//TxPDO1-02 - actual velocity (4Byte)
			d2[motor_id-1] = (int)(can_motor.data[4] + (can_motor.data[5]<<8)
					+ (can_motor.data[6]<<16) + (can_motor.data[7]<<24));
		}
		else if(cob == COB_TxPDO2)
		{
			// printf("[TxPDO2] motor id: 0x%03x, 0x%03x\n", can_motor.id, cob);
			//TxPDO2-01 - actual torque (2Byte)
			d3[motor_id-1] = (short)(can_motor.data[0] + (can_motor.data[1]<<8));
			//TxPDO2-02 - statusword (2Byte)
			d4[motor_id-1] = (int)(can_motor.data[2] + (can_motor.data[3]<<8));
			//TxPDO2-03 - mode of operation (1Byte)
			d5[motor_id-1] = (int)(can_motor.data[4]);
		}
		else if(cob == COB_TxPDO3)
		{

		}
		else if(cob == COB_TxPDO4)
		{

		}
		else
		{
			
		}

	}
	// printf("\n\n");
	
}

void Motor_CiA402::motor_activate(int ID)
{
	if (DEBUG_PRINT) printf("Controlword: Shutdown\n");
	SDO_CONTROLWORD(ID, OBJ_WRITE, CONTROL_COMMAND_SHUTDOWN);
	usleep(10000);
	if (DEBUG_PRINT) printf("Controlword: Switch On\n");
	SDO_CONTROLWORD(ID, OBJ_WRITE, CONTROL_COMMAND_SWITCH_ON);
	usleep(10000);
	if (DEBUG_PRINT) printf("Controlword: Enable Operation\n");
	SDO_CONTROLWORD(ID, OBJ_WRITE, CONTROL_COMMAND_ENABLE_OPERATION);
	usleep(10000);
}

void Motor_CiA402::motor_deactivate(int ID)
{

	SDO_CONTROLWORD(ID, OBJ_WRITE, CONTROL_COMMAND_SHUTDOWN);
	usleep(10000);
	SDO_CONTROLWORD(ID, OBJ_WRITE, CONTROL_COMMAND_DISABLE_VOLTAGE);
	usleep(10000);
	//NMT_STATE(ID, NMT_RESET_NODE );
}

void Motor_CiA402::activate_all(const std::string &_device_id, Config_t &_config, int nodes)
{
	n_Nodes = nodes;
	int sock = Open(_device_id, _config, false);

    if (!sock)
    {
        printf("PCAN open error\n");
        return;
    }
    else
    {
		NMT_STATE(0, NMT_PREOP_MODE);
        // [Check]
        for(int i=1; i<=n_Nodes; ++i)
        {
            for(int j=1; j<=4; ++j)
			{
				if (DEBUG_PRINT) printf("pdo stop node: %d, pdo: %d\n",i,j);
                PDO_STOP(i, j);
			}

            TxPDO1_MAPPING(i);
            TxPDO2_MAPPING(i);
            RxPDO1_MAPPING(i);
            SDO_MODES_OPERTAION(i, OBJ_WRITE, OP_MODE_CYCLIC_SYNC_TORQUE);

            motor_activate(i);

        }

	    NMT_STATE(0, NMT_START_NODE);
    }
}

void Motor_CiA402::deactivate_all(void)
{
    // [Check]
	for(int i=1; i<=n_Nodes; ++i)
	{
		SDO_CONTROLWORD(i, OBJ_WRITE, CONTROL_COMMAND_SHUTDOWN);
		usleep(10000);
		SDO_CONTROLWORD(i, OBJ_WRITE, CONTROL_COMMAND_DISABLE_VOLTAGE);
		usleep(10000);
		//NMT_STATE(i, NMT_RESET_NODE );
	}

	Close();
}
