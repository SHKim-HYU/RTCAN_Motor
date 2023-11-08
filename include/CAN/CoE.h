/*! 
 *  @file CiA402_Object_Dictionary.h
 *  @brief CiA402_Object_Dictionary
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Oct. 26. 2023
 *  @Comm
 */

#ifndef CIA402_OBJECT_DICTIONARY_H_
#define CIA402_OBJECT_DICTIONARY_H_

// Definition
#define OBJ_READ 	1
#define OBJ_WRITE 	2

#define READ_REQUEST 			0x40
#define READ_1BYTE_RESPONSE 	0x4F
#define READ_2BYTE_RESPONSE 	0x4B
#define READ_4BYTE_RESPONSE 	0x43

#define WRITE_REQUEST_1BYTE 	0x2F
#define WRITE_REQUEST_2BYTE 	0x2B
#define WRITE_REQUEST_4BYTE 	0x23
#define WRITE_REQUEST_RESPONSE 	0x60

#define COB_NMT 0x000
#define COB_SYNC 0x80
#define COB_TPDO1 0x180
#define COB_RPDO1 0x200
#define COB_TPDO2 0x280
#define COB_RPDO2 0x300
#define COB_TPDO3 0x380
#define COB_RPDO3 0x400
#define COB_TPDO4 0x480
#define COB_RPDO4 0x500
#define COB_SDO 0x600

// Network Management (NMT)
#define NMT_START_NODE 0x01
#define NMT_STOP_NODE 0x02
#define NMT_PREOP_MODE 0x80
#define NMT_RESET_NODE 0x81
#define NMT_RESET_COMMU 0x82

// Statusword bits
#define STATUSWORD_READY_TO_SWITCH_ON_BIT 		0
#define STATUSWORD_SWITCHED_ON_BIT 				1
#define STATUSWORD_OPERATION_ENABLE_BIT 		2
#define STATUSWORD_FAULT_BIT 					3
#define STATUSWORD_VOLTAGE_ENABLE_BIT 			4
#define STATUSWORD_QUICK_STOP_BIT 				5
#define STATUSWORD_SWITCH_ON_DISABLE_BIT 		6
#define STATUSWORD_NO_USED_WARNING_BIT 			7
#define STATUSWORD_ELMO_NOT_USED_BIT 			8
#define STATUSWORD_REMOTE_BIT 					9
#define STATUSWORD_TARGET_REACHED_BIT 			10
#define STATUSWORD_INTERNAL_LIMIT_ACTIVE_BIT	11

// Object Dictionary
#define OBJ_STATUSWORD				0x6041U		// R,	UINT16
#define OBJ_CONTROLWORD				0x6040U		// RW,	UNIT16
#define OBJ_OPERATIONMODE			0x6060U		// RW,	UNIT8
#define OBJ_OPERATIONMODE_MONITOR	0x6061U		// R,	UINT8
#define OBJ_POSITION_ACTUAL 		0x6063U
#define OBJ_VELOCITY_ACTUAL 		0x6069U
#define OBJ_RATE_CURRENT			0x6075U
#define OBJ_CURRENT_ACTUAL 			0x6078U

// Status
#define STATUS_NOT_SPECIFIED					0xffff
#define STATUS_NOT_READY_TO_SWITCH_ON 			0x0000
#define STATUS_SWITCH_ON_DISABLED 				0x0040
#define STATUS_READY_TO_SWITCH_ON 				0x0021
#define STATUS_SWITCHED_ON						0x0233
#define STATUS_OPERATION_ENABLED 				0x0237
#define STATUS_QUICK_STOP_ACTIVE 				0x0007
#define STATUS_FAULT_REACTION_ACTIVE 			0x000f
#define STATUS_FAULT 							0x0008

// Control CoE FSM(Finite State Machine)
#define CONTROL_COMMAND_DISABLE_VOLTAGE				0x0000
#define CONTROL_COMMAND_QUICK_STOP					0x0002
#define CONTROL_COMMAND_SHUTDOWN					0x0006
#define CONTROL_COMMAND_SWITCH_ON					0x0007
#define CONTROL_COMMAND_ENABLE_OPERATION			0x000f
#define CONTROL_COMMAND_SWITCH_ON_ENABLE_OPERATION	0x000f
#define CONTROL_COMMAND_DISABLE_OPERATION			0x0007
#define CONTROL_COMMAND_FAULT_RESET					0x0080

// Operation mode
#define OP_MODE_NO_MODE					0x00
#define OP_MODE_PROFILE_POSITION		0x01
#define OP_MODE_VELOCITY				0x02
#define OP_MODE_PROFILE_VELOCITY		0x03
#define OP_MODE_TORQUE_PROFILE			0x04
#define OP_MODE_HOMING					0x06
#define OP_MODE_INTERPOLATED_POSITION	0x07
#define OP_MODE_CYCLIC_SYNC_POSITION	0x08
#define OP_MODE_CYCLIC_SYNC_VELOCITY	0x09
#define OP_MODE_CYCLIC_SYNC_TORQUE		0x0a

typedef union{
	struct{
		unsigned char 	type;
		unsigned char 	index_low;
		unsigned char 	index_high;
		unsigned char 	subindex;
		unsigned char 	data[4];
	}info;
	unsigned char value[8];
}SDO_PACKET;

typedef union{
	unsigned char uint8Value[4];
	unsigned short uint16Value[2];
	unsigned long uint32Value;
}DATA_OBJECT;

typedef uint64_t 		UINT64;
typedef int64_t 		INT64;
typedef unsigned int 	UINT32;
typedef int32_t 		INT32;
typedef int16_t 		INT16;
typedef uint16_t 		UINT16;
typedef uint8_t 		UINT8;
typedef int8_t 			INT8;

#endif /* CIA402_OBJECT_DICTIONARY_H_ */
