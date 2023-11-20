/*! 
 *  @file MotorCiA402.h
 *  @brief MotorCiA402
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Nov. 20. 2023
 *  @Comm
 */

#ifndef MOTORCIA402_H_
#define MOTORCIA402_H_

#include <string.h>
#include <unistd.h>
#include "PCANDevice.h"
#include "CiA402_Object_Dictionary.h"

#define DEBUG_PRINT 1



class Motor_CiA402: public PCANDevice {
public:
	Motor_CiA402();
	virtual ~Motor_CiA402();

	void SDO_CONTROLWORD(int NodeID, int RW, unsigned char data);
	void SDO_MODES_OPERTAION(unsigned char NodeID, int RW, unsigned char data);
	void SDO_TARGET_TORQUE(unsigned char NodeID, int val);
	int SDO_RATE_CURRENT(unsigned char NodeID);
	int SDO_RECEIVE(void);
	void SDO_SEND(uint32_t _cob_id, uint8_t *_s_packet, uint16_t _length);

	void NMT_STATE(unsigned char NodeID, unsigned char data);

    // Process Data Objects (PDO)
	void PDO_STOP(unsigned char NodeID, unsigned char PDO_VAL);
    void TxPDO1_MAPPING(unsigned char NodeID);
    void TxPDO2_MAPPING(unsigned char NodeID);
    void RxPDO1_MAPPING(unsigned char NodeID);

    void RxPDO1_SEND(unsigned char NodeID, short RPDO_VAL);

    void TxPDO1_READ(int *d1, int *d2, int *d3);
	void TxPDO1_READ(int *d1, int *d2);   

	void SYNC(void);
	void Motor_STATE(int *d1, int *d2, int *d3);

    // Debug
	void Print_CAN_FRAME(int type);

	void motor_activate(int ID);
	void motor_deactivate(int ID);

	void activate_all(const std::string &_device_id, Config_t &_config);
	void deactivate_all(void);
private:
	int res;
	uint32_t cob;
	SDO_PACKET s_packet;
	DATA_OBJECT s_obj;
	int motor_id;
	unsigned char motor_c[4], motor_p[4];
	struct CAN_msg_t can_motor;

    std::string device_id;
    Config_t config;
};

#endif /* MOTORCIA402_H_ */
