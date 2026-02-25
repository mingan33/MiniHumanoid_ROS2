#include "encos_motor_driver.hpp"

float fmaxf(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
    }

float fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
    }

float fmaxf3(float x, float y, float z){
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
    }

float fminf3(float x, float y, float z){
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
    }
    
void limit_norm(float *x, float *y, float limit){
    /// Scales the lenght of vector (x, y) to be <= limit ///
    float norm = sqrt(*x * *x + *y * *y);
    if(norm > limit){
        *x = *x * limit/norm;
        *y = *y * limit/norm;
        }
    }

int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }
	
union F32
{
	float v_float;
	unsigned int v_int;
	unsigned char buf[4];
}f32;

void float32_to_float16(float *float32,unsigned short int *float16)
{
	unsigned short int temp=0;
	f32.v_float=*float32;
	temp=(f32.buf[3]&0x7F)<<1|((f32.buf[2]&0x80)>>7);
	temp-=112;
	*float16=temp<<10|(f32.buf[2]&0x7F)<<3|f32.buf[1]>>5;
    *float16 |= ((f32.v_int & 0x80000000) >> 16);
}

void float16_to_float32(unsigned short int *float16,float *float32)
{
	unsigned short int temp2=0;
	f32.v_int=0;
	temp2=(((*float16&0x7C00)>>10)+112);
	f32.buf[3]=temp2>>1;
	f32.buf[2]=((temp2&0x01)<<7)|(*float16&0x03FC)>>3;
	f32.buf[1]=(*float16&0x03)<<6;
	f32.v_int |= ((*float16 & 0x8000) << 16);
	*float32=f32.v_float;
}


EncosMotorDriver::EncosMotorDriver(uint16_t motor_id, std::string can_interface, uint16_t master_id)
                             : can_(SocketCAN::get(can_interface)){   
    motor_id_ = motor_id;
    master_id_ = master_id;
    can_interface_ = can_interface;
    CanCbkFunc can_callback = std::bind(&EncosMotorDriver::CanRxMsgCallback, this, std::placeholders::_1);
    can_->add_can_callback(can_callback, master_id_);
}

EncosMotorDriver::~EncosMotorDriver() { can_->remove_can_callback(master_id_); }


//MOTOR SETTING
/*
cmd:
0x00:NON
0x01:set the communication mode to automatic feedback.
0x02:set the communication mode to response.
0x03:set the current position to zero.
*/
void EncosMotorDriver::MotorSetting(uint8_t cmd)
{
    can_frame tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x04;

	if(cmd==0) return;

    tx_frame.data[0] = motor_id_ >> 8;
    tx_frame.data[1] = motor_id_ & 0xFF;
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = cmd;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}


// This function use in ask communication mode.
/*
motor_id:1~0x7FE
kp:0~500
kd:0~50
pos:-12.5rad~12.5rad
spd:-18rad/s~18rad/s
tor:-30Nm~30Nm
*/
void EncosMotorDriver::MotorMitCtrlCmd(float pos,float spd,float kp,float kd,float tor)
{
	int kp_int;
	int kd_int;
	int pos_int;            
	int spd_int;
	int tor_int;
	
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;
		
	if(kp>KP_MAX) kp=KP_MAX;
		else if(kp<KP_MIN) kp=KP_MIN;
	if(kd>KD_MAX ) kd=KD_MAX;
		else if(kd<KD_MIN) kd=KD_MIN;	
	if(pos>POS_MAX)	pos=POS_MAX;
		else if(pos<POS_MIN) pos=POS_MIN;
	if(spd>SPD_MAX)	spd=SPD_MAX;
		else if(spd<SPD_MIN) spd=SPD_MIN;
	if(tor>T_MAX)	tor=T_MAX;
		else if(tor<T_MIN) tor=T_MIN;

    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9);
	pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);            
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);
    tor_int = float_to_uint(tor, T_MIN, T_MAX, 12);	
	
    tx_frame.data[0] = 0x00|(kp_int>>7);//kp5
	tx_frame.data[1] = ((kp_int&0x7F)<<1)|((kd_int&0x100)>>8);//kp7+kd1
	tx_frame.data[2] = kd_int&0xFF;
	tx_frame.data[3] = pos_int>>8;
	tx_frame.data[4] = pos_int&0xFF;
	tx_frame.data[5] = spd_int>>4;
	tx_frame.data[6] = (spd_int&0x0F)<<4|(tor_int>>8);
	tx_frame.data[7] = tor_int&0xff;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EncosMotorDriver::CanRxMsgCallback(const can_frame& rx_frame){
        uint8_t ack_status=0;
        int pos_int=0;            
        int spd_int=0;
        int cur_int=0;

		ack_status = rx_frame.data[0]>>5;
		error_id_ = rx_frame.data[0]&0x1F;
        
		if(ack_status==1)//response frame 1
		{
			pos_int = rx_frame.data[1]<<8|rx_frame.data[2];
			spd_int = rx_frame.data[3]<<4|(rx_frame.data[4]&0xF0)>>4;
			cur_int = (rx_frame.data[4]&0x0F)<<8|rx_frame.data[5];
			
			motor_pos_ = uint_to_float(pos_int,POS_MIN,POS_MAX,16);
			motor_spd_ = uint_to_float(spd_int,SPD_MIN,SPD_MAX,12);
			motor_current_ = uint_to_float(cur_int,I_MIN,I_MAX,12);

			motor_temperature_ = (rx_frame.data[6]-50)/2;
			mos_temperature_ = (rx_frame.data[7]-50)/2;
        }

}

