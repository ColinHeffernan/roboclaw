/*
 * roboclaw.cpp - implementation of the roboclaw C++ library using ROS serial
 *
 * This code is a modified version of the Ion Motion Control Arduino library.
 * To view the code in its original form, download it at
 *     http://downloads.ionmc.com/code/arduino.zip 
 */

#include <roboclaw/roboclaw.h>

/*
 * Macros taken directly from Arduino Library
 */
#define MAXRETRY 2
#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg)>>24),(uint8_t)(((uint32_t)arg)>>16),(uint8_t)(((uint32_t)arg)>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg

/*
 * Constructor opens port at desired baudrate
 */
Roboclaw::Roboclaw(std::string &port, uint32_t baudrate)
{
    /* initialize pointer to a new Serial port object */
    port_ = new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(100));
    port_->open();
}

/*
 * Destructor closes serial port and frees the associated memory
 */
Roboclaw::~Roboclaw()
{
    port_->close();
    delete port_;
}

/*
 * writes a single byte to the serial port
 */
void Roboclaw::write(uint8_t byte)
{
    port_->write(&byte, 1);
}

/*
 * reads and returns a single byte from the serial port or -1 in error or timeout
 * returns an int16_t to be consistent with the Arduino library and allow for
 *       returning -1 error
 */
int16_t Roboclaw::read()
{
    uint8_t buff[1];
    if (port_->read(buff, 1) == 1)
    {
        return buff[0];
    }
    else
    {
        return -1;
    }
}

/*
 * flushes the serial port's input and output buffers
 */
void Roboclaw::flush()
{
    port_->flush();
}

/*
 * resets the crc calculation
 */
void Roboclaw::crc_clear()
{
    crc_ = 0;
}

/*
 * updates the crc calculation with based on the specified byte
 * see the Roboclaw sheet and user manual for more on this 
 */
void Roboclaw::crc_update (uint8_t data)
{
    int i;
    crc_ = crc_^ ((uint16_t)data << 8);
    for (i=0; i<8; i++)
    {
        if (crc_ & 0x8000)
            crc_ = (crc_ << 1) ^ 0x1021;
        else
            crc_ <<= 1;
    }
}

/*
 * returns the current value of the crc
 * this is not necessary as private methods can directly access the crc_ attribute,
 *      but it is being kept for now to keep the original Arduino code intact
 */
uint16_t Roboclaw::crc_get()
{
    return crc_;
}

/*
 * writes n bytes to the serial port as specified by function arguments
 */
bool Roboclaw::write_n(uint8_t cnt, ... )
{
    uint8_t trys=MAXRETRY;
    do{
        crc_clear();

        va_list marker;
        va_start(marker, cnt);     /* Initialize variable arguments */
        /* read each argument after cnt, update crc, and send the data */
        for(uint8_t index=0;index<cnt;index++){
            uint8_t data = va_arg(marker, int);
            crc_update(data);
            write(data);
        }
        va_end( marker );              /* Reset variable arguments */
        /* send the crc to the Roboclaw and check for return value */
        port_->write((uint8_t *) &crc_, 2);
        if(read()==0xFF)
            return true;
    }while(trys--);
    return false;
}

/*
 * reads the output of a command into n uint32_t variables specified by pointer arguments
 */
bool Roboclaw::read_n(uint8_t cnt,uint8_t address,uint8_t cmd,...)
{
    uint32_t value=0;
    uint8_t trys=MAXRETRY;
    int16_t data;
    do{
        flush();

        data=0;
        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        /* read each four byte output of the command into a uint32_t */
        va_list marker;
        va_start( marker, cmd );     /* Initialize variable arguments. */
        for(uint8_t index=0;index<cnt;index++){
            /* retrieve the pointer to the next uint32_t to be updated */
            uint32_t *ptr = va_arg(marker, uint32_t *);

            if(data!=-1){
                data = read();
                crc_update(data);
                value=(uint32_t)data<<24;
            }
            else{
                break;
            }

            if(data!=-1){
                data = read();
                crc_update(data);
                value|=(uint32_t)data<<16;
            }
            else{
                break;
            }

            if(data!=-1){
                data = read();
                crc_update(data);
                value|=(uint32_t)data<<8;
            }
            else{
                break;
            }

            if(data!=-1){
                data = read();
                crc_update(data);
                value|=(uint32_t)data;
            }
            else{
                break;
            }

            *ptr = value;
        }
        va_end( marker );              /* Reset variable arguments.      */

        /* read crc from the roboclaw and double check with our calculation */
        if(data!=-1){
            uint16_t ccrc;
            data = read();
            if(data!=-1){
                ccrc = data << 8;
                data = read();
                if(data!=-1){
                    ccrc |= data;
                    return crc_get()==ccrc;
                }
            }
        }
    }while(trys--);

    return false;
}

/*
 * reads a one byte register returning its contents or false in error
 */
uint8_t Roboclaw::read1(uint8_t address,uint8_t cmd,bool *valid)
{
    if(valid)
        *valid = false;

    uint8_t value=0;
    uint8_t trys=MAXRETRY;
    /* data is a signed int to allow for -1 assignment in error */
    int16_t data; 
    do{
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read();
        crc_update(data);
        value=data;

        if(data!=-1){
            uint16_t ccrc;
            data = read();
            if(data!=-1){
                ccrc = data << 8;
                data = read();
                if(data!=-1){
                    ccrc |= data;
                    if(crc_get()==ccrc){
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    }while(trys--);

    return false;
}

/*
 * reads a two byte register, returning its contents or false in error
 */
uint16_t Roboclaw::read2(uint8_t address,uint8_t cmd,bool *valid)
{
    if(valid)
        *valid = false;

    uint16_t value=0;
    uint8_t trys=MAXRETRY;
    int16_t data;
    do{
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read();
        crc_update(data);
        value=(uint16_t)data<<8;

        if(data!=-1){
            data = read();
            crc_update(data);
            value|=(uint16_t)data;
        }

        if(data!=-1){
            uint16_t ccrc;
            data = read();
            if(data!=-1){
                ccrc = data << 8;
                data = read();
                if(data!=-1){
                    ccrc |= data;
                    if(crc_get()==ccrc){
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    }while(trys--);

    return false;
}

/*
 * reads a four byte register
 *      returns register contents or false in error
 */
uint32_t Roboclaw::read4(uint8_t address, uint8_t cmd, bool *valid)
{
    if(valid)
        *valid = false;

    uint16_t value=0;
    uint8_t trys=MAXRETRY;
    int16_t data;
    do{
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read();
        crc_update(data);
        value=(uint16_t)data<<8;

        if(data!=-1){
            data = read();
            crc_update(data);
            value|=(uint16_t)data;
        }

        if(data!=-1){
            uint16_t ccrc;
            data = read();
            if(data!=-1){
                ccrc = data << 8;
                data = read();
                if(data!=-1){
                    ccrc |= data;
                    if(crc_get()==ccrc){
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    }while(trys--);

    return false;
}

/*
 * Reads a four byte register along with the roboclaw's status
 * returns the value of the register or false in the event of a timeout or error
 * updates value of status pointer argument
 * indicates success/failure through the valid argument
 */
uint32_t Roboclaw::read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid)
{
    if(valid)
        *valid = false;

    uint16_t value=0;
    uint8_t trys=MAXRETRY;
    int16_t data;
    do{
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read();
        crc_update(data);
        value=(uint16_t)data<<8;

        if(data!=-1){
            data = read();
            crc_update(data);
            value|=(uint16_t)data;
        }

        if(data!=-1){
            uint16_t ccrc;
            data = read();
            if(data!=-1){
                ccrc = data << 8;
                data = read();
                if(data!=-1){
                    ccrc |= data;
                    if(crc_get()==ccrc){
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    }while(trys--);

    return false;
}

/************************************************************
 * original methods unchanged from original Arduino library *
 ************************************************************/
bool Roboclaw::ForwardM1(uint8_t address, uint8_t speed){
    return write_n(3,address,M1FORWARD,speed);
}

bool Roboclaw::BackwardM1(uint8_t address, uint8_t speed){
    return write_n(3,address,M1BACKWARD,speed);
}

bool Roboclaw::SetMinVoltageMainBattery(uint8_t address, uint8_t voltage){
    return write_n(3,address,SETMINMB,voltage);
}

bool Roboclaw::SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage){
    return write_n(3,address,SETMAXMB,voltage);
}

bool Roboclaw::ForwardM2(uint8_t address, uint8_t speed){
    return write_n(3,address,M2FORWARD,speed);
}

bool Roboclaw::BackwardM2(uint8_t address, uint8_t speed){
    return write_n(3,address,M2BACKWARD,speed);
}

bool Roboclaw::ForwardBackwardM1(uint8_t address, uint8_t speed){
    return write_n(3,address,M17BIT,speed);
}

bool Roboclaw::ForwardBackwardM2(uint8_t address, uint8_t speed){
    return write_n(3,address,M27BIT,speed);
}

bool Roboclaw::ForwardMixed(uint8_t address, uint8_t speed){
    return write_n(3,address,MIXEDFORWARD,speed);
}

bool Roboclaw::BackwardMixed(uint8_t address, uint8_t speed){
    return write_n(3,address,MIXEDBACKWARD,speed);
}

bool Roboclaw::TurnRightMixed(uint8_t address, uint8_t speed){
    return write_n(3,address,MIXEDRIGHT,speed);
}

bool Roboclaw::TurnLeftMixed(uint8_t address, uint8_t speed){
    return write_n(3,address,MIXEDLEFT,speed);
}

bool Roboclaw::ForwardBackwardMixed(uint8_t address, uint8_t speed){
    return write_n(3,address,MIXEDFB,speed);
}

bool Roboclaw::LeftRightMixed(uint8_t address, uint8_t speed){
    return write_n(3,address,MIXEDLR,speed);
}

uint32_t Roboclaw::ReadEncM1(uint8_t address, uint8_t *status,bool *valid){
    return read4_1(address,GETM1ENC,status,valid);
}

uint32_t Roboclaw::ReadEncM2(uint8_t address, uint8_t *status,bool *valid){
    return read4_1(address,GETM2ENC,status,valid);
}

uint32_t Roboclaw::ReadSpeedM1(uint8_t address, uint8_t *status,bool *valid){
    return read4_1(address,GETM1SPEED,status,valid);
}

uint32_t Roboclaw::ReadSpeedM2(uint8_t address, uint8_t *status,bool *valid){
    return read4_1(address,GETM2SPEED,status,valid);
}

bool Roboclaw::ResetEncoders(uint8_t address){
    return write_n(2,address,RESETENC);
}

bool Roboclaw::ReadVersion(uint8_t address,char *version){
    uint8_t data;
    uint8_t trys=MAXRETRY;
    do{
        flush();

        data = 0;

        crc_clear();
        write(address);
        crc_update(address);
        write(GETVERSION);
        crc_update(GETVERSION);

        uint8_t i;
        for(i=0;i<48;i++){
            if(data!=-1){
                data=read();
                version[i] = data;
                crc_update(version[i]);
                if(version[i]==0){
                    uint16_t ccrc;
                    data = read();
                    if(data!=-1){
                        ccrc = data << 8;
                        data = read();
                        if(data!=-1){
                            ccrc |= data;
                            return crc_get()==ccrc;
                        }
                    }
                    break;
                }
            }
            else{
                break;
            }
        }
    }while(trys--);

    return false;
}

bool Roboclaw::SetEncM1(uint8_t address, int val){
    return write_n(6,address,SETM1ENCCOUNT,SetDWORDval(val));
}

bool Roboclaw::SetEncM2(uint8_t address, int val){
    return write_n(6,address,SETM2ENCCOUNT,SetDWORDval(val));
}

uint16_t Roboclaw::ReadMainBatteryVoltage(uint8_t address,bool *valid){
    return read2(address,GETMBATT,valid);
}

uint16_t Roboclaw::ReadLogicBatteryVoltage(uint8_t address,bool *valid){
    return read2(address,GETLBATT,valid);
}

bool Roboclaw::SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage){
    return write_n(3,address,SETMINLB,voltage);
}

bool Roboclaw::SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage){
    return write_n(3,address,SETMAXLB,voltage);
}

bool Roboclaw::SetM1VelocityPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps){
    uint32_t kp = kp_fp*65536;
    uint32_t ki = ki_fp*65536;
    uint32_t kd = kd_fp*65536;
    return write_n(18,address,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

bool Roboclaw::SetM2VelocityPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps){
    uint32_t kp = kp_fp*65536;
    uint32_t ki = ki_fp*65536;
    uint32_t kd = kd_fp*65536;
    return write_n(18,address,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

uint32_t Roboclaw::ReadISpeedM1(uint8_t address,uint8_t *status,bool *valid){
    return read4_1(address,GETM1ISPEED,status,valid);
}

uint32_t Roboclaw::ReadISpeedM2(uint8_t address,uint8_t *status,bool *valid){
    return read4_1(address,GETM2ISPEED,status,valid);
}

bool Roboclaw::DutyM1(uint8_t address, uint16_t duty){
    return write_n(4,address,M1DUTY,SetWORDval(duty));
}

bool Roboclaw::DutyM2(uint8_t address, uint16_t duty){
    return write_n(4,address,M2DUTY,SetWORDval(duty));
}

bool Roboclaw::DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2){
    return write_n(6,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(duty2));
}

bool Roboclaw::SpeedM1(uint8_t address, uint32_t speed){
    return write_n(6,address,M1SPEED,SetDWORDval(speed));
}

bool Roboclaw::SpeedM2(uint8_t address, uint32_t speed){
    return write_n(6,address,M2SPEED,SetDWORDval(speed));
}

bool Roboclaw::SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2){
    return write_n(10,address,MIXEDSPEED,SetDWORDval(speed1),SetDWORDval(speed2));
}

bool Roboclaw::SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed){
    return write_n(10,address,M1SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}

bool Roboclaw::SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed){
    return write_n(10,address,M2SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}
bool Roboclaw::SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2){
    return write_n(14,address,MIXEDSPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(speed2));
}

bool Roboclaw::SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag){
    return write_n(11,address,M1SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool Roboclaw::SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag){
    return write_n(11,address,M2SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool Roboclaw::SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){
    return write_n(19,address,MIXEDSPEEDDIST,SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool Roboclaw::SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){
    return write_n(15,address,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool Roboclaw::SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){
    return write_n(15,address,M2SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool Roboclaw::SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){
    return write_n(23,address,MIXEDSPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool Roboclaw::ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2){
    bool valid;
    uint16_t value = read2(address,GETBUFFERS,&valid);
    if(valid){
        depth1 = value>>8;
        depth2 = value;
    }
    return valid;
}

bool Roboclaw::ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2){
    bool valid;
    uint32_t value = read4(address,GETPWMS,&valid);
    if(valid){
        pwm1 = value>>16;
        pwm2 = value&0xFFFF;
    }
    return valid;
}

bool Roboclaw::ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2){
    bool valid;
    uint32_t value = read4(address,GETCURRENTS,&valid);
    if(valid){
        current1 = value>>16;
        current2 = value&0xFFFF;
    }
    return valid;
}

bool Roboclaw::SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2){
    return write_n(18,address,MIXEDSPEED2ACCEL,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(accel2),SetDWORDval(speed2));
}

bool Roboclaw::SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag){
    return write_n(27,address,MIXEDSPEED2ACCELDIST,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool Roboclaw::DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel){
    return write_n(8,address,M1DUTYACCEL,SetWORDval(duty),SetDWORDval(accel));
}

bool Roboclaw::DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel){
    return write_n(8,address,M2DUTYACCEL,SetWORDval(duty),SetDWORDval(accel));
}

bool Roboclaw::DutyAccelM1M2(uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2){
    return write_n(14,address,MIXEDDUTYACCEL,SetWORDval(duty1),SetDWORDval(accel1),SetWORDval(duty2),SetDWORDval(accel2));
}

bool Roboclaw::ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
    uint32_t Kp,Ki,Kd;
    bool valid = read_n(4,address,READM1PID,&Kp,&Ki,&Kd,&qpps);
    Kp_fp = ((float)Kp)/65536;
    Ki_fp = ((float)Ki)/65536;
    Kd_fp = ((float)Kd)/65536;
    return valid;
}

bool Roboclaw::ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
    uint32_t Kp,Ki,Kd;
    bool valid = read_n(4,address,READM2PID,&Kp,&Ki,&Kd,&qpps);
    Kp_fp = ((float)Kp)/65536;
    Ki_fp = ((float)Ki)/65536;
    Kd_fp = ((float)Kd)/65536;
    return valid;
}

bool Roboclaw::SetMainVoltages(uint8_t address,uint16_t min,uint16_t max){
    return write_n(6,address,SETMAINVOLTAGES,SetWORDval(min),SetWORDval(max));
}

bool Roboclaw::SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max){
    return write_n(6,address,SETLOGICVOLTAGES,SetWORDval(min),SetWORDval(max));
}

bool Roboclaw::ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max){
    bool valid;
    uint32_t value = read4(address,GETMINMAXMAINVOLTAGES,&valid);
    if(valid){
        min = value>>16;
        max = value&0xFFFF;
    }
    return valid;
}

bool Roboclaw::ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max){
    bool valid;
    uint32_t value = read4(address,GETMINMAXLOGICVOLTAGES,&valid);
    if(valid){
        min = value>>16;
        max = value&0xFFFF;
    }
    return valid;
}

bool Roboclaw::SetM1PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max){
    uint32_t kp=kp_fp*1024;
    uint32_t ki=ki_fp*1024;
    uint32_t kd=kd_fp*1024;
    return write_n(30,address,SETM1POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

bool Roboclaw::SetM2PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max){
    uint32_t kp=kp_fp*1024;
    uint32_t ki=ki_fp*1024;
    uint32_t kd=kd_fp*1024;
    return write_n(30,address,SETM2POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

bool Roboclaw::ReadM1PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
    uint32_t Kp,Ki,Kd;
    bool valid = read_n(7,address,READM1POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
    Kp_fp = ((float)Kp)/1024;
    Ki_fp = ((float)Ki)/1024;
    Kd_fp = ((float)Kd)/1024;
    return valid;
}

bool Roboclaw::ReadM2PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
    uint32_t Kp,Ki,Kd;
    bool valid = read_n(7,address,READM2POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
    Kp_fp = ((float)Kp)/1024;
    Ki_fp = ((float)Ki)/1024;
    Kd_fp = ((float)Kd)/1024;
    return valid;
}

bool Roboclaw::SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
    return write_n(19,address,M1SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

bool Roboclaw::SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
    return write_n(19,address,M2SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

bool Roboclaw::SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag){
    return write_n(35,address,MIXEDSPEEDACCELDECCELPOS,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(deccel1),SetDWORDval(position1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(deccel2),SetDWORDval(position2),flag);
}

bool Roboclaw::SetM1DefaultAccel(uint8_t address, uint32_t accel){
    return write_n(6,address,SETM1DEFAULTACCEL,SetDWORDval(accel));
}

bool Roboclaw::SetM2DefaultAccel(uint8_t address, uint32_t accel){
    return write_n(6,address,SETM2DEFAULTACCEL,SetDWORDval(accel));
}

bool Roboclaw::SetPinFunctions(uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode){
    return write_n(5,address,SETPINFUNCTIONS,S3mode,S4mode,S5mode);
}

bool Roboclaw::GetPinFunctions(uint8_t address, uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode){
    uint8_t crc;
    bool valid = false;
    uint8_t val1,val2,val3;
    uint8_t trys=MAXRETRY;
    int16_t data;
    do{
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(GETPINFUNCTIONS);
        crc_update(GETPINFUNCTIONS);

        data = read();
        crc_update(data);
        val1=data;

        if(data!=-1){
            data = read();
            crc_update(data);
            val2=data;
        }

        if(data!=-1){
            data = read();
            crc_update(data);
            val3=data;
        }

        if(data!=-1){
            uint16_t ccrc;
            data = read();
            if(data!=-1){
                ccrc = data << 8;
                data = read();
                if(data!=-1){
                    ccrc |= data;
                    if(crc_get()==ccrc){
                        S3mode = val1;
                        S4mode = val2;
                        S5mode = val3;
                        return true;
                    }
                }
            }
        }
    }while(trys--);

    return false;
}

bool Roboclaw::SetDeadBand(uint8_t address, uint8_t Min, uint8_t Max){
    return write_n(4,address,SETDEADBAND,Min,Max);
}

bool Roboclaw::GetDeadBand(uint8_t address, uint8_t &Min, uint8_t &Max){
    bool valid;
    uint16_t value = read2(address,GETDEADBAND,&valid);
    if(valid){
        Min = value>>8;
        Max = value;
    }
    return valid;
}

bool Roboclaw::ReadEncoders(uint8_t address,uint32_t &enc1,uint32_t &enc2){
    bool valid = read_n(2,address,GETENCODERS,&enc1,&enc2);
    return valid;
}

bool Roboclaw::ReadISpeeds(uint8_t address,uint32_t &ispeed1,uint32_t &ispeed2){
    bool valid = read_n(2,address,GETISPEEDS,&ispeed1,&ispeed2);
    return valid;
}

bool Roboclaw::RestoreDefaults(uint8_t address){
    return write_n(2,address,RESTOREDEFAULTS);
}

bool Roboclaw::ReadTemp(uint8_t address, uint16_t &temp){
    bool valid;
    temp = read2(address,GETTEMP,&valid);
    return valid;
}

bool Roboclaw::ReadTemp2(uint8_t address, uint16_t &temp){
    bool valid;
    temp = read2(address,GETTEMP2,&valid);
    return valid;
}

uint16_t Roboclaw::ReadError(uint8_t address,bool *valid){
    return read2(address,GETERROR,valid);
}

bool Roboclaw::ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode){
    bool valid;
    uint16_t value = read2(address,GETENCODERMODE,&valid);
    if(valid){
        M1mode = value>>8;
        M2mode = value;
    }
    return valid;
}

bool Roboclaw::SetM1EncoderMode(uint8_t address,uint8_t mode){
    return write_n(3,address,SETM1ENCODERMODE,mode);
}

bool Roboclaw::SetM2EncoderMode(uint8_t address,uint8_t mode){
    return write_n(3,address,SETM2ENCODERMODE,mode);
}

bool Roboclaw::WriteNVM(uint8_t address){
    return write_n(6,address,WRITENVM, SetDWORDval(0xE22EAB7A) );
}

bool Roboclaw::ReadNVM(uint8_t address){
    return write_n(2,address,READNVM);
}

bool Roboclaw::SetConfig(uint8_t address, uint16_t config){
    return write_n(4,address,SETCONFIG,SetWORDval(config));
}

bool Roboclaw::GetConfig(uint8_t address, uint16_t &config){
    bool valid;
    uint16_t value = read2(address,GETCONFIG,&valid);
    if(valid){
        config = value;
    }
    return valid;
}

bool Roboclaw::SetM1MaxCurrent(uint8_t address,uint32_t max){
    return write_n(10,address,SETM1MAXCURRENT,SetDWORDval(max),SetDWORDval(0));
}

bool Roboclaw::SetM2MaxCurrent(uint8_t address,uint32_t max){
    return write_n(10,address,SETM2MAXCURRENT,SetDWORDval(max),SetDWORDval(0));
}

bool Roboclaw::ReadM1MaxCurrent(uint8_t address,uint32_t &max){
    uint32_t tmax,dummy;
    bool valid = read_n(2,address,GETM1MAXCURRENT,&tmax,&dummy);
    if(valid)
        max = tmax;
    return valid;
}

bool Roboclaw::ReadM2MaxCurrent(uint8_t address,uint32_t &max){
    uint32_t tmax,dummy;
    bool valid = read_n(2,address,GETM2MAXCURRENT,&tmax,&dummy);
    if(valid)
        max = tmax;
    return valid;
}

bool Roboclaw::SetPWMMode(uint8_t address, uint8_t mode){
    return write_n(3,address,SETPWMMODE,mode);
}

bool Roboclaw::GetPWMMode(uint8_t address, uint8_t &mode){
    bool valid;
    uint8_t value = read1(address,GETPWMMODE,&valid);
    if(valid){
        mode = value;
    }
    return valid;
}
