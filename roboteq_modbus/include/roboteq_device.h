#ifndef _roboteq_device_dot_h
#define _roboteq_device_dot_h

#include <vector>
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "serial/serial.h"

// MODBUS ASCII command/reg mapping
#define COMMAND_AC 0x00C0
#define COMMAND_AO 0x0320
#define COMMAND_ASW 0x03C0
#define COMMAND_AX 0x0240
#define COMMAND_B 0x02A0
#define COMMAND_C 0x0060
#define COMMAND_CB 0x0080
#define COMMAND_CG 0x0000
#define COMMAND_CSW 0x0380
#define COMMAND_D0 0x0140
#define COMMAND_D1 0x0120
#define COMMAND_DC 0x00E0
#define COMMAND_DS 0x0100
#define COMMAND_DX 0x0260
#define COMMAND_EES 0x02E0
#define COMMAND_EX 0x0180
#define COMMAND_G 0x0000
#define COMMAND_H 0x0160
#define COMMAND_MG 0x01A0
#define COMMAND_MS 0x01C0
#define COMMAND_P 0x0020
#define COMMAND_PR 0x01E0
#define COMMAND_PRX 0x0220
#define COMMAND_PSW 0x03A0
#define COMMAND_PX 0x0200
#define COMMAND_R 0x0300
#define COMMAND_RC 0x02C0
#define COMMAND_S 0x0040
#define COMMAND_SX 0x0280
#define COMMAND_TV 0x0360
#define COMMAND_TX 0x0340
#define COMMAND_VAR 0x00A0
#define QUERY_A 0x2000
#define QUERY_AI 0x8020
#define QUERY_AIC 0x8040
#define QUERY_ANG 0x2640
#define QUERY_B 0x22A0
#define QUERY_BA 0x2180
#define QUERY_BCR 0x2120
#define QUERY_BS 0x2140
#define QUERY_BSR 0x2160
#define QUERY_C 0x2080
#define QUERY_CB 0x20A0
#define QUERY_CIA 0x22E0
#define QUERY_CIP 0x2300
#define QUERY_CIS 0x22C0
#define QUERY_CR 0x2100
#define QUERY_D 0x21C0
#define QUERY_DI 0x8000
#define QUERY_DO 0x2260
#define QUERY_DR 0x2360
#define QUERY_E 0x2280
#define QUERY_EO 0x24E0
#define QUERY_F 0x2200
#define QUERY_FC 0x26A0
#define QUERY_FF 0x2240
#define QUERY_FIN 0x26E0
#define QUERY_FM 0x2440
#define QUERY_FS 0x2220
#define QUERY_GY 0x2620
#define QUERY_HS 0x2460
#define QUERY_ICL 0x2680
#define QUERY_K 0x2340
#define QUERY_LK 0x2480
#define QUERY_M 0x2020
#define QUERY_MA 0x2380
#define QUERY_MGD 0x23A0
#define QUERY_MGM 0x23E0
#define QUERY_MGS 0x2400
#define QUERY_MGT 0x23C0
#define QUERY_MGY 0x2420
#define QUERY_ML 0x2560
#define QUERY_MRS 0x25A0
#define QUERY_MZ 0x25C0
#define QUERY_P 0x2040
#define QUERY_PI 0x8060
#define QUERY_PIC 0x8080
#define QUERY_PK 0x25E0
#define QUERY_QO 0x24C0
#define QUERY_RF 0x2600
#define QUERY_RMA 0x2500
#define QUERY_RMG 0x2520
#define QUERY_RMM 0x2540
#define QUERY_S 0x2060
#define QUERY_SCC 0x2660
#define QUERY_SL 0x26C0
#define QUERY_SR 0x20E0
#define QUERY_T 0x21E0
#define QUERY_TM 0x2320
#define QUERY_TR 0x24A0
#define QUERY_TS 0x2580
#define QUERY_V 0x21A0
#define QUERY_VAR 0x20C0

const int ROBOTEQ_CH1 = 16;
const int ROBOTEQ_CH2 = 32;

const int ROBOTEQ_MODE_OPEN_LOOP = 0;
const int ROBOTEQ_MODE_SPEED = 1;
const int ROBOTEQ_MODE_SPEED_POSITION = 6;

const int ROBOTEQ_FEEDBACK_OTHER = 0;
const int ROBOTEQ_FEEDBACK_HALL = 1;

const int ROBOTEQ_AMP_TRIGGER_NONE = 0;
const int ROBOTEQ_AMP_TRIGGER_SAFETY_STOP = 1;
const int ROBOTEQ_AMP_TRIGGER_EMERGENCY_STOP = 2;
const int ROBOTEQ_AMP_TRIGGER_MOTOR_STOP = 3;

const int ROBOTEQ_FF_OVERHEAT = 1;
const int ROBOTEQ_FF_OVERVOLTAGE = 2;
const int ROBOTEQ_FF_UNDERVOLTAGE = 4;
const int ROBOTEQ_FF_SHORT_CIRCUIT = 8;
const int ROBOTEQ_FF_EMERGENCY_STOP = 16;
const int ROBOTEQ_FF_BRUSHLESS_FAULT = 32;
const int ROBOTEQ_FF_MOSFET_FAILURE = 64;

const int ROBOTEQ_FM_AMPS_LIMIT = 1;
const int ROBOTEQ_FM_MOTOR_STALLED = 2;
const int ROBOTEQ_FM_LOOP_ERROR = 4;
const int ROBOTEQ_FM_SAFETY_STOP = 8;
const int ROBOTEQ_FM_FORWARD_LIMIT = 16;
const int ROBOTEQ_FM_REVERSE_LIMIT = 32;
const int ROBOTEQ_FM_AMPS_TRIGGER = 64;

const int ROBOTEQ_FS_SERIAL_MODE = 1;
const int ROBOTEQ_FS_PULSE_MODE = 2;
const int ROBOTEQ_FS_ANALOG_MODE = 4;
const int ROBOTEQ_FS_POWER_STAGE_OFF = 8;
const int ROBOTEQ_FS_STALL_DETECTED = 16;
const int ROBOTEQ_FS_AT_LIMIT = 32;
const int ROBOTEQ_FS_UNUSED = 64;
const int ROBOTEQ_FS_SCRIPT_RUNNING = 128;

class RoboteqDevice {
  public:
    RoboteqDevice(std::string _device, int _baud, int _num_channels);
    ~RoboteqDevice();

    void                             commandGo(unsigned int channel, int value);
    const std::vector<int32_t>       getBrushlessCount();
    const std::vector<float>         getBrushlessSpeed();
    const std::vector<int32_t>       getClosedLoopError();
    const std::vector<float>         getCurrent();
    const std::vector<uint16_t>      getFirmwareID();
    const std::vector<uint16_t>      getFlagsRuntime();
    const uint16_t                   getFlagsStatus();
    const uint16_t                   getFlagsFault();
    const std::vector<float>         getRotorAngle();
    const std::vector<float>         getTemperature();
    const std::vector<float>         getVoltage();

  private:
    bool                             connect();
    bool                             disconnect();
    unsigned char                    LRC(std::vector<unsigned char> msg);
    uint32_t                         modbusParseQueryResponse(std::string input);
    const std::string                modbusReadInputRegisters(unsigned int reg, unsigned int offset);
    const std::string                modbusWriteHoldingRegisters(unsigned int reg, unsigned int offset, uint32_t value);
    bool writeLine(std::string line);
    std::string readLine();

    int                              baud;
    int                              num_channels;
    std::string                      port;
    serial::Serial                   ser;
};

#endif // _roboteq_device_dot_h
