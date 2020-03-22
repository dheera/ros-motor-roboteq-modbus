#include "roboteq_device.h"

void log(std::string text) {
  std::cout << text << "\n";
}

RoboteqDevice::RoboteqDevice(std::string _port, int _baud, int _num_channels) {
  port = _port;
  baud = _baud;
  num_channels = _num_channels;
  if(!connect()) log("could not connect to roboteq");
}

RoboteqDevice::~RoboteqDevice() {
  disconnect();
}

bool RoboteqDevice::connect() {
  try {
      ser.setPort(port);
      ser.setBaudrate(baud);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();

      // put roboteq in modbus ascii mode
      writeLine("");
      writeLine("^DMOD 3");
      readLine();

  } catch(serial::IOException& e) {
      return false;
  }

  if(!ser.isOpen()) {
      return false;
  }

  return true;
}

bool RoboteqDevice::disconnect() {
  ser.close();
  return true;
}

void RoboteqDevice::commandGo(unsigned int channel, int value) {
  writeLine(modbusWriteHoldingRegisters(COMMAND_G, channel, value));
  // TODO: verify command was successful. example successful outputs:
  // :011000020002EB
  // :011000010002EC
}

const std::vector<int32_t> RoboteqDevice::getBrushlessCount() {
  std::vector<int32_t> result(num_channels, 0);
  for(int i=1;i<=num_channels;i++) {
    writeLine(modbusReadInputRegisters(QUERY_CB, i));
    result[i-1] = (int32_t)modbusParseQueryResponse(readLine());
  }
  return result;
}

const std::vector<float> RoboteqDevice::getBrushlessSpeed() {
  std::vector<float> result(num_channels, 0.0);
  for(int i=1;i<=num_channels;i++) {
    writeLine(modbusReadInputRegisters(QUERY_BS, i));
    result[i-1] = (float)(int16_t)(uint16_t)modbusParseQueryResponse(readLine());
  }
  return result;
}

const std::vector<int32_t> RoboteqDevice::getClosedLoopError() {
  std::vector<int32_t> result(num_channels, 0);
  for(int i=1;i<=num_channels;i++) {
    writeLine(modbusReadInputRegisters(QUERY_E, i));
    result[i-1] = (int32_t)modbusParseQueryResponse(readLine());
  }
  return result;
}

const std::vector<float> RoboteqDevice::getCurrent() {
  std::vector<float> result(num_channels, 0.0);
  for(int i=1;i<=num_channels;i++) {
    writeLine(modbusReadInputRegisters(QUERY_A, i));
    result[i-1] = (float)(int16_t)(uint16_t)modbusParseQueryResponse(readLine()) / 10.0;
  }
  return result;
}

const std::vector<uint16_t> RoboteqDevice::getFirmwareID() {
  std::vector<uint16_t> result(4, 0);
  for(int i=1;i<=4;i++) {
    writeLine(modbusReadInputRegisters(QUERY_FIN, i));
    result[i-1] = (uint16_t)modbusParseQueryResponse(readLine());
  }
  return result;
}

const std::vector<uint16_t> RoboteqDevice::getFlagsRuntime() {
  std::vector<uint16_t> result(num_channels, 0);
  for(int i=1;i<=num_channels;i++) {
    writeLine(modbusReadInputRegisters(QUERY_FM, i));
    result[i-1] = (uint16_t)modbusParseQueryResponse(readLine());
  }
  return result;
}

const uint16_t RoboteqDevice::getFlagsStatus() {
  writeLine(modbusReadInputRegisters(QUERY_FS, 0));
  return (uint16_t)modbusParseQueryResponse(readLine());
}

const uint16_t RoboteqDevice::getFlagsFault() {
  writeLine(modbusReadInputRegisters(QUERY_FF, 0));
  return (uint16_t)modbusParseQueryResponse(readLine());
}

const std::vector<float> RoboteqDevice::getRotorAngle() {
  std::vector<float> result(num_channels, 0);
  for(int i=1;i<=num_channels;i++) {
    writeLine(modbusReadInputRegisters(QUERY_ANG, i));
    result[i-1] = (float)(uint16_t)modbusParseQueryResponse(readLine()) / 512.0 * 6.283185307179586;
  }
  return result;
}

const std::vector<float> RoboteqDevice::getTemperature() {
  std::vector<float> result(num_channels+1, 0.0);
  for(int i=1;i<=num_channels+1;i++) {
    writeLine(modbusReadInputRegisters(QUERY_T, i));
    result[i-1] = (float)(int16_t)(uint16_t)modbusParseQueryResponse(readLine());
  }
  return result;
}

const std::vector<float> RoboteqDevice::getVoltage() {
  if(!ser.isOpen()) connect();

  std::vector<float> result(3, 0.0);

  writeLine(modbusReadInputRegisters(QUERY_V, 1));
  result[0] = ((float)(int16_t)(uint16_t)modbusParseQueryResponse(readLine())) / 10.0;

  writeLine(modbusReadInputRegisters(QUERY_V, 2));
  result[1] = ((float)(int16_t)(uint16_t)modbusParseQueryResponse(readLine())) / 10.0;

  writeLine(modbusReadInputRegisters(QUERY_V, 3));
  result[2] = ((float)(int16_t)(uint16_t)modbusParseQueryResponse(readLine())) / 1000.0;

  return result;
}

bool RoboteqDevice::writeLine(std::string line) {
  if(!ser.isOpen()) return false;

  ser.flush();
  ser.read(ser.available());
  ser.write(line + "\r\n");

  return true;
}

std::string RoboteqDevice::readLine() {
  return ser.readline((size_t)65536, "\r\n");
}

unsigned char RoboteqDevice::LRC(std::vector<unsigned char> msg) {
  unsigned char sum = 0;
  for(auto i=0;i<(int)msg.size();i++) sum += msg[i];
  return (unsigned char)(-(char)sum);
}  

uint32_t RoboteqDevice::modbusParseQueryResponse(std::string input) {
  std::istringstream iss(input);

  char start;
  iss >> start;
  if(start != ':') return 0;
  
  iss >> std::hex;
  
  std::string s;

  iss >> std::setw(2) >> s;
  unsigned int node = std::stoul(s, nullptr, 16);
  
  iss >> std::setw(2) >> s;
  unsigned int function_code = std::stoul(s, nullptr, 16);
  
  iss >> std::setw(2) >> s;
  unsigned int data_length = std::stoul(s, nullptr, 16);

  iss >> std::setw(8) >> s;
  unsigned long int data = std::stoul(s, nullptr, 16);
  
  iss >> std::setw(2) >> s;
  unsigned int lrc = std::stoul(s, nullptr, 16);

  return data;
}

const std::string RoboteqDevice::modbusWriteHoldingRegisters(unsigned int reg, unsigned int offset, uint32_t value) {
  std::vector<unsigned char> msg_bytes = {
    0x01, // node address
    0x10, // function code (write multiple holding registers)
    0x00, 0x00, // register address -- to be filled in
    0x00, 0x02, // write 2 registers
    0x04, // write 4 bytes
    0x00, 0x00, 0x00, 0x00 // value -- to be filled in
  };

  reg += offset;

  msg_bytes[2] = reg >> 8;
  msg_bytes[3] = reg & 0xFF;

  // roboteq is big indian
  msg_bytes[7] = (value & 0xFF000000) >> 24;
  msg_bytes[8] = (value & 0x00FF0000) >> 16;
  msg_bytes[9] = (value & 0x0000FF00) >> 8;
  msg_bytes[10] = (value & 0x000000FF);

  msg_bytes.push_back(LRC(msg_bytes));
  std::stringstream output;

  output << ":";
  output << std::hex << std::setfill('0');

  for(auto i=0;i < (int)msg_bytes.size(); i++) {
    output << std::setw(2) << static_cast<unsigned>(msg_bytes[i]);
  }

  return output.str();
}

const std::string RoboteqDevice::modbusReadInputRegisters(unsigned int reg, unsigned int offset) {
  std::vector<unsigned char> msg_bytes = {
    0x01, // node address
    0x04, // function code (read input registers)
    0x00, 0x00, // register address -- to be filled in
    0x00, 0x02, // read 2 bytes
  };

  reg += offset;

  msg_bytes[2] = reg >> 8;
  msg_bytes[3] = reg & 0xFF;

  msg_bytes.push_back(LRC(msg_bytes));
  std::stringstream output;

  output << ":";
  output << std::hex << std::setfill('0');

  for(auto i=0;i < (int)msg_bytes.size(); i++) {
    output << std::setw(2) << static_cast<unsigned>(msg_bytes[i]);
  }

  return output.str();
}

