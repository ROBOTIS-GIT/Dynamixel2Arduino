


#include "port_handler.h"

using namespace DYNAMIXEL;

PortHandler::PortHandler()
 : open_state_(false)
{}

/* PortHandler */
bool PortHandler::getOpenState()
{
  return open_state_;
}

void PortHandler::setOpenState(bool state)
{
  open_state_ = state;
}


/* SerialPortHandler */
SerialPortHandler::SerialPortHandler(HardwareSerial& port, const int dir_pin)
 : PortHandler(), port_(port), dir_pin_(dir_pin), baud_(57600)
{}

void SerialPortHandler::begin()
{
  begin(baud_);
}

void SerialPortHandler::begin(unsigned long baud)
{
  baud_ = baud;
  port_.begin(baud_);
  dir_pin_ != -1 ? pinMode(dir_pin_, OUTPUT), digitalWrite(dir_pin_, LOW):(void)(dir_pin_);
  setOpenState(true);
}

void SerialPortHandler::end(void)
{
  port_.end();
  setOpenState(false);
}

int SerialPortHandler::available(void)
{
  return port_.available();
}

int SerialPortHandler::read()
{
  return port_.read();
}

size_t SerialPortHandler::write(uint8_t c)
{
  size_t ret = 0;
  dir_pin_ != -1 ? digitalWrite(dir_pin_, HIGH):(void)(dir_pin_);
  ret = port_.write(c);
  dir_pin_ != -1 ? port_.flush(), digitalWrite(dir_pin_, LOW):(void)(dir_pin_);

  return ret;
}

size_t SerialPortHandler::write(uint8_t *buf, size_t len)
{
  size_t ret;
  dir_pin_ != -1 ? digitalWrite(dir_pin_, HIGH):(void)(dir_pin_);
  ret = port_.write(buf, len);
  dir_pin_ != -1 ? port_.flush(), digitalWrite(dir_pin_, LOW):(void)(dir_pin_);

  return ret;      
}

unsigned long SerialPortHandler::getBaud() const
{
  return baud_;
}