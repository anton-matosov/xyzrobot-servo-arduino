#include "TcpSerial.h"
#include <boost/asio/write.hpp>
#include <boost/asio/read.hpp>
#include <iostream>
#include <vector>
#include <boost/asio.hpp>
#include <boost/thread.hpp>


tcp::resolver::iterator resolve(boost::asio::io_service& ioService, const std::string& ip, uint16_t port)
{
    tcp::resolver resolver(ioService);
    tcp::resolver::query query(ip, std::to_string(port));
    return resolver.resolve(query);
}

TcpSerial::TcpSerial(const std::string& ip, uint16_t port)
    : socket_(ioService_)
    , everRead_(false)
    , lastRead_(0)
{
  connect(socket_, resolve(ioService_, ip, port));
}

void TcpSerial::begin(const unsigned long baudRate, const uint8_t transferConfig)
{

}

size_t TcpSerial::write(uint8_t byte)
{
    return write(&byte, 1);
}

size_t TcpSerial::write(const uint8_t *data, size_t size)
{
    return boost::asio::write(socket_, boost::asio::buffer(data, size));
}

bool TcpSerial::available()
{
  boost::asio::socket_base::bytes_readable command(true);
  socket_.io_control(command);
  const std::size_t bytesReadable = command.get();

  return bytesReadable != 0;
}

uint8_t TcpSerial::peek()
{
    if (!available())
    {
        return kNoData;
    }
    if (!everRead_)
    {
        return read();
    }
    return lastRead_;
}

uint8_t TcpSerial::read()
{
    if (!available()) {
        return kNoData;
    }
    everRead_ = true;
    boost::asio::read(socket_, boost::asio::buffer(&lastRead_, 1));
    return lastRead_;
}

size_t TcpSerial::readBytes(uint8_t *buffer, size_t size)
{
    everRead_ = true;
    return boost::asio::read(socket_, boost::asio::buffer(buffer, size));
}

