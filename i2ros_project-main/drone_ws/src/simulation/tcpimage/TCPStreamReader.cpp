#include "TCPStreamReader.h"
#include <iostream>
#include <vector>

#define CHUNK_SIZE 4096

using namespace std;
using namespace libsocket;

TCPStreamReader::TCPStreamReader(string const &host, string const &port)
  : m_good(false), m_host(host), m_port(port), m_server(host, port, LIBSOCKET_IPv4)
{
  m_server_set.add_fd(m_server, LIBSOCKET_READ);
  m_server_stream = nullptr;
}


void TCPStreamReader::WaitConnect() {
  m_readypair = m_server_set.wait();
  m_ready_server = dynamic_cast<inet_stream_server *>(m_readypair.first.back());
  m_readypair.first.pop_back();
  m_server_stream = m_ready_server->accept();
  m_good = true;
}

int TCPStreamReader::ReadInt() {
  size_t bytes_received = 0;
  int value;

  if(m_server_stream != nullptr) {
    bytes_received = m_server_stream->rcv(&value, 4, 0);
  }

  if(bytes_received != 4) {
    throw socket_exception("TCPStreamReader", 54 , "Failed to read int from server stream");
  }

  return value;
}

uint32_t TCPStreamReader::ReadUInt() {
  size_t bytes_received = 0;
  uint32_t value;

  if(m_server_stream != nullptr) {
    bytes_received = m_server_stream->rcv(&value, sizeof(uint32_t), 0);
  }

  if(bytes_received != sizeof(uint32_t)) {
    throw socket_exception("TCPStreamReader", 54 , "Failed to read unsigned int from server stream");
  }

  return value;
}

uint64_t TCPStreamReader::ReadUInt64() {
  size_t bytes_received = 0;
  uint64_t value;

  if(m_server_stream != nullptr) {
    bytes_received = m_server_stream->rcv(&value, sizeof(uint64_t), 0);
  }

  if(bytes_received != sizeof(uint64_t)) {
    throw socket_exception("TCPStreamReader", 54 , "Failed to read unsigned 64-bit int from server stream");
  }

  return value;
}

float TCPStreamReader::ReadFloat() {
  size_t bytes_received = 0;
  float value;

  if(m_server_stream != nullptr) {
    bytes_received = m_server_stream->rcv(&value, 4, 0);
  }

  if(bytes_received != 4) {
    throw socket_exception("TCPStreamReader", 54 , "Failed to read float from server stream");
  }

  return value;
}

string TCPStreamReader::ReadString() {
  vector<char> string_vec;
  if(m_server_stream != nullptr) {
    uint8_t byte = 255;
    while (byte != 0) {
      m_server_stream->rcv(&byte, 1, 0);
      string_vec.push_back(byte);
    }
  }
  return string(string_vec.begin(), string_vec.end() - 1);
}

shared_ptr<uint8_t> TCPStreamReader::ReadBytes(const size_t num_bytes) {
  int data_remaining = num_bytes;
  int current_offset = 0;
  size_t bytes_received = 0;
  shared_ptr<uint8_t> bytes;

  if (m_server_stream != nullptr) {
    bytes = shared_ptr<uint8_t>(new uint8_t[data_remaining], default_delete<uint8_t[]>());
    while (data_remaining > 0) {
      bytes_received = m_server_stream->rcv(bytes.get() + current_offset, min(data_remaining, CHUNK_SIZE), 0);
      data_remaining -= bytes_received;
      current_offset += bytes_received;
    }
  }

  if(data_remaining != 0) {
    throw socket_exception("TCPStreamReader", 77, "Failed to read bytes from server stream");
  }

  return bytes;
}

void TCPStreamReader::Shutdown() {
  m_good = false;
  m_server.destroy();
}

TCPStreamReader::~TCPStreamReader() {
  Shutdown();
}

bool TCPStreamReader::Good() {
  return m_good;
}
