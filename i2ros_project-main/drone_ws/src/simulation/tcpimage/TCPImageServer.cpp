#include "TCPImageServer.h"
#include <iostream>

#define CHUNK_SIZE 4096
#define BYTES_PER_PIXEL 3

using namespace std;
using namespace libsocket;

TCPImageServer::TCPImageServer(TCPStreamReader* stream_reader, const bool flip_image)
  : m_stream_reader(stream_reader), m_flip(flip_image) {

}

void TCPImageServer::WaitConnect() {
  m_stream_reader->WaitConnect();
}

ImageData TCPImageServer::GetImage() {
  ImageData img;
  try {
    // uint32_t time_sec = m_stream_reader->ReadUInt();
    // uint32_t time_nsec = m_stream_reader->ReadUInt();
    int width =  m_stream_reader->ReadInt();
    int height = m_stream_reader->ReadInt();
    img = m_flip ? FlipImage(ReadImage(width, height)) : ReadImage(width, height);
    img.time_sec = 0;
    img.time_nsec = 0;
  } catch (const socket_exception& ex) {
    cerr << ex.mesg << endl;
  }
  return img;
}

ImageData TCPImageServer::ReadImage(const int width, const int height) {
  ImageData img;
  img.width = width;
  img.height = height;
  img.data = m_stream_reader->ReadBytes(width * height * BYTES_PER_PIXEL);
  return img;
}

bool TCPImageServer::Good() {
  return m_stream_reader->Good();
}

ImageData TCPImageServer::FlipImage(ImageData const &img) {
  const int stride = img.width * BYTES_PER_PIXEL;
  ImageData flipped_image;
  flipped_image.width = img.width;
  flipped_image.height = img.height;
  flipped_image.data = shared_ptr<uint8_t>(new uint8_t[img.height * stride], default_delete<uint8_t[]>());

  for(int row = 0; row < img.height ; row++) {
    memcpy(flipped_image.data.get() + row * stride, img.data.get() + (img.height - row - 1) * stride, stride);
  }
  return flipped_image;
}
