/*=========================================================================

  Program:   OpenIGTLink -- Example for Tracker Server Program
  Language:  C++

  Copyright (c) Insight Software Consortium. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include <iostream>
#include <math.h>
#include <cstdlib>

#include "igtlOSUtil.h"
#include "igtlTransformMessage.h"
#include "igtlImageMessage.h"
#include "igtlPointMessage.h"
#include "igtlStringMessage.h"

#include "igtlServerSocket.h"

void SendTransform(igtl::Socket* socket);
void SendPoint(igtl::Socket* socket, int n);
void SendImage(igtl::Socket* socket, int len);
void SendString(igtl::Socket* socket, int len);

void GetRandomTestMatrix(igtl::Matrix4x4& matrix, bool positionOnly=false);
void GetRandomString(int size, std::string& str);
void GetRandomImageInt8(int size, char* ptr);

int main(int argc, char* argv[])
{
  //------------------------------------------------------------
  // Parse Arguments

  if (argc < 5) // check number of arguments
    {
    // If not correct, print usage
    std::cerr << "Usage: " << argv[0] << " <port> <fps> <type>"    << std::endl;
    std::cerr << "    <port>     : Port # (18944 in Slicer default)"   << std::endl;
    std::cerr << "    <fps>      : Frequency (fps) to send coordinate" << std::endl;
    std::cerr << "    <type>     : Data type (TRANSFORM, POINT, IMAGE, STRING)" << std::endl;
    std::cerr << "    <iter>     : Number of iteration" << std::endl;
    std::cerr << "    <size>     : Number of data (or data length in case of string/image)" << std::endl;
    
    exit(0);
    }

  int    port     = atoi(argv[1]);
  double fps      = atof(argv[2]);
  double interval = (double) (1000.0 / fps);
  const char * type = argv[3];
  int    size     = 1;
  int    nIter    = atoi(argv[4]);
  if (argc == 6)
    {
    size = atoi(argv[5]);
    }

  igtl::ServerSocket::Pointer serverSocket;
  serverSocket = igtl::ServerSocket::New();
  int r = serverSocket->CreateServer(port);

  if (r < 0)
    {
    std::cerr << "Cannot create a server socket." << std::endl;
    exit(0);
    }

  igtl::Socket::Pointer socket;

  // Time stamp to control the interval
  igtl::TimeStamp::Pointer ts;
  ts = igtl::TimeStamp::New();
  ts->GetTime();

  while (1)
    {
    //------------------------------------------------------------
    // Waiting for Connection
    socket = serverSocket->WaitForConnection(1000);
    
    if (socket.IsNotNull()) // if client connected
      {
      //------------------------------------------------------------
      // loop
      for (int i = 0; i < nIter; i ++)
        {
        ts->GetTime();
        double start = ts->GetTimeStamp();

        if (strcmp(type, "TRANSFORM") == 0)
          {
          SendTransform(socket);
          }
        else if (strcmp(type, "POINT") == 0)
          {
          SendPoint(socket, size);
          }
        else if (strcmp(type, "IMAGE") == 0)
          {
          SendImage(socket, size);
          }
        else if (strcmp(type, "STRING") == 0)
          {
          SendString(socket, size);
          }
        else
          {
          std::cerr << "Wrong type is specified. " << std::endl;
          socket->CloseSocket();
          break;
          }
        
        ts->GetTime();
        double end = ts->GetTimeStamp();
        double elapsed = (end - start) * 1000.0; // ms
        double remain = interval - elapsed; // ms
        if (remain < 0.0)
          {
          std::cerr << "[WARMING] Sending delay: " << remain << std::endl;
          }
        else
          {
          igtl::Sleep((int)(remain)); // wait
          }
        }
      }
    }
    
  //------------------------------------------------------------
  // Close connection (The example code never reachs to this section ...)
  
  socket->CloseSocket();

}

void SendTransform(igtl::Socket* socket)
{
  igtl::TimeStamp::Pointer ts;
  ts = igtl::TimeStamp::New();
  ts->GetTime();

  igtl::TransformMessage::Pointer transMsg;
  transMsg = igtl::TransformMessage::New();
  transMsg->SetDeviceName("Tracker");

  igtl::Matrix4x4 matrix;
  GetRandomTestMatrix(matrix);
  transMsg->SetMatrix(matrix);
  transMsg->SetTimeStamp(ts);
  transMsg->Pack();
  socket->Send(transMsg->GetPackPointer(), transMsg->GetPackSize());
}

void SendPoint(igtl::Socket* socket, int n)
{
  igtl::Matrix4x4 matrix;

  igtl::TimeStamp::Pointer ts;
  ts = igtl::TimeStamp::New();
  ts->GetTime();

  igtl::PointMessage::Pointer pointMsg;
  pointMsg = igtl::PointMessage::New();
  pointMsg->SetDeviceName("PointSender");

  for (int i = 0; i < n; i ++)
    {
    //---------------------------
    // Create point
    GetRandomTestMatrix(matrix, true);

    igtl::PointElement::Pointer point0;
    point0 = igtl::PointElement::New();
    point0->SetName("POINT_0");
    point0->SetGroupName("GROUP_0");
    point0->SetRGBA(0xFF, 0x00, 0x00, 0xFF);
    point0->SetPosition(matrix[0][3], matrix[1][3], matrix[2][3]);
    point0->SetRadius(15.0);
    point0->SetOwner("IMAGE_0");
    pointMsg->AddPointElement(point0);
    }

  //---------------------------
  // Pack into the point message
  pointMsg->SetTimeStamp(ts);
  pointMsg->Pack();
  
  //------------------------------------------------------------
  // Send
  socket->Send(pointMsg->GetPackPointer(), pointMsg->GetPackSize());

}

void SendImage(igtl::Socket* socket, int len)
{
  igtl::TimeStamp::Pointer ts;
  ts = igtl::TimeStamp::New();
  ts->GetTime();

  //------------------------------------------------------------
  // size parameters
  int   size[]     = {256, 256, 1};       // image dimension
  float spacing[]  = {1.0, 1.0, 5.0};     // spacing (mm/pixel)
  int   svsize[]   = {256, 256, 1};       // sub-volume size
  int   svoffset[] = {0, 0, 0};           // sub-volume offset
  int   scalarType = igtl::ImageMessage::TYPE_UINT8;// scalar type

  size[1] = len / (256*4);
  size[2] = 4;
  
  //------------------------------------------------------------
  // Create a new IMAGE type message
  igtl::ImageMessage::Pointer imgMsg = igtl::ImageMessage::New();
  imgMsg->SetDimensions(size);
  imgMsg->SetSpacing(spacing);
  imgMsg->SetScalarType(scalarType);
  imgMsg->SetDeviceName("ImagerClient");
  imgMsg->SetSubVolume(svsize, svoffset);
  imgMsg->AllocateScalars();
  
  //------------------------------------------------------------
  // Set image data (See GetTestImage() bellow for the details)
  //GetTestImage(imgMsg, filedir, i % 5);
  GetRandomImageInt8((int)imgMsg->GetImageSize(), (char*)imgMsg->GetScalarPointer());
  
  //------------------------------------------------------------
  // Get random orientation matrix and set it.
  igtl::Matrix4x4 matrix;
  GetRandomTestMatrix(matrix);
  imgMsg->SetMatrix(matrix);

  imgMsg->SetTimeStamp(ts);
  
  //------------------------------------------------------------
  // Pack (serialize) and send
  imgMsg->Pack();
  socket->Send(imgMsg->GetPackPointer(), imgMsg->GetPackSize());
  
}

void SendString(igtl::Socket* socket, int len)
{
  igtl::TimeStamp::Pointer ts;
  ts = igtl::TimeStamp::New();
  ts->GetTime();
  
  //------------------------------------------------------------
  // Allocate Transform Message Class
  std::string str;

  GetRandomString(len, str);
  
  igtl::StringMessage::Pointer stringMsg;
  stringMsg = igtl::StringMessage::New();
  stringMsg->SetDeviceName("StringMessage");
  stringMsg->SetString(str.c_str());
  stringMsg->SetTimeStamp(ts);
  stringMsg->Pack();
  socket->Send(stringMsg->GetPackPointer(), stringMsg->GetPackSize());
}

void GetRandomTestMatrix(igtl::Matrix4x4& matrix, bool positionOnly)
{
  float position[3];
  float orientation[4];

  // random position
  static float phi = 0.0;
  position[0] = 50.0 * cos(phi);
  position[1] = 50.0 * sin(phi);
  position[2] = 50.0 * cos(phi);
  phi = phi + 0.2;

  if (positionOnly)
    {
    return;
    }
    
  
  // random orientation
  static float theta = 0.0;
  orientation[0]=0.0;
  orientation[1]=0.6666666666*cos(theta);
  orientation[2]=0.577350269189626;
  orientation[3]=0.6666666666*sin(theta);
  theta = theta + 0.1;

  //igtl::Matrix4x4 matrix;
  igtl::QuaternionToMatrix(orientation, matrix);

  matrix[0][3] = position[0];
  matrix[1][3] = position[1];
  matrix[2][3] = position[2];
  
  //igtl::PrintMatrix(matrix);
}

void GetRandomString(int size, std::string& str)
{
  const char alphabets[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  int nalphabets = sizeof(alphabets) - 1;

  str.clear();
  for (int i = 0; i < size; i ++)
    {
      str += alphabets[rand() % nalphabets];
    }
}

void GetRandomImageInt8(int size, char* ptr)
{
  char* p = ptr;
  char* ptr_end = p + size;
  while (p < ptr_end)
    {
    *p = rand() % 255;
    p++;
    }
}
