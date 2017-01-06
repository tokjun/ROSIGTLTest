/*=========================================================================

  Program:   Open IGT Link -- Example for Data Receiving Server Program
  Module:    $RCSfile: $
  Language:  C++
  Date:      $Date: $
  Version:   $Revision: $

  Copyright (c) Insight Software Consortium. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <cstdlib>
#include <cstring>

#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlTransformMessage.h"
#include "igtlImageMessage.h"
#include "igtlServerSocket.h"
#include "igtlStatusMessage.h"
#include "igtlPositionMessage.h"

#if OpenIGTLink_PROTOCOL_VERSION >= 2
#include "igtlPointMessage.h"
#include "igtlTrajectoryMessage.h"
#include "igtlStringMessage.h"
#include "igtlBindMessage.h"
#include "igtlCapabilityMessage.h"
#endif //OpenIGTLink_PROTOCOL_VERSION >= 2

int ReceiveTransform(igtl::Socket * socket, igtl::MessageHeader * header);
int ReceivePosition(igtl::Socket * socket, igtl::MessageHeader * header);
int ReceiveImage(igtl::Socket * socket, igtl::MessageHeader * header);
int ReceiveStatus(igtl::Socket * socket, igtl::MessageHeader * header);

#if OpenIGTLink_PROTOCOL_VERSION >= 2
int ReceivePoint(igtl::Socket * socket, igtl::MessageHeader * header);
int ReceiveTrajectory(igtl::Socket * socket, igtl::MessageHeader::Pointer& header);
int ReceiveString(igtl::Socket * socket, igtl::MessageHeader * header);
int ReceiveBind(igtl::Socket * socket, igtl::MessageHeader * header);
int ReceiveCapability(igtl::Socket * socket, igtl::MessageHeader * header);
#endif //OpenIGTLink_PROTOCOL_VERSION >= 2


// Global variables

// Verbose level
int VerboseLevel;


int main(int argc, char* argv[])
{

  // Log file
  std::ofstream logFile;

  //------------------------------------------------------------
  // Parse Arguments

  if (argc != 4) // check number of arguments
    {
    // If not correct, print usage
    std::cerr << "Usage: " << argv[0] << " <port> <logfile> <verbose>"    << std::endl;
    std::cerr << "    <port>     : Port # (18944 in Slicer default)"   << std::endl;
    std::cerr << "    <logfile>  : Log file name"   << std::endl;
    std::cerr << "    <verbose>  : Verbose level (0: None; 1: Header; 2: Header + Body)"   << std::endl;
    exit(0);
    }

  int    port  = atoi(argv[1]);
  VerboseLevel = atoi(argv[3]);
  const char * logFileName = argv[2];


  igtl::ServerSocket::Pointer serverSocket;
  serverSocket = igtl::ServerSocket::New();
  int r = serverSocket->CreateServer(port);

  if (r < 0)
    {
    std::cerr << "Cannot create a server socket." << std::endl;
    exit(0);
    }

  igtl::Socket::Pointer socket;
  
  while (1)
    {
    //------------------------------------------------------------
    // Waiting for Connection
    socket = serverSocket->WaitForConnection(1000);
    
    if (socket.IsNotNull()) // if client connected
      {
      // Open a log file
      logFile.open(logFileName, std::ios::app);
      if (logFile.is_open())
        {
        // Print header
        logFile << "Type, Name, Origin Time, Delivery Time, Latency" << std::endl;
        }
      
        
      // Create a message buffer to receive header
      igtl::MessageHeader::Pointer headerMsg;
      headerMsg = igtl::MessageHeader::New();

      //------------------------------------------------------------
      // Allocate time stamps

      // Message time stamp
      igtl::TimeStamp::Pointer ts;
      ts = igtl::TimeStamp::New();

      // Current time
      igtl::TimeStamp::Pointer cts;
      cts = igtl::TimeStamp::New();


      //------------------------------------------------------------
      // loop
      //for (int i = 0; i < 100; i ++)
      while(1)
        {

        // Initialize receive buffer
        headerMsg->InitPack();

        // Receive generic header from the socket
        int r = socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
        if (r == 0)
          {
          socket->CloseSocket();
          break;
          }
        if (r != headerMsg->GetPackSize())
          {
          continue;
          }

        // Deserialize the header
        headerMsg->Unpack();

        // Check data type and receive data body
        if (strcmp(headerMsg->GetDeviceType(), "TRANSFORM") == 0)
          {
          ReceiveTransform(socket, headerMsg);
          }
        else if (strcmp(headerMsg->GetDeviceType(), "POSITION") == 0)
          {
          ReceivePosition(socket, headerMsg);
          }
        else if (strcmp(headerMsg->GetDeviceType(), "IMAGE") == 0)
          {
          ReceiveImage(socket, headerMsg);
          }
        else if (strcmp(headerMsg->GetDeviceType(), "STATUS") == 0)
          {
          ReceiveStatus(socket, headerMsg);
          }
#if OpenIGTLink_PROTOCOL_VERSION >= 2
        else if (strcmp(headerMsg->GetDeviceType(), "POINT") == 0)
          {
          ReceivePoint(socket, headerMsg);
          }
        else if (strcmp(headerMsg->GetDeviceType(), "TRAJ") == 0)
          {
          ReceiveTrajectory(socket, headerMsg);
          }
        else if (strcmp(headerMsg->GetDeviceType(), "STRING") == 0)
          {
          ReceiveString(socket, headerMsg);
          }
        else if (strcmp(headerMsg->GetDeviceType(), "BIND") == 0)
          {
          ReceiveBind(socket, headerMsg);
          }
        else if (strcmp(headerMsg->GetDeviceType(), "CAPABILITY") == 0)
          {
          ReceiveCapability(socket, headerMsg);
          }
#endif //OpenIGTLink_PROTOCOL_VERSION >= 2
        else
          {
          // if the data type is unknown, skip reading.
          std::cerr << "Receiving : " << headerMsg->GetDeviceType() << std::endl;
          std::cerr << "Size : " << headerMsg->GetBodySizeToRead() << std::endl;
          socket->Skip(headerMsg->GetBodySizeToRead(), 0);
          }

        if (logFile.is_open())
          {
          headerMsg->GetTimeStamp(ts);

          // Get current time
          cts->GetTime();
          
          // Compute latency -- assuming that the clocks of the sender and the receiver are synchronized.
          double dts = ts->GetTimeStamp();
          double dcts = cts->GetTimeStamp();
          double latency = dcts-dts;

          // Get time stamp
          igtlUint32 sec;
          igtlUint32 nanosec;
          ts->GetTimeStamp(&sec, &nanosec);
          
          igtlUint32 csec;
          igtlUint32 cnanosec;
          cts->GetTimeStamp(&csec, &cnanosec);

          // Type, Name, Origin Time, Delivery Time, Latency" << std::endl;
          logFile << headerMsg->GetDeviceType() << ", "
                  << headerMsg->GetDeviceName() << ", "
                  << sec << "."  << std::setw(9) << std::setfill('0') << nanosec << ", "
                  << csec << "." << std::setw(9) << std::setfill('0') << cnanosec << ", "
                  << latency << std::endl;
          }

        }
      logFile.close();
      }
    }
    
  //------------------------------------------------------------
  // Close connection (The example code never reaches to this section ...)
  
  socket->CloseSocket();

}


int ReceiveTransform(igtl::Socket * socket, igtl::MessageHeader * header)
{
  if (VerboseLevel > 0)
    {
    std::cerr << "Receiving TRANSFORM data type." << std::endl;
    }
  
  // Create a message buffer to receive transform data
  igtl::TransformMessage::Pointer transMsg;
  transMsg = igtl::TransformMessage::New();
  transMsg->SetMessageHeader(header);
  transMsg->AllocatePack();
  
  // Receive transform data from the socket
  socket->Receive(transMsg->GetPackBodyPointer(), transMsg->GetPackBodySize());
  
  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = transMsg->Unpack(1);
  
  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
    // Retrive the transform data
    igtl::Matrix4x4 matrix;
    transMsg->GetMatrix(matrix);
    if (VerboseLevel >= 2)
      {
      igtl::PrintMatrix(matrix);
      }
    return 1;
    }

  return 0;

}


int ReceivePosition(igtl::Socket * socket, igtl::MessageHeader * header)
{
  if (VerboseLevel > 0)
    {
    std::cerr << "Receiving POSITION data type." << std::endl;
    }
  
  // Create a message buffer to receive transform data
  igtl::PositionMessage::Pointer positionMsg;
  positionMsg = igtl::PositionMessage::New();
  positionMsg->SetMessageHeader(header);
  positionMsg->AllocatePack();
  
  // Receive position position data from the socket
  socket->Receive(positionMsg->GetPackBodyPointer(), positionMsg->GetPackBodySize());
  
  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = positionMsg->Unpack(1);
  
  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
    // Retrive the transform data
    float position[3];
    float quaternion[4];

    positionMsg->GetPosition(position);
    positionMsg->GetQuaternion(quaternion);

    if (VerboseLevel > 1)
      {
      std::cerr << "position   = (" << position[0] << ", " << position[1] << ", " << position[2] << ")" << std::endl;
      std::cerr << "quaternion = (" << quaternion[0] << ", " << quaternion[1] << ", "
                << quaternion[2] << ", " << quaternion[3] << ")" << std::endl << std::endl;
      }

    return 1;
    }

  return 0;

}


int ReceiveImage(igtl::Socket * socket, igtl::MessageHeader * header)
{
  if (VerboseLevel > 0)
    {
    std::cerr << "Receiving IMAGE data type." << std::endl;
    }

  // Create a message buffer to receive transform data
  igtl::ImageMessage::Pointer imgMsg;
  imgMsg = igtl::ImageMessage::New();
  imgMsg->SetMessageHeader(header);
  imgMsg->AllocatePack();
  
  // Receive transform data from the socket
  socket->Receive(imgMsg->GetPackBodyPointer(), imgMsg->GetPackBodySize());
  
  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = imgMsg->Unpack(1);
  
  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
    // Retrive the image data
    int   size[3];          // image dimension
    float spacing[3];       // spacing (mm/pixel)
    int   svsize[3];        // sub-volume size
    int   svoffset[3];      // sub-volume offset
    int   scalarType;       // scalar type

    scalarType = imgMsg->GetScalarType();
    imgMsg->GetDimensions(size);
    imgMsg->GetSpacing(spacing);
    imgMsg->GetSubVolume(svsize, svoffset);

    if (VerboseLevel > 1)
      {
      std::cerr << "Device Name           : " << imgMsg->GetDeviceName() << std::endl;
      std::cerr << "Scalar Type           : " << scalarType << std::endl;
      std::cerr << "Dimensions            : ("
                << size[0] << ", " << size[1] << ", " << size[2] << ")" << std::endl;
      std::cerr << "Spacing               : ("
                << spacing[0] << ", " << spacing[1] << ", " << spacing[2] << ")" << std::endl;
      std::cerr << "Sub-Volume dimensions : ("
                << svsize[0] << ", " << svsize[1] << ", " << svsize[2] << ")" << std::endl;
      std::cerr << "Sub-Volume offset     : ("
                << svoffset[0] << ", " << svoffset[1] << ", " << svoffset[2] << ")" << std::endl;
      }
    return 1;
    }

  return 0;

}


int ReceiveStatus(igtl::Socket * socket, igtl::MessageHeader * header)
{
  if (VerboseLevel > 0)
    {
    std::cerr << "Receiving STATUS data type." << std::endl;
    }

  // Create a message buffer to receive transform data
  igtl::StatusMessage::Pointer statusMsg;
  statusMsg = igtl::StatusMessage::New();
  statusMsg->SetMessageHeader(header);
  statusMsg->AllocatePack();
  
  // Receive transform data from the socket
  socket->Receive(statusMsg->GetPackBodyPointer(), statusMsg->GetPackBodySize());
  
  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = statusMsg->Unpack(1);
  
  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
    if (VerboseLevel > 1)
      {
      std::cerr << "========== STATUS ==========" << std::endl;
      std::cerr << " Code      : " << statusMsg->GetCode() << std::endl;
      std::cerr << " SubCode   : " << statusMsg->GetSubCode() << std::endl;
      std::cerr << " Error Name: " << statusMsg->GetErrorName() << std::endl;
      std::cerr << " Status    : " << statusMsg->GetStatusString() << std::endl;
      std::cerr << "============================" << std::endl;
      }
    }

  return 0;

}


#if OpenIGTLink_PROTOCOL_VERSION >= 2
int ReceivePoint(igtl::Socket * socket, igtl::MessageHeader * header)
{
  if (VerboseLevel > 0)
    {
    std::cerr << "Receiving POINT data type." << std::endl;
    }

  // Create a message buffer to receive transform data
  igtl::PointMessage::Pointer pointMsg;
  pointMsg = igtl::PointMessage::New();
  pointMsg->SetMessageHeader(header);
  pointMsg->AllocatePack();

  // Receive transform data from the socket
  socket->Receive(pointMsg->GetPackBodyPointer(), pointMsg->GetPackBodySize());

  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = pointMsg->Unpack(1);

  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
    int nElements = pointMsg->GetNumberOfPointElement();
    for (int i = 0; i < nElements; i ++)
      {
      igtl::PointElement::Pointer pointElement;
      pointMsg->GetPointElement(i, pointElement);

      igtlUint8 rgba[4];
      pointElement->GetRGBA(rgba);

      igtlFloat32 pos[3];
      pointElement->GetPosition(pos);

      if (VerboseLevel > 1)
        {
        std::cerr << "========== Element #" << i << " ==========" << std::endl;
        std::cerr << " Name      : " << pointElement->GetName() << std::endl;
        std::cerr << " GroupName : " << pointElement->GetGroupName() << std::endl;
        std::cerr << " RGBA      : ( " << (int)rgba[0] << ", " << (int)rgba[1] << ", " << (int)rgba[2] << ", " << (int)rgba[3] << " )" << std::endl;
        std::cerr << " Position  : ( " << std::fixed << pos[0] << ", " << pos[1] << ", " << pos[2] << " )" << std::endl;
        std::cerr << " Radius    : " << std::fixed << pointElement->GetRadius() << std::endl;
        std::cerr << " Owner     : " << pointElement->GetOwner() << std::endl;
        std::cerr << "================================" << std::endl;
        }
      }
    }

  return 1;
}

int ReceiveTrajectory(igtl::Socket * socket, igtl::MessageHeader::Pointer& header)
{
  if (VerboseLevel > 0)
    {
    std::cerr << "Receiving TRAJECTORY data type." << std::endl;
    }

  // Create a message buffer to receive transform data
  igtl::TrajectoryMessage::Pointer trajectoryMsg;
  trajectoryMsg = igtl::TrajectoryMessage::New();
  trajectoryMsg->SetMessageHeader(header);
  trajectoryMsg->AllocatePack();

  // Receive transform data from the socket
  socket->Receive(trajectoryMsg->GetPackBodyPointer(), trajectoryMsg->GetPackBodySize());

  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = trajectoryMsg->Unpack(1);

  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
    int nElements = trajectoryMsg->GetNumberOfTrajectoryElement();
    for (int i = 0; i < nElements; i ++)
      {
      igtl::TrajectoryElement::Pointer trajectoryElement;
      trajectoryMsg->GetTrajectoryElement(i, trajectoryElement);

      igtlUint8 rgba[4];
      trajectoryElement->GetRGBA(rgba);

      igtlFloat32 entry[3];
      igtlFloat32 target[3];
      trajectoryElement->GetEntryPosition(entry);
      trajectoryElement->GetTargetPosition(target);

      if (VerboseLevel > 1)
        {
        std::cerr << "========== Element #" << i << " ==========" << std::endl;
        std::cerr << " Name      : " << trajectoryElement->GetName() << std::endl;
        std::cerr << " GroupName : " << trajectoryElement->GetGroupName() << std::endl;
        std::cerr << " RGBA      : ( " << (int)rgba[0] << ", " << (int)rgba[1] << ", " << (int)rgba[2] << ", " << (int)rgba[3] << " )" << std::endl;
        std::cerr << " Entry Pt  : ( " << std::fixed << entry[0] << ", " << entry[1] << ", " << entry[2] << " )" << std::endl;
        std::cerr << " Target Pt : ( " << std::fixed << target[0] << ", " << target[1] << ", " << target[2] << " )" << std::endl;
        std::cerr << " Radius    : " << std::fixed << trajectoryElement->GetRadius() << std::endl;
        std::cerr << " Owner     : " << trajectoryElement->GetOwner() << std::endl;
        std::cerr << "================================" << std::endl << std::endl;
        }
      }
    }

  return 1;
}


int ReceiveString(igtl::Socket * socket, igtl::MessageHeader * header)
{
  if (VerboseLevel > 0)
    {
    std::cerr << "Receiving STRING data type." << std::endl;
    }

  // Create a message buffer to receive transform data
  igtl::StringMessage::Pointer stringMsg;
  stringMsg = igtl::StringMessage::New();
  stringMsg->SetMessageHeader(header);
  stringMsg->AllocatePack();

  // Receive transform data from the socket
  socket->Receive(stringMsg->GetPackBodyPointer(), stringMsg->GetPackBodySize());

  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = stringMsg->Unpack(1);

  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
    if (VerboseLevel > 1)
      {
      std::cerr << "Encoding: " << stringMsg->GetEncoding() << "; "
                << "String: " << stringMsg->GetString() << std::endl;
      }
    }

  return 1;
}


int ReceiveBind(igtl::Socket * socket, igtl::MessageHeader * header)
{
  if (VerboseLevel > 0)
    {
    std::cerr << "Receiving BIND data type." << std::endl;
    }

  // Create a message buffer to receive transform data
  igtl::BindMessage::Pointer bindMsg;
  bindMsg = igtl::BindMessage::New();
  bindMsg->SetMessageHeader(header);
  bindMsg->AllocatePack();

  // Receive transform data from the socket
  socket->Receive(bindMsg->GetPackBodyPointer(), bindMsg->GetPackBodySize());

  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = bindMsg->Unpack(1);

  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
    int n = bindMsg->GetNumberOfChildMessages();

    for (int i = 0; i < n; i ++)
      {
      if (strcmp(bindMsg->GetChildMessageType(i), "STRING") == 0)
        {
        igtl::StringMessage::Pointer stringMsg;
        stringMsg = igtl::StringMessage::New();
        bindMsg->GetChildMessage(i, stringMsg);
        stringMsg->Unpack(0);
        if (VerboseLevel > 1)
          {
          std::cerr << "Message type: STRING" << std::endl;
          std::cerr << "Message name: " << stringMsg->GetDeviceName() << std::endl;
          std::cerr << "Encoding: " << stringMsg->GetEncoding() << "; "
                    << "String: " << stringMsg->GetString() << std::endl;
          }
        }
      else if (strcmp(bindMsg->GetChildMessageType(i), "TRANSFORM") == 0)
        {
        igtl::TransformMessage::Pointer transMsg;
        transMsg = igtl::TransformMessage::New();
        bindMsg->GetChildMessage(i, transMsg);
        transMsg->Unpack(0);
        if (VerboseLevel > 1)
          {
          std::cerr << "Message type: TRANSFORM" << std::endl;
          std::cerr << "Message name: " << transMsg->GetDeviceName() << std::endl;
          igtl::Matrix4x4 matrix;
          transMsg->GetMatrix(matrix);
          igtl::PrintMatrix(matrix);
          }
        }
      }
    }

  return 1;
}


int ReceiveCapability(igtl::Socket * socket, igtl::MessageHeader * header)
{
  if (VerboseLevel > 0)
    {
    std::cerr << "Receiving CAPABILITY data type." << std::endl;
    }

  // Create a message buffer to receive transform data
  igtl::CapabilityMessage::Pointer capabilMsg;
  capabilMsg = igtl::CapabilityMessage::New();
  capabilMsg->SetMessageHeader(header);
  capabilMsg->AllocatePack();

  // Receive transform data from the socket
  socket->Receive(capabilMsg->GetPackBodyPointer(), capabilMsg->GetPackBodySize());

  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = capabilMsg->Unpack(1);
  
  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
    int nTypes = capabilMsg->GetNumberOfTypes();
    if (VerboseLevel > 1)
      {
      for (int i = 0; i < nTypes; i ++)
        {
        std::cerr << "Typename #" << i << ": " << capabilMsg->GetType(i) << std::endl;
        }
      }
    }

  return 1;
  
}


#endif //OpenIGTLink_PROTOCOL_VERSION >= 2
