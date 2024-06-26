#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sstream>
#include <unistd.h>

#include "smb_driver/RoboteqDevice.h"
#include "smb_driver/ErrorCodes.h"

using namespace std;

#define BUFFER_SIZE 1024
#define MISSING_VALUE -1024

// #define ROBOTEQ_DEBUG
// #define PRINTING_ON

RoboteqDevice::RoboteqDevice()
{
  handle = RQ_INVALID_HANDLE;
}
RoboteqDevice::~RoboteqDevice()
{
  Disconnect();
}

bool RoboteqDevice::IsConnected()
{
#ifdef ROBOTEQ_DEBUG
  return true;
#endif
  return handle != RQ_INVALID_HANDLE;
}

int RoboteqDevice::Connect(string port)
{
  if(IsConnected())
  {
    cout<<"[RoboteqDriverLib]Device is connected, attempting to disconnect."<<endl;
    Disconnect();
  }
#ifdef ROBOTEQ_DEBUG
  cout<<"[RoboteqDriverLib]Simulating serial connections\n";
#endif
#ifndef ROBOTEQ_DEBUG
  //Open port.
  cout<<"[RoboteqDriverLib]Opening port: '"<<port<<"'...";
  handle = open(port.c_str(), O_RDWR |O_NOCTTY | O_NDELAY);
  if(handle == RQ_INVALID_HANDLE)
  {
    cout<<"[RoboteqDriverLib]failed."<<endl;
    return RQ_ERR_OPEN_PORT;
  }

  cout<<"[RoboteqDriverLib]succeeded."<<endl;
  fcntl (handle, F_SETFL, O_APPEND | O_NONBLOCK & ~FNDELAY);

  cout<<"[RoboteqDriverLib]Initializing port...";
  InitPort();
  cout<<"...done."<<endl;

  int status;
  string response;
  cout<<"[RoboteqDriverLib]Detecting device version...";
  int z;

  for(z=0;z<5;z++){
    status = IssueCommand("?", "FID", 50000, response);

    if(status != RQ_SUCCESS)
      {continue;}
    if(status == RQ_SUCCESS)
      {break;}
  }

  if(status != RQ_SUCCESS)
  {
    cout<<"[RoboteqDriverLib]failed (issue ?FID response: "<<status<<")."<<endl;
    Disconnect();
    return RQ_UNRECOGNIZED_DEVICE;
  }


  // cout<<response.substr(8, 4)<<"."<<endl;
#endif
  return RQ_SUCCESS;
}

void RoboteqDevice::Disconnect()
{
  if(IsConnected())
    close(handle);

  handle = RQ_INVALID_HANDLE;
}

void RoboteqDevice::InitPort()
{
  if(!IsConnected())
    return;

  //Get the existing Comm Port Attributes in cwrget
  int BAUDRATE = B115200;
  struct termios newtio;
  tcgetattr (handle, &newtio);

  //Set the Tx and Rx Baud Rate to 115200
  cfsetospeed (&newtio, (speed_t)BAUDRATE);
  cfsetispeed (&newtio, (speed_t)BAUDRATE);

  //Enable the Receiver and  Set local Mode
  newtio.c_iflag = IGNBRK;    /* Ignore Break Condition & no processing under input options*/
  newtio.c_lflag = 0;      /* Select the RAW Input Mode through Local options*/
  newtio.c_oflag = 0;      /* Select the RAW Output Mode through Local options*/
  newtio.c_cflag |= (CLOCAL | CREAD);  /* Select the Local Mode & Enable Receiver through Control options*/

  //Make RAW Mode more explicit by turning Canonical Mode off, Echo off, Echo Erase off and Signals off*/
  newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  //Disable Software Flow Control
  newtio.c_iflag &= ~(IXON | IXOFF | IXANY);

  //Set Data format to 8N1
  newtio.c_cflag &= ~CSIZE;    /* Mask the Character Size Bits through Control options*/
  newtio.c_cflag |= CS8;      /* Select Character Size to 8-Bits through Control options*/
  newtio.c_cflag &= ~PARENB;    /* Select Parity Disable through Control options*/
  newtio.c_cflag &= ~PARODD;    /* Select the Even Parity (Disabled) through Control options*/
  newtio.c_cflag &= ~CSTOPB;    /*Set number of Stop Bits to 1*/

  //Timout Parameters. Set to 0 characters (VMIN) and 10 second (VTIME) timeout. This was done to prevent the read call from blocking indefinitely.*/
  newtio.c_cc[VMIN] = 0;
  newtio.c_cc[VTIME] = 100;

  /* Flush the Input buffer and set the attribute NOW without waiting for Data to Complete*/
  tcflush (handle, TCIFLUSH);
  tcsetattr (handle, TCSANOW, &newtio);
}

int RoboteqDevice::Write(string str)
{
//  cout<<"[RoboteqDriverLib]=======Inside Write========\n";
  if(!IsConnected())
    return RQ_ERR_NOT_CONNECTED;

#ifndef ROBOTEQ_DEBUG
#ifdef PRINTING_ON
  cout<<"[RoboteqDriverLib]writing....."<<str<<"\n";
#endif
  int countSent = write(handle, str.c_str(), str.length());

  //Verify weather the Transmitting Data on UART was Successful or Not
  if(countSent < 0)
    return RQ_ERR_TRANSMIT_FAILED;
#endif

#ifdef ROBOTEQ_DEBUG
  cout<<"[RoboteqDriverLib]Sending Data on serial="<<str<<"\n";
#endif

  return RQ_SUCCESS;
}
int RoboteqDevice::ReadAll(string &str)
{
  int countRcv;
  if(!IsConnected())
    return RQ_ERR_NOT_CONNECTED;

  char buf[BUFFER_SIZE + 1] = "";

  str = "";
#ifndef ROBOTEQ_DEBUG
  while((countRcv = read(handle, buf, BUFFER_SIZE)) > 0)
  {
    str.append(buf, countRcv);

    //No further data.
    if(countRcv < BUFFER_SIZE)
      break;
  }

  if(countRcv < 0)
  {
    if(errno == EAGAIN)
      return RQ_ERR_SERIAL_IO;
    else
      return RQ_ERR_SERIAL_RECEIVE;
  }
#endif

#ifdef ROBOTEQ_DEBUG
  cout<<"[RoboteqDriverLib]Simulating Read - \n";
#endif

  return RQ_SUCCESS;
}

/*
 * Commands including IDs for RoboCAN protocol
 */

int RoboteqDevice::IssueCommandId(int id, string commandType, string command, string args, int waitus, string &response, bool isplusminus)
{
    int status;
    string read;
    response = "";
    char id_str[4];

    sprintf(id_str,"%02d",id);

    string id_stdstr(id_str);

    string cmdstr;
    if(args == "")
        cmdstr = ("@" + id_stdstr + commandType + command + "\r");
    else
        cmdstr = ("@" + id_stdstr + commandType + command + " " + args + "\r");
#ifdef PRINTING_ON
    cout<<"[RoboteqDriverLib]Issuing command = "<<cmdstr<<"\n";
#endif
    status = Write(cmdstr);
    if(status != RQ_SUCCESS)
        return status;
#ifdef PRINTING_ON
    cout<<"[RoboteqDriverLib]Writing done\n";
#endif
    usleep(waitus);

    status = ReadAll(read);

    if(status != RQ_SUCCESS)
        return status;

    if(isplusminus) // isplusminus is for commands where the response is only a + or -
    {
        if(read.length() < 2) // Every response returns a + or - followed by \r\n = 2 chars?
            return RQ_INVALID_RESPONSE;

        response = read.substr(read.length() - 2, 1); //
        return RQ_SUCCESS;
    }

    // the response is like this
    // AI=123
    // where AI is the command and 123 is the value
    // For a RoboCan command with ID, the return value looks like this
    // @04 AI=123
    // Where 04 is the id
    //
    // How the following code works
    // Find the position of 'AI=' which is (command+"=")
    // then add command length + 1(for the =) to get to the starting index of the actual value
    // then find the position of '\r', ie the last character
    // now length of the value is last char position - first char position
    //
    // position of id = original pos - 3 [considering that id is always a 2 digit number]

    string::size_type pos = read.rfind(command + "=");
    if(pos == string::npos)
        return RQ_INVALID_RESPONSE;

    int id_pos = pos - 3;
    string response_id = read.substr(id_pos, 2); // Id is always 2 digits
#ifdef PRINTING_ON
    if(response_id.compare(id_stdstr) == 0) {
      cout<<"[RoboteqDriverLib]Response and request ids match<<"<<read<<"\n";
    } else {
      cout<<"[RoboteqDriverLib]Response and request id mismatch"<<read<<"\n";
    }
#endif
    pos += command.length() + 1;

    string::size_type carriage = read.find("\r", pos);
    if(carriage == string::npos)
        return RQ_INVALID_RESPONSE;

    response = read.substr(pos, carriage - pos);

    return RQ_SUCCESS;
}
int RoboteqDevice::IssueCommandId(int id, string commandType, string command, int waitus, string &response, bool isplusminus)
{
    return IssueCommandId(id, commandType, command, "", waitus, response, isplusminus);
}

int RoboteqDevice::SetConfigId(int id, int configItem, int index, int value)
{
    string response;
    char command[10];
    char args[50];

    if(configItem < 0 || configItem > 255)
        return RQ_INVALID_CONFIG_ITEM;

    sprintf(command, "$%02X", configItem);
    sprintf(args, "%i %i", index, value);
    if(index == MISSING_VALUE)
    {
        sprintf(args, "%i", value);
        index = 0;
    }

    if(index < 0)
        return RQ_INDEX_OUT_RANGE;

    int status = IssueCommandId(id, "^", command, args, wait_response_us, response, true);
    if(status != RQ_SUCCESS)
        return status;
    if(response != "+")
        return RQ_SET_CONFIG_FAILED;

    return RQ_SUCCESS;
}
int RoboteqDevice::SetConfigId(int id, int configItem, int value)
{
    return SetConfigId(id, configItem, MISSING_VALUE, value);
}

int RoboteqDevice::SetCommandId(int id, int commandItem, int index, int value)
{
    string response;
    char command[10];
    char args[50];

    if(commandItem < 0 || commandItem > 255)
        return RQ_INVALID_COMMAND_ITEM;

    sprintf(command, "$%02X", commandItem);
    sprintf(args, "%i %i", index, value);
    if(index == MISSING_VALUE)
    {
        if(value != MISSING_VALUE)
            sprintf(args, "%i", value);
        index = 0;
    }

    if(index < 0)
        return RQ_INDEX_OUT_RANGE;

    int status = IssueCommandId(id, "!", command, args, wait_response_us, response, true);
    if(status != RQ_SUCCESS)
        return status;
    if(response != "+")
        return RQ_SET_COMMAND_FAILED;

    return RQ_SUCCESS;
}
int RoboteqDevice::SetCommandId(int id, int commandItem, int value)
{
    return SetCommandId(id, commandItem, MISSING_VALUE, value);
}
int RoboteqDevice::SetCommandId(int id, int commandItem)
{
    return SetCommandId(id, commandItem, MISSING_VALUE, MISSING_VALUE);
}

int RoboteqDevice::GetConfigId(int id, int configItem, int index, int &result)
{
    string response;
    char command[10];
    char args[50];

    if(configItem < 0 || configItem > 255)
        return RQ_INVALID_CONFIG_ITEM;

    if(index < 0)
        return RQ_INDEX_OUT_RANGE;

    sprintf(command, "$%02X", configItem);
    sprintf(args, "%i", index);

    int status = IssueCommandId(id, "~", command, args, wait_response_us, response);
    if(status != RQ_SUCCESS)
        return status;

    istringstream iss(response);
    iss>>result;

    if(iss.fail())
        return RQ_GET_CONFIG_FAILED;

    return RQ_SUCCESS;
}
int RoboteqDevice::GetConfigId(int id, int configItem, int &result)
{
    return GetConfigId(id, configItem, 0, result);
}

int RoboteqDevice::GetValueId(int id, int operatingItem, int index, int &result)
{
    string response;
    char command[10];
    char args[50];

    if(operatingItem < 0 || operatingItem > 255)
        return RQ_INVALID_OPER_ITEM;

    if(index < 0)
        return RQ_INDEX_OUT_RANGE;

    sprintf(command, "$%02X", operatingItem);
    sprintf(args, "%i", index);

    int status = IssueCommandId(id, "?", command, args, wait_response_us, response);
    if(status != RQ_SUCCESS)
        return status;

    istringstream iss(response);
    iss>>result;

    if(iss.fail())
        return RQ_GET_VALUE_FAILED;

    return RQ_SUCCESS;
}
int RoboteqDevice::GetValueId(int id, int operatingItem, int &result)
{
    return GetValueId(id, operatingItem, 0, result);
}



/*
 * No ID Commands
 */

int RoboteqDevice::IssueCommand(string commandType, string command, string args, int waitus, string &response, bool isplusminus)
{
  int status;
  string read;
  response = "";

  if(args == "")
    status = Write(commandType + command + "\r");
  else
    status = Write(commandType + command + " " + args + "\r");

  if(status != RQ_SUCCESS)
    return status;

  usleep(waitus);

  status = ReadAll(read);
  if(status != RQ_SUCCESS)
    return status;

#ifndef ROBOTEQ_DEBUG
// Only check the status if not in debug mode
  if(isplusminus)
  {
    if(read.length() < 2)
      return RQ_INVALID_RESPONSE;

    response = read.substr(read.length() - 2, 1);
    return RQ_SUCCESS;
  }


  string::size_type pos = read.rfind(command + "=");
  if(pos == string::npos)
    return RQ_INVALID_RESPONSE;

  pos += command.length() + 1;

  string::size_type carriage = read.find("\r", pos);
  if(carriage == string::npos)
    return RQ_INVALID_RESPONSE;

  response = read.substr(pos, carriage - pos);
#endif
  return RQ_SUCCESS;
}
int RoboteqDevice::IssueCommand(string commandType, string command, int waitus, string &response, bool isplusminus)
{
  return IssueCommand(commandType, command, "", waitus, response, isplusminus);
}

int RoboteqDevice::SetConfig(int configItem, int index, int value)
{
  string response;
  char command[10];
  char args[50];

  if(configItem < 0 || configItem > 255)
    return RQ_INVALID_CONFIG_ITEM;

  sprintf(command, "$%02X", configItem);
  sprintf(args, "%i %i", index, value);
  if(index == MISSING_VALUE)
  {
    sprintf(args, "%i", value);
    index = 0;
  }

  if(index < 0)
    return RQ_INDEX_OUT_RANGE;

  int status = IssueCommand("^", command, args, wait_response_us, response, true);
  if(status != RQ_SUCCESS)
    return status;
  if(response != "+")
    return RQ_SET_CONFIG_FAILED;

  return RQ_SUCCESS;
}
int RoboteqDevice::SetConfig(int configItem, int value)
{
  return SetConfig(configItem, MISSING_VALUE, value);
}

int RoboteqDevice::SetCommand(int commandItem, int index, int value)
{
  string response;
  char command[10];
  char args[50];

  if(commandItem < 0 || commandItem > 255)
    return RQ_INVALID_COMMAND_ITEM;

  sprintf(command, "$%02X", commandItem);
  sprintf(args, "%i %i", index, value);
  if(index == MISSING_VALUE)
  {
    if(value != MISSING_VALUE)
      sprintf(args, "%i", value);
    index = 0;
  }

  if(index < 0)
    return RQ_INDEX_OUT_RANGE;

  int status = IssueCommand("!", command, args, wait_response_us, response, true);
  if(status != RQ_SUCCESS)
    return status;
  if(response != "+")
    return RQ_SET_COMMAND_FAILED;

  return RQ_SUCCESS;
}
int RoboteqDevice::SetCommand(int commandItem, int value)
{
  return SetCommand(commandItem, MISSING_VALUE, value);
}
int RoboteqDevice::SetCommand(int commandItem)
{
  return SetCommand(commandItem, MISSING_VALUE, MISSING_VALUE);
}

int RoboteqDevice::GetConfig(int configItem, int index, int &result)
{
  string response;
  char command[10];
  char args[50];

  if(configItem < 0 || configItem > 255)
    return RQ_INVALID_CONFIG_ITEM;

  if(index < 0)
    return RQ_INDEX_OUT_RANGE;

  sprintf(command, "$%02X", configItem);
  sprintf(args, "%i", index);

  int status = IssueCommand("~", command, args, wait_response_us, response);
  if(status != RQ_SUCCESS)
    return status;
#ifndef ROBOTEQ_DEBUG
  istringstream iss(response);
  iss>>result;

  if(iss.fail())
    return RQ_GET_CONFIG_FAILED;
#endif

  return RQ_SUCCESS;
}
int RoboteqDevice::GetConfig(int configItem, int &result)
{
  return GetConfig(configItem, 0, result);
}

int RoboteqDevice::GetValue(int operatingItem, int index, int &result)
{
  string response;
  char command[10];
  char args[50];

  if(operatingItem < 0 || operatingItem > 255)
    return RQ_INVALID_OPER_ITEM;

  if(index < 0)
    return RQ_INDEX_OUT_RANGE;

  sprintf(command, "$%02X", operatingItem);
  sprintf(args, "%i", index);

  int status = IssueCommand("?", command, args, wait_response_us, response);
  if(status != RQ_SUCCESS)
    return status;

  istringstream iss(response);
  iss>>result;

  if(iss.fail())
    return RQ_GET_VALUE_FAILED;

  return RQ_SUCCESS;
}
int RoboteqDevice::GetValue(int operatingItem, int &result)
{
  return GetValue(operatingItem, 0, result);
}


string ReplaceString(string source, string find, string replacement)
{
  string::size_type pos = 0;
    while((pos = source.find(find, pos)) != string::npos)
  {
        source.replace(pos, find.size(), replacement);
        pos++;
    }

  return source;
}

void sleepms(int milliseconds)
{
  usleep(milliseconds / 1000);
}
