#ifndef __RoboteqDevice_H_
#define __RoboteqDevice_H_

using namespace std;

string ReplaceString(string source, string find, string replacement);
void sleepms(int milliseconds);

class RoboteqDevice
{
private:
  int device_fd;
  int fd0;
  int handle;

protected:
  void InitPort();

  int Write(string str);
  int ReadAll(string &str);

  int IssueCommandId(int id, string commandType, string command, string args, int waitms, string &response, bool isplusminus = false);
  int IssueCommandId(int id, string commandType, string command, int waitms, string &response, bool isplusminus = false);

  int IssueCommand(string commandType, string command, string args, int waitms, string &response, bool isplusminus = false);
  int IssueCommand(string commandType, string command, int waitms, string &response, bool isplusminus = false);

public:
  bool IsConnected();
  int Connect(string port);
  void Disconnect();

  int SetConfig(int configItem, int index, int value);
  int SetConfig(int configItem, int value);

  int SetCommand(int commandItem, int index, int value);
  int SetCommand(int commandItem, int value);
  int SetCommand(int commandItem);

  int GetConfig(int configItem, int index, int &result);
  int GetConfig(int configItem, int &result);

  int GetValue(int operatingItem, int index, int &result);
  int GetValue(int operatingItem, int &result);

  int SetConfigId(int id, int configItem, int index, int value);
  int SetConfigId(int id, int configItem, int value);

  int SetCommandId(int id, int commandItem, int index, int value);
  int SetCommandId(int id, int commandItem, int value);
  int SetCommandId(int id, int commandItem);

  int GetConfigId(int id, int configItem, int index, int &result);
  int GetConfigId(int id, int configItem, int &result);

  int GetValueId(int id, int operatingItem, int index, int &result);
  int GetValueId(int id, int operatingItem, int &result);


  RoboteqDevice();
  ~RoboteqDevice();
};

#endif
