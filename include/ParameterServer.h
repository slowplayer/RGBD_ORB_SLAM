#ifndef PARAMETER_SERVER_H_
#define PARAMETER_SERVER_H_
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{
class ParameterServer
{
public:
  static ParameterServer* instance();
  bool setFilePath(const std::string filename);
  
  float inline getParam(const std::string param){return config[param];}
private:
  static ParameterServer* _instance;
  ParameterServer();
  bool loadParam(const std::string filename);
  void inline setParam(const std::string param,float value){config[param]=value;}
 
  std::map<std::string,float> config;
 
};
}
#endif