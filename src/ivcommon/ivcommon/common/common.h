/*!
* \file common.h
* \brief 常用操作，文件创建及路径、角度弧度。
*
*　常用操作，文件创建及路径、角度弧度。
*
* \author KaiJin Ji
* \version v1.2.1
* \date 2018/07/27
*/
#pragma once
#include <cmath>
#include <time.h>
#include <string.h>
#include <iostream>
#include <sys/stat.h>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

namespace common{

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

#define MAX(a,b) (a)>(b)?(a):(b)
#define MIN(a,b) (a)<(b)?(a):(b)


/// \brief 解析linux目录的 ~/符号
/// \param path　~/目录
/// \return 解析好的全目录
inline std::string expand_user(std::string path) {

  if (!path.empty() && path[0] == '~') {
    assert(path.size() == 1 or path[1] == '/');  // or other error handling
    char const* home = getenv("HOME");
    if (home || ((home = getenv("USERPROFILE")))) {
      path.replace(0, 1, home);
    }
    else {
      char const *hdrive = getenv("HOMEDRIVE"),
        *hpath = getenv("HOMEPATH");
      assert(hdrive);  // or other error handling
      assert(hpath);
      path.replace(0, 1, std::string(hdrive) + hpath);
    }
  }
  return path;
}


inline bool SetRecordDir(const std::string& p) {
	std::string path= expand_user(p);
    boost::filesystem::path dir(path);
    if(boost::filesystem::create_directory(dir))
	return true;

    if(boost::filesystem::exists(dir)==true)
	return true;
    else
	return false;
}

/// \brief 利用当前时间创建文件夹
/// \param path　母目录
/// \return 创建好的文件夹路径
inline std::string createstampeddir(std::string path)
{
  path = expand_user(path);

  std::string dirname;
  time_t timel;
  time(&timel);
  tm* pTmp=localtime(&timel);
  char readable_start_time[100];
  memset(readable_start_time,0,21);
  sprintf(readable_start_time, "/%04d-%02d-%02d_%02d-%02d-%02d",
	  pTmp->tm_year + 1900,
	  pTmp->tm_mon + 1,
	  pTmp->tm_mday,
	  pTmp->tm_hour,
	  pTmp->tm_min,
	  pTmp->tm_sec);
  if(!SetRecordDir(path))
    std::cerr<<"can't create directory:"<<path.c_str();
  dirname = path + readable_start_time;
  if(!SetRecordDir(dirname))
    std::cerr<<"can't create directory:"<<dirname.c_str();
  return dirname;

}

inline int mkpath(std::string s, mode_t mode=0755)
{
    size_t pre=0,pos;
    std::string dir;
    int mdret;
    if(s[s.size()-1]!='/')
    {
        s+='/';
    }
    while((pos=s.find_first_of('/',pre))!=std::string::npos){
        dir=s.substr(0,pos++);
        pre=pos;
        if(dir.size()==0) continue;
        if((mdret=::mkdir(dir.c_str(),mode)) && errno!=EEXIST){
            return mdret;
        }
    }
    return mdret;
}
}
