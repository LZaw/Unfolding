
#include "FileUtil.hpp"

#if defined(WIN32) || defined(_WIN32)
  #define PATH_SEPARATOR "\\"
#else
  #define PATH_SEPARATOR "/"
#endif

std::string FileUtil::calculateFileTitle(const std::string& path){
  int slashPos = path.find_last_of(PATH_SEPARATOR);
  int dotPos = path.find_last_of(".");
  std::string ret = path.substr(slashPos + 1, dotPos - slashPos - 1);
  return ret;
}

bool FileUtil::hasFileEnding(const std::string& path){
  size_t pos = path.find_last_of(".");
  if(pos == std::string::npos){
    return false;
  }
  if(path.size() - pos > 5){
    return false;
  }
  return true;
}

std::string FileUtil::removeFileEnding(const std::string& path){
  if(!hasFileEnding(path)){
    return path;
  }
  std::string ret = path.substr(0, path.find_last_of("."));
  return ret;
}

bool FileUtil::stringEndsWith(const std::string& string, const std::string& ending){
  if(string.length() >= ending.length()) {
    return (0 == string.compare(string.length() - ending.length(), ending.length(), ending));
  }else{
    return false;
  }
}
