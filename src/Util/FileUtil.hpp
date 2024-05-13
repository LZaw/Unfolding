
#pragma once

// C++ Headers
#include <string>

namespace FileUtil
{
  std::string calculateFileTitle(const std::string& path);

  bool hasFileEnding(const std::string& path);

  std::string removeFileEnding(const std::string& path);

  bool stringEndsWith(const std::string& string, const std::string& ending);
}
