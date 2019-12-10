// Copyright Andrin Jenal ETHZ Zurich
#ifndef BASE_FILE_UTILS_H_
#define BASE_FILE_UTILS_H_

#include <string>

namespace base {

class FileUtils {

static bool IsDir(const std::string& dir);
static bool CreateDirRecursivelyOrDie(const std::string& dir,
                                      const std::string& delimiter = "/");

};

}  // namespace base

#endif  // BASE_FILE_UTILS_H_
