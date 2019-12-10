#include "file_utils.h"

#include <iostream>

namespace base {

bool FileUtils::IsDir(const std::string& dir) {
    return (dir.find(".") == std::string::npos);
}

bool FileUtils::CreateDirRecursivelyOrDie(const std::string& path,
                                          const std::string& delimiter) {
    if (!IsDir(path)) return false;

    std::string dir = path;
    size_t pos = 0;
    std::string root_dir = "";
    while ((pos = path.find(delimiter)) != std::string::npos) {
        const std::string sub_dir = dir.substr(0, pos);
        std::cout << "sub dir: " << sub_dir << std::endl;
        root_dir = root_dir + delimiter + sub_dir,
        std::cout << "root dir: " << root_dir << std::endl;
        dir.erase(0, pos + delimiter.length());
    }
    return false;
}

}  // namespace base
