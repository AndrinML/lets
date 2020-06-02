#include <iostream>
#include <sstream>

#include "VideoProcessing.hpp"

int main (int argc, char** argv) {
    std::string file;
    std::string fileName;
    std::string fileType;

    if (argc > 1) {
        file = argv[1];
        fileName = file.substr(0, file.size() - 4);
        fileType = file.substr(file.size() - 4, file.size());

        std::cout << "open file: " << fileName << std::endl;
        std::cout << "file type: " << fileType << std::endl;
    } else {
        fileName = "mini_waterfall";
        fileType = ".avi";
    }

    VideoProcessing VidProc(fileName + fileType, fileName);

    return 0;
}
