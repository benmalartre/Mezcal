//--------------------------------------------------------------------------------
// FILE DECLARATION
//--------------------------------------------------------------------------------
#ifndef _FILE_H_
#define _FILE_H_

#include <fstream>
#include "stringutils.h"

namespace BOB{
    enum FILE_MODE
    {
        FILE_READ,
        FILE_WRITE
    };
#define SIZE_HEADER 66
#define SIZE_BUFFER 256

#ifdef _WIN32
	#define SEPARATOR "\\"
#else
	#define SEPARATOR "/"
#endif

int getFileSize(const std::string& filePath);
inline bool directoryExists(std::string path);
inline bool fileExists(const std::string& name);
bool createDirectory(const std::string& path);

class File
{
private:
    std::string path;
    std::fstream* file;
    std::string content;
    char* buffer;
    
public:
    File(){};
    File(const std::string& path);
    void createPath(const std::string& directory, const std::string& in_name, const std::string& extension);
    bool open(FILE_MODE mode);
    bool close();
    unsigned long getFileLength();
    
    void setPath(const std::string& in_path){path = in_path;};
    
    void write(const std::string& s);
    std::string read();
    std::string readAll();
};
} // namespace BOB
#endif //_FILE_H_
