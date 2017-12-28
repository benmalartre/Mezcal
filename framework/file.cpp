#include "file.h"
namespace BOB{
    // CONSTRUCTOR
    //---------------------------------------------------------------------------------------
    File::File(const std::string& fileName)
    {
        if(fileExists(fileName))
        	path = fileName;
        else
            std::cout << "This file does NOT exists : "<< fileName << std::endl;
    }
    // DIRECTORY EXISTS
    //---------------------------------------------------------------------------------------
    inline bool directoryExists(std::string path)
    {
        struct stat info;
        
        if( stat( path.c_str(), &info ) != 0 ) return false;
        else if( info.st_mode & S_IFDIR ) return true;
        else return false;
    }
    
    // FILE EXISTS
    //---------------------------------------------------------------------------------------
    inline bool fileExists(const std::string& name){
        struct stat buffer;
        return (stat (name.c_str(), &buffer) == 0);
    }

    // CREATE DIRECTORY
    //---------------------------------------------------------------------------------------
    bool createDirectory(const std::string& path)
    {
        if(directoryExists(path)) return true;
        
        std::vector<std::string> splitted = splitString(path, SEPARATOR);
        std::string currentPath = "";
        if(startsWithString(path, SEPARATOR))currentPath = SEPARATOR;
        mode_t nMode = 0733; // UNIX style permissions
        int nError = 0;
        for(unsigned int i=0;i<splitted.size();i++)
        {
            currentPath += splitted[i];
            if(!directoryExists(currentPath))
            {
#if defined(_WIN32)
                nError = _mkdir(currentPath.c_str());
#else
                nError = mkdir(currentPath.c_str(),nMode);
#endif
                if (nError != 0) return false;
            }
            currentPath += SEPARATOR;
        }
        return true;
    }

    // GET FILE SIZE
    //---------------------------------------------------------------------------------------
    int getFileSize(const std::string& filePath)
    {
        
        struct stat results;
        
        if (stat((const char*)filePath.c_str(), &results) == 0)
            return results.st_size;
        else
            return -1;
    }
    
    // CREATE PATH
    //---------------------------------------------------------------------------------------
    void File::createPath(const std::string& directory, const std::string& in_name, const std::string& extension)
    {
        if(endsWithString(in_name, extension)) path = directory+SEPARATOR+in_name;
        else path = directory+SEPARATOR+in_name+extension;
    }

    // OPEN FILE FOR READ OR WRITE
    //---------------------------------------------------------------------------------------
    bool File::open(FILE_MODE mode)
    {
        file = new std::fstream();
        if(mode == FILE_WRITE)
            file->open(path.c_str(), std::ios::out | std::ios::trunc);
        else if(mode == FILE_READ)
            file->open(path.c_str(), std::ios::in);
        if(file->is_open()) return true;
        else return false;
    }
    
    // CLOSE FILE
    //---------------------------------------------------------------------------------------
    bool File::close()
    {
        if(file->is_open()) file->close();
        return true;
    }
    
    // WRITE
    //---------------------------------------------------------------------------------------
    void File::write(const std::string& s)
    {
        *file << s << "\n";
    }
    
    // GET FILE LENGTH
    //---------------------------------------------------------------------------------------
    unsigned long File::getFileLength()
    {
        if(!file->good()) return 0;
        file->seekg(0,std::ios::end);
        unsigned long len = file->tellg();
        file->seekg(std::ios::beg);
        
        return len;
    }


    // READ
    //---------------------------------------------------------------------------------------
    std::string File::read()
    {
        if (file->is_open())
        {
            file->seekg(0);
            std::string line;
            while ( getline (*file,line) )
            {
                content+=line+"\n";
            }
            file->close();
        }
        return content;
    }
    
    std::string File::readAll()
    {
        file->seekg(0, std::ios::end);
        content.reserve(file->tellg());
        file->seekg(0, std::ios::beg);
        
        content.assign((std::istreambuf_iterator<char>(*file)),
                       std::istreambuf_iterator<char>());
        return content;
    }
}// end namespace BOB


