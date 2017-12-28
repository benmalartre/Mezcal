//--------------------------------------------------------
// STRING UTILS IMPLEMENTATION
//--------------------------------------------------------
#include "stringutils.h"

using namespace std;
namespace BOB{
    // TOKENIZE STRING
    //---------------------------------------------------------------------------------------
    void tokenizeString(const std::string& str,
                     std::vector<std::string>& tokens,
                     const std::string& delimiters)
    {
        tokens.clear();
        
        // Skip delimiters at beginning.
        size_t lastPos = str.find_first_not_of(delimiters, 0);
        // Find first "non-delimiter".
        size_t pos     = str.find_first_of(delimiters, lastPos);
        
        while(std::string::npos != pos || std::string::npos != lastPos)
        {
            // Found a token, add it to the vector.
            tokens.push_back(str.substr(lastPos, pos - lastPos));
            // Skip delimiters.  Note the "not_of"
            lastPos = str.find_first_not_of(delimiters, pos);
            // Find next "non-delimiter"
            pos = str.find_first_of(delimiters, lastPos);
        }
    }

    // REPLACE STRING
    //---------------------------------------------------------------------------------------
    std::string replaceString(const std::string& str, const std::string& from, const std::string& to)
    {
        string rs(str);
        size_t start_pos = rs.find(from);
        if(start_pos == std::string::npos)
            return rs;
        rs.replace(start_pos, from.length(), to);
        return rs;
    }

    // FRAME TO STRING (INT TO STRING WITH PADDING)
    //---------------------------------------------------------------------------------------
    std::string frameToString(int frame, int padding)
    {
        std::stringstream ss;
        ss << std::setw(padding) << std::setfill('0') << frame;
        std::string fs = ss.str();
        return fs;
    }
    
    // GET PADDING STRING
    //---------------------------------------------------------------------------------------
    std::string getPaddingString(const std::string& path)
    {
        string tmp(path);
        size_t start = tmp.find(".");
        if(start!=std::string::npos)
            tmp.erase(0, start+1);
        size_t end = tmp.find(".");
        if(end!=std::string::npos)
            tmp.erase(end, tmp.length());
        return tmp;
    }
    
    // GET PADDING (STRING TO INT)
    //---------------------------------------------------------------------------------------
    int getPadding(const std::string& padding_string)
    {
        char nb = padding_string[2];
        return  atoi(&nb);
    }
    
    // IS FIXED FILE
    //---------------------------------------------------------------------------------------
    bool isFixedFile(const std::string& padding_string)
    {
        if(padding_string.find("%") != std::string::npos)
            return false;
        return true;
    }

    // GET CURRENT FILE FOR FRAME
    //---------------------------------------------------------------------------------------
    std::string getCurrentCacheFile(const std::string& path, int frame)
    {
        std::string padding_string = getPaddingString(path);
        bool fixed = isFixedFile(padding_string);
        size_t padding = padding_string.length();
        if(!fixed)
            padding = getPadding(padding_string);
        
        std::string fs = frameToString(frame, padding);
        return replaceString(path, padding_string, fs);
    }


    // NEXT WORD IN STRING
    //---------------------------------------------------------------------------------------
    std::string nextWordInString(std::string &in, size_t start=0, size_t *pEnd=NULL)
    {
        std::string ret;
        size_t len = in.length();
        if(len > start)
        {
            while(start<len && (in[start]==' ' || in[start]=='\t'))
                start++;
            if(start!=len)
            {
                size_t end = start;
                while(end<len && !(in[end]==' ' || in[end]=='\t'))
                    end++;
                ret = in.substr(start, (end-start));
                if(pEnd)
                    *pEnd = end;
            }
        }
        return ret;
    }
    
    
    // STRING END WITH
    //---------------------------------------------------------------------------------------
    bool endsWithString(std::string const & value, std::string const & ending)
    {
        if (ending.size() > value.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    }
    
    // STRING START WITH
    //---------------------------------------------------------------------------------------
    bool startsWithString(std::string const & value, std::string const & starting)
    {
        if (starting.size() > value.size()) return false;
        return std::equal(starting.begin(), starting.end(), value.begin());
    }
    
    // SPLIT STRING
    //---------------------------------------------------------------------------------------
    std::vector<std::string> splitString(const std::string& s, const std::string& delim)
    {
        std::vector<std::string> tokens;
        std::size_t start = s.find_first_not_of(delim), end = 0;
        
        while((end = s.find_first_of(delim, start)) != std::string::npos)
        {
            tokens.push_back(s.substr(start, end - start));
            start = s.find_first_not_of(delim, end);
        }
        if(start != std::string::npos)
            tokens.push_back(s.substr(start));
        
        return tokens;
    }
    
    // COUNT STRING
    //---------------------------------------------------------------------------------------
    int countString(const std::string & str, const std::string & find)
    {
        int count = 0;
        size_t nPos = str.find(find, 0); // fist occurrence
        while(nPos != std::string::npos)
        {
            count++;
            nPos = str.find(find, nPos+1);
        }
        return count;
    }
    
    // REPLACE STRING
    //---------------------------------------------------------------------------------------
    std::string replaceString(std::string& str, const std::string& from, const std::string& to)
    {
        size_t start_pos = str.find(from);
        
        if(start_pos == std::string::npos)
            return str;
        std::string out_str = str;
        out_str.replace(start_pos, from.length(), to);
        return out_str;
    }
    
    // REPLACE ALL STRING
    //---------------------------------------------------------------------------------------
    std::string replaceAllString(std::string& str, const std::string& from, const std::string& to)
    {
        size_t start_pos;
        bool search = true;
        std::string out_str = str;
        
        while(search == true)
        {
            start_pos = out_str.find(from);
            if(start_pos != std::string::npos)
                out_str.replace(start_pos, from.length(), to);
            else
                search = false;
        }
        return out_str;
    }
    
    /*
     // RANDOM STRING
     //---------------------------------------------------------------------------------------
     std::string randomString(size_t length)
     {
     const char* charmap = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
     const size_t charmapLength = strlen(charmap);
     std::string result;
     auto generator = [&](){ return charmap[rand()%charmapLength]; };
     string result;
     result.reserve(length);
     std::generate_n(std::back_inserter(result), length, generator);
     return result;
     }
     */
}

