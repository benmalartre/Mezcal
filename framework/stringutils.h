//--------------------------------------------------------
// STRING UTILS
//--------------------------------------------------------
#ifndef _STRING_UTILS_H_
#define _STRING_UTILS_H_


#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>
#include <map>
#include <limits.h>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdlib.h>
#include <stdio.h>

namespace BOB{
    
	std::string replaceString(const std::string& str, const std::string& from, const std::string& to);
	std::string frameToString(int frame, int padding=4);
	std::string getPaddingString(const std::string& path);
	int getPadding(const std::string& padding_string);
	bool isFixedFile(const std::string& padding_string);
	std::string getCurrentFile(const std::string& path, int frame);
    void tokenizeString(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters = " \t\n\r");
    
    bool endsWithString(std::string const & value, std::string const & ending);
    bool startsWithString(std::string const & value, std::string const & starting);
    int countString(const std::string & str, const std::string & find);
    std::vector<std::string> splitString(const std::string& s, const std::string& delim);
    std::string replaceString(std::string& str, const std::string& from, const std::string& to);
    std::string replaceAllString(std::string& str, const std::string& from, const std::string& to);
     
}
#endif /* _STRING_UTILS_H_ */
