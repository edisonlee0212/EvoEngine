#ifndef STRINGHELPER_H
#define STRINGHELPER_H
#include <string>
#include <vector>
#include <iterator>
#include <sstream>
//Borrowed from http://www.cplusplus.com/faq/sequences/strings/trim/
inline std::string trim_right_copy(
  const std::string& s,
  const std::string& delimiters = " \f\n\r\t\v" )
{
  return s.substr( 0, s.find_last_not_of( delimiters ) + 1 );
}

inline std::string trim_left_copy(
  const std::string& s,
  const std::string& delimiters = " \f\n\r\t\v" )
{
  return s.substr( s.find_first_not_of( delimiters ) );
}

inline std::string trim_copy(
  const std::string& s,
  const std::string& delimiters = " \f\n\r\t\v" )
{
  return trim_left_copy( trim_right_copy( s, delimiters ), delimiters );
}

typedef std::string::size_type (std::string::*find_t)(const std::string& delim,
                                                std::string::size_type offset) const;

std::vector<std::string> split(const std::string& s,
                         const std::string& match,
                         bool removeEmpty=false,
                         bool fullMatch=false);

//http://stackoverflow.com/a/13636164/195722
template <typename T>
  std::string ObjToString ( T obj )
  {
     std::ostringstream ss;
     ss << obj;
     return ss.str();
  }

#endif
