#ifndef CSVPP_H
#define CSVPP_H

#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <istream>


#define VERSION "2.2"

namespace csvpp {

	class RowWriter;

	class RowReader : public std::map<std::string, std::string> {
		private:
			std::vector<std::string> header;
			bool skipheader;
			std::string delimiter_char; // this is a string because the split function helper is expecting a string, but really this is just a char
		public:
			const char * newline;
			// Adding support for custom delimiter character
			// Based on the patch by Hanifa
			// https://code.google.com/p/csvpp/issues/detail?id=2
			RowReader(std::string delimiter_char = ",", bool skipheader=false,const char * newline="\n") : delimiter_char(delimiter_char), skipheader(skipheader), newline(newline) { }
			void clear() { header.clear(); }
			friend std::istream & operator>>(std::istream & os, RowReader & r);
			friend std::ostream & operator<<(std::ostream & os, const RowWriter & r);
	};

	class RowWriter : public std::vector<RowReader>
	{
		public:
			friend std::ostream & operator<<(std::ostream & os, const RowWriter & r);
	};
	
	typedef RowReader::const_iterator rowiterator;
}

#endif