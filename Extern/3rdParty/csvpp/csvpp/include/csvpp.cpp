#include <string>
#include <iostream>
#include <istream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include "csvpp.h"
#include "stringhelper.h"

using namespace std;
using namespace csvpp;

namespace csvpp {

	std::ostream & operator<<(std::ostream & os, const RowWriter & r)
	{
		
		rowiterator it;
		if (r.size() == 0)
			return os;

		if (!r[0].skipheader)
		{
			for(unsigned int i = 0; i < r[0].header.size() - 1; i++)
			{
				os << r[0].header[i] << r[0].delimiter_char;;
			}
			os << r[0].header[r[0].header.size() - 1] << r[0].newline;
		}
		for(unsigned int i = 0; i < r.size(); i++)
		{
			for(it = r[i].begin(); it != r[i].end(); it++)
			{
				if (distance(r[i].begin(), it) != (int)(r[i].size() - 1))
					os << it->second << r[i].delimiter_char;
				else
					os << it->second;
			}
			if (!r[0].skipheader && i != 0)
				os << r[i].newline;
		}

		return os;
	}

	std::istream & operator>>(std::istream & is, RowReader & r) 
	{ 
		string buffer;
		stringstream buffer2;
		int currentheader = 0;
		getline(is, buffer);
		// Patch by damienlmoore - https://code.google.com/p/csvpp/issues/detail?id=1
		if(!is.good() || is.eof())
		{
			return is;
		}
		
		buffer = trim_copy(buffer);
		char c;
		bool startquote = false;
		if(r.header.size() == 0 && !r.skipheader)
		{
			
			vector<string> sections = split(buffer, r.delimiter_char);
			for(unsigned int i = 0; i < sections.size(); i++)
				r.header.push_back(sections[i]);
		} else {
			for(unsigned int i = 0; i < buffer.length(); i++)
			{
				c = buffer[i];
				/*
					If the current character is a comma then we may have found the start of the next column
					however we do need to test if we are inside of a quote

					If we aren't inside of a quote - store the value using the current header 'pointer' and keep scanning
				*/
				if (c == r.delimiter_char[0])
				{
					if (startquote)
					{
						buffer2 << c;
						continue;
					}
					if (!r.skipheader)
					{
						r[r.header[currentheader]] = buffer2.str();
					}
					else
					{
						r[ObjToString(currentheader)] = buffer2.str();
					}
					buffer2.str(string());
					currentheader++;
					continue;
				}


				// If the character is a quote then we need to note this and use that to ignore commas
				// added logic to ignore whitespace before and after the whitespace
				if (c == '"')
				{
					if (startquote)
					{
						buffer2 << '"';
						buffer2.str(trim_left_copy(buffer2.str()));
					}
					if ( (((int)i-1) >= 0 && buffer[i-1] == '\\'))
					{
						buffer2 << c;
						continue;
					}
					startquote = !startquote;
					//find , and move i to it
					if (!startquote)
					{
						for(unsigned int x = i; x < buffer.length(); x++)
						{
							if (buffer[x] == r.delimiter_char[0] || x == buffer.length())
							{
								i = x-1;
								break;
							}
						}
					}
				}

				buffer2 << c;
			}
			if (!r.skipheader)
			{
				r[r.header[currentheader]] = buffer2.str();
			} else {
				r[ObjToString(currentheader)] = buffer2.str();
			}
		}
		
		return is; 
	}
}
