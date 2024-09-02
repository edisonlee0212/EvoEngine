#include <string>
#include <vector>
#include "stringhelper.h"

using namespace std;

vector<string> split(const string& s,
                         const string& match,
                         bool removeEmpty,
                         bool fullMatch)
{
        vector<string> result;                 // return container for tokens
        string::size_type start = 0,           // starting position for searches
                          skip = 1;            // positions to skip after a match
        find_t pfind = &string::find_first_of; // search algorithm for matches

        if (fullMatch)
        {
            // use the whole match string as a key
            // instead of individual characters
            // skip might be 0. see search loop comments
            skip = match.length();
            pfind = &string::find;
        }

        while (start != string::npos)
        {
            // get a complete range [start..end)
            string::size_type end = (s.*pfind)(match, start);

            // null strings always match in string::find, but
            // a skip of 0 causes infinite loops. pretend that
            // no tokens were found and extract the whole string
            if (skip == 0) end = string::npos;

            string token = s.substr(start, end - start);

            if (!(removeEmpty && token.empty()))
            {
                // extract the token and add it to the result list
                result.push_back(token);
            }

            // start the next range
            if ((start = end) != string::npos) start += skip;
        }

        return result;
}
