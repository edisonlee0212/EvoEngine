//
//  base64 encoding and decoding with C++.
//  Version: 2.rc.04 (release candidate)
//

#ifndef BASE64_H_C0CE2A47_D10E_42C9_A27C_C883944E704A
#define BASE64_H_C0CE2A47_D10E_42C9_A27C_C883944E704A

#include <string>
#include <vector>

namespace base64
{

#if __cplusplus >= 201703L
#include <string_view>
#endif  // __cplusplus >= 201703L

std::string base64_encode(const std::string &s, bool url = false);
std::string base64_encode(const std::vector<uint8_t> &s, bool url = false);

std::string base64_encode_pem(const std::string &s);
std::string base64_encode_pem(const std::vector<uint8_t> &s);

std::string base64_encode_mime(const std::string &s);
std::string base64_encode_mime(const std::vector<uint8_t> &s);

std::string base64_decode(const std::string &s, bool remove_linebreaks = false);
std::string base64_decode(const std::vector<uint8_t> &s, bool remove_linebreaks = false);

std::string base64_encode(const unsigned char *s, size_t len, bool url = false);

#if __cplusplus >= 201703L
//
// Interface with std::string_view rather than const std::string&
// Requires C++17
// Provided by Yannic Bonenberger (https://github.com/Yannic)
//
std::string base64_encode     (std::string_view s, bool url = false);
std::string base64_encode_pem (std::string_view s);
std::string base64_encode_mime(std::string_view s);

std::string base64_decode(std::string_view s, bool remove_linebreaks = false);
#endif  // __cplusplus >= 201703L

} // namespace base64

#endif /* BASE64_H_C0CE2A47_D10E_42C9_A27C_C883944E704A */
