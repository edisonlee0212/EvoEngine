/**
 * @author Tomas Polasek
 * @date 11.20.2019
 * @version 1.0
 * @brief Utilities and statistics for the treeio::Tree class.
 */

#include "TreeIOUtils.hpp"

#include <fstream>
#include <streambuf>
#include <string>

#include <base64.h>

namespace treeutil {

std::array<float, 3u> BoundingBox::center() const {
  return {
      position[0u] + size[0u] / 2.0f,
      position[1u] + size[1u] / 2.0f,
      position[2u] + size[2u] / 2.0f,
  };
}

float BoundingBox::diameter() const {
  // Calculate diameter of the bounding box = sqrt(width^2 + height^2 + depth^2).
  return std::sqrt(size[0u] * size[0u] + size[1u] * size[1u] + size[2u] * size[2u]);
}

ProgressBar::ProgressBar(const std::string &text, std::size_t width, bool rewrite)
    : mText{text}, mWidth{width}, mRewrite{rewrite} {
}

std::string ProgressBar::progress(float progress) {
  const auto progressBarString{
      mText + progressString(mWidth, progress, EMPTY_SYMBOL, FULL_SYMBOL, LEFT_BORDER_SYMBOL, RIGHT_BORDER_SYMBOL)};
  const auto reversion{std::string(progressBarString.size() * mRewrite * mProgressGenerated, '\b')};

  mProgressGenerated = true;

  return reversion + progressBarString;
}

std::string ProgressBar::emptyProgressString(std::size_t width, unsigned char symbol) {
  return std::string(width, symbol);
}

std::string ProgressBar::progressString(std::size_t width, float progress, unsigned char emptySymbol,
                                        unsigned char fullSymbol, unsigned char leftBorderSymbol,
                                        unsigned char rightBorderSymbol) {
  // Subtract 2 characters for the borders.
  const auto progressWidth{width - 2u};
  const auto emptyProgress{emptyProgressString(progressWidth, emptySymbol)};

  // Generate filled progress bar.
  const auto filledCharacters{static_cast<std::size_t>(progressWidth * progress)};
  auto filledProgress{emptyProgress};
  filledProgress.replace(0u, filledCharacters, filledCharacters, fullSymbol);

  // Add borders.
  std::stringstream ss{};
  ss << leftBorderSymbol << filledProgress << rightBorderSymbol;

  return ss.str();
}

std::string strToLower(const std::string &str) {
  auto copy{str};
  std::transform(copy.begin(), copy.end(), copy.begin(), [](const auto c) {
    return std::tolower(c);
  });
  return copy;
}

bool equalCaseInsensitive(const std::string &first, const std::string &second) {
  if (first.size() != second.size()) {
    return false;
  }

  for (std::size_t iii = 0u; iii < first.size(); ++iii) {
    if (std::tolower(first[iii]) != std::tolower(second[iii])) {
      return false;
    }
  }

  return true;
}

std::string capPath(const std::string &filePath) {
  if (filePath.size() < 0)
    return "";
  auto last = filePath.c_str()[filePath.size() - 1];
  if (last != '\\' && last != '/') {
    auto newFile = std::string(filePath);
    return newFile.append("/");
  } else
    return filePath;
}

std::string fileExtension(const std::string &filePath) {
  return std::filesystem::path(filePath).extension().string();
}

std::string filePath(const std::string &filePath) {
  return std::filesystem::path(filePath).parent_path().string();
}

std::string fileBaseName(const std::string &filePath) {
  return std::filesystem::path(filePath).stem().string();
}

std::vector<std::string> listFiles(const std::string &extension, const std::string &path, bool recursive,
                                   bool relative) {
  std::vector<std::string> files{};

  auto searchPath{path};
  if (path.empty()) {
    searchPath = std::filesystem::current_path().string();
  }

  std::stack<std::filesystem::path> paths{};
  std::set<std::filesystem::path> processed{};
  paths.push({searchPath});
  while (!paths.empty()) {  // Process paths recursively, if requested.
    const auto currentPath{paths.top()};
    paths.pop();
    // Stop loops, processing each path only once.
    if (processed.find(currentPath) != processed.end()) {
      continue;
    }

    for (const auto &entry :
         std::filesystem::directory_iterator(currentPath)) {  // Process all files in the current directory.
      // Skip directories, adding them to the stack if recursive processing is requested.
      if (std::filesystem::is_directory(entry)) {
        if (recursive) {
          paths.push(entry);
        }
        continue;
      }

      auto ext{entry.path().extension().string()};
      std::transform(ext.begin(), ext.end(), ext.begin(), [](auto c) {
        return std::tolower(c);
      });
      if (extension.empty() || !ext.compare(extension)) {
        const auto pathStr{relative ?
                                    // Skip the common part of the path.
                               entry.path().string().substr(searchPath.size() /*+ 1u*/)
                                    : entry.path().string()};
        files.push_back(pathStr);
      }
    }

    processed.emplace(currentPath);
  }

  return files;
}

std::vector<std::string> listFiles(const std::vector<std::string> &extensions, const std::string &path, bool recursive,
                                   bool relative) {
  std::vector<std::string> files{};

  for (const auto &extension : extensions) {
    const auto newFiles{listFiles(extension, path, recursive, relative)};
    files.insert(files.end(), newFiles.begin(), newFiles.end());
  }

  if (extensions.empty()) {
    files = listFiles("", path, recursive);
  }

  return files;
}

bool fileExists(const std::string &path) {
  return std::filesystem::exists(path);
}

bool deleteFile(const std::string &path) {
  return std::filesystem::remove(path);
}

std::string readWholeFile(const std::string &path) {
  std::string result{};
  std::ifstream ifs(path, std::ios::in | std::ios::binary | std::ios::ate);

  if (ifs.is_open() || true) {
    const auto filesize{static_cast<std::size_t>(ifs.tellg())};
    ifs.seekg(0, std::ios::beg);
    result.resize(filesize);
    ifs.read(result.data(), filesize);
  } else {
    std::cerr << "Failed to read file from \"" << path << "\"!" << std::endl;
  }

  return result;
}

std::string replaceExtension(const std::string &path, const std::string &extension) {
  const auto oldExtension{fileExtension(path)};
  return path.substr(0, path.size() - oldExtension.size()) + extension;
}

std::string relativePath(const std::string &path, const std::string &relativePath) {
  const auto findIt{path.find(relativePath.empty() ? std::filesystem::current_path().string() : relativePath)};
  return findIt != std::string::npos ? path.substr(findIt + relativePath.size() + 1u) : path;
}

bool containsOnlyWhiteSpaces(const std::string &str) {
  for (auto it = str.begin(); it != str.end(); ++it) {
    if (!isspace(*it)) {
      return false;
    }
  }
  return true;
}

std::string encodeBinaryJSON(const std::vector<uint8_t> &data) {
  return base64::base64_encode_pem(data);
}

}  // namespace treeutil
