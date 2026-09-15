#include "PackageManager.hpp"

EVOENGINE_PACKAGE_EXPORT const void* EVOENGINE_RUNTIME_IDENTITY_FUNCTION() {
  static const char identity = 0;
  return &identity;
}
