#pragma once

#if defined(_WIN32) && defined(EVOENGINE_RUNTIME_PACKAGES)
#  if defined(EVOENGINE_EDITOR_SDK_EXPORTS)
#    define EVOENGINE_EDITOR_API __declspec(dllexport)
#  else
#    define EVOENGINE_EDITOR_API __declspec(dllimport)
#  endif
#else
#  define EVOENGINE_EDITOR_API
#endif
