// Intentionally no #pragma once: include this after any platform header that may pull in X11/X.h.
// X11 exposes common words as preprocessor macros, which can corrupt unrelated C++ APIs.

#if defined(__linux__) || defined(__unix__) || defined(EVOENGINE_LINUX)
#  ifdef Success
#    undef Success
#  endif
#  ifdef Status
#    undef Status
#  endif
#  ifdef None
#    undef None
#  endif
#  ifdef Bool
#    undef Bool
#  endif
#  ifdef True
#    undef True
#  endif
#  ifdef False
#    undef False
#  endif
#  ifdef Always
#    undef Always
#  endif
#  ifdef Above
#    undef Above
#  endif
#  ifdef Below
#    undef Below
#  endif
#endif
