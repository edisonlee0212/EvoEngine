#if defined(_MSC_VER) && defined(EVOENGINE_RUNTIME_PACKAGES)
// CMake's WINDOWS_EXPORT_ALL_SYMBOLS can emit an undecorated "__" export
// when scanning MSVC PCH objects from linked static libraries.
extern "C" __declspec(dllexport) int __ = 0;
#endif
