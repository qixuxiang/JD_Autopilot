#pragma  once

#if defined _WIN32 || defined __CYGWIN__
#ifdef MAPDATAENGINE_EXPORTS
#ifdef __GNUC__
#define MAPDATAENGINE_API __attribute__ ((dllexport))
#else
#define MAPDATAENGINE_API __declspec(dllexport) // Note: actually gcc seems to also supports this syntax.
#endif
#else //MAPDATAENGINE_EXPORTS
#ifdef __GNUC__
#define MAPDATAENGINE_API __attribute__ ((dllimport))
#else
#define MAPDATAENGINE_API __declspec(dllimport) // Note: actually gcc seems to also supports this syntax.
#endif
#endif //else
#define DLL_LOCAL
#else
#if __GNUC__ >= 4
#define MAPDATAENGINE_API __attribute__ ((visibility ("default")))
#define DLL_LOCAL  __attribute__ ((visibility ("hidden")))
#else
#define MAPDATAENGINE_API  "do not suport"
#define DLL_LOCAL "do not suport"
#endif
#endif