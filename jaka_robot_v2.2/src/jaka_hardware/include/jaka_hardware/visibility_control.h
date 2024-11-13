#ifndef JAKA_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define JAKA_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define JAKA_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
#define JAKA_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define JAKA_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
#define JAKA_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef JAKA_HARDWARE_INTERFACE_BUILDING_DLL
#define JAKA_HARDWARE_INTERFACE_PUBLIC JAKA_HARDWARE_INTERFACE_EXPORT
#else
#define JAKA_HARDWARE_INTERFACE_PUBLIC JAKA_HARDWARE_INTERFACE_IMPORT
#endif
#define JAKA_HARDWARE_INTERFACE_PUBLIC_TYPE JAKA_HARDWARE_INTERFACE_PUBLIC
#define JAKA_HARDWARE_INTERFACE_LOCAL
#else
#define JAKA_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
#define JAKA_HARDWARE_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define JAKA_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define JAKA_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define JAKA_HARDWARE_INTERFACE_PUBLIC
#define JAKA_HARDWARE_INTERFACE_LOCAL
#endif
#define JAKA_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // JAKA_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
