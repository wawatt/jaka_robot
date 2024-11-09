#ifndef JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION__VISIBILITY_CONTROL_H_
#define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_EXPORT __attribute__ ((dllexport))
    #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_IMPORT __attribute__ ((dllimport))
  #else
    #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_EXPORT __declspec(dllexport)
    #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_IMPORT __declspec(dllimport)
  #endif
  #ifdef JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_BUILDING_DLL
    #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_PUBLIC JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_EXPORT
  #else
    #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_PUBLIC JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_IMPORT
  #endif
  #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_PUBLIC_TYPE JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_PUBLIC
  #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_LOCAL
#else
  #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_EXPORT __attribute__ ((visibility("default")))
  #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_IMPORT
  #if __GNUC__ >= 4
    #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_PUBLIC __attribute__ ((visibility("default")))
    #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_PUBLIC
    #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_LOCAL
  #endif
  #define JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // JAKA_FOLLOW_JOINT_TRAJECTORY_ACTION__VISIBILITY_CONTROL_H_