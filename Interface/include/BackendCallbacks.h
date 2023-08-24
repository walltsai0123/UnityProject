#pragma once
#include <PluginAPI/IUnityInterface.h>
#include <sstream>

typedef void(UNITY_INTERFACE_API *StringCallback)(const char* message);

extern StringCallback DebugLog;
extern StringCallback DebugLogWarning;
extern StringCallback DebugLogError;

#define STR(message) static_cast<std::ostringstream &&>((std::ostringstream() << message)).str().data()
#ifndef NDEBUG

#define LOG(message) if(DebugLog) { DebugLog(STR(message)); }

#define LOGWARN(message) if(DebugLogWarning) { DebugLogWarning(STR(message)); }

#define LOGERR(message) if(DebugLogError) { DebugLogError(STR(message)); }

#else
#define LOG(m)
#define LOGWARN(m)
#define LOGERR(m)
#endif