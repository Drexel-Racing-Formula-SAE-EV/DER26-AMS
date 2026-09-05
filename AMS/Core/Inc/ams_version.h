/* One source of firmware release identity. Build commit remains independent. */
#ifndef AMS_VERSION_H_
#define AMS_VERSION_H_

#define AMS_VERSION_MAJOR 0
#define AMS_VERSION_MINOR 5
#define AMS_VERSION_PATCH 23
#define AMS_RELEASE_DATE "20260905"
#define AMS_VERSION_STRINGIFY_(value) #value
#define AMS_VERSION_STRINGIFY(value) AMS_VERSION_STRINGIFY_(value)
#define AMS_VERSION_STRING AMS_VERSION_STRINGIFY(AMS_VERSION_MAJOR) "." \
    AMS_VERSION_STRINGIFY(AMS_VERSION_MINOR) "." AMS_VERSION_STRINGIFY(AMS_VERSION_PATCH)
#ifndef AMS_SOURCE_REVISION
#define AMS_SOURCE_REVISION "DER26-AMS-v" AMS_VERSION_STRING "-" AMS_RELEASE_DATE
#endif

/* GCC honors SOURCE_DATE_EPOCH for these built-ins. IDE builds without it
 * deliberately retain their real compilation timestamp. Only app.c stores it;
 * every diagnostic consumer reads that same manifest instance. */
#ifndef AMS_BUILD_DATE
#define AMS_BUILD_DATE __DATE__
#endif
#ifndef AMS_BUILD_TIME
#define AMS_BUILD_TIME __TIME__
#endif

#endif
