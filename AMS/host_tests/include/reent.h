#ifndef AMS_HOST_REENT_SHIM_H
#define AMS_HOST_REENT_SHIM_H
/*
 * Host-test shim for FreeRTOS/Newlib builds.
 *
 * The STM32 ARM toolchain normally provides <reent.h>. Desktop GCC usually
 * does not. FreeRTOS portable headers only need the struct name for host-side
 * parsing here, so a dummy definition is enough for the test harness.
 */
struct _reent { int _ams_host_dummy; };
extern struct _reent *_impure_ptr;
#define _REENT_INIT_PTR(p) ((void)(p))
void _reclaim_reent(struct _reent *p);
#endif
