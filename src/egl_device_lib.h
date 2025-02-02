//============================================================================
// EmberGL
//
// Copyright (c) 2022, Jarkko Lempiainen
// All rights reserved.
//============================================================================

#ifndef EGL_DEVICE_LIB_H
#define EGL_DEVICE_LIB_H
//----------------------------------------------------------------------------


//============================================================================
// interface
//============================================================================
// external
#if defined(ARDUINO_ARCH_RP2040)
#include "drivers/egl_device_ssd1283a.h"
#elif defined(ARDUINO)
#include "drivers/egl_device_ili9341.h"
#include "drivers/egl_device_ili9488.h"
#else
#include "egl_core.h"
#endif
EGL_NAMESPACE_BEGIN

// new
//----------------------------------------------------------------------------

//============================================================================
EGL_NAMESPACE_END
#endif
