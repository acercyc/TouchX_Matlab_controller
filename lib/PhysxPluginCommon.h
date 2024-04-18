//---------------------------------------------------------------------
//
// Copyright 2009, SensAble Technologies, Inc.
//
// File: PHYSXPLUGINCOMMON.h
//
// Author: Venkat Gourishankar	
//
// Created: 12/28/2009
//
// Description: The QtGlWidget class interfaces OpenGL with Qt windows 
// for rendering scenes in the uAPI
//
//---------------------------------------------------------------------
#ifndef PHYSXPLUGINCOMMON_H_
#define PHYSXPLUGINCOMMON_H_

#if defined(WIN32)
#define NOMINMAX
#endif

#ifdef WIN32
#include <windows.h>
#endif

#include "Globals.h"
#include "DeviceSpace.h"
#include "QHRenderer.h"

#include "NxPhysics.h"
#include "UpdateTime.h"
#include "NxVersionNumber.h"
#include "NxCooking.h"
#include "Stream.h"
#include "ContactReport.h"


#endif