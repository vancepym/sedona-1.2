//
// Copyright (c) 2012 Tridium, Inc.
// Licensed under the Academic Free License version 3.0
//
// History:
//   29 Mar 12  Elizabeth McKenney  Creation
//

#include "sedona.h"
#include "sedonaPlatform.h"

#include <windows.h>


// Str PlatformService.doPlatformId()
Cell sys_PlatformService_doPlatformId(SedonaVM* vm, Cell* params)
{         
  Cell result;
  result.aval = PLATFORM_ID;
  return result;
}


// Str PlatformService.getPlatVersion()
Cell sys_PlatformService_getPlatVersion(SedonaVM* vm, Cell* params)
{         
  Cell result;
  result.aval = PLAT_BUILD_VERSION;  
  return result;
}


// long PlatformService.getNativeMemAvailable()
int64_t sys_PlatformService_getNativeMemAvailable(SedonaVM* vm, Cell* params)
{
  int64_t totalmem;
  MEMORYSTATUSEX memstatus;
  GlobalMemoryStatusEx( &memstatus );

  // Use MB resolution but report bytes (reduce event freq)
  totalmem = (memstatus.ullAvailPhys + memstatus.ullAvailVirtual) >> 20;
  return totalmem << 20;
}

