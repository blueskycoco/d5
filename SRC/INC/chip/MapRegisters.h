

#ifndef __MAPREGISTERS_H__
#define __MAPREGISTERS_H__

#include <ceddk.h>

static __inline PVOID MapRegisters (DWORD dwPhysAddr,DWORD dwSize)
{
	register PHYSICAL_ADDRESS TempPhysAddr;
	TempPhysAddr.QuadPart = (UINT64) dwPhysAddr;
	return MmMapIoSpace(TempPhysAddr,dwSize,FALSE);
}
static __inline void UnMapRegisters (PVOID pReg,DWORD dwSize)
{
	MmUnmapIoSpace(pReg,dwSize);
}
#endif //__MAPREGISTERS_H__