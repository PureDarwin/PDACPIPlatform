/*
*
* Copyright (c) 2007-Present The PureDarwin Project.
* All rights reserved.
*
* @PUREDARWIN_LICENSE_HEADER_START@
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
* IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
* PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
* PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* @PUREDARWIN_LICENSE_HEADER_END@
*
* PDACPIPlatform Open Source Version of Apple's AppleACPIPlatform
* Created by github.com/csekel (InSaneDarwin)
*
* This specific file was created by Zormeister
*/

/* These are our functions so we can hook into IOKit properly. */

#include "acpica/acpi.h"
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IORegistryEntry.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOCommand.h>
#include <IOKit/IOCommandGate.h>
#include <libkern/c++/OSSet.h>
#include <libkern/c++/OSCollectionIterator.h>
#include <pexpert/i386/efi.h>
#include <pexpert/i386/boot.h>

/* for some reason Xcode has disabled any and all forms of auto completion. */
/* ZORMEISTER: Consider using CLion or VS Code with C++ extensions for better completion */

extern "C" void *AcpiOsExtMapMemory(ACPI_PHYSICAL_ADDRESS, ACPI_SIZE);
extern "C" void AcpiOsExtUnmapMemory(void *, ACPI_SIZE);  /* Fixed: Added missing size parameter */
extern "C" ACPI_STATUS AcpiOsExtInitialize(void);
extern "C" ACPI_PHYSICAL_ADDRESS AcpiOsExtGetRootPointer(void);
extern "C" ACPI_STATUS AcpiOsExtExecute(ACPI_EXECUTE_TYPE Type, ACPI_OSD_EXEC_CALLBACK Function, void *Context);
extern "C" void AcpiOsExtWaitEventsComplete(void);

IOWorkLoop *gAcpiOsThreadWorkLoop;
IOCommandGate *gAcpiOsThreadCommandGate;

/* PCI config space stuff. */
ACPI_MCFG_ALLOCATION gPCIFromPE;
ACPI_MCFG_ALLOCATION *gPCIDataFromMCFG;
size_t gPCIMCFGEntryCount;

/* Enhanced PCI access state */
static boolean_t gPCIEcamAvailable = false;
static IOMemoryMap *gPCIEcamMap = NULL;
static void *gPCIEcamVirtualBase = NULL;

IOLock *gAcpiOsExtMemoryMapLock;
OSSet *gAcpiOsExtMemoryMapSet;
OSCollectionIterator *gAcpiOsExtMemoryMapIterator;

/* Enhanced execution tracking */
static UInt32 gPendingExecutions = 0;
static IOLock *gExecutionLock;

struct _iocmdq_callback_data
{
    ACPI_OSD_EXEC_CALLBACK Callback;
    void *Context;
};

IOReturn AcpiOsThreadDispatch(OSObject *, void *field0, void *, void *, void *)
{
    _iocmdq_callback_data *d = (_iocmdq_callback_data *)field0;
    
    ml_set_interrupts_enabled(false); /* disable interrupts */
    d->Callback(d->Context);
    ml_set_interrupts_enabled(true); /* enable interrupts */

    /* Update pending execution count */
    IOLockLock(gExecutionLock);
    if (gPendingExecutions > 0) {
        gPendingExecutions--;
    }
    IOLockUnlock(gExecutionLock);

    /* we no longer have need for our callback data, and we have exited the cautious period without interrupts. */
    IOFree(d, sizeof(_iocmdq_callback_data));
    return kIOReturnSuccess;
}

/*
 * Initialize ECAM/MMIO PCI Configuration Space Access
 * This sets up memory mapping for the PCI configuration space base
 * obtained from boot arguments or MCFG table.
 */
static ACPI_STATUS InitializePCIEcam(void)
{
    if (gPCIEcamAvailable) {
        return AE_OK;
    }
    
    /* Use boot args data if available */
    if (gPCIFromPE.Address != 0) {
        ACPI_SIZE ecam_size = (gPCIFromPE.EndBusNumber - gPCIFromPE.StartBusNumber + 1) * 256 * 4096;
        
        IOMemoryDescriptor *desc = IOMemoryDescriptor::withAddressRange(
            gPCIFromPE.Address, ecam_size, 
            kIOMemoryDirectionInOut | kIOMemoryMapperNone, 
            kernel_task
        );
        
        if (desc) {
            gPCIEcamMap = desc->map();
            if (gPCIEcamMap) {
                gPCIEcamVirtualBase = (void *)gPCIEcamMap->getVirtualAddress();
                gPCIEcamAvailable = true;
                
                IOLog("ACPI: ECAM initialized at 0x%llx, buses %d-%d\n", 
                      gPCIFromPE.Address, gPCIFromPE.StartBusNumber, gPCIFromPE.EndBusNumber);
            }
            desc->release();
        }
    }
    
    return gPCIEcamAvailable ? AE_OK : AE_NOT_FOUND;
}

ACPI_STATUS AcpiOsExtInitialize(void)
{
    /* Initialize local resources. */
    gAcpiOsExtMemoryMapLock = IOLockAlloc();
    gAcpiOsExtMemoryMapSet = OSSet::withCapacity(4); /* OSSet's can expand if need be, right? */
    gAcpiOsExtMemoryMapIterator = OSCollectionIterator::withCollection(gAcpiOsExtMemoryMapSet);

    /* Initialize execution tracking */
    gExecutionLock = IOLockAlloc();
    gPendingExecutions = 0;

    /* init the execution system */
    gAcpiOsThreadWorkLoop = IOWorkLoop::workLoop();
    gAcpiOsThreadCommandGate = IOCommandGate::commandGate(NULL);
    gAcpiOsThreadCommandGate->setWorkLoop(gAcpiOsThreadWorkLoop);
    
    /* Fetch MCFG data from PE boot args, at least until PlatformExpert updates the data. */
    boot_args *args = (boot_args *)PE_state.bootArgs;
    gPCIFromPE.Address = args->pciConfigSpaceBaseAddress;
    gPCIFromPE.PciSegment = 0;
    gPCIFromPE.StartBusNumber = args->pciConfigSpaceStartBusNumber;
    gPCIFromPE.EndBusNumber = args->pciConfigSpaceEndBusNumber;
    
    /* Initialize ECAM if available */
    InitializePCIEcam();
    
    return AE_OK;
}

void *AcpiOsExtMapMemory(ACPI_PHYSICAL_ADDRESS addr, ACPI_SIZE size)
{
    IOMemoryDescriptor *desc = IOMemoryDescriptor::withAddressRange(addr, size, kIOMemoryDirectionInOut | kIOMemoryMapperNone, kernel_task);
    if (desc) {
        IOMemoryMap *map = desc->map();
        if (map) {
            IOVirtualAddress va = map->getVirtualAddress();
            IOLockLock(gAcpiOsExtMemoryMapLock);
            gAcpiOsExtMemoryMapSet->setObject(map);
            IOLockUnlock(gAcpiOsExtMemoryMapLock);
            /* Don't release the map here - we need it for unmapping */
            desc->release();
            return (void *)va;
        }
        desc->release();
    }
    return NULL;
}

void AcpiOsExtUnmapMemory(void *p, ACPI_SIZE size)
{
    IOVirtualAddress va = (IOVirtualAddress)p;

    IOLockLock(gAcpiOsExtMemoryMapLock);
    gAcpiOsExtMemoryMapIterator->reset();
    while (IOMemoryMap *map = OSDynamicCast(IOMemoryMap, gAcpiOsExtMemoryMapIterator->getNextObject())) {
        if (map->getVirtualAddress() == va && map->getLength() == size) {
            gAcpiOsExtMemoryMapSet->removeObject(map);
            map->release(); /* Now we can release it */
            break;
        }
    }
    IOLockUnlock(gAcpiOsExtMemoryMapLock);
}


/* Locate the EFI configuration table */
ACPI_PHYSICAL_ADDRESS AcpiOsExtGetRootPointer(void)
{
    EFI_PHYSICAL_ADDRESS tableAddr = 0;
#if __LP64__
    EFI_CONFIGURATION_TABLE_64 tbl;
#else
    EFI_CONFIGURATION_TABLE_32 tbl;
#endif
    
    /* These are populated at boot time by the UEFI bootloader (boot.efi to be specific) */
    static const char *table_paths[] = {
        "/efi/configuration-table/8868E871-E4F1-11D3-BC22-0080C73C8881", /* ACPI_20 */
        "/efi/configuration-table/EB9D2D30-2D88-11D3-9A16-0090273FC14D"  /* ACPI */
    };
    
    for (int i = 0; i < 2; i++) {
        IORegistryEntry *reg = IORegistryEntry::fromPath(table_paths[i], gIODTPlane);

        if (reg) {
            /* The kernel interprets this field as a data entry, it's the physical address to the EFI config table */
            /* I referenced AppleSMBIOS for this. */
            OSData *d = OSDynamicCast(OSData, reg->getProperty("table"));
            if (d && (d->getLength() <= sizeof(tableAddr))) {
                bcopy(d->getBytesNoCopy(), &tableAddr, d->getLength());
                IOMemoryDescriptor *desc = IOMemoryDescriptor::withAddressRange(tableAddr, sizeof(tbl), kIODirectionOutIn | kIOMemoryMapperNone, kernel_task);
                if (desc) {
                    bzero(&tbl, sizeof(tbl));
                    desc->readBytes(0, &tbl, sizeof(tbl));
                    desc->release();
                    reg->release();
                    return (ACPI_PHYSICAL_ADDRESS)tbl.VendorTable;
                }
            }
            reg->release();
        }
    }
    AcpiOsPrintf("ACPI: No RSDP found.\n");
    return 0;
}

ACPI_STATUS AcpiOsExtExecute(ACPI_EXECUTE_TYPE Type, ACPI_OSD_EXEC_CALLBACK Function, void *Context)
{
    _iocmdq_callback_data *d = (_iocmdq_callback_data *)IOMalloc(sizeof(_iocmdq_callback_data));
    if (!d) {
        return AE_NO_MEMORY;
    }
    
    d->Callback = Function;
    d->Context = Context;
    
    /* Track pending executions */
    IOLockLock(gExecutionLock);
    gPendingExecutions++;
    IOLockUnlock(gExecutionLock);
    
    IOReturn result = gAcpiOsThreadCommandGate->runAction(&AcpiOsThreadDispatch, d);
    
    /* If execution failed, decrement counter */
    if (result != kIOReturnSuccess) {
        IOLockLock(gExecutionLock);
        if (gPendingExecutions > 0) {
            gPendingExecutions--;
        }
        IOLockUnlock(gExecutionLock);
        IOFree(d, sizeof(_iocmdq_callback_data));
        return AE_ERROR;
    }
    
    return AE_OK;
}

/*
 * Wait for all pending ACPI executions to complete
 * This addresses the previous comment: "How do I check that my IOCommandGate has finished all of it's runAction calls?"
 */
void AcpiOsExtWaitEventsComplete(void)
{
    const int max_wait_ms = 5000; /* Maximum wait time: 5 seconds */
    const int poll_interval_ms = 10; /* Poll every 10ms */
    int waited_ms = 0;
    
    /* Poll until all executions complete or timeout */
    while (waited_ms < max_wait_ms) {
        IOLockLock(gExecutionLock);
        UInt32 pending = gPendingExecutions;
        IOLockUnlock(gExecutionLock);
        
        if (pending == 0) {
            break; /* All executions completed */
        }
        
        IOSleep(poll_interval_ms);
        waited_ms += poll_interval_ms;
    }
    
    if (waited_ms >= max_wait_ms) {
        IOLockLock(gExecutionLock);
        UInt32 remaining = gPendingExecutions;
        IOLockUnlock(gExecutionLock);
        
        IOLog("ACPI: Warning - %u executions still pending after %d ms timeout\n", 
              remaining, max_wait_ms);
    }
}

/*
 * Enhanced PCI Configuration Space Access using ECAM/MMIO
 * This implements the missing AcpiOsReadPCIConfigSpace function
 */
ACPI_STATUS AcpiOsReadPCIConfigSpace(ACPI_PCI_ID *PciId, UInt32 Reg, UInt64 *Value, UInt32 Width) 
{
    if (!PciId || !Value) {
        return AE_BAD_PARAMETER;
    }
    
    if (Width != 8 && Width != 16 && Width != 32) {
        return AE_BAD_PARAMETER;
    }
    
    /* Try ECAM/MMIO first if available */
    if (gPCIEcamAvailable && gPCIEcamVirtualBase &&
        PciId->Bus >= gPCIFromPE.StartBusNumber && 
        PciId->Bus <= gPCIFromPE.EndBusNumber) {
        
        /* Calculate ECAM offset: (Bus << 20) + (Device << 15) + (Function << 12) + Register */
        UInt32 bus_offset = (PciId->Bus - gPCIFromPE.StartBusNumber) << 20;
        UInt32 device_offset = PciId->Device << 15;
        UInt32 function_offset = PciId->Function << 12;
        UInt32 total_offset = bus_offset + device_offset + function_offset + Reg;
        
        void *config_addr = (char *)gPCIEcamVirtualBase + total_offset;
        
        switch (Width) {
            case 8:
                *Value = *(UInt8 *)config_addr;
                break;
            case 16:
                *Value = *(UInt16 *)config_addr;
                break;
            case 32:
                *Value = *(UInt32 *)config_addr;
                break;
        }
        
        return AE_OK;
    }
    
    /* Fallback: This should call the legacy port I/O method */
    /* For now, return error - the caller should use AcpiOsReadPciConfiguration from osdarwin.c */
    return AE_NOT_IMPLEMENTED;
}

/*
 * Write PCI Configuration Space using ECAM/MMIO
 * Companion function to AcpiOsReadPCIConfigSpace
 */
ACPI_STATUS AcpiOsWritePCIConfigSpace(ACPI_PCI_ID *PciId, UInt32 Reg, UInt64 Value, UInt32 Width)
{
    if (!PciId) {
        return AE_BAD_PARAMETER;
    }
    
    if (Width != 8 && Width != 16 && Width != 32) {
        return AE_BAD_PARAMETER;
    }
    
    /* Try ECAM/MMIO first if available */
    if (gPCIEcamAvailable && gPCIEcamVirtualBase &&
        PciId->Bus >= gPCIFromPE.StartBusNumber && 
        PciId->Bus <= gPCIFromPE.EndBusNumber) {
        
        /* Calculate ECAM offset */
        UInt32 bus_offset = (PciId->Bus - gPCIFromPE.StartBusNumber) << 20;
        UInt32 device_offset = PciId->Device << 15;
        UInt32 function_offset = PciId->Function << 12;
        UInt32 total_offset = bus_offset + device_offset + function_offset + Reg;
        
        void *config_addr = (char *)gPCIEcamVirtualBase + total_offset;
        
        switch (Width) {
            case 8:
                *(UInt8 *)config_addr = (UInt8)Value;
                break;
            case 16:
                *(UInt16 *)config_addr = (UInt16)Value;
                break;
            case 32:
                *(UInt32 *)config_addr = (UInt32)Value;
                break;
        }
        
        return AE_OK;
    }
    
    /* Fallback: This should call the legacy port I/O method */
    return AE_NOT_IMPLEMENTED;
}

/*
 * Complete ACPI Status to IOReturn mapping
 * This addresses the large comment block with all ACPI error codes
 */
IOReturn AcpiStatus2IOReturn(ACPI_STATUS stat) 
{
    switch (stat) {
        case AE_OK:
            return kIOReturnSuccess;
            
        /* Memory related errors */
        case AE_NO_MEMORY:
            return kIOReturnNoMemory;
        case AE_BUFFER_OVERFLOW:
            return kIOReturnOverrun;
        case AE_STACK_OVERFLOW:
        case AE_STACK_UNDERFLOW:
            return kIOReturnOverrun;
            
        /* Parameter errors */
        case AE_BAD_PARAMETER:
            return kIOReturnBadArgument;
        case AE_NULL_OBJECT:
        case AE_NULL_ENTRY:
            return kIOReturnBadArgument;
            
        /* Resource errors */
        case AE_NOT_FOUND:
        case AE_NOT_EXIST:
            return kIOReturnNotFound;
        case AE_ALREADY_EXISTS:
            return kIOReturnExclusiveAccess;
        case AE_LIMIT:
            return kIOReturnNoResources;
        case AE_NO_HARDWARE_RESPONSE:
            return kIOReturnNoDevice;
            
        /* Access and permission errors */
        case AE_ACCESS:
            return kIOReturnNotPrivileged;
        case AE_NOT_ACQUIRED:
        case AE_RELEASE_DEADLOCK:
        case AE_ACQUIRE_DEADLOCK:
            return kIOReturnCannotLock;
        case AE_ALREADY_ACQUIRED:
            return kIOReturnExclusiveAccess;
            
        /* Timing errors */
        case AE_TIME:
            return kIOReturnTimeout;
            
        /* Implementation errors */
        case AE_NOT_IMPLEMENTED:
        case AE_SUPPORT:
            return kIOReturnUnsupported;
        case AE_NOT_CONFIGURED:
            return kIOReturnNotReady;
            
        /* Data errors */
        case AE_TYPE:
            return kIOReturnBadMessageID;
        case AE_NUMERIC_OVERFLOW:
        case AE_HEX_OVERFLOW:
        case AE_DECIMAL_OVERFLOW:
        case AE_OCTAL_OVERFLOW:
            return kIOReturnMessageTooLarge;
            
        /* Table and namespace errors */
        case AE_NO_ACPI_TABLES:
        case AE_NO_NAMESPACE:
            return kIOReturnNotReady;
        case AE_END_OF_TABLE:
            return kIOReturnSuccess; /* End of iteration, not an error */
            
        /* I/O errors */
        case AE_IO_ERROR:
            return kIOReturnIOError;
            
        /* Method execution errors */
        case AE_ABORT_METHOD:
            return kIOReturnAborted;
        case AE_NO_GLOBAL_LOCK:
        case AE_NO_HANDLER:
        case AE_SAME_HANDLER:
            return kIOReturnNoResources;
        case AE_OWNER_ID_LIMIT:
            return kIOReturnNoResources;
            
        /* Generic errors */
        case AE_ERROR:
        default:
            return kIOReturnError;
    }
}

/*
 * Cleanup function for AcpiOsLayer resources
 * Should be called during termination
 */
ACPI_STATUS AcpiOsExtTerminate(void)
{
    /* Wait for all pending executions to complete */
    AcpiOsExtWaitEventsComplete();
    
    /* Cleanup ECAM mapping */
    if (gPCIEcamMap) {
        gPCIEcamMap->release();
        gPCIEcamMap = NULL;
        gPCIEcamVirtualBase = NULL;
        gPCIEcamAvailable = false;
    }
    
    /* Cleanup work loop and command gate */
    if (gAcpiOsThreadCommandGate) {
        gAcpiOsThreadCommandGate->release();
        gAcpiOsThreadCommandGate = NULL;
    }
    
    if (gAcpiOsThreadWorkLoop) {
        gAcpiOsThreadWorkLoop->release();
        gAcpiOsThreadWorkLoop = NULL;
    }
    
    /* Cleanup memory maps */
    if (gAcpiOsExtMemoryMapIterator) {
        gAcpiOsExtMemoryMapIterator->release();
        gAcpiOsExtMemoryMapIterator = NULL;
    }
    
    if (gAcpiOsExtMemoryMapSet) {
        gAcpiOsExtMemoryMapSet->release();
        gAcpiOsExtMemoryMapSet = NULL;
    }
    
    /* Cleanup locks */
    if (gAcpiOsExtMemoryMapLock) {
        IOLockFree(gAcpiOsExtMemoryMapLock);
        gAcpiOsExtMemoryMapLock = NULL;
    }
    
    if (gExecutionLock) {
        IOLockFree(gExecutionLock);
        gExecutionLock = NULL;
    }
    
    return AE_OK;
}
