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
* This specific file was created by Zormeister w/ csekel (InSaneDarwin) adjustments
*/

/* standard includes... */
#include "acpica/acpi.h"
#include "acpica/actables.h"  /* For MCFG table definitions */

#include <mach/semaphore.h>
#include <machine/machine_routines.h>
#include <mach/machine.h>
#include <IOKit/IOLib.h>
#include <mach/thread_status.h>

/* ACPI OS Layer implementations because yes */
#define _COMPONENT ACPI_OS_SERVICES
ACPI_MODULE_NAME("osdarwin");

/* TODO:
 * - AcpiOsCreateSemaphore - DONE
 * - AcpiOsDeleteSemaphore - DONE (implemented as AcpiOsDestroySemaphore)
 * - AcpiOsGetTimer - DONE
 * - AcpiOsSignalSemaphore - DONE
 * - AcpiOsWaitSemaphore - DONE
 *
 * PCI I/O accessing too - DONE Implemented
 * Cache functions - DONE added Implementation
 */

/* track memory allocations, otherwise all hell will break loose in XNU. */
struct _memory_tag {
    UINT32 magic;
    ACPI_SIZE size;
};

/* Cache management structures for ACPICA object caching */
struct _cache_object {
    struct _cache_object *next;
    /* Object data follows this header */
};

struct _acpi_cache {
    UINT32 magic;
    char name[16];
    ACPI_SIZE object_size;
    UINT16 max_depth;
    UINT16 current_depth;
    struct _cache_object *list_head;
    IOSimpleLock *lock;
    UINT32 requests;
    UINT32 hits;
};

#define ACPI_CACHE_MAGIC 'cach'

#define ACPI_OS_PRINTF_USE_KPRINTF 0x1
#define ACPI_OS_PRINTF_USE_IOLOG   0x2

#if DEBUG
UInt32 gAcpiOsPrintfFlags = ACPI_OS_PRINTF_USE_KPRINTF | ACPI_OS_PRINTF_USE_IOLOG;
#else
/* ZORMEISTER: silence ACPICA's terror from the bad ACPI of HP, Lenovo and various other companies */
UInt32 gAcpiOsPrintfFlags = ACPI_OS_PRINTF_USE_IOLOG;
#endif

/* External functions - see PDACPIPlatform/AcpiOsLayer.cpp */
extern void *AcpiOsExtMapMemory(ACPI_PHYSICAL_ADDRESS, ACPI_SIZE);
extern void AcpiOsExtUnmapMemory(void *);
extern ACPI_STATUS AcpiOsExtInitialize(void);
extern ACPI_PHYSICAL_ADDRESS AcpiOsExtGetRootPointer(void);
extern ACPI_STATUS AcpiOsExtExecute(ACPI_EXECUTE_TYPE Type, ACPI_OSD_EXEC_CALLBACK Function, void *Context);

ACPI_STATUS AcpiOsInitialize(void)
{
    ACPI_STATUS status;
    
    PE_parse_boot_argn("acpi_os_log", &gAcpiOsPrintfFlags, sizeof(UInt32));
    
    status = AcpiOsExtInitialize(); /* dispatch to AcpiOsLayer.cpp to establish the memory map tracking + PCI access. */
    if (ACPI_FAILURE(status)) {
        return status;
    }
    
    /* Initialize ECAM support - this will be done lazily on first PCI access if ACPI tables aren't ready yet */
    /* AcpiOsInitializePciEcam(); */
    
    return AE_OK;
}

/* AcpiOsValidateCache (Debug helper) - Validate cache integrity (debug builds only) */
#if DEBUG
ACPI_STATUS
AcpiOsValidateCache(ACPI_CACHE_T *Cache)
{
    struct _acpi_cache *cache = (struct _acpi_cache *)Cache;
    struct _cache_object *object;
    UINT16 count = 0;
    
    if (!cache || cache->magic != ACPI_CACHE_MAGIC) {
        AcpiOsPrintf("ACPI: Invalid cache object\n");
        return AE_BAD_PARAMETER;
    }
    
    IOSimpleLockLock(cache->lock);
    
    /* Count objects in cache */
    object = cache->list_head;
    while (object && count < cache->max_depth + 10) { /* Prevent infinite loops */
        count++;
        object = object->next;
    }
    
    if (count != cache->current_depth) {
        AcpiOsPrintf("ACPI: Cache '%s' depth mismatch: reported %d, actual %d\n",
                     cache->name, cache->current_depth, count);
        IOSimpleLockUnlock(cache->lock);
        return AE_ERROR;
    }
    
    IOSimpleLockUnlock(cache->lock);
    
    AcpiOsPrintf("ACPI: Cache '%s' validated: %d/%d objects, %u requests, %u hits\n",
                 cache->name, cache->current_depth, cache->max_depth,
                 cache->requests, cache->hits);
    
    return AE_OK;
}
#endif

/* AcpiOsGetCacheStatistics (Debug helper) -  Get cache statistics */
ACPI_STATUS
AcpiOsGetCacheStatistics(ACPI_CACHE_T *Cache, UINT32 *Requests, UINT32 *Hits)
{
    struct _acpi_cache *cache = (struct _acpi_cache *)Cache;
    
    if (!cache || cache->magic != ACPI_CACHE_MAGIC) {
        return AE_BAD_PARAMETER;
    }
    
    if (Requests) {
        *Requests = cache->requests;
    }
    
    if (Hits) {
        *Hits = cache->hits;
    }
    
    return AE_OK;
}

ACPI_STATUS AcpiOsTerminate(void)
{
    /* Cleanup any OS-specific resources if needed */
    gPciEcamInitialized = FALSE;
    gPciEcamBase = 0;
    gPciEcamSize = 0;
    gPciStartBus = 0;
    gPciEndBus = 0;
    
    /* Note: ACPICA should clean up its own caches via AcpiOsDeleteCache() */
    /* but we could add cache leak detection here in debug builds */
    
    return AE_OK;
}

ACPI_PHYSICAL_ADDRESS AcpiOsGetRootPointer(void)
{
    return AcpiOsExtGetRootPointer();
}

#pragma mark Memory-related services

/* This is an XNU private API. I'd much rather a public function but that seems impossible. */
extern vm_offset_t ml_vtophys(vm_offset_t);

ACPI_STATUS AcpiOsGetPhysicalAddress(void *LogicalAddress, ACPI_PHYSICAL_ADDRESS *PhysicalAddress)
{
    IOVirtualAddress va = (IOVirtualAddress)LogicalAddress;
    *PhysicalAddress = ml_vtophys(va); /* i sure do hope this is compatible */
    return AE_OK;
}

void *
AcpiOsMapMemory(ACPI_PHYSICAL_ADDRESS Where, ACPI_SIZE Length)
{
    return AcpiOsExtMapMemory(Where, Length);
}

void
AcpiOsUnmapMemory(void *LogicalAddress, ACPI_SIZE Length)
{
    AcpiOsExtUnmapMemory(LogicalAddress);
}

void *
AcpiOsAllocate(ACPI_SIZE Size)
{
    void *alloc = IOMalloc(Size + sizeof(struct _memory_tag));
    struct _memory_tag *mem = alloc;
    mem->magic = 'mema';
    mem->size = Size + sizeof(struct _memory_tag);
    return (alloc + sizeof(struct _memory_tag));
}

void *
AcpiOsAllocateZeroed(ACPI_SIZE Size)
{
    void *alloc = AcpiOsAllocate(Size);
    memset(alloc, 0, Size);
    return alloc;
}

void
AcpiOsFree(void *p)
{
    struct _memory_tag *m = p - sizeof(struct _memory_tag);
    if (m->magic == 'mema') {
        IOFree(m, m->size);
    } else {
        /* induce panic? */
        return;
    }
}

/* ZORMEISTER: me is kernel. i can write and read as i want. */
BOOLEAN AcpiOsReadable(void *Memory, ACPI_SIZE Length) { return true; }
BOOLEAN AcpiOsWriteable(void *Memory, ACPI_SIZE Length) { return true; }

/* ZORMEISTER: import KPIs because otherwise this won't work. */
extern unsigned int ml_phys_read_byte(vm_offset_t paddr);
extern unsigned int ml_phys_read_byte_64(addr64_t paddr);
extern unsigned int ml_phys_read_half(vm_offset_t paddr);
extern unsigned int ml_phys_read_half_64(addr64_t paddr);
extern unsigned int ml_phys_read_word(vm_offset_t paddr);
extern unsigned int ml_phys_read_word_64(addr64_t paddr);
extern unsigned long long ml_phys_read_double(vm_offset_t paddr);
extern unsigned long long ml_phys_read_double_64(addr64_t paddr);

/*
 * ZORMEISTER:
 * mind you this is my local machine using the following:
 *
 * SDK: MacOSX15.4.sdk
 * Xcode Version: 16.3 build 16E140
 * Apple Clang version: clang-1700.0.13.3
 *
 * i need to establish a build server for my projects
 *
 */

ACPI_STATUS
AcpiOsReadMemory(
    ACPI_PHYSICAL_ADDRESS   Address,
    UINT64                  *Value,
    UINT32                  Width)
{
    switch (Width) {
        case 8:
#if __LP64__
            *Value = ml_phys_read_byte_64(Address);
#else
            *Value = ml_phys_read_byte(Address);
#endif
            return AE_OK;
        case 16:
#if __LP64__
            *Value = ml_phys_read_half_64(Address);
#else
            *Value = ml_phys_read_half(Address);
#endif
            return AE_OK;
        case 32:
#if __LP64__
            *Value = ml_phys_read_word_64(Address);
#else
            *Value = ml_phys_read_word(Address);
#endif
            return AE_OK;
        case 64:
#if __LP64__
            *Value = ml_phys_read_double_64(Address);
#else
            *Value = ml_phys_read_double(Address);
#endif
        default:
            AcpiOsPrintf("ACPI: bad width value\n");
            return AE_ERROR;
    }
}

extern uint8_t ml_port_io_read8(uint16_t ioport);
extern uint16_t ml_port_io_read16(uint16_t ioport);
extern uint32_t ml_port_io_read32(uint16_t ioport);

ACPI_STATUS
AcpiOsReadPort(ACPI_IO_ADDRESS Address,
               UINT32 *Value,
               UINT32 Width)
{
    switch (Width) {
        case 8:
            *Value = ml_port_io_read8(Address);
            return AE_OK;
        case 16:
            *Value = ml_port_io_read16(Address);
            return AE_OK;
        case 32:
            *Value = ml_port_io_read32(Address);
            return AE_OK;
        default:
            return AE_BAD_PARAMETER;
    }
}


extern void ml_phys_write_byte(vm_offset_t paddr, unsigned int data);
extern void ml_phys_write_byte_64(addr64_t paddr, unsigned int data);
extern void ml_phys_write_half(vm_offset_t paddr, unsigned int data);
extern void ml_phys_write_half_64(addr64_t paddr, unsigned int data);
extern void ml_phys_write_word(vm_offset_t paddr, unsigned int data);
extern void ml_phys_write_word_64(addr64_t paddr, unsigned int data);
extern void ml_phys_write_double(vm_offset_t paddr, unsigned long long data);
extern void ml_phys_write_double_64(addr64_t paddr, unsigned long long data);

ACPI_STATUS
AcpiOsWriteMemory(ACPI_PHYSICAL_ADDRESS Address,
                  UINT64 Value,
                  UINT32 Width)
{
    switch (Width) {
        case 8:
#if __LP64__
            ml_phys_write_byte_64(Address, (UINT32)Value);
#else
            ml_phys_write_byte(Address, (UINT32)Value);
#endif
            return AE_OK;
        case 16:
#if __LP64__
            ml_phys_write_half_64(Address, (UINT32)Value);
#else
            ml_phys_write_half(Address, (UINT32)Value);
#endif
            return AE_OK;
        case 32:
#if __LP64__
            ml_phys_write_word_64(Address, (UINT32)Value);
#else
            ml_phys_write_word(Address, (UINT32)Value);
#endif
            return AE_OK;
        case 64:
#if __LP64__
            ml_phys_write_double_64(Address, Value);
#else
            ml_phys_write_double(Address, Value);
#endif
        default:
            AcpiOsPrintf("ACPI: bad width value\n");
            return AE_ERROR;
    }
}

extern void ml_port_io_write8(uint16_t ioport, uint8_t val);
extern void ml_port_io_write16(uint16_t ioport, uint16_t val);
extern void ml_port_io_write32(uint16_t ioport, uint32_t val);

ACPI_STATUS
AcpiOsWritePort(ACPI_IO_ADDRESS Address,
                UINT32 Value,
                UINT32 Width)
{
    switch (Width) {
        case 8:
            ml_port_io_write8(Address, (UINT8)Value);
            return AE_OK;
        case 16:
            ml_port_io_write16(Address, (UINT16)Value);
            return AE_OK;
        case 32:
            ml_port_io_write32(Address, Value);
            return AE_OK;
        default:
            return AE_BAD_PARAMETER;
    }
}

/* PCI Configuration Space Access - MMIO and Port I/O Implementation */

/* Global variables for ECAM/MMIO support */
static ACPI_PHYSICAL_ADDRESS gPciEcamBase = 0;
static UINT32 gPciEcamSize = 0;
static UINT16 gPciStartBus = 0;
static UINT16 gPciEndBus = 0;
static boolean_t gPciEcamInitialized = FALSE;

/* Initialize ECAM (Enhanced Configuration Access Mechanism) support */
static ACPI_STATUS
AcpiOsInitializePciEcam(void)
{
    ACPI_TABLE_MCFG *mcfg_table = NULL;
    ACPI_STATUS status;
    
    if (gPciEcamInitialized) {
        return AE_OK;
    }
    
    /* Try to get MCFG table for ECAM base address */
    status = AcpiGetTable(ACPI_SIG_MCFG, 0, (ACPI_TABLE_HEADER **)&mcfg_table);
    if (ACPI_SUCCESS(status) && mcfg_table) {
        ACPI_MCFG_ALLOCATION *allocation;
        
        /* Get first allocation entry */
        allocation = (ACPI_MCFG_ALLOCATION *)((UINT8 *)mcfg_table + sizeof(ACPI_TABLE_MCFG));
        
        if ((UINT8 *)allocation < (UINT8 *)mcfg_table + mcfg_table->Header.Length) {
            gPciEcamBase = allocation->Address;
            gPciStartBus = allocation->PciSegment;  /* Actually start bus */
            gPciEndBus = allocation->EndBusNumber;
            gPciEcamSize = (gPciEndBus - gPciStartBus + 1) * 256 * 4096; /* Each bus has 256 devices, 4KB each */
            
#if DEBUG
            AcpiOsPrintf("ACPI: ECAM base 0x%llX, buses %d-%d, size 0x%X\n", 
                        gPciEcamBase, gPciStartBus, gPciEndBus, gPciEcamSize);
#endif
        }
    }
    
    gPciEcamInitialized = TRUE;
    return AE_OK;
}

/* MMIO-based PCI configuration space access */
static ACPI_STATUS
AcpiOsReadPciConfigMmio(ACPI_PCI_ID *PciId, UINT32 Register, UINT64 *Value, UINT32 Width)
{
    ACPI_PHYSICAL_ADDRESS config_addr;
    void *mapped_addr;
    UINT64 data = 0;
    
    /* Calculate ECAM address: Base + (Bus << 20) + (Device << 15) + (Function << 12) + Register */
    config_addr = gPciEcamBase + 
                  ((UINT64)PciId->Bus << 20) + 
                  ((UINT64)PciId->Device << 15) + 
                  ((UINT64)PciId->Function << 12) + 
                  Register;
    
    /* Map the configuration space */
    mapped_addr = AcpiOsMapMemory(config_addr, Width / 8);
    if (!mapped_addr) {
        return AE_NO_MEMORY;
    }
    
    /* Read the value */
    switch (Width) {
        case 8:
            data = *(UINT8 *)mapped_addr;
            break;
        case 16:
            data = *(UINT16 *)mapped_addr;
            break;
        case 32:
            data = *(UINT32 *)mapped_addr;
            break;
        default:
            AcpiOsUnmapMemory(mapped_addr, Width / 8);
            return AE_BAD_PARAMETER;
    }
    
    *Value = data;
    AcpiOsUnmapMemory(mapped_addr, Width / 8);
    
#if DEBUG
    AcpiOsPrintf("PCI MMIO read: %02X:%02X:%02X reg 0x%02X width %d = 0x%X\n",
                 PciId->Bus, PciId->Device, PciId->Function, 
                 Register, Width, (UINT32)*Value);
#endif
    
    return AE_OK;
}

static ACPI_STATUS
AcpiOsWritePciConfigMmio(ACPI_PCI_ID *PciId, UINT32 Register, UINT64 Value, UINT32 Width)
{
    ACPI_PHYSICAL_ADDRESS config_addr;
    void *mapped_addr;
    
#if DEBUG
    AcpiOsPrintf("PCI MMIO write: %02X:%02X:%02X reg 0x%02X width %d = 0x%X\n",
                 PciId->Bus, PciId->Device, PciId->Function, 
                 Register, Width, (UINT32)Value);
#endif
    
    /* Calculate ECAM address */
    config_addr = gPciEcamBase + 
                  ((UINT64)PciId->Bus << 20) + 
                  ((UINT64)PciId->Device << 15) + 
                  ((UINT64)PciId->Function << 12) + 
                  Register;
    
    /* Map the configuration space */
    mapped_addr = AcpiOsMapMemory(config_addr, Width / 8);
    if (!mapped_addr) {
        return AE_NO_MEMORY;
    }
    
    /* Write the value */
    switch (Width) {
        case 8:
            *(UINT8 *)mapped_addr = (UINT8)Value;
            break;
        case 16:
            *(UINT16 *)mapped_addr = (UINT16)Value;
            break;
        case 32:
            *(UINT32 *)mapped_addr = (UINT32)Value;
            break;
        default:
            AcpiOsUnmapMemory(mapped_addr, Width / 8);
            return AE_BAD_PARAMETER;
    }
    
    AcpiOsUnmapMemory(mapped_addr, Width / 8);
    return AE_OK;
}

/* Legacy Port I/O based PCI configuration space access */
static ACPI_STATUS
AcpiOsReadPciConfigPortIo(ACPI_PCI_ID *PciId, UINT32 Register, UINT64 *Value, UINT32 Width)
{
    UINT32 pci_address;
    UINT32 data = 0;
    
    /* Construct PCI configuration address for legacy method */
    pci_address = (1U << 31) |                  /* Enable bit */
                  (PciId->Bus << 16) |          /* Bus number */
                  (PciId->Device << 11) |       /* Device number */
                  (PciId->Function << 8) |      /* Function number */
                  (Register & 0xFC);            /* Register (aligned to 32-bit) */
    
    /* Write address to CONFIG_ADDRESS port (0xCF8) */
    ml_port_io_write32(0xCF8, pci_address);
    
    /* Read data from CONFIG_DATA port (0xCFC) with appropriate offset */
    switch (Width) {
        case 8:
            data = ml_port_io_read8(0xCFC + (Register & 3));
            break;
        case 16:
            data = ml_port_io_read16(0xCFC + (Register & 2));
            break;
        case 32:
            data = ml_port_io_read32(0xCFC);
            break;
    }
    
    *Value = data;
    
#if DEBUG
    AcpiOsPrintf("PCI Port I/O read: %02X:%02X:%02X reg 0x%02X width %d = 0x%X\n",
                 PciId->Bus, PciId->Device, PciId->Function, 
                 Register, Width, (UINT32)*Value);
#endif
    
    return AE_OK;
}

static ACPI_STATUS
AcpiOsWritePciConfigPortIo(ACPI_PCI_ID *PciId, UINT32 Register, UINT64 Value, UINT32 Width)
{
    UINT32 pci_address;
    
#if DEBUG
    AcpiOsPrintf("PCI Port I/O write: %02X:%02X:%02X reg 0x%02X width %d = 0x%X\n",
                 PciId->Bus, PciId->Device, PciId->Function, 
                 Register, Width, (UINT32)Value);
#endif
    
    /* Construct PCI configuration address */
    pci_address = (1U << 31) |                  /* Enable bit */
                  (PciId->Bus << 16) |          /* Bus number */
                  (PciId->Device << 11) |       /* Device number */
                  (PciId->Function << 8) |      /* Function number */
                  (Register & 0xFC);            /* Register (aligned to 32-bit) */
    
    /* Write address to CONFIG_ADDRESS port (0xCF8) */
    ml_port_io_write32(0xCF8, pci_address);
    
    /* Write data to CONFIG_DATA port (0xCFC) with appropriate offset */
    switch (Width) {
        case 8:
            ml_port_io_write8(0xCFC + (Register & 3), (UINT8)Value);
            break;
        case 16:
            ml_port_io_write16(0xCFC + (Register & 2), (UINT16)Value);
            break;
        case 32:
            ml_port_io_write32(0xCFC, (UINT32)Value);
            break;
    }
    
    return AE_OK;
}

/* Main PCI Configuration Space Access Functions */
ACPI_STATUS
AcpiOsReadPciConfiguration(ACPI_PCI_ID *PciId,
                           UINT32 Register,
                           UINT64 *Value,
                           UINT32 Width)
{
    if (!PciId || !Value) {
        return AE_BAD_PARAMETER;
    }
    
    if (Width != 8 && Width != 16 && Width != 32) {
        return AE_BAD_PARAMETER;
    }
    
    /* Initialize ECAM support if not done already */
    AcpiOsInitializePciEcam();
    
    /* Try MMIO/ECAM first if available and bus is in range */
    if (gPciEcamBase != 0 && 
        PciId->Bus >= gPciStartBus && 
        PciId->Bus <= gPciEndBus) {
        return AcpiOsReadPciConfigMmio(PciId, Register, Value, Width);
    }
    
    /* Fall back to legacy Port I/O method */
    return AcpiOsReadPciConfigPortIo(PciId, Register, Value, Width);
}

ACPI_STATUS
AcpiOsWritePciConfiguration(ACPI_PCI_ID *PciId,
                            UINT32 Register,
                            UINT64 Value,
                            UINT32 Width)
{
    if (!PciId) {
        return AE_BAD_PARAMETER;
    }
    
    if (Width != 8 && Width != 16 && Width != 32) {
        return AE_BAD_PARAMETER;
    }
    
    /* Initialize ECAM support if not done already */
    AcpiOsInitializePciEcam();
    
    /* Try MMIO/ECAM first if available and bus is in range */
    if (gPciEcamBase != 0 && 
        PciId->Bus >= gPciStartBus && 
        PciId->Bus <= gPciEndBus) {
        return AcpiOsWritePciConfigMmio(PciId, Register, Value, Width);
    }
    
    /* Fall back to legacy Port I/O method */
    return AcpiOsWritePciConfigPortIo(PciId, Register, Value, Width);
}

#pragma mark OS time related functions

void AcpiOsSleep(UINT64 ms)
{
    IOSleep((UINT32)ms);
}

void AcpiOsStall(UINT32 us)
{
    IODelay(us);
}

/* ZORMEISTER: I think? - 'The current value of the system timer in 100-nanosecond units. '*/
UInt64 AcpiOsGetTimer(void)
{
    UInt64 abs = mach_absolute_time();
    UInt64 ns = 0;
    absolutetime_to_nanoseconds(abs, &ns);
    return (ns / 100);
}

#pragma mark Lock functions

ACPI_STATUS AcpiOsCreateLock(ACPI_SPINLOCK *Lock)
{
    IOSimpleLock *lck = IOSimpleLockAlloc();
    if (!lck) {
        return AE_NO_MEMORY;
    }
    
    *Lock = lck;
    
    return AE_OK;
};

void AcpiOsDeleteLock(ACPI_SPINLOCK Lock)
{
    IOSimpleLockFree(Lock);
}


/* 'May be called from interrupt handlers, GPE handlers, and Fixed event handlers.' */
/* Fun way of saying I should disable interrupts until the lock is released. */
ACPI_CPU_FLAGS AcpiOsAcquireLock(ACPI_SPINLOCK Lock)
{
    ml_set_interrupts_enabled(false);
    IOSimpleLockLock(Lock);
    return 0;
}

void AcpiOsReleaseLock(ACPI_SPINLOCK Lock, ACPI_CPU_FLAGS Flags)
{
    IOSimpleLockUnlock(Lock);
    ml_set_interrupts_enabled(true);
}

#pragma mark Semaphore code

ACPI_STATUS AcpiOsCreateSemaphore(UInt32 InitialUnits, UInt32 MaxUnits, ACPI_SEMAPHORE *Handle)
{
    if (Handle == NULL) {
        return AE_BAD_PARAMETER;
    }
    
    if (semaphore_create(current_task(), Handle, 0, MaxUnits) == KERN_SUCCESS) {
        return AE_OK;
    }

    return AE_NO_MEMORY;
}

ACPI_STATUS AcpiOsDeleteSemaphore(ACPI_SEMAPHORE Handle)
{
    return AcpiOsDestroySemaphore(Handle);
}

ACPI_STATUS AcpiOsDestroySemaphore(ACPI_SEMAPHORE Semaphore)
{
    if (Semaphore == NULL) {
        return AE_BAD_PARAMETER;
    }

    semaphore_destroy(current_task(), Semaphore);

    return AE_OK;
}

ACPI_STATUS AcpiOsWaitSemaphore(ACPI_SEMAPHORE Semaphore, UInt32 Units, UInt16 Timeout)
{
    if (Semaphore == NULL) {
        return AE_BAD_PARAMETER;
    }
    
    if (Timeout == 0xFFFF) {
        if (semaphore_wait(Semaphore) != KERN_SUCCESS) {
            return AE_TIME;
        }
    } else {
        if (semaphore_wait_deadline(Semaphore, (Timeout * NSEC_PER_MSEC)) != KERN_SUCCESS) {
            return AE_TIME;
        }
    }
    
    return AE_OK;
}

ACPI_STATUS AcpiOsSignalSemaphore(ACPI_SEMAPHORE Semaphore, UInt32 Units)
{
    if (Semaphore == NULL) {
        return AE_BAD_PARAMETER;
    }
    
    if (Units > 1) {
        semaphore_signal_all(Semaphore);
    } else {
        semaphore_signal(Semaphore);
    }
    
    return AE_OK;
}


#pragma mark Cache management functions

/* adjustments by csekel (InSaneDarwin)
 * ACPICA Object Cache Implementation for Darwin
 * 
 * ACPICA uses object caching to improve performance by reusing frequently
 * allocated/freed objects like parse tree nodes, namespace entries, etc.
 *
 * Cache objects are stored as a simple linked list (LIFO) for fast
 * acquire/release operations. Each cache maintains its own lock and
 * statistics to minimize contention between different object types.
 */

/* AcpiOsCreateCache - Create a cache object for ACPICA */
ACPI_STATUS
AcpiOsCreateCache(char *CacheName,
                  UINT16 ObjectSize,
                  UINT16 MaxDepth,
                  ACPI_CACHE_T **ReturnCache)
{
    struct _acpi_cache *cache;
    
    if (!CacheName || !ReturnCache || ObjectSize == 0) {
        return AE_BAD_PARAMETER;
    }
    
    /* Allocate cache structure */
    cache = (struct _acpi_cache *)IOMalloc(sizeof(struct _acpi_cache));
    if (!cache) {
        return AE_NO_MEMORY;
    }
    
    /* Initialize cache structure */
    memset(cache, 0, sizeof(struct _acpi_cache));
    cache->magic = ACPI_CACHE_MAGIC;
    strncpy(cache->name, CacheName, sizeof(cache->name) - 1);
    cache->name[sizeof(cache->name) - 1] = '\0';
    cache->object_size = ObjectSize;
    cache->max_depth = MaxDepth;
    cache->current_depth = 0;
    cache->list_head = NULL;
    cache->requests = 0;
    cache->hits = 0;
    
    /* Create lock for thread safety */
    cache->lock = IOSimpleLockAlloc();
    if (!cache->lock) {
        IOFree(cache, sizeof(struct _acpi_cache));
        return AE_NO_MEMORY;
    }
    
    *ReturnCache = (ACPI_CACHE_T *)cache;
    
#if DEBUG
    AcpiOsPrintf("ACPI: Created cache '%s', object size %d, max depth %d\n",
                 CacheName, ObjectSize, MaxDepth);
#endif
    
    return AE_OK;
}

/* AcpiOsDeleteCache - Free all objects within a cache and delete the cache object */
ACPI_STATUS
AcpiOsDeleteCache(ACPI_CACHE_T *Cache)
{
    struct _acpi_cache *cache = (struct _acpi_cache *)Cache;
    struct _cache_object *object, *next;
    
    if (!cache || cache->magic != ACPI_CACHE_MAGIC) {
        return AE_BAD_PARAMETER;
    }
    
#if DEBUG
    AcpiOsPrintf("ACPI: Deleting cache '%s', requests %u, hits %u (%.1f%%)\n",
                 cache->name, cache->requests, cache->hits,
                 cache->requests ? (cache->hits * 100.0 / cache->requests) : 0.0);
#endif
    
    /* Purge all objects from cache */
    IOSimpleLockLock(cache->lock);
    
    object = cache->list_head;
    while (object) {
        next = object->next;
        IOFree(object, sizeof(struct _cache_object) + cache->object_size);
        object = next;
    }
    
    IOSimpleLockUnlock(cache->lock);
    
    /* Free the lock and cache structure */
    IOSimpleLockFree(cache->lock);
    cache->magic = 0; /* Invalidate */
    IOFree(cache, sizeof(struct _acpi_cache));
    
    return AE_OK;
}

/* AcpiOsPurgeCache - Free all objects within a cache */
ACPI_STATUS
AcpiOsPurgeCache(ACPI_CACHE_T *Cache)
{
    struct _acpi_cache *cache = (struct _acpi_cache *)Cache;
    struct _cache_object *object, *next;
    UINT16 purged = 0;
    
    if (!cache || cache->magic != ACPI_CACHE_MAGIC) {
        return AE_BAD_PARAMETER;
    }
    
    IOSimpleLockLock(cache->lock);
    
    object = cache->list_head;
    while (object) {
        next = object->next;
        IOFree(object, sizeof(struct _cache_object) + cache->object_size);
        object = next;
        purged++;
    }
    
    cache->list_head = NULL;
    cache->current_depth = 0;
    
    IOSimpleLockUnlock(cache->lock);
    
#if DEBUG
    AcpiOsPrintf("ACPI: Purged %d objects from cache '%s'\n", purged, cache->name);
#endif
    
    return AE_OK;
}

/* AcpiOsAcquireObject - Get an object from the cache or allocate a new one */
void *
AcpiOsAcquireObject(ACPI_CACHE_T *Cache)
{
    struct _acpi_cache *cache = (struct _acpi_cache *)Cache;
    struct _cache_object *object;
    void *return_object = NULL;
    
    if (!cache || cache->magic != ACPI_CACHE_MAGIC) {
        return NULL;
    }
    
    IOSimpleLockLock(cache->lock);
    
    cache->requests++;
    
    /* Try to get an object from the cache first */
    if (cache->list_head) {
        object = cache->list_head;
        cache->list_head = object->next;
        cache->current_depth--;
        cache->hits++;
        
        /* Return pointer to data area (after the header) */
        return_object = (void *)((char *)object + sizeof(struct _cache_object));
        
        IOSimpleLockUnlock(cache->lock);
        
        /* Clear the object data */
        memset(return_object, 0, cache->object_size);
        
        return return_object;
    }
    
    IOSimpleLockUnlock(cache->lock);
    
    /* Cache is empty, allocate a new object */
    object = (struct _cache_object *)IOMalloc(sizeof(struct _cache_object) + cache->object_size);
    if (!object) {
        return NULL;
    }
    
    /* Clear the entire object */
    memset(object, 0, sizeof(struct _cache_object) + cache->object_size);
    
    /* Return pointer to data area */
    return_object = (void *)((char *)object + sizeof(struct _cache_object));
    
    return return_object;
}

/* AcpiOsReleaseObject - Release an object back to the cache */
ACPI_STATUS
AcpiOsReleaseObject(ACPI_CACHE_T *Cache, void *Object)
{
    struct _acpi_cache *cache = (struct _acpi_cache *)Cache;
    struct _cache_object *cache_object;
    
    if (!cache || cache->magic != ACPI_CACHE_MAGIC || !Object) {
        /* If cache is invalid, just free the object */
        if (Object) {
            cache_object = (struct _cache_object *)((char *)Object - sizeof(struct _cache_object));
            IOFree(cache_object, sizeof(struct _cache_object) + (cache ? cache->object_size : 0));
        }
        return AE_BAD_PARAMETER;
    }
    
    /* Get pointer to cache object header */
    cache_object = (struct _cache_object *)((char *)Object - sizeof(struct _cache_object));
    
    IOSimpleLockLock(cache->lock);
    
    /* If cache is full, just free the object */
    if (cache->current_depth >= cache->max_depth) {
        IOSimpleLockUnlock(cache->lock);
        IOFree(cache_object, sizeof(struct _cache_object) + cache->object_size);
        return AE_OK;
    }
    
    /* Add object back to cache */
    cache_object->next = cache->list_head;
    cache->list_head = cache_object;
    cache->current_depth++;
    
    IOSimpleLockUnlock(cache->lock);
    
    return AE_OK;
}

#pragma mark thread related stuff

ACPI_THREAD_ID
AcpiOsGetThreadId(void)
{
    return thread_tid(current_thread()); /* I think? */
}

ACPI_STATUS AcpiOsExecute(ACPI_EXECUTE_TYPE Type, ACPI_OSD_EXEC_CALLBACK Function, void *Context)
{
    return AcpiOsExtExecute(Type, Function, Context);
}

void AcpiOsWaitEventsComplete(void)
{
    /* Wait for all queued asynchronous events to complete */
    /* Implementation depends on how AcpiOsExecute queues work */
    /* For now, this is a no-op since AcpiOsExecute delegates to external implementation */
}

#pragma mark Override functions - they do nothing.

ACPI_STATUS AcpiOsPredefinedOverride(const ACPI_PREDEFINED_NAMES *PredefinedObject, ACPI_STRING *NewValue)
{
    *NewValue = NULL;
    return AE_OK;
}

ACPI_STATUS AcpiOsTableOverride(ACPI_TABLE_HEADER *ExistingTable, ACPI_TABLE_HEADER **NewTable)
{
    *NewTable = NULL;
    return AE_OK;
}

ACPI_STATUS AcpiOsPhysicalTableOverride(ACPI_TABLE_HEADER *ExistingTable,
                                        ACPI_PHYSICAL_ADDRESS *NewAddress,
                                        UINT32 *NewTableLength)
{
    if (!ExistingTable || !NewAddress || !NewTableLength) {
        return AE_BAD_PARAMETER;
    }
    
    *NewAddress = 0;
    *NewTableLength = 0;
    return AE_OK;
}

#pragma mark Misc. OSL services

ACPI_STATUS AcpiOsSignal(UINT32 Function, void *Info)
{
    switch (Function) {
        case ACPI_SIGNAL_BREAKPOINT: {
            if (Info) {
                char *breakpt = (char *)Info;
                AcpiOsPrintf("ACPI: recieved breakpoint signal: %s", breakpt);
            } else {
                AcpiOsPrintf("ACPI: recieved breakpoint signal");
            }
            break;
        }

        case ACPI_SIGNAL_FATAL: {
            if (Info) {
                ACPI_SIGNAL_FATAL_INFO *ftl = (ACPI_SIGNAL_FATAL_INFO *)Info;
                AcpiOsPrintf("ACPI: recieved AML fatal signal, type: %d, code: %d, arg: %d\n", ftl->Type, ftl->Code, ftl->Argument);
            } else {
                AcpiOsPrintf("ACPI: recieved AML fatal signal");
            }
            break;
        }

        default: {
            AcpiOsPrintf("ACPI: unknown signal recieved (%d)\n", Function);
            break;
        }
    }
    return AE_OK;
}

void AcpiOsPrintf(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    AcpiOsVprintf(fmt, va);
    va_end(va);
}

void AcpiOsVprintf(const char *fmt, va_list list)
{
    char msg[4096]; /* I don't think a message will exceed this size in one go. */
    vsnprintf(msg, 4096, fmt, list);

    if (gAcpiOsPrintfFlags & ACPI_OS_PRINTF_USE_KPRINTF) {
        kprintf("%s", msg);
    }
    
    if (gAcpiOsPrintfFlags & ACPI_OS_PRINTF_USE_IOLOG) {
        /* Allegedly IOLog can't be used within an interrupt context. I believe. */
        if (!ml_at_interrupt_context()) {
            IOLog("%s", msg);
        }
    }
}
