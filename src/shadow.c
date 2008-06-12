// Support for enabling/disabling BIOS ram shadowing.
//
// Copyright (C) 2008  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2006 Fabrice Bellard
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "util.h" // memcpy
#include "pci.h" // pci_config_writeb

// Test if 'addr' is in the range from 'start'..'start+size'
#define IN_RANGE(addr, start, size) ({   \
            u32 __addr = (addr);         \
            u32 __start = (start);       \
            u32 __size = (size);         \
            (__addr - __start < __size); \
        })

// Enable shadowing and copy bios.
static void
copy_bios(PCIDevice d)
{
    int v = pci_config_readb(d, 0x59);
    v |= 0x30;
    pci_config_writeb(d, 0x59, v);
    memcpy((void *)0x000f0000, (void *)BUILD_BIOS_TMP_ADDR, 0x10000);
}

// Make the BIOS code segment area (0xf0000) writable.
void
make_bios_writable()
{
    if (CONFIG_COREBOOT)
        return;

    dprintf(3, "enabling shadow ram\n");

    // Locate chip controlling ram shadowing.
    PCIDevice d;
    int ret = pci_find_device(0x8086, 0x1237, 0, &d);
    if (ret) {
        dprintf(1, "Unable to unlock ram - bridge not found\n");
        return;
    }

    // Copy the bios to a temporary area.
    memcpy((void *)BUILD_BIOS_TMP_ADDR, (void *)0x000f0000, 0x10000);

    // Enable shadowing and copy bios.
    if (IN_RANGE((u32)copy_bios, 0xf0000, 0x10000)) {
        // Jump to shadow enable function - use the copy in the
        // temporary storage area so that memory does not change under
        // the executing code.
        u32 pos = (u32)copy_bios - 0xf0000 + BUILD_BIOS_TMP_ADDR;
        void (*func)(PCIDevice) = (void*)pos;
        func(d);
    } else {
        copy_bios(d);
    }

    // Clear the temporary area.
    memset((void *)BUILD_BIOS_TMP_ADDR, 0, 0x10000);
}

// Make the BIOS code segment area (0xf0000) read-only.
void
make_bios_readonly()
{
    if (CONFIG_COREBOOT)
        return;

    dprintf(3, "locking shadow ram\n");

    PCIDevice d;
    int ret = pci_find_device(0x8086, 0x1237, 0, &d);
    if (ret) {
        dprintf(1, "Unable to lock ram - bridge not found\n");
        return;
    }

    wbinvd();
    int v = pci_config_readb(d, 0x59);
    v = (v & 0x0f) | (0x10);
    pci_config_writeb(d, 0x59, v);
}
