// System Management Mode support (on emulators)
//
// Copyright (C) 2008-2014  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2006 Fabrice Bellard
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "config.h" // CONFIG_*
#include "dev-q35.h"
#include "dev-piix.h"
#include "hw/pci.h" // pci_config_writel
#include "hw/pci_ids.h" // PCI_VENDOR_ID_INTEL
#include "hw/pci_regs.h" // PCI_DEVICE_ID
#include "output.h" // dprintf
#include "paravirt.h" // PORT_SMI_STATUS
#include "string.h" // memcpy
#include "util.h" // smm_setup
#include "x86.h" // wbinvd

#define SMM_REV_I32 0x00020000
#define SMM_REV_I64 0x00020064

struct smm_state {
    union {
        struct {
            u8 pad_000[0xf8];
            u32 smm_base;
            u32 smm_rev;
            u8 pad_100[0xd0];
            u32 eax, ecx, edx, ebx, esp, ebp, esi, edi, eip, eflags;
            u8 pad_1f8[0x08];
        } i32;
        struct {
            u8 pad_000[0xfc];
            u32 smm_rev;
            u32 smm_base;
            u8 pad_104[0x6c];
            u64 rflags, rip, r15, r14, r13, r12, r11, r10, r9, r8;
            u64 rdi, rsi, rbp, rsp, rbx, rdx, rcx, rax;
        } i64;
    };
};

struct smm_layout {
    u8 stack[0x8000];
    u64 codeentry;
    u8 pad_8008[0x7df8];
    struct smm_state cpu;
};

void VISIBLE32FLAT
handle_smi(u16 cs)
{
    u8 cmd = inb(PORT_SMI_CMD);
    struct smm_layout *smm = MAKE_FLATPTR(cs, 0);
    dprintf(DEBUG_HDL_smi, "handle_smi cmd=%x smbase=%p\n", cmd, smm);

    if (smm == (void*)BUILD_SMM_INIT_ADDR) {
        // relocate SMBASE to 0xa0000
        if (smm->cpu.i32.smm_rev == SMM_REV_I32) {
            smm->cpu.i32.smm_base = BUILD_SMM_ADDR;
        } else if (smm->cpu.i64.smm_rev == SMM_REV_I64) {
            smm->cpu.i64.smm_base = BUILD_SMM_ADDR;
        } else {
            warn_internalerror();
            return;
        }
        // indicate to smm_relocate_and_restore() that the SMM code was executed
        outb(0x00, PORT_SMI_STATUS);
        return;
    }
}

extern void entry_smi(void);
// movw %cs, %ax; ljmpw $SEG_BIOS, $(entry_smi - BUILD_BIOS_ADDR)
#define SMI_INSN (0xeac88c | ((u64)SEG_BIOS<<40) \
                  | ((u64)((u32)entry_smi - BUILD_BIOS_ADDR) << 24))

static void
smm_save_and_copy(void)
{
    // save original memory content
    struct smm_layout *initsmm = (void*)BUILD_SMM_INIT_ADDR;
    struct smm_layout *smm = (void*)BUILD_SMM_ADDR;
    memcpy(&smm->cpu, &initsmm->cpu, sizeof(smm->cpu));
    memcpy(&smm->codeentry, &initsmm->codeentry, sizeof(smm->codeentry));

    // Setup code entry point.
    initsmm->codeentry = SMI_INSN;
}

static void
smm_relocate_and_restore(void)
{
    /* init APM status port */
    outb(0x01, PORT_SMI_STATUS);

    /* raise an SMI interrupt */
    outb(0x00, PORT_SMI_CMD);

    /* wait until SMM code executed */
    while (inb(PORT_SMI_STATUS) != 0x00)
        ;

    /* restore original memory content */
    struct smm_layout *initsmm = (void*)BUILD_SMM_INIT_ADDR;
    struct smm_layout *smm = (void*)BUILD_SMM_ADDR;
    memcpy(&initsmm->cpu, &smm->cpu, sizeof(initsmm->cpu));
    memcpy(&initsmm->codeentry, &smm->codeentry, sizeof(initsmm->codeentry));

    // Setup code entry point.
    smm->codeentry = SMI_INSN;
    wbinvd();
}

// This code is hardcoded for PIIX4 Power Management device.
static void piix4_apmc_smm_setup(int isabdf, int i440_bdf)
{
    /* check if SMM init is already done */
    u32 value = pci_config_readl(isabdf, PIIX_DEVACTB);
    if (value & PIIX_DEVACTB_APMC_EN)
        return;

    /* enable the SMM memory window */
    pci_config_writeb(i440_bdf, I440FX_SMRAM, 0x02 | 0x48);

    smm_save_and_copy();

    /* enable SMI generation when writing to the APMC register */
    pci_config_writel(isabdf, PIIX_DEVACTB, value | PIIX_DEVACTB_APMC_EN);

    /* enable SMI generation */
    value = inl(acpi_pm_base + PIIX_PMIO_GLBCTL);
    outl(acpi_pm_base + PIIX_PMIO_GLBCTL,
	 value | PIIX_PMIO_GLBCTL_SMI_EN);

    smm_relocate_and_restore();

    /* close the SMM memory window and enable normal SMM */
    pci_config_writeb(i440_bdf, I440FX_SMRAM, 0x02 | 0x08);
}

/* PCI_VENDOR_ID_INTEL && PCI_DEVICE_ID_INTEL_ICH9_LPC */
void ich9_lpc_apmc_smm_setup(int isabdf, int mch_bdf)
{
    /* check if SMM init is already done */
    u32 value = inl(acpi_pm_base + ICH9_PMIO_SMI_EN);
    if (value & ICH9_PMIO_SMI_EN_APMC_EN)
        return;

    /* enable the SMM memory window */
    pci_config_writeb(mch_bdf, Q35_HOST_BRIDGE_SMRAM, 0x02 | 0x48);

    smm_save_and_copy();

    /* enable SMI generation when writing to the APMC register */
    outl(value | ICH9_PMIO_SMI_EN_APMC_EN | ICH9_PMIO_SMI_EN_GLB_SMI_EN,
         acpi_pm_base + ICH9_PMIO_SMI_EN);

    /* lock SMI generation */
    value = pci_config_readw(isabdf, ICH9_LPC_GEN_PMCON_1);
    pci_config_writel(isabdf, ICH9_LPC_GEN_PMCON_1,
                      value | ICH9_LPC_GEN_PMCON_1_SMI_LOCK);

    smm_relocate_and_restore();

    /* close the SMM memory window and enable normal SMM */
    pci_config_writeb(mch_bdf, Q35_HOST_BRIDGE_SMRAM, 0x02 | 0x08);
}

static int SMMISADeviceBDF = -1, SMMPMDeviceBDF = -1;

void
smm_device_setup(void)
{
    if (!CONFIG_USE_SMM)
        return;

    struct pci_device *isapci, *pmpci;
    isapci = pci_find_device(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82371AB_3);
    pmpci = pci_find_device(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82441);
    if (isapci && pmpci) {
        SMMISADeviceBDF = isapci->bdf;
        SMMPMDeviceBDF = pmpci->bdf;
        return;
    }
    isapci = pci_find_device(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_ICH9_LPC);
    pmpci = pci_find_device(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_Q35_MCH);
    if (isapci && pmpci) {
        SMMISADeviceBDF = isapci->bdf;
        SMMPMDeviceBDF = pmpci->bdf;
    }
}

void
smm_setup(void)
{
    if (!CONFIG_USE_SMM || SMMISADeviceBDF < 0)
        return;

    dprintf(3, "init smm\n");
    u16 device = pci_config_readw(SMMISADeviceBDF, PCI_DEVICE_ID);
    if (device == PCI_DEVICE_ID_INTEL_82371AB_3)
        piix4_apmc_smm_setup(SMMISADeviceBDF, SMMPMDeviceBDF);
    else
        ich9_lpc_apmc_smm_setup(SMMISADeviceBDF, SMMPMDeviceBDF);
}
