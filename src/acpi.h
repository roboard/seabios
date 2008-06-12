#ifndef __ACPI_H
#define __ACPI_H

#include "types.h" // u32

void acpi_bios_init(void);

// XXX - move to better header.
extern int smp_cpus;

struct rsdp_descriptor         /* Root System Descriptor Pointer */
{
	u8                            signature [8];          /* ACPI signature, contains "RSD PTR " */
	u8                              checksum;               /* To make sum of struct == 0 */
	u8                            oem_id [6];             /* OEM identification */
	u8                              revision;               /* Must be 0 for 1.0, 2 for 2.0 */
	u32                             rsdt_physical_address;  /* 32-bit physical address of RSDT */
	u32                             length;                 /* XSDT Length in bytes including hdr */
	u64                             xsdt_physical_address;  /* 64-bit physical address of XSDT */
	u8                              extended_checksum;      /* Checksum of entire table */
	u8                            reserved [3];           /* Reserved field must be 0 */
};

#endif // acpi.h
