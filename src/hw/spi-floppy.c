// This file may be distributed under the terms of the GNU LGPLv3 license.

// This option should be moved to Kconfig.
#define CONFIG_SPI_DISK 1

#include "biosvar.h" // GET_GLOBALFLAT
#include "block.h" // struct drive_s
#include "bregs.h" // struct bregs
#include "malloc.h" // malloc_fseg
#include "memmap.h" // add_e820
#include "output.h" // dprintf
#include "romfile.h" // romfile_findprefix
#include "stacks.h" // call16_int
#include "std/disk.h" // DISK_RET_SUCCESS
#include "string.h" // memset
#include "util.h"
#include "pci.h"

u16 spi_baseaddr VARFSEG;
static void set_spi_baseaddr(void)
{
    spi_baseaddr = pci_config_readl(0x00, 0x40) & 0x0000fffe;
    //pci_config_writel(0x00, 0x40, pci_config_readl(0x00, 0x40) | 1);
}
static u16 get_spi_baseaddr(void)
{
    return GET_GLOBAL(spi_baseaddr);
}


static u8 spi_inb(u16 offset)
{
    return inb(get_spi_baseaddr() + offset);
}
static void spi_outb(u16 offset, u8 d)
{
    outb(d, get_spi_baseaddr() + offset);
}
static void enable_cs(void)
{
    spi_outb(4, 0);
}
static void disable_cs(void)
{
    spi_outb(4, 1);
}

static void write_spi_byte(u8 n)
{
    spi_outb(0, n);
    // waiting for ODC bit set
    while((spi_inb(3) & 0x10) == 0);
}
static void write_spi_24bit_addr(unsigned long addr)
{
    write_spi_byte((u8)((addr >> 16) & 0xff));
    write_spi_byte((u8)((addr >> 8) & 0xff));
    write_spi_byte((u8)((addr) & 0xff));
}
static void spi_one_byte_cmd(u8 cmd)
{
    enable_cs();
    write_spi_byte(cmd);
    disable_cs();
}
static u8 read_spi_byte(void)
{
    spi_outb(1, 0);  // triggle SPI read
    // waiting for IDR bit set
    while((spi_inb(3) & 0x20) == 0);
    return spi_inb(1);  // read SPI data
}
static void set_spi_ctrl_reg(u8 n)
{
    spi_outb(2, n);
}
static u8 get_spi_ctrl_reg(void)
{
    return spi_inb(2);
}


int ext_mem_copy(u32 dst, u32 src, u16 size)
{
    u64 gdt[6];
    gdt[2] = GDT_DATA | GDT_LIMIT(0xfffff) | GDT_BASE(src);
    gdt[3] = GDT_DATA | GDT_LIMIT(0xfffff) | GDT_BASE(dst);

    // Call int 1587 to copy data.
    struct bregs br;
    memset(&br, 0, sizeof(br));
    br.flags = F_CF|F_IF;
    br.ah = 0x87;
    br.es = GET_SEG(SS);
    br.si = (u32)gdt;
    br.cx = size / 2;
    call16_int(0x15, &br);

    if (br.flags & F_CF) return 1;
    return 0;
}


// allocate 4KB buffer for data backup before an earsing operation
void *spi_4kbuf VARFSEG;
static int alloc_spi_4kbuf(void)
{
    spi_4kbuf = memalign_tmphigh(PAGE_SIZE, 4096);
    if (!spi_4kbuf) {
        warn_noalloc();
        return 1;
    }
    add_e820((u32)spi_4kbuf, 4096, E820_RESERVED);
    return 0;
}


static u32 lba2slba(u32 lba)
{
    return ((lba & 0x003f) >> 3) + ((lba & 0x0007) << 3) + (lba & (~0x0000003f));
}

#define set_flash_writable() u32 reg_sb_c4; u8 spi_ctrl_reg; _SET_flash_writable(&reg_sb_c4, &spi_ctrl_reg)
#define set_flash_unwritable() _SET_flash_unwritable(&reg_sb_c4, &spi_ctrl_reg)

static void _SET_flash_writable(u32* reg_sb_c4, u8* spi_ctrl_reg)
{
    *reg_sb_c4 = pci_config_readl(0x38, 0xc4);
    pci_config_writel(0x38, 0xc4, *reg_sb_c4 | 1);

    //set_spi_ckdiv and spi_afdis_bit 0x0c for EX
    *spi_ctrl_reg = get_spi_ctrl_reg();
    set_spi_ctrl_reg(((*spi_ctrl_reg | 0x20) & 0xf0) | 0x0c);
}
static void _SET_flash_unwritable(u32* reg_sb_c4, u8* spi_ctrl_reg)
{
    set_spi_ctrl_reg(*spi_ctrl_reg);
    pci_config_writel(0x38, 0xc4, *reg_sb_c4);
}


static u8 spi_flash_read_byte(u32 addr)
{
    unsigned char value;
    
    set_flash_writable();
    {
        enable_cs();
        write_spi_byte(0x03); //READ
        write_spi_24bit_addr(addr); //address
        value = read_spi_byte();
        disable_cs();
    }
    set_flash_unwritable();

    return value;
}

static void spi_flash_read_sector(u32 sector_idx, u32 buf)
{
    u32 flash_addr = sector_idx * 512;
    u8  tmp_buf[128];
    u32 tmp_buf_fptr = (u32)(MAKE_FLATPTR(GET_SEG(SS), tmp_buf));
    int i, j;

    for (i=0; i<512; i=i+128)
    {
        for (j=0; j<128; j++)
        {
            tmp_buf[j] = spi_flash_read_byte((flash_addr + i) + j);
        }
        ext_mem_copy(buf + i, tmp_buf_fptr, 128);
    }
}


static void spi_flash_write_enable(void)
{
    spi_one_byte_cmd(6);  // WREN
}
static void spi_flash_write_disable(void)
{
    spi_one_byte_cmd(4);  // WRDI
}
static void spi_flash_wait_op_finish(void)
{
    enable_cs();
    write_spi_byte(5); // RDSR
    while((read_spi_byte() & 0x01) != 0);
    disable_cs();
}

static void spi_flash_erase_block(u32 addr)
{
    set_flash_writable();
    spi_flash_write_enable();  // enable write
    {
        enable_cs();
        write_spi_byte(0x20);  // SECTOR ERASE
        write_spi_24bit_addr(addr);  // address
        disable_cs();
        
        spi_flash_wait_op_finish();
    }
    spi_flash_write_disable();
    set_flash_unwritable();
}

static void spi_flash_write_data(u32 addr, u8 *data, unsigned size)  // size must <= 256
{
    set_flash_writable();
    spi_flash_write_enable();  // enable write
    {
        enable_cs();
        write_spi_byte(0x02);  // PAGE PROGRAM
        write_spi_24bit_addr(addr);  // address
        while(size != 0)
        {
            write_spi_byte(*data);
            data++;
            size--;
        }
        disable_cs();

        spi_flash_wait_op_finish();
    }
    spi_flash_write_disable();
    set_flash_unwritable();
}

static void spi_flash_write_sector(u32 sector_idx, u32 buf)
{
    u32 flash_sector_idx = sector_idx & (~0x00000007);
    u32 flash_sector_idx_offset = sector_idx & 0x07;

    u32 spi_4kbuf_ptr = (u32)GET_GLOBAL(spi_4kbuf);
    u8  tmp_buf[128];
    u32 tmp_buf_fptr = (u32)(MAKE_FLATPTR(GET_SEG(SS), tmp_buf));
    u32 i;

    // backup the target 4k-byte block
    for (i=0; i<8; i++)
        spi_flash_read_sector(flash_sector_idx + i, spi_4kbuf_ptr + i*512);
    // update the target sector
    ext_mem_copy(spi_4kbuf_ptr + flash_sector_idx_offset*512, buf, 512);

    // erase the target block
    spi_flash_erase_block(flash_sector_idx * 512);

    // write back the target block
    for (i=0; i<4096; i=i+128)
    {
        ext_mem_copy(tmp_buf_fptr, spi_4kbuf_ptr + i, 128);
        spi_flash_write_data(flash_sector_idx*512 + i, tmp_buf, 128);
    }
}


static int spi_floppy_read(struct disk_op_s *op)
{
    u32 slba, lba = (u32)op->lba;
    u32 buf_fl = (u32)op->buf_fl;
    int i;

    for(i = 0; i < op->count; i++)
    {
            slba = lba2slba(lba);
            spi_flash_read_sector(slba, buf_fl);
            lba++;
            buf_fl = buf_fl + 512;
    }
    return DISK_RET_SUCCESS;
}

static int spi_floppy_write(struct disk_op_s *op)
{
    u32 slba, lba = (u32)op->lba;
    u32 buf_fl = (u32)op->buf_fl;
    int i;

    for(i = 0; i < op->count; i++)
    {
        slba = lba2slba(lba);
        spi_flash_write_sector(slba, buf_fl);
        lba++;
        buf_fl = buf_fl + 512;
    }
    return DISK_RET_SUCCESS;
}


static struct drive_s *init_non_std_floppy(int floppyid, struct chs_s *c)
{
    struct drive_s *drv = malloc_fseg(sizeof(*drv));
    if (!drv) {
        warn_noalloc();
        return NULL;
    }
    memset(drv, 0, sizeof(*drv));
    drv->cntl_id = floppyid;
    drv->type = DTYPE_FLOPPY;
    drv->blksize = DISK_SECTOR_SIZE;
    drv->floppy_type = 0;  // non standard type.
    drv->sectors = (u64)-1;

    memcpy(&drv->lchs, c, sizeof(struct chs_s));
    return drv;
}

int process_spifloppy_op(struct disk_op_s *op)
{
    if (!CONFIG_SPI_DISK) return 0;

    switch (op->command) {
    case CMD_READ:
        return spi_floppy_read(op);
    case CMD_WRITE:
        return spi_floppy_write(op);
    case CMD_VERIFY:
    case CMD_FORMAT:
    case CMD_RESET:
        return DISK_RET_SUCCESS;
    default:
        op->count = 0;
        return DISK_RET_EPARAM;
    }
}

void spi_floppy_setup(void)
{
    if (!CONFIG_SPI_DISK) return;

    set_spi_baseaddr();
    spi_outb(5, 0x1f);  // clear the spi error status register

    if (alloc_spi_4kbuf()) return;

    // set disk CHS size to match the 86Duino's SPI flash; note the size should reserve 512KB for BIOS area
    struct chs_s spi_chs;
    spi_chs.head = 2;
    spi_chs.cylinder = 236;
    spi_chs.sector = 32;

    // setup disk driver
    struct drive_s *drv_spi = init_non_std_floppy(0, &spi_chs);
    if (!drv_spi) return;
    drv_spi->type = DTYPE_SPIDISK;
    char *desc = znprintf(MAXDESCSIZE, "86Duino SPI flash disk");
    boot_add_floppy(drv_spi, desc, bootprio_find_spidisk());
}
