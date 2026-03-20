#ifndef _DRV_W25Q128_H_
#define _DRV_W25Q128_H_

#include <stdint.h>

/* W25Q128 容量 16MB = 128Mbit */
#define W25Q128_FLASH_SIZE          (16 * 1024 * 1024)  // 16M字节
#define W25Q128_SECTOR_SIZE         4096                 // 4K 扇区
#define W25Q128_BLOCK_SIZE          65536                // 64K 块

/* 指令集 */
#define CMD_WRITE_ENABLE            0x06
#define CMD_VOLATILE_SR_WRITE_EN    0x50
#define CMD_WRITE_DISABLE           0x04
#define CMD_READ_STATUS_REG1        0x05
#define CMD_READ_STATUS_REG2        0x35
#define CMD_WRITE_STATUS_REG        0x01
#define CMD_PAGE_PROGRAM            0x02
#define CMD_QUAD_PAGE_PROGRAM       0x32
#define CMD_BLOCK_ERASE_64K         0xD8
#define CMD_BLOCK_ERASE_32K         0x52
#define CMD_SECTOR_ERASE_4K         0x20
#define CMD_CHIP_ERASE              0xC7
#define CMD_ERASE_SUSPEND            0x75
#define CMD_ERASE_RESUME             0x7A
#define CMD_POWER_DOWN               0xB9
#define CMD_HIGH_PERFORM_MODE        0xA3
#define CMD_READ_DATA                0x03
#define CMD_FAST_READ                0x0B
#define CMD_FAST_READ_DUAL_OUT       0x3B
#define CMD_FAST_READ_DUAL_IO        0xBB
#define CMD_FAST_READ_QUAD_OUT       0x6B
#define CMD_FAST_READ_QUAD_IO        0xEB
#define CMD_WORD_READ_QUAD_IO        0xE7
#define CMD_READ_JEDEC_ID            0x9F
#define CMD_READ_UNIQUE_ID           0x4B
#define CMD_READ_SFDP_REG            0x5A
#define CMD_ERASE_SECURITY_REG       0x44
#define CMD_PROGRAM_SECURITY_REG     0x42
#define CMD_READ_SECURITY_REG        0x48
#define CMD_ENABLE_RESET             0x66
#define CMD_RESET_DEVICE             0x99

/* 状态寄存器位 */
#define SR1_BUSY                     (1 << 0)
#define SR1_WEL                      (1 << 1)
#define SR1_BP0                      (1 << 2)
#define SR1_BP1                      (1 << 3)
#define SR1_BP2                      (1 << 4)
#define SR1_BP3                      (1 << 5)
#define SR1_TB                       (1 << 6)
#define SR1_SRP                      (1 << 7)

/* 函数声明 */
int w25q128_init(void);
uint32_t w25q128_read_jedec_id(void);
uint8_t w25q128_read_status_reg(void);
void w25q128_wait_busy(void);
void w25q128_write_enable(void);
void w25q128_sector_erase(uint32_t addr);
void w25q128_block_erase_64k(uint32_t addr);
void w25q128_chip_erase(void);
void w25q128_page_program(uint32_t addr, const uint8_t *data, uint16_t len);
void w25q128_read_data(uint32_t addr, uint8_t *buf, uint32_t len);

#endif
