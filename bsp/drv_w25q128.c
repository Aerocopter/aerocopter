#include "stm32f4xx.h"
#include "bsp_spi.h"
#include "drv_w25q128.h"
#include <string.h>

/* 片选控制宏，直接调用 bsp 层函数 */
#define W25Q128_CS_LOW()    bsp_spi1_cs_low()
#define W25Q128_CS_HIGH()   bsp_spi1_cs_high()

int w25q128_init(void)
{

    /* 读取 JEDEC ID 检查通信是否正常 */
    uint32_t id = w25q128_read_jedec_id();
    if (id != 0xEF4018)   // W25Q128 的 JEDEC ID 为 0xEF4018
        return -1;
    else
        return 0;
}

uint32_t w25q128_read_jedec_id(void)
{
    uint8_t tx[4] = {CMD_READ_JEDEC_ID, 0, 0, 0};
    uint8_t rx[4];

    W25Q128_CS_LOW();
    bsp_spi1_transfer_buffer(tx, rx, 4);
    W25Q128_CS_HIGH();

    // 返回第1~3字节，即 (rx[1]<<16) | (rx[2]<<8) | rx[3]
    return ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
}

/* 读状态寄存器1 */
uint8_t w25q128_read_status_reg(void)
{
    uint8_t tx[2] = {CMD_READ_STATUS_REG1, 0xFF};
    uint8_t rx[2];

    W25Q128_CS_LOW();
    bsp_spi1_transfer_buffer(tx, rx, 2);
    W25Q128_CS_HIGH();

    return rx[1];
}

/* 等待 BUSY 位清除 */
void w25q128_wait_busy(void)
{
    while (w25q128_read_status_reg() & SR1_BUSY);
}

/* 写使能 */
void w25q128_write_enable(void)
{
    uint8_t cmd = CMD_WRITE_ENABLE;

    W25Q128_CS_LOW();
    bsp_spi1_transfer_buffer(&cmd, NULL, 1);
    W25Q128_CS_HIGH();
}

/* 扇区擦除（4KB），addr 为任意扇区起始地址 */
void w25q128_sector_erase(uint32_t addr)
{
    uint8_t tx[4] = {CMD_SECTOR_ERASE_4K, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF};

    w25q128_write_enable();
    w25q128_wait_busy();   // 确保之前操作完成

    W25Q128_CS_LOW();
    bsp_spi1_transfer_buffer(tx, NULL, 4);
    W25Q128_CS_HIGH();

    w25q128_wait_busy();   // 等待擦除完成
}

/* 64KB 块擦除 */
void w25q128_block_erase_64k(uint32_t addr)
{
    uint8_t tx[4] = {CMD_BLOCK_ERASE_64K, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF};

    w25q128_write_enable();
    w25q128_wait_busy();

    W25Q128_CS_LOW();
    bsp_spi1_transfer_buffer(tx, NULL, 4);
    W25Q128_CS_HIGH();

    w25q128_wait_busy();
}

/* 全片擦除 */
void w25q128_chip_erase(void)
{
    uint8_t cmd = CMD_CHIP_ERASE;

    w25q128_write_enable();
    w25q128_wait_busy();

    W25Q128_CS_LOW();
    bsp_spi1_transfer_buffer(&cmd, NULL, 1);
    W25Q128_CS_HIGH();

    w25q128_wait_busy();   // 全片擦除耗时较长（数十秒）
}

/* 页编程（一页最多256字节），addr 必须是页对齐的（即低8位为0） */
void w25q128_page_program(uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (len > 256) len = 256;   // 单页最大256字节

    uint8_t tx[4 + 256];
    tx[0] = CMD_PAGE_PROGRAM;
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >> 8) & 0xFF;
    tx[3] = addr & 0xFF;
    memcpy(&tx[4], data, len);

    w25q128_write_enable();
    w25q128_wait_busy();

    W25Q128_CS_LOW();
    bsp_spi1_transfer_buffer(tx, NULL, 4 + len);
    W25Q128_CS_HIGH();

    w25q128_wait_busy();
}

/* 读取任意长度数据（使用普通读命令 0x03） */
void w25q128_read_data(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t tx[4] = {CMD_READ_DATA, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF};

    W25Q128_CS_LOW();
    bsp_spi1_transfer_buffer(tx, NULL, 4);          // 发送命令+地址

    /* 连续读取 len 字节，此时 MOSI 持续发送 0xFF 以提供时钟 */
    uint8_t dummy = 0xFF;
    for (uint32_t i = 0; i < len; i++)
    {
        buf[i] = bsp_spi1_transfer(dummy);
    }
    // 或者使用批量传输函数：
    // bsp_spi1_transfer_buffer(NULL, buf, len);   // 需要修改 bsp_spi1_transfer_buffer 支持 NULL tx 的情况

    W25Q128_CS_HIGH();
}
