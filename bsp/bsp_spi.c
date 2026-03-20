#include "stm32f4xx.h"
#include "bsp_spi.h"


/* =========================================================
   SPI1  (W25Q128)
   SCK  = PA5 (AF5)
   MISO = PA6 (AF5)
   MOSI = PA7 (AF5)
   CS   = PA4
   ========================================================= */

/* ================= SPI1 CS Control ================= */

void bsp_spi1_cs_low(void)
{
    GPIOA->BSRR = (1 << (4 + 16));   // PA4 LOW
}

void bsp_spi1_cs_high(void)
{
    GPIOA->BSRR = (1 << 4);          // PA4 HIGH
}

/* ================= SPI1 Init ================= */

void bsp_spi1_init(bsp_spi_mode_t mode)
{
    /* Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;    // SPI1 clock (APB2)

    /* PA5/6/7 AF5 */
    GPIOA->MODER &= ~(0xFF << (5*2));      // clear PA5,6,7 mode bits (bits 10-15)
    GPIOA->MODER |=  (0xAA << (5*2));      // set all three to alternate function (10)

    GPIOA->AFR[0] &= ~(0xFFF << 20);       // clear AF bits for PA5 (bit20-23), PA6 (24-27), PA7 (28-31)
    GPIOA->AFR[0] |=  (5 << 20) | (5 << 24) | (5 << 28);  // AF5 for all

    /* PA4 CS as output */
    GPIOA->MODER &= ~(3 << (4*2));         // clear PA4 mode bits
    GPIOA->MODER |=  (1 << (4*2));         // set as output (01)
    GPIOA->OSPEEDR |= (3 << (4*2));        // high speed

    bsp_spi1_cs_high();                    // default CS high

    /* SPI1 configuration */
    SPI1->CR1 = 0;
    SPI1->CR1 |= SPI_CR1_MSTR;             // master mode
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // software slave management
    SPI1->CR1 |= SPI_CR1_BR_1;              // prescaler 8 (adjust as needed)

    /* Clock polarity/phase */
    switch(mode)
    {
        case SPI_MODE_0:
            break;
        case SPI_MODE_1:
            SPI1->CR1 |= SPI_CR1_CPHA;
            break;
        case SPI_MODE_2:
            SPI1->CR1 |= SPI_CR1_CPOL;
            break;
        case SPI_MODE_3:
            SPI1->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;
            break;
    }

    SPI1->CR1 |= SPI_CR1_SPE;               // enable SPI
}

/* ================= SPI1 Single Byte ================= */

uint8_t bsp_spi1_transfer(uint8_t data)
{
    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = data;

    while(!(SPI1->SR & SPI_SR_RXNE));
    return SPI1->DR;
}

/* ================= SPI1 Burst Transfer ================= */

void bsp_spi1_transfer_buffer(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++)
    {
        uint8_t send = 0xFF;
        if(tx) send = tx[i];

        uint8_t recv = bsp_spi1_transfer(send);

        if(rx) rx[i] = recv;
    }
}

/* ================= CS Control ================= */

void bsp_spi2_cs_low(void)
{
    GPIOB->BSRR = (1 << (12 + 16));
}

void bsp_spi2_cs_high(void)
{
    GPIOB->BSRR = (1 << 12);
}

/* ================= SPI Init ================= */

void bsp_spi2_init(bsp_spi_mode_t mode)
{
    /* Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* PB13/14/15 AF5 */
    GPIOB->MODER &= ~(0xFF << (13*2));
    GPIOB->MODER |=  (0xAA << (13*2));

    GPIOB->AFR[1] &= ~(0xFFF << 20);
    GPIOB->AFR[1] |=  (5 << 20) | (5 << 24) | (5 << 28);

    /* PB12 CS as output */
    GPIOB->MODER &= ~(3 << (12*2));
    GPIOB->MODER |=  (1 << (12*2));
    GPIOB->OSPEEDR |= (3 << (12*2));
    bsp_spi2_cs_high();

    SPI2->CR1 = 0;
    SPI2->CR1 |= SPI_CR1_MSTR;
    SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    SPI2->CR1 |= SPI_CR1_BR_1;  // prescaler 8

    /* Mode setting */
    switch(mode)
    {
        case SPI_MODE_0:
            break;

        case SPI_MODE_1:
            SPI2->CR1 |= SPI_CR1_CPHA;
            break;

        case SPI_MODE_2:
            SPI2->CR1 |= SPI_CR1_CPOL;
            break;

        case SPI_MODE_3:
            SPI2->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;
            break;
    }

    SPI2->CR1 |= SPI_CR1_SPE;
}

/* ================= Single Byte ================= */

uint8_t bsp_spi2_transfer(uint8_t data)
{
    while(!(SPI2->SR & SPI_SR_TXE));
    SPI2->DR = data;

    while(!(SPI2->SR & SPI_SR_RXNE));
    return SPI2->DR;
}

/* ================= Burst Transfer ================= */

void bsp_spi2_transfer_buffer(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++)
    {
        uint8_t send = 0xFF;
        if(tx) send = tx[i];

        uint8_t recv = bsp_spi2_transfer(send);

        if(rx) rx[i] = recv;
    }
}

/* =========================================================
   SPI3  (ICM-42688)
   SCK  = PC10
   MISO = PC11
   MOSI = PC12
   CS   = PA15
   INT  = PC2
   ========================================================= */

/* ================= SPI3 CS ================= */

void bsp_spi3_cs_low(void)
{
    GPIOA->BSRR = (1 << (15 + 16));   // PA15 LOW
}

void bsp_spi3_cs_high(void)
{
    GPIOA->BSRR = (1 << 15);          // PA15 HIGH
}

/* ================= SPI3 Init ================= */

void bsp_spi3_init(bsp_spi_mode_t mode)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

    /* PC10/11/12 AF6 */
    GPIOC->MODER &= ~(0xFF << (10*2));
    GPIOC->MODER |=  (0xAA << (10*2));

    GPIOC->AFR[1] &= ~(0xFFF << 8);
    GPIOC->AFR[1] |=  (6 << 8) | (6 << 12) | (6 << 16);

    /* PA15 CS output */
    GPIOA->MODER &= ~(3 << (15*2));
    GPIOA->MODER |=  (1 << (15*2));
    GPIOA->OSPEEDR |= (3 << (15*2));

    bsp_spi3_cs_high();

    SPI3->CR1 = 0;
    SPI3->CR1 |= SPI_CR1_MSTR;
    SPI3->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    SPI3->CR1 |= SPI_CR1_BR_1;   // Prescaler 8 (~5MHz)

    /* ICM-42688 必须 Mode 0 */
    if(mode == SPI_MODE_1) SPI3->CR1 |= SPI_CR1_CPHA;
    if(mode == SPI_MODE_2) SPI3->CR1 |= SPI_CR1_CPOL;
    if(mode == SPI_MODE_3) SPI3->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;

    SPI3->CR1 |= SPI_CR1_SPE;
}

/* ================= SPI3 Transfer ================= */

uint8_t bsp_spi3_transfer(uint8_t data)
{
    while(!(SPI3->SR & SPI_SR_TXE));
    SPI3->DR = data;

    while(!(SPI3->SR & SPI_SR_RXNE));
    return SPI3->DR;
}

void bsp_spi3_transfer_buffer(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    for(uint16_t i=0;i<len;i++)
    {
        uint8_t send = tx ? tx[i] : 0xFF;
        uint8_t recv = bsp_spi3_transfer(send);
        if(rx) rx[i] = recv;
    }
}

