#include "oled.h"

#include "main.h"
#include "oled_gfx.h"

#define OLED_SCL_HIGH()  (OLED_SCL_GPIO_Port->BSRR = (uint32_t)OLED_SCL_Pin)
#define OLED_SCL_LOW()   (OLED_SCL_GPIO_Port->BSRR = ((uint32_t)OLED_SCL_Pin) << 16)
#define OLED_SDA_HIGH()  (OLED_SDA_GPIO_Port->BSRR = (uint32_t)OLED_SDA_Pin)
#define OLED_SDA_LOW()   (OLED_SDA_GPIO_Port->BSRR = ((uint32_t)OLED_SDA_Pin) << 16)
#define OLED_DC_CMD()    (OLED_DC_GPIO_Port->BSRR = ((uint32_t)OLED_DC_Pin) << 16)
#define OLED_DC_DATA()   (OLED_DC_GPIO_Port->BSRR = (uint32_t)OLED_DC_Pin)
#define OLED_RES_LOW()   (OLED_RES_GPIO_Port->BSRR = ((uint32_t)OLED_RES_Pin) << 16)
#define OLED_RES_HIGH()  (OLED_RES_GPIO_Port->BSRR = (uint32_t)OLED_RES_Pin)

static void oled_spi_byte(uint8_t value)
{
  uint8_t i;

  for (i = 0; i < 8U; i++)
  {
    OLED_SCL_LOW();
    if ((value & 0x80U) != 0U)
    {
      OLED_SDA_HIGH();
    }
    else
    {
      OLED_SDA_LOW();
    }
    value = (uint8_t)(value << 1);
    OLED_SCL_HIGH();
  }
}

static void oled_write_cmd(uint8_t cmd)
{
  OLED_DC_CMD();
  oled_spi_byte(cmd);
}

static void oled_write_data(const uint8_t *data, uint16_t len)
{
  OLED_DC_DATA();
  while (len-- > 0U)
  {
    oled_spi_byte(*data++);
  }
}

static void oled_delay(volatile uint32_t cycles)
{
  while (cycles-- > 0U)
  {
  }
}

static void oled_hw_reset(void)
{
  OLED_RES_LOW();
  oled_delay(40000U);   /* ≈ 1 ms @168 MHz */
  OLED_RES_HIGH();
  oled_delay(400000U);  /* ≈ 10 ms，等电荷泵稳定 */
}

void oled_init(void)
{
  oled_hw_reset();

  oled_write_cmd(0xAE);        /* 关显示 */
  oled_write_cmd(0xD5); oled_write_cmd(0x80);
  oled_write_cmd(0xA8); oled_write_cmd(0x3F);
  oled_write_cmd(0xD3); oled_write_cmd(0x00);
  oled_write_cmd(0x40);        /* 起始行 0 */
  oled_write_cmd(0x8D); oled_write_cmd(0x14);  /* 电荷泵 */
  oled_write_cmd(0x20); oled_write_cmd(0x00);  /* 水平寻址 */
#if OLED_MIRROR_X
  oled_write_cmd(0xA1);
#else
  oled_write_cmd(0xA0);
#endif
#if OLED_MIRROR_Y
  oled_write_cmd(0xC8);
#else
  oled_write_cmd(0xC0);
#endif
  oled_write_cmd(0xDA); oled_write_cmd(0x12);
  oled_write_cmd(0x81); oled_write_cmd(0xCF);
  oled_write_cmd(0xD9); oled_write_cmd(0xF1);
  oled_write_cmd(0xDB); oled_write_cmd(0x40);
  oled_write_cmd(0xA4);
  oled_write_cmd(0xA6);
  oled_write_cmd(0x2E);        /* 关滚动 */
  oled_write_cmd(0xAF);        /* 开显示 */

  oled_gfx_clear();
  oled_flush();
}

void oled_flush(void)
{
  oled_write_cmd(0x21); oled_write_cmd(0x00); oled_write_cmd(0x7F);
  oled_write_cmd(0x22); oled_write_cmd(0x00); oled_write_cmd(0x07);
  oled_write_data(oled_gfx_buffer(), (uint16_t)(OLED_WIDTH * OLED_HEIGHT / 8U));
}
