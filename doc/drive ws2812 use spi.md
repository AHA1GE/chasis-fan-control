通过对 ws2812 时序的介绍，完成一个 ws2812 控制需要发送 24bit GRB 的颜色数据，比特率为 800Kbps。

为了可以使用 SPI 模拟 ws2812 的时序，需要将 GRB 颜色数据中每 1 个 bit 膨胀为 4 个 bit，即：

1 表示为：1110
0 表示为：1000
这样 0 bit 中高电平约占 1/4，低电平约占 3/4。1 bit 中高电平约占 3/4，低电平约占 1/4。符合通讯协议。

此时，驱动一个 ws2812，SPI MOSI 引脚需要发送 4 x 24 bits = 12 Bytes。SPI 的时钟频率设置为 800 x 4 = 3.2MHz 左右。

CH32V003 主频设置为 48MHz， SPI 设置 16 分频，为 3MHz，在误差范围内，实测可以正常驱动WS2812。

```C
/**
 * @brief 
 * 
 * @param ws2812_bit_buffer 
 * @param ws2812_byte_buffer 
 * 
 *  1bit 膨胀位 4bit
 *  1：1110
 *  0：1000
 */
void ws2812_set_grb(ws2812_bit_buffer_t *ws2812_bit_buffer, ws2812_byte_buffer_t *ws2812_byte_buffer)
{
    ws2812_byte_buffer_t ws2812_color_data = 
    {
        .green = ws2812_byte_buffer->green,
        .red   = ws2812_byte_buffer->red,
        .blue  = ws2812_byte_buffer->blue
    };
 
    for(uint8_t i = 0; i<4; i++)
    {
        ws2812_bit_buffer->green >>= 8;
        ws2812_bit_buffer->red   >>= 8;
        ws2812_bit_buffer->blue  >>= 8;
        /**
         * @brief 
         * 每 2 bit 的 RGB 数据膨胀为 1byte 的 spi 数据 
         * 每个 byte 中，第 8bit 和第 3bit 位固定为 1，第 4bit 和第 0bit 位固定为 0，剩余 bit 根据颜色值设定
         */
        ws2812_bit_buffer->green |= ( 0x88 | ((ws2812_color_data.green & 0x80)>>1) | ((ws2812_color_data.green & 0x80)>>2)| \
                                             ((ws2812_color_data.green & 0x40)>>4) | ((ws2812_color_data.green & 0x40)>>5) )<<24;
        ws2812_bit_buffer->red   |= ( 0x88 | ((ws2812_color_data.red & 0x80)>>1)   | ((ws2812_color_data.red & 0x80)>>2)| \
                                             ((ws2812_color_data.red & 0x40)>>4)   | ((ws2812_color_data.red & 0x40)>>5) )<<24;
        ws2812_bit_buffer->blue  |= ( 0x88 | ((ws2812_color_data.blue & 0x80)>>1)  | ((ws2812_color_data.blue & 0x80)>>2)| \
                                             ((ws2812_color_data.blue & 0x40)>>4)  | ((ws2812_color_data.blue & 0x40)>>5) )<<24;
 
        ws2812_color_data.green <<= 2;
        ws2812_color_data.red   <<= 2;
        ws2812_color_data.blue  <<= 2;                              
    } 
}
```