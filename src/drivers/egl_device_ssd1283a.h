//============================================================================

//============================================================================

#ifndef EGL_DEVICE_SSD1283A_H
#define EGL_DEVICE_SSD1283A_H
//----------------------------------------------------------------------------


//============================================================================
// interface
//============================================================================
// external
#include "../egl_device.h"
#include <SPI.h>
#include <HardwareSerial.h>
#include "hardware/dma.h"
#include "hardware/irq.h"
#include <SSD1283A.h> //Hardware-specific library
#include <LCDWIKI_GUI.h> //Core graphics library
EGL_NAMESPACE_BEGIN

// new
class graphics_device_ssd1283a;
//----------------------------------------------------------------------------

#define SPI_CLOCK_TRANSFER 16000000
// #define SPI_CLOCK_TRANSFER 1000000
#define SPI_CLOCK_DISPLAY 27000000

//============================================================================
// graphics_device_ssd1283a
//============================================================================
class graphics_device_ssd1283a: public graphics_device<graphics_device_ssd1283a>
{
public:
  //
  SSD1283A* display_lcd;
  uint32_t pin_cs;
  uint32_t pin_dc;

  // properties
  enum {supports_dma=true};
  // enum {supports_dma=false};
  //--------------------------------------------------------------------------

  // device properties
  enum {fb_width=130, fb_height=130};
  enum {fb_format=pixfmt_b5g6r5};
  typedef pixel<e_pixel_format(fb_format)> fb_format_t;
  //--------------------------------------------------------------------------

	// construction
  graphics_device_ssd1283a();
	graphics_device_ssd1283a(uint32_t pin_cs_, uint32_t pin_dc_, uint32_t pin_sclk_, uint32_t pin_mosi_, uint32_t pin_miso_, uint32_t pin_reset_=0xff);
  ~graphics_device_ssd1283a();
  void set_spi(uint8_t spi_, uint32_t spi_clock_=SPI_CLOCK_TRANSFER);
  void init(uint32_t pin_cs_, uint32_t pin_dc_, uint32_t pin_sclk_, uint32_t pin_mosi_, uint32_t pin_miso_, uint32_t pin_reset_=0xff);
  void init_rasterizer(const rasterizer_cfg&, const rasterizer_tiling_cfg&, const rasterizer_vertex_cache_cfg&);
  void init_dma(rasterizer_data_transfer*, uint8_t num_transfers_, fb_format_t *dma_buffer_, size_t dma_buffer_size_);
  //--------------------------------------------------------------------------

  // immediate rendering interface
  void flush_dma();
  template<class IPShader> EGL_INLINE void fast_draw_hline(uint16_t x_, uint16_t y_, uint16_t width_, const IPShader&);
  template<class IPShader> EGL_INLINE void fast_draw_vline(uint16_t x_, uint16_t y_, uint16_t height_, const IPShader&);
  template<class IPShader> EGL_INLINE void fast_draw_rect(uint16_t x_, uint16_t y_, uint16_t width_, uint16_t height_, const IPShader&);
  //--------------------------------------------------------------------------

private:
  graphics_device_ssd1283a(const graphics_device_ssd1283a&); // not implemented
  void operator=(const graphics_device_ssd1283a&); // not implemented
  virtual void submit_tile(uint8_t tx_, uint8_t ty_, const vec2u16 &reg_min_, const vec2u16 &reg_end_, uint16_t thread_idx_);
#if EGL_PLATFORM_NUM_DMA>0 && EGL_BUILDOP_DMA_TRANSFER==1
  static void dma_interrupt();
  void dma_interrupt_impl();
  bool start_dma_transfer(const rasterizer_data_transfer&);
  // void dma_interrupt_handler();
#endif
  //--------------------------------------------------------------------------

  // HW commands
  EGL_INLINE void begin_spi_transition();
  EGL_INLINE void end_spi_transition();
  EGL_INLINE void set_window_address(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
  EGL_INLINE void update_tcr_data8();
  EGL_INLINE void update_tcr_data16();
  EGL_INLINE void set_data_window(uint16_t x0_, uint16_t y0_, uint16_t x1_, uint16_t y1_);
  EGL_INLINE void writecmd_cont(uint8_t);
  EGL_INLINE void writecmd_last(uint8_t);
  EGL_INLINE void writedata8_cont(uint8_t);
  EGL_INLINE void writedata8_last(uint8_t);
  EGL_INLINE void writedata16_cont(uint16_t);
  EGL_INLINE void writedata16_cont(uint16_t data_, uint16_t n_);
  EGL_INLINE void writedata16_last(uint16_t);
  EGL_INLINE void wait_spi_fifo();
  EGL_INLINE void wait_spi_complete();
#ifdef EGL_PLATFORM_TEENSY4X
  EGL_INLINE void update_tcr(uint32_t tcr_state_, bool state_transit_);
  EGL_INLINE void direct_write_lo(volatile uint32_t *base_, uint32_t mask_) {*(base_+34)=mask_;}
  EGL_INLINE void direct_write_hi(volatile uint32_t *base_, uint32_t mask_) {*(base_+35)=mask_;}
#endif
  //--------------------------------------------------------------------------

  static graphics_device_ssd1283a *s_active_dev;
  SPIClass *m_spi;
  uint32_t m_spi_clock;
  uint8_t m_spi_chl;
#ifdef EGL_PLATFORM_TEENSY4X
  IMXRT_LPSPI_t *m_spi_imxrt;
  uint8_t m_num_pending_rx;
  volatile uint32_t *m_cs_port;
  uint32_t m_cs_mask;
  volatile uint32_t *m_dc_port;
  uint32_t m_dc_mask;
#elif defined(KINETISK)
  KINETISK_SPI_t *m_spi_kinetisk;
  uint32_t m_spi_kinetisk_fifo_wait;
  uint32_t m_pcs_command;
  uint32_t m_pcs_data;
  volatile uint8_t *m_cs_port;
  uint8_t m_cs_mask;
#else
  volatile uint8_t *m_cs_port;
  uint8_t m_cs_mask;
  volatile uint8_t *m_dc_port;
  uint8_t m_dc_mask;
#endif
  fb_format_t *m_tile_rt0;
  rasterizer_tile_size_t m_tile_width;
  rasterizer_tile_size_t m_tile_height;
#if EGL_PLATFORM_NUM_DMA>0 && EGL_BUILDOP_DMA_TRANSFER==1 && ARDUINO_ARCH_RP2040
  // DMA
  uint m_dma_chl;
  dma_channel_config dma_config;
  uint8_t *m_dma_display_data;
  rasterizer_data_transfer *m_dma_transfers;
  uint8_t m_dma_transfers_size;
  volatile uint8_t m_dma_transfers_rpos;
  uint8_t m_dma_transfers_wpos;
  fb_format_t *m_dma_buffer;
  size_t m_dma_buffer_size;
  volatile size_t m_dma_buffer_rpos;
  size_t m_dma_buffer_wpos;
#elif EGL_PLATFORM_NUM_DMA>0 && EGL_BUILDOP_DMA_TRANSFER==1
  // DMA
  DMAChannel m_dma_chl;
  DMASetting m_dma_settings;
  rasterizer_data_transfer *m_dma_transfers;
  uint8_t m_dma_transfers_size;
  volatile uint8_t m_dma_transfers_rpos;
  uint8_t m_dma_transfers_wpos;
  fb_format_t *m_dma_buffer;
  size_t m_dma_buffer_size;
  volatile size_t m_dma_buffer_rpos;
  size_t m_dma_buffer_wpos;
#endif
};
//----------------------------------------------------------------------------




//============================================================================
//============================================================================
// inline & template implementations
//============================================================================
//============================================================================


//============================================================================
// graphics_device_ssd1283a
//============================================================================
template<class IPShader>
// ??? take a look - implement like rect
void graphics_device_ssd1283a::fast_draw_hline(uint16_t x_, uint16_t y_, uint16_t width_, const IPShader &ips_)
{
  // // draw horizontal line
  // // begin_spi_transition();
	// uint16_t x_max=x_+width_-1;
	// set_data_window(x_, y_, x_max, y_);
	// writecmd_cont(ILI9341_RAMWR);
  // update_tcr_data16();
  // fb_format_t res;
  // while(true)
  // {
  //   ips_.exec(res, x_, y_);
  //   if(x_==x_max)
  //   {
  //     writedata16_last(res.v);
  //     break;
  //   }
  //   writedata16_cont(res.v);
  //   ++x_;
  // }
  // end_spi_transition();

  // this wont work, because it draws a hline of a single color
  // display_lcd->drawFastHLine(x_, y_, width_, )

  fb_format_t res;
  for (uint16_t x = x_; x <= x_ + width_ -1; ++x) {
    ips_.exec(res, x, y_);
    display_lcd->drawPixel(x, y_, res.v); // Draw each pixel with the computed color
  }
  EGL_LOG("Hline draw!\r\n"); // ????



}
//----

// ??? take a look - implement like rect
template<class IPShader>
void graphics_device_ssd1283a::fast_draw_vline(uint16_t x_, uint16_t y_, uint16_t height_, const IPShader &ips_)
{
  // // draw vertical line
  // begin_spi_transition();
  // uint16_t y_max=y_+height_-1;
  // set_data_window(x_, y_, x_, y_max);
  // writecmd_cont(ILI9341_RAMWR);
  // update_tcr_data16();
  // fb_format_t res;
  // while(true)
  // {
  //   ips_.exec(res, x_, y_);
  //   if(y_==y_max)
  //   {
  //     writedata16_last(res.v);
  //     break;
  //   }
  //   writedata16_cont(res.v);
  //   ++y_;
  // }
  // end_spi_transition();


  fb_format_t res;
  for (uint16_t y = y_; y <= y_ + height_ -1; ++y) {
    ips_.exec(res, x_, y);
    display_lcd->drawPixel(x_, y, res.v); // Draw each pixel with the computed color
  }
  EGL_LOG("Vline draw!\r\n"); // ????
}
//----

template<class IPShader>
void graphics_device_ssd1283a::fast_draw_rect(uint16_t x_, uint16_t y_, uint16_t width_, uint16_t height_, const IPShader &ips_)
{
  // // draw rectangle
  // begin_spi_transition();
  // uint16_t x_max=x_+width_-1, y_max=y_+height_-1;
  // set_data_window(x_, y_, x_max, y_max);
  // writecmd_cont(ILI9341_RAMWR);
  // update_tcr_data16();
  // fb_format_t res;
  // while(true)
  // {
  //   uint16_t x=x_;
  //   while(true)
  //   {
  //     ips_.exec(res, x, y_);
  //     if(x==x_max)
  //       break;
  //     writedata16_cont(res.v);
  //     ++x;
  //   }
  //   if(y_==y_max)
  //   {
  //     writedata16_last(res.v);
  //     break;
  //   }
  //   writedata16_cont(res.v);
  //   ++y_;
  // }
  // end_spi_transition();

  // fb_format_t res;
  // for (uint16_t y = y_; y <= y_ + height_ -1; ++y) {
  //   for (uint16_t x = x_; x <= x_ + width_ -1; ++x) {
  //     ips_.exec(res, x, y);
  //     display_lcd->drawPixel(x, y, res.v); // Draw each pixel with the computed color
  //   }
  // }


  graphics_device_ssd1283a::begin_spi_transition();
  graphics_device_ssd1283a::set_window_address(x_, y_, x_ + width_ - 1, y_ + height_ - 1);
  graphics_device_ssd1283a::writecmd_cont(0x22);

  fb_format_t res;
  for (uint16_t y = y_; y <= y_ + height_ -1; ++y) {
    for (uint16_t x = x_; x <= x_ + width_ -1; ++x) {
      ips_.exec(res, x, y);
      graphics_device_ssd1283a::writedata16_cont(res.v);
    }
  }
  graphics_device_ssd1283a::end_spi_transition();
}
//----------------------------------------------------------------------------

void graphics_device_ssd1283a::begin_spi_transition()
{
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DISPLAY, MSBFIRST, SPI_MODE0));
  if (pin_cs >= 0) digitalWrite(pin_cs, LOW);
}
//----

void graphics_device_ssd1283a::end_spi_transition()
{
  if (pin_cs >= 0) digitalWrite(pin_cs, HIGH);
  SPI.endTransaction();
}
//----

void graphics_device_ssd1283a::set_window_address(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
  switch (display_lcd->getRotation())
  {
    case 0:
      graphics_device_ssd1283a::writecmd_cont(0x44);
      graphics_device_ssd1283a::writedata8_cont(x2 + 2);
      graphics_device_ssd1283a::writedata8_cont(x1 + 2);
      graphics_device_ssd1283a::writecmd_cont(0x45);
      graphics_device_ssd1283a::writedata8_cont(y2 + 2);
      graphics_device_ssd1283a::writedata8_cont(y1 + 2);
      graphics_device_ssd1283a::writecmd_cont(0x21);
      graphics_device_ssd1283a::writedata8_cont(y1 + 2);
      graphics_device_ssd1283a::writedata8_cont(x1 + 2);
      //Serial.print("_setWindowAddress "); Serial.print(y1 + 2); Serial.print(", "); Serial.println(x1 + 2);
      break;
    case 1:
      graphics_device_ssd1283a::writecmd_cont(0x44);
      graphics_device_ssd1283a::writedata8_cont(fb_height - y1 + 1);
      graphics_device_ssd1283a::writedata8_cont(fb_height - y2 + 1);
      graphics_device_ssd1283a::writecmd_cont(0x45);
      graphics_device_ssd1283a::writedata8_cont(fb_width - x1 - 1);
      graphics_device_ssd1283a::writedata8_cont(fb_width - x2 - 1);
      graphics_device_ssd1283a::writecmd_cont(0x21);
      graphics_device_ssd1283a::writedata8_cont(fb_width - x1 - 1);
      graphics_device_ssd1283a::writedata8_cont(fb_height - y1 + 1);
      //Serial.print("_setWindowAddress "); Serial.print(WIDTH - x1 - 1); Serial.print(", "); Serial.println(HEIGHT - y1 + 1);
      break;
    case 2:
      graphics_device_ssd1283a::writecmd_cont(0x44);
      graphics_device_ssd1283a::writedata8_cont(fb_width - x1 + 1);
      graphics_device_ssd1283a::writedata8_cont(fb_width - x2 + 1);
      graphics_device_ssd1283a::writecmd_cont(0x45);
      graphics_device_ssd1283a::writedata8_cont(fb_height - y1 + 1);
      graphics_device_ssd1283a::writedata8_cont(fb_height - y2 + 1);
      graphics_device_ssd1283a::writecmd_cont(0x21);
      graphics_device_ssd1283a::writedata8_cont(fb_height - y1 + 1);
      graphics_device_ssd1283a::writedata8_cont(fb_width - x1 + 1);
      //Serial.print("_setWindowAddress "); Serial.print(HEIGHT - y1 + 1); Serial.print(", "); Serial.println(WIDTH - x1 + 1);
      break;
    case 3:
      graphics_device_ssd1283a::writecmd_cont(0x44);
      graphics_device_ssd1283a::writedata8_cont(y2 + 2);
      graphics_device_ssd1283a::writedata8_cont(y1 + 2);
      graphics_device_ssd1283a::writecmd_cont(0x45);
      graphics_device_ssd1283a::writedata8_cont(x2);
      graphics_device_ssd1283a::writedata8_cont(x1);
      graphics_device_ssd1283a::writecmd_cont(0x21);
      graphics_device_ssd1283a::writedata8_cont(x1);
      graphics_device_ssd1283a::writedata8_cont(y1 + 2);
      //Serial.print("_setWindowAddress "); Serial.print(x1); Serial.print(", "); Serial.println(y1 + 2);
      break;
  }
  graphics_device_ssd1283a::writecmd_cont(0x22);
}
//----

void graphics_device_ssd1283a::update_tcr_data8()
{
#ifdef EGL_PLATFORM_TEENSY4X
  update_tcr(LPSPI_TCR_PCS(1)|LPSPI_TCR_FRAMESZ(7)|LPSPI_TCR_CONT, true);
#elif defined(KINETISK)
#else
  if(m_dc_port)
    *m_dc_port|=m_dc_mask;
#endif
}
//----

void graphics_device_ssd1283a::update_tcr_data16()
{
#ifdef EGL_PLATFORM_TEENSY4X
  update_tcr(LPSPI_TCR_PCS(1)|LPSPI_TCR_FRAMESZ(15)|LPSPI_TCR_CONT, true);
#elif defined(KINETISK)
#else
  if(m_dc_port)
    *m_dc_port|=m_dc_mask;
#endif
}
//----

// ??? take a look - hmm
void graphics_device_ssd1283a::set_data_window(uint16_t x0_, uint16_t y0_, uint16_t x1_, uint16_t y1_)
{
  // // setup data window
  // writecmd_cont(ILI9341_CASET);
  // update_tcr_data16();
  // writedata16_cont(x0_);
  // writedata16_cont(x1_);
  // writecmd_cont(ILI9341_PASET);
  // update_tcr_data16();
  // writedata16_cont(y0_);
  // writedata16_cont(y1_);
}
//----

void graphics_device_ssd1283a::writecmd_cont(uint8_t cmd_)
{
  digitalWrite(pin_dc, LOW);
  SPI.transfer(cmd_);
  digitalWrite(pin_dc, HIGH);
}
//----

// ??? take a look - maybe important
void graphics_device_ssd1283a::writecmd_last(uint8_t cmd_)
{
// #ifdef EGL_PLATFORM_TEENSY4X
//   update_tcr(LPSPI_TCR_PCS(0)|LPSPI_TCR_FRAMESZ(7), true);
//   m_spi_imxrt->TDR=cmd_;
//   ++m_num_pending_rx;
// #elif defined(KINETISK)
//   m_spi_kinetisk->PUSHR=cmd_|(m_pcs_command<<16)|SPI_PUSHR_CTAS(0)|SPI_PUSHR_EOQ;
// #else
//   if(m_dc_port)
//     *m_dc_port&=~m_dc_mask;
//   m_spi->transfer(cmd_);
// #endif
//   wait_spi_complete();
}
//----

void graphics_device_ssd1283a::writedata8_cont(uint8_t data_)
{
  digitalWrite(pin_dc, HIGH);
  SPI.transfer(data_);
}
//----

void graphics_device_ssd1283a::writedata8_last(uint8_t data_)
{
#ifdef EGL_PLATFORM_TEENSY4X
  update_tcr(LPSPI_TCR_PCS(1)|LPSPI_TCR_FRAMESZ(7), false);
  m_spi_imxrt->TDR=data_;
  ++m_num_pending_rx;
#elif defined(KINETISK)
  m_spi_kinetisk->PUSHR=data_|(m_pcs_data<<16)|SPI_PUSHR_CTAS(0)|SPI_PUSHR_EOQ;
#else
  m_spi->transfer(data_);
#endif
  wait_spi_complete();
}
//----

void graphics_device_ssd1283a::writedata16_cont(uint16_t data_)
{
  digitalWrite(pin_dc, HIGH);
#if (defined (ESP8266) || defined(ESP32)) && true // faster
  SPI.write16(data_);
#else
  //SPI.transfer(data_ >> 8);
  //SPI.transfer(data_);
  SPI.transfer16(data_);
#endif
}
//----

void graphics_device_ssd1283a::writedata16_cont(uint16_t data_, uint16_t n_)
{
  digitalWrite(pin_dc, HIGH);
#if (defined (ESP8266) || defined(ESP32)) && true // fastest
  uint16_t swapped = ((data_ << 8) | (data_ >> 8));
  SPI.writePattern((uint8_t*)&swapped, 2, n_);
#elif (defined (ESP8266) || defined(ESP32)) && true // faster
  uint16_t swapped = ((data_ << 8) | (data_ >> 8));
  while (n_-- > 0)
  {
    SPI.write16(swapped);
  }
#else
  while (n_-- > 0)
  {
    SPI.transfer(data_ >> 8);
    SPI.transfer(data_);
  }
#endif
}
//----

void graphics_device_ssd1283a::writedata16_last(uint16_t data_)
{
#ifdef EGL_PLATFORM_TEENSY4X
  update_tcr(LPSPI_TCR_PCS(1)|LPSPI_TCR_FRAMESZ(15), false);
  m_spi_imxrt->TDR=data_;
  ++m_num_pending_rx;
#elif defined(KINETISK)
  m_spi_kinetisk->PUSHR=data_|(m_pcs_data<<16)|SPI_PUSHR_CTAS(1)|SPI_PUSHR_EOQ;
#else
  m_spi->transfer16(data_);
#endif
  wait_spi_complete();
}
//----

// ??? take a look -
void graphics_device_ssd1283a::wait_spi_fifo()
{
#ifdef EGL_PLATFORM_TEENSY4X
  do
  {
    if(!(m_spi_imxrt->RSR&LPSPI_RSR_RXEMPTY))
    {
      uint32_t tmp __attribute__((unused))=m_spi_imxrt->RDR;
      if(m_num_pending_rx)
        --m_num_pending_rx;
    }
  } while(!(m_spi_imxrt->SR&LPSPI_SR_TDF));
#elif defined(KINETISK)
  uint32_t sr;
  do
  {
    sr=m_spi_kinetisk->SR;
    if(sr&0xF0)
      uint32_t tmp __attribute__((unused))=m_spi_kinetisk->POPR;
  } while((sr&(15<<12))>m_spi_kinetisk_fifo_wait);
#endif
}
//----

void graphics_device_ssd1283a::wait_spi_complete()
{
#ifdef EGL_PLATFORM_TEENSY4X
  while(m_num_pending_rx)
  {
    if(!(m_spi_imxrt->RSR&LPSPI_RSR_RXEMPTY))
    {
      uint32_t tmp __attribute__((unused))=m_spi_imxrt->RDR;
      --m_num_pending_rx;
    }
  }
  m_spi_imxrt->CR=LPSPI_CR_MEN|LPSPI_CR_RRF;
#elif defined(KINETISK)
  while(1)
  {
    uint32_t sr=m_spi_kinetisk->SR;
    if(sr&SPI_SR_EOQF)
      break;
    if(sr&0xF0)
      uint32_t tmp __attribute__((unused))=m_spi_kinetisk->POPR;
  }
  m_spi_kinetisk->SR=SPI_SR_EOQF;
  while(m_spi_kinetisk->SR&0xF0)
  	uint32_t tmp __attribute__((unused))=m_spi_kinetisk->POPR;
#endif
}
//----

#ifdef EGL_PLATFORM_TEENSY4X
void graphics_device_ssd1283a::update_tcr(uint32_t tcr_state_, bool state_transit_)
{
  if(!m_dc_mask || !state_transit_)
  {
    while((m_spi_imxrt->FSR&0x1f));
    m_spi_imxrt->TCR=tcr_state_;
  }
  else
  {
    wait_spi_complete();
    if(tcr_state_&LPSPI_TCR_PCS(3))
      direct_write_hi(m_dc_port, m_dc_mask);
    else
      direct_write_lo(m_dc_port, m_dc_mask);
    m_spi_imxrt->TCR=tcr_state_&~(LPSPI_TCR_PCS(3)|LPSPI_TCR_CONT);
  }
}
#endif
//----------------------------------------------------------------------------

//============================================================================
EGL_NAMESPACE_END
#endif
