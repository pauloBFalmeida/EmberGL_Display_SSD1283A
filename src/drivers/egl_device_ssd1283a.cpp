//============================================================================

//============================================================================

#ifdef ARDUINO
#include "egl_device_ssd1283a.h"
EGL_USING_NAMESPACE
//----------------------------------------------------------------------------

//============================================================================
// graphics_device_ssd1283a
//============================================================================
graphics_device_ssd1283a *graphics_device_ssd1283a::s_active_dev=0;
//----------------------------------------------------------------------------

graphics_device_ssd1283a::graphics_device_ssd1283a()
{
  EGL_ASSERT(!s_active_dev);
  s_active_dev=this;
  m_tile_rt0=0;
  set_spi(0);
#ifdef KINETISK
  m_pcs_data=0;
  m_pcs_command=0;
#endif

#if EGL_PLATFORM_NUM_DMA>0 && EGL_BUILDOP_DMA_TRANSFER==1
  // init DMA variables
  m_dma_transfers=0;
#endif
}
//----

graphics_device_ssd1283a::graphics_device_ssd1283a(uint32_t pin_cs_, uint32_t pin_dc_, uint32_t pin_sclk_, uint32_t pin_mosi_, uint32_t pin_miso_, uint32_t pin_reset_)
{
  EGL_ASSERT(!s_active_dev);
  s_active_dev=this;
  m_tile_rt0=0;
  set_spi(0);
#ifdef KINETISK
	m_pcs_data=0;
	m_pcs_command=0;
#endif

#if EGL_PLATFORM_NUM_DMA>0 && EGL_BUILDOP_DMA_TRANSFER==1
  // init DMA variables
  m_dma_transfers=0;
#endif

  // setup SPI pins
  init(pin_cs_, pin_dc_, pin_sclk_, pin_mosi_, pin_miso_, pin_reset_);
}
//----

graphics_device_ssd1283a::~graphics_device_ssd1283a()
{
  s_active_dev=0;
}
//----

void graphics_device_ssd1283a::set_spi(uint8_t spi_, uint32_t spi_clock_)
{
  m_spi_clock=spi_clock_;
  m_spi=0;
  m_spi_chl=0;
  switch(spi_)
  {
#if EGL_PLATFORM_NUM_SPI>=1
    // SPI0
    case 0:
    {
      m_spi=&SPI;
      m_spi_chl=0;
#ifdef EGL_PLATFORM_TEENSY4X
      m_spi_imxrt=&IMXRT_LPSPI4_S;
#elif defined(KINETISK)
      m_spi_kinetisk=&KINETISK_SPI0;
      m_spi_kinetisk_fifo_wait=3<<12;
#endif
    } break;
#endif

#if EGL_PLATFORM_NUM_SPI>=2
    // SPI1
    case 1:
    {
      m_spi=&SPI1;
      m_spi_chl=1;
#ifdef EGL_PLATFORM_TEENSY4X
      m_spi_imxrt=&IMXRT_LPSPI3_S;
#elif defined(KINETISK)
      m_spi_kinetisk=&KINETISK_SPI1;
      m_spi_kinetisk_fifo_wait=0<<12;
#endif
    } break;
#endif

#if EGL_PLATFORM_NUM_SPI>=3
    // SPI2
    case 2:
    {
      m_spi=&SPI2;
      m_spi_chl=2;
#ifdef EGL_PLATFORM_TEENSY4X
      m_spi_imxrt=&IMXRT_LPSPI1_S;
#elif defined(KINETISK)
      m_spi_kinetisk=&KINETISK_SPI2;
      m_spi_kinetisk_fifo_wait=0<<12;
#endif
    } break;
#endif

    // unsupported
    default: EGL_ERRORF("Unsupported SPI %i.\r\n", spi_);
  }
}
//----

void graphics_device_ssd1283a::init(uint32_t pin_cs_, uint32_t pin_dc_, uint32_t pin_sclk_, uint32_t pin_mosi_, uint32_t pin_miso_, uint32_t pin_reset_)
{
  pin_cs = pin_cs_;
  pin_dc = pin_dc_;
#if defined(ARDUINO_ARCH_RP2040)
  // Initialize SPI
  spi_init(spi_default, SPI_CLOCK_TRANSFER);

  // Set up SPI GPIO functions
  gpio_set_function(pin_sclk_, GPIO_FUNC_SPI);
  gpio_set_function(pin_miso_, GPIO_FUNC_SPI);
  gpio_set_function(pin_mosi_, GPIO_FUNC_SPI);

  // Set up CS and DC pins as GPIO
  pinMode(pin_cs_, OUTPUT);
  digitalWrite(pin_cs_, HIGH);

  pinMode(pin_dc_, OUTPUT);
  digitalWrite(pin_dc_, HIGH);

  // Initialize SPI
  SPI.begin();
#else
  m_cs_port=portOutputRegister(pin_cs_);
  m_cs_mask=digitalPinToBitMask(pin_cs_);
  pinMode(pin_cs_, OUTPUT);
  *m_cs_port|=m_cs_mask;
  m_dc_port=0;
  m_dc_mask=0;
  m_spi->setMOSI(pin_mosi_);
  m_spi->setMISO(pin_miso_);
  m_spi->setSCK(pin_sclk_);
  m_spi->begin();
  if(m_spi->pinIsChipSelect(pin_dc_))
    m_spi->setCS(pin_dc_);
  else
  {
    m_dc_port=portOutputRegister(pin_dc_);
    m_dc_mask=digitalPinToBitMask(pin_dc_);
    pinMode(pin_dc_, OUTPUT);
    *m_dc_port|=m_dc_mask;
  }
#endif

  // setup reset pin
  if(pin_reset_<0xff)
  {
    pinMode(pin_reset_, OUTPUT);
    digitalWrite(pin_reset_, HIGH);
    delay(5);
    digitalWrite(pin_reset_, LOW);
    delay(20);
    digitalWrite(pin_reset_, HIGH);
    delay(150);
  }

  // Initialize TFT
  display_lcd = new SSD1283A(pin_cs_, pin_dc_, pin_reset_, -1);
  display_lcd->init();
  display_lcd->fillScreen(0x0);
  display_lcd->setRotation(0);
  EGL_LOG("SSD1283A graphics device initialized!\r\n");
}
//----

void graphics_device_ssd1283a::init_rasterizer(const rasterizer_cfg &rcfg_, const rasterizer_tiling_cfg &tcfg_, const rasterizer_vertex_cache_cfg &vccfg_)
{
  graphics_device<graphics_device_ssd1283a>::init(rcfg_, tcfg_, vccfg_);
  m_tile_rt0=(fb_format_t*)rcfg_.rts[0];
  m_tile_width=tcfg_.tile_width;
  m_tile_height=tcfg_.tile_height;
}
//----

void graphics_device_ssd1283a::init_dma(rasterizer_data_transfer *transfers_, uint8_t num_transfers_, fb_format_t *dma_buffer_, size_t dma_buffer_size_)
{
#if EGL_PLATFORM_NUM_DMA>0 && EGL_BUILDOP_DMA_TRANSFER==1 && defined(ARDUINO_ARCH_RP2040)
  // setup DMA buffers
  EGL_ASSERT(transfers_ && dma_buffer_);
  m_dma_transfers=transfers_;
  m_dma_transfers_size=num_transfers_;
  m_dma_transfers_rpos=0;
  m_dma_transfers_wpos=0;
  m_dma_buffer=dma_buffer_;
  m_dma_buffer_size=dma_buffer_size_/sizeof(*m_dma_buffer);
  m_dma_buffer_rpos=0;
  m_dma_buffer_wpos=0;

  // Get a free channel
  m_dma_chl = dma_claim_unused_channel(true);

  dma_config = dma_channel_get_default_config(m_dma_chl);
  channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_8);
  channel_config_set_dreq(&dma_config, spi_get_dreq(spi_default, true)); // Use Default SPI as data request
  channel_config_set_read_increment(&dma_config, true);
  channel_config_set_write_increment(&dma_config, false);

  dma_channel_configure(
      m_dma_chl,                    // Channel to be configured
      &dma_config,                  // The configuration
      &spi_get_hw(spi_default)->dr, // Write address
      dma_buffer_,                  // Read address (Source)
      dma_buffer_size_,             // Number of transfers
      false                         // Dont start immediately
  );

  // Set up the DMA interrupt handler
  dma_channel_set_irq0_enabled(m_dma_chl, true);
  irq_set_exclusive_handler(DMA_IRQ_0, dma_interrupt);
  irq_set_enabled(DMA_IRQ_0, true);

#else // EGL_BUILDOP_DMA_TRANSFER
  EGL_ERROR("DMA support disabled\r\n");
#endif
}
//----------------------------------------------------------------------------

void graphics_device_ssd1283a::flush_dma()
{
#if EGL_PLATFORM_NUM_DMA>0 && EGL_BUILDOP_DMA_TRANSFER==1
  // wait for all DMA transfers to complete
  while(m_dma_transfers_rpos!=m_dma_transfers_wpos);
#endif
}
//----------------------------------------------------------------------------

void graphics_device_ssd1283a::submit_tile(uint8_t tx_, uint8_t ty_, const vec2u16 &reg_min_, const vec2u16 &reg_end_, uint16_t thread_idx_)
{
  // access update pos, size and data
  uint16_t x=tx_*m_tile_width+reg_min_.x;
  uint16_t y=ty_*m_tile_height+reg_min_.y;
  uint16_t update_width=reg_end_.x-reg_min_.x;
  uint16_t update_height=reg_end_.y-reg_min_.y;
  fb_format_t *data=m_tile_rt0+reg_min_.x+reg_min_.y*m_tile_width;

#if EGL_PLATFORM_NUM_DMA>0 && EGL_BUILDOP_DMA_TRANSFER==1
  // check for DMA transfer
  if(m_dma_transfers)
  {
    // add DMA transfers for the tile
    do
    {
      // get DMA transfer size
      size_t max_transfer_px=m_dma_buffer_size-m_dma_buffer_wpos;
      if(max_transfer_px<update_width*2)
      {
        m_dma_buffer_wpos=0;
        max_transfer_px=m_dma_buffer_size;
      }
      uint16_t transfer_height=(uint16_t)min(size_t(update_height), max_transfer_px/update_width);
      size_t transfer_px=update_width*transfer_height;
      update_height-=transfer_height;

      // wait for space in transfer/data buffers for the DMA transfer (spinlock)
      uint8_t next_dma_transfers_wpos=m_dma_transfers_wpos+1;
      if(next_dma_transfers_wpos==m_dma_transfers_size)
        next_dma_transfers_wpos=0;
      while(   (m_dma_buffer_rpos>=m_dma_buffer_wpos && m_dma_buffer_wpos+transfer_px>m_dma_buffer_rpos && m_dma_transfers_rpos!=m_dma_transfers_wpos)
            || (m_dma_transfers_rpos==next_dma_transfers_wpos));

      // copy tile data to the DMA buffer
      fb_format_t *dma_buf=m_dma_buffer+m_dma_buffer_wpos;
      if(m_tile_shader)
      {
        m_tile_shader->copy_region(dma_buf, render_targets(), depth_target(), size_t(data-m_tile_rt0), x, y, update_width, transfer_height, m_tile_width);
        data+=m_tile_width*transfer_height;
      }
      else
      {
        if(m_tile_width==update_width)
        {
          // copy the full buffer at once
          mem_copy(dma_buf, data, transfer_px*sizeof(*m_dma_buffer));
          data+=transfer_px;
        }
        else
        {
          // copy the buffer scanline at the time
          uint16_t dma_scan_size=update_width*sizeof(*m_dma_buffer);
          fb_format_t *dma_buf_end=dma_buf+transfer_px;
          do
          {
            mem_copy(dma_buf, data, dma_scan_size);
            dma_buf+=update_width;
            data+=m_tile_width;
          } while(dma_buf<dma_buf_end);
        }
      }

      // setup DMA transfer
      rasterizer_data_transfer &transfer=m_dma_transfers[m_dma_transfers_wpos];
      transfer.data_offset=m_dma_buffer_wpos;
      transfer.x=x;
      transfer.y=y;
      y+=transfer_height;
      transfer.width=update_width;
      transfer.height=transfer_height;

      // queue transfer and try to start DMA transfer if none active
      noInterrupts();
      if(m_dma_transfers_wpos!=m_dma_transfers_rpos)
        m_dma_transfers_wpos=next_dma_transfers_wpos;
      interrupts();
      if(   m_dma_transfers_wpos!=m_dma_transfers_rpos
         || start_dma_transfer(transfer))
      {
        m_dma_transfers_wpos=next_dma_transfers_wpos;
        m_dma_buffer_wpos+=transfer_px;
      }
    } while(update_height);
  }
  else
#endif // EGL_BUILDOP_DMA_TRANSFER
  {
    // synchronously SPI transfer tile update region
    if(m_tile_shader) {
      m_tile_shader->transfer_region(render_targets(), depth_target(), size_t(data-m_tile_rt0), x, y, update_width, update_height, m_tile_width);
    } else {

      begin_spi_transition();
      set_window_address(x, y, x+update_width-1, y+update_height-1);
      writecmd_cont(0x22);

      fb_format_t *data_end=data+m_tile_width*update_height;
      while(1)
      {
        fb_format_t *data_scan=data, *data_scan_end=data_scan+update_width-1;
        while(data_scan<data_scan_end)
          writedata16_cont((data_scan++)->v);
        data+=m_tile_width;
        if(data==data_end)
        {
          writedata16_cont(data_scan->v);
          break;
        }
        else
          writedata16_cont(data_scan->v);
      }
      end_spi_transition();
    }
  }
}
//----

#if EGL_PLATFORM_NUM_DMA>0 && EGL_BUILDOP_DMA_TRANSFER==1
void graphics_device_ssd1283a::dma_interrupt()
{
  s_active_dev->dma_interrupt_impl();
}
//----

// ??? take a look - dma
void graphics_device_ssd1283a::dma_interrupt_impl()
{
#if defined(ARDUINO_ARCH_RP2040)
  dma_channel_acknowledge_irq0(m_dma_chl);  // Clear the interrupt
  end_spi_transition();

  free(m_dma_display_data); // free malloc used in DMA transfer

  rasterizer_data_transfer &transfer=m_dma_transfers[m_dma_transfers_rpos];
  size_t data_size=transfer.width*transfer.height-1;
  m_dma_buffer_rpos+=data_size;

  // move to the next DMA transfer (loop until end-of-transfers or able to start DMA transfer)
    do
    {
      if(++m_dma_transfers_rpos==m_dma_transfers_size)
        m_dma_transfers_rpos=0;
    } while(   m_dma_transfers_rpos!=m_dma_transfers_wpos
            && !start_dma_transfer(m_dma_transfers[m_dma_transfers_rpos]));
#endif
}
//----

bool graphics_device_ssd1283a::start_dma_transfer(const rasterizer_data_transfer &transfer_)
{
  // setup SPI transfer
  enum {min_dma_pixels=16}; // minimum pixels required to trigger DMA transfer
  EGL_STATIC_ASSERT(sizeof(*m_dma_buffer)==2);
  // begin spi transfer by setting window in display that we want to update
  begin_spi_transition();
  set_window_address(transfer_.x, transfer_.y, transfer_.x+transfer_.width-1, transfer_.y+transfer_.height-1);
  writecmd_cont(0x22);

  m_dma_buffer_rpos=transfer_.data_offset;  // sets the read postion from dma buffer
  size_t data_size=transfer_.width*transfer_.height; // amount of pixels to send

  // check for synchronous transfer
  if(data_size<min_dma_pixels)
  {
    // write pixels synchronously over SPI
    update_tcr_data16();
    const fb_format_t *buf=m_dma_buffer+m_dma_buffer_rpos, *buf_end=buf+data_size-1;
    while(buf<buf_end)
      writedata16_cont((buf++)->v);
    writedata16_last(buf->v);
    end_spi_transition();
    m_dma_buffer_rpos+=data_size;

    return false;
  }

  // const fb_format_t *dma_buf=m_dma_buffer+m_dma_buffer_rpos; // start address of source dma buffer
  update_tcr_data16();
  // create buffer of bytes to dma transfer to display
  m_dma_display_data = (uint8_t*) malloc(data_size * sizeof(uint16_t));
  // put the uint16_t color from pixel struct  in the buffer for the dma transfer
  for (int i=0; i<data_size; i++) {
    m_dma_display_data[2*i+1] = (m_dma_buffer[m_dma_buffer_rpos+i].v)    & 0xFF;
    m_dma_display_data[2*i]   = (m_dma_buffer[m_dma_buffer_rpos+i].v>>8) & 0xFF;
  }
  // configure and start dma
  dma_channel_configure(m_dma_chl, &dma_config,
      &spi_get_hw(spi_default)->dr, // Write address
      m_dma_display_data,           // Read address (Source)
      data_size*sizeof(uint16_t),   // Number of transfers
      false                         // Dont start immediately
  );
  dma_channel_start(m_dma_chl);

  return true;
}
#endif // EGL_BUILDOP_DMA_TRANSFER
//----------------------------------------------------------------------------
#endif // ARDUINO
