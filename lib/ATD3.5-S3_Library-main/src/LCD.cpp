#include "LCD.h"
#include <Arduino.h>

#include <SPI.h>
#include "esp32-hal-spi.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "soc/dport_reg.h"
#include "driver/periph_ctrl.h"

#define LCD_OFFSET_X (0)
#define LCD_OFFSET_Y (0)

LCD::LCD() {
  
}

volatile spi_dev_t *spi_dev = (volatile spi_dev_t *)(DR_REG_SPI2_BASE);

void LCD::initSPI() {
  // SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, -1); // SCK, MISO, MOSI, CS

  // Data bus
  pinMatrixOutAttach(SPI_MOSI_PIN, FSPID_OUT_IDX, false, false);
  pinMatrixOutAttach(SPI_SCK_PIN, FSPICLK_OUT_IDX, false, false);

  // Setup SPI
  periph_module_reset(PERIPH_SPI2_MODULE);
  periph_module_enable(PERIPH_SPI2_MODULE);

  // spiInitBus
  spi_dev->clk_gate.clk_en = 1;
  spi_dev->clk_gate.mst_clk_sel = 1;
  spi_dev->clk_gate.mst_clk_active = 1;
  spi_dev->dma_conf.tx_seg_trans_clr_en = 1;
  spi_dev->dma_conf.rx_seg_trans_clr_en = 1;
  spi_dev->dma_conf.dma_seg_trans_en = 0;
  spi_dev->misc.val = 0;
  spi_dev->user.val = 0;
  spi_dev->user1.val = 0;
  spi_dev->ctrl.val = 0;
  spi_dev->clock.val = 0;

  // Set SPI Mode 0
  spi_dev->misc.ck_idle_edge = 0;
  spi_dev->user.ck_out_edge = 1;

  // Set MSBFIRST
  spi_dev->ctrl.wr_bit_order = 0;
  spi_dev->ctrl.rd_bit_order = 0;

  // Set Clock
  this->_div = spiFrequencyToClockDiv(this->clock); // 40 MHz
  spi_dev->clock.val = this->_div;

  // Ref : 8.4.6 Access 8-bit I8080/MT6800 LCD in Master Half-Duplex Mode
  // Setup 1 : Add command 0x27
  // spi_dev->ctrl.fcmd_oct = 1; // 8-bit data bus (Command)
  spi_dev->user.usr_command = 0;
  spi_dev->user2.usr_command_bitlen = 0;
  spi_dev->user2.usr_command_value = 0;

  // Setup 2 : Disable Address
  spi_dev->user.usr_addr = 0; // Disable Address
  spi_dev->user.usr_dummy = 0;
  spi_dev->user.usr_miso = 0; // Disable MISO

  // Setup 3 : Set dataout
  // spi_dev->user.fwrite_oct = 1; // 8-bit data bus (Data)
  // spi_dev->user.usr_mosi = 0; // Enable MOSI

  // Config CS delay
  spi_dev->user.cs_setup = 0;
  spi_dev->user.cs_hold = 0;

  // Send dummy for SCK pin state update
  spi_dev->cmd.update = 1;
  while (spi_dev->cmd.update);
  spi_dev->cmd.usr = 1;
  while (spi_dev->cmd.usr) ;
}

void LCD::write_spi(uint8_t data) {
  this->write_spi(&data, 1);
}

void LCD::write_spi(uint8_t *buf, size_t len) {
  uint32_t offset = 0;
  while(len) {
    spi_dev->user.usr_mosi = 1; // Enable MISO

    size_t byte_write = min(len, (size_t)(16 * 4));
    uint8_t buffIndex = 0;
    uint8_t byteIndex = 0;
    for (size_t i=0;i<byte_write;i++) {
      if (byteIndex == 0) {
        spi_dev->data_buf[buffIndex] = buf[offset + i];
      } else {
        spi_dev->data_buf[buffIndex] |= buf[offset + i] << (byteIndex * 8);
      }
      byteIndex++;
      if (byteIndex == 4) {
        byteIndex = 0;
        buffIndex++;
      }
    }

    spi_dev->ms_dlen.ms_data_bitlen = (byte_write * 8) - 1;
    
    spi_dev->cmd.update = 1;
    while (spi_dev->cmd.update);
    spi_dev->cmd.usr = 1;
    while (spi_dev->cmd.usr) ;

    // spi_dev->user.usr_mosi = 0; // Disable MISO

    len -= byte_write;
    offset += byte_write;
  }
}

// ST7796 specific commands
#define ST7796_NOP     0x00
#define ST7796_SWRESET 0x01
#define ST7796_RDDID   0x04
#define ST7796_RDDST   0x09

#define ST7796_SLPIN   0x10
#define ST7796_SLPOUT  0x11
#define ST7796_PTLON   0x12
#define ST7796_NORON   0x13

#define ST7796_RDMODE  0x0A
#define ST7796_RDMADCTL  0x0B
#define ST7796_RDPIXFMT  0x0C
#define ST7796_RDIMGFMT  0x0A
#define ST7796_RDSELFDIAG  0x0F

#define ST7796_INVOFF  0x20
#define ST7796_INVON   0x21

#define ST7796_DISPOFF 0x28
#define ST7796_DISPON  0x29

#define ST7796_CASET   0x2A
#define ST7796_RASET   0x2B
#define ST7796_RAMWR   0x2C
#define ST7796_RAMRD   0x2E

#define ST7796_PTLAR   0x30
#define ST7796_VSCRDEF 0x33
#define ST7796_MADCTL  0x36
#define ST7796_VSCRSADD 0x37
#define ST7796_PIXFMT  0x3A

#define ST7796_WRDISBV  0x51
#define ST7796_RDDISBV  0x52
#define ST7796_WRCTRLD  0x53

#define ST7796_FRMCTR1 0xB1
#define ST7796_FRMCTR2 0xB2
#define ST7796_FRMCTR3 0xB3
#define ST7796_INVCTR  0xB4
#define ST7796_DFUNCTR 0xB6

#define ST7796_PWCTR1  0xC0
#define ST7796_PWCTR2  0xC1
#define ST7796_PWCTR3  0xC2

#define ST7796_VMCTR1  0xC5
#define ST7796_VMCOFF  0xC6

#define ST7796_RDID4   0xD3

#define ST7796_GMCTRP1 0xE0
#define ST7796_GMCTRN1 0xE1

#define ST7796_MADCTL_MY  0x80
#define ST7796_MADCTL_MX  0x40
#define ST7796_MADCTL_MV  0x20
#define ST7796_MADCTL_ML  0x10
#define ST7796_MADCTL_RGB 0x00
#define ST7796_MADCTL_BGR 0x08
#define ST7796_MADCTL_MH  0x04

#define ST7796_MADCTL_MY  0x80
#define ST7796_MADCTL_MX  0x40
#define ST7796_MADCTL_MV  0x20
#define ST7796_MADCTL_ML  0x10
#define ST7796_MADCTL_BGR 0x08
#define ST7796_MADCTL_MH  0x04
#define ST7796_MADCTL_RGB 0x00

void LCD::write_cmd(uint8_t cmd) {
  this->write_cmd(cmd, NULL, 0);
}

void LCD::write_cmd(uint8_t cmd, uint8_t data) {
  this->write_cmd(cmd, &data, 1);
}

void LCD::write_cmd(uint8_t cmd, uint8_t *data, int len) {
    digitalWrite(LCD_CS_PIN, LOW);
    digitalWrite(LCD_DC_PIN, LOW);
    write_spi(cmd);
    if (data && (len > 0)) {
        digitalWrite(LCD_DC_PIN, HIGH);
        write_spi(data, len);
    }
    digitalWrite(LCD_CS_PIN, HIGH);
}

void LCD::initST7796S() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, LOW);

  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, LOW);

  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);

  pinMode(LCD_DC_PIN, OUTPUT);

  // Reset
  digitalWrite(LCD_RST_PIN, LOW);
  delay(10);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(100);

  write_cmd(ST7796_SWRESET);
  delay(10);

  write_cmd(ST7796_SLPOUT);
  delay(10);

  // write_cmd(ST7796_MADCTL, ST7796_MADCTL_MX | ST7796_MADCTL_MV | ST7796_MADCTL_BGR); // Rotation
  // ST7796_MADCTL_MX | ST7796_MADCTL_MY
  // ST7796_MADCTL_MY | ST7796_MADCTL_MV
  // ---
  // ST7796_MADCTL_MX | ST7796_MADCTL_MV
  // write_cmd(ST7796_MADCTL, ST7796_MADCTL_MV | ST7796_MADCTL_BGR); // Rotation
  this->setRotation(this->rotation);

  write_cmd(ST7796_PIXFMT, 0x55); // 16-bit color mode
    
  delay(10);
  write_cmd(ST7796_INVON);
  write_cmd(ST7796_DISPON);
  digitalWrite(LCD_BL_PIN, HIGH);
  delay(200);
}

void LCD::begin(uint8_t rotation, uint32_t speed) {
  this->rotation = rotation;
  this->clock = speed;
  this->initSPI();
  this->initST7796S();
}

void LCD::setRotation(int m) {
  if (m == 0) {
    write_cmd(ST7796_MADCTL, ST7796_MADCTL_MV | ST7796_MADCTL_BGR); // Rotation
    lcd_width = LCD_WIDTH;
    lcd_height = LCD_HEIGHT;
  } else if (m == 1) {
    write_cmd(ST7796_MADCTL,  ST7796_MADCTL_MX | ST7796_MADCTL_MY | ST7796_MADCTL_MV | ST7796_MADCTL_BGR); // Rotation
    lcd_width = LCD_WIDTH;
    lcd_height = LCD_HEIGHT;
  } else if (m == 2) {
    write_cmd(ST7796_MADCTL, ST7796_MADCTL_MY | ST7796_MADCTL_BGR); // Rotation
    lcd_width = LCD_HEIGHT;
    lcd_height = LCD_WIDTH;
  } else if (m == 3) {
    write_cmd(ST7796_MADCTL, ST7796_MADCTL_MX | ST7796_MADCTL_BGR); // Rotation
    lcd_width = LCD_HEIGHT;
    lcd_height = LCD_WIDTH;
  }
  this->rotation = m;
}

uint8_t LCD::getRotation() {
  return this->rotation;
}

void LCD::setWindow(int x_start, int y_start, int x_end, int y_end) {
  spi_dev->clock.val = this->_div;
  
  x_start += LCD_OFFSET_X;
  x_end += LCD_OFFSET_X;
  y_start += LCD_OFFSET_Y;
  y_end += LCD_OFFSET_Y;
  {
    uint8_t buff_x[] = {
      (uint8_t)((x_start >> 8) & 0xFF),
      (uint8_t)(x_start & 0xFF),
      (uint8_t)((x_end >> 8) & 0xFF),
      (uint8_t)(x_end & 0xFF)
    };
    this->write_cmd(ST7796_CASET, buff_x, 4);
  }

  {
    uint8_t buff_y[] = {
      (uint8_t)((y_start >> 8) & 0xFF),
      (uint8_t)(y_start & 0xFF),
      (uint8_t)((y_end >> 8) & 0xFF),
      (uint8_t)(y_end & 0xFF)
    };
    this->write_cmd(ST7796_RASET, buff_y, 4);
  }
}

void LCD::drawBitmap(int x_start, int y_start, int x_end, int y_end, uint16_t* color_data) {
  this->setWindow(x_start, y_start, x_end, y_end);
  // this->write_cmd(ST7796_RAMWR, (uint8_t*) color_data, (x_end - x_start + 1) * (y_end - y_start + 1) * 2);
  
  digitalWrite(LCD_CS_PIN, LOW);

  // Start write command
  digitalWrite(LCD_DC_PIN, LOW);
  this->write_spi(ST7796_RAMWR);
  
  // Write Color data
  digitalWrite(LCD_DC_PIN, HIGH);

  spi_dev->user.usr_command = 0; // No Command
  
  uint32_t index = 0;
  uint32_t len = (x_end - x_start + 1) * (y_end - y_start + 1) * 2u;
  int i;
  uint32_t write_bit;
  while(len) {
    write_bit = min((uint32_t)(len * 8), (uint32_t)(16 * 32));

    spi_dev->user.usr_mosi = 1;
    for (i=0;i<16;i++) {
      spi_dev->data_buf[i] = (len >= 4 ? (color_data[index + 1] << 16) : 0x0) | color_data[index];
      index += 2;
    }
    spi_dev->ms_dlen.ms_data_bitlen = write_bit - 1;
    spi_dev->cmd.update = 1;
    while (spi_dev->cmd.update);
    spi_dev->cmd.usr = 1;
    while(spi_dev->cmd.usr);
    // spi_dev->user.usr_mosi = 0;

    len -= (write_bit / 8);
  }
  
  digitalWrite(LCD_CS_PIN, HIGH);
}

void LCD::drawPixel(uint16_t x, uint16_t y, uint16_t color) {
  if (x >= this->lcd_width) return;
	if (y >= this->lcd_height) return;

  this->setWindow(x, y, x, y);
  uint8_t buff[] = {
    (uint8_t)((color >> 8) & 0xFF),
    (uint8_t)(color & 0xFF)
  };
  this->write_cmd(ST7796_RAMWR, buff, sizeof(buff));
}

uint16_t LCD::color565(uint8_t red, uint8_t green, uint8_t blue) {
  return ((red & 0b11111000) << 8) | ((green & 0b11111100) << 3) | (blue >> 3);
}

uint32_t LCD::color24to16(uint32_t color888) {
  return this->color565((color888 >> 16) & 0xFF, (color888 >> 8) & 0xFF, color888 & 0xFF);
}

void LCD::fillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	if (x1 >= this->lcd_width) return;
	if (x2 >= this->lcd_width) x2 = this->lcd_width - 1;
	if (y1 >= this->lcd_height) return;
	if (y2 >= this->lcd_height) y2 = this->lcd_height - 1;

	this->setWindow(x1, y1, x2, y2);

  digitalWrite(LCD_CS_PIN, LOW);

  // Start write command
  digitalWrite(LCD_DC_PIN, LOW);
  this->write_spi(ST7796_RAMWR);
  
  // Write Color data
  digitalWrite(LCD_DC_PIN, HIGH);

  spi_dev->user.usr_command = 0; // No Command

  // Fill buffer by color
  for (uint8_t i=0;i<16;i++) {
    spi_dev->data_buf[i] = (color >> 8) & 0x00FF;
    spi_dev->data_buf[i] |= (color & 0x00FF) << 8;
    spi_dev->data_buf[i] |= ((color & 0xFF00) >> 8) << 16;
    spi_dev->data_buf[i] |= (color & 0xFF00) << 24;
  }
  
  uint32_t len = (x2 - x1 + 1) * (y2 - y1 + 1);
  while(len) {
    uint8_t write_count = min((int)len, 32); // 16 * 2 = 32

    spi_dev->user.usr_mosi = 1;
    spi_dev->ms_dlen.ms_data_bitlen = (write_count * 16) - 1;
    spi_dev->cmd.update = 1;
    while (spi_dev->cmd.update);
    spi_dev->cmd.usr = 1;
    while(spi_dev->cmd.usr);
    // spi_dev->user.usr_mosi = 0;

    len -= write_count;
  }
  
  digitalWrite(LCD_CS_PIN, HIGH);
}

// Display OFF
void LCD::off() {
	this->write_cmd(ST7796_DISPOFF);	//Display off
}
 
// Display ON
void LCD::on() {
	this->write_cmd(ST7796_DISPON);	//Display on
}

// Fill screen
// color:color
void LCD::fillScreen(uint16_t color) {
	this->fillRect(0, 0, this->lcd_width - 1, this->lcd_height - 1, color);
}

// Draw line
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// color:color 
void LCD::drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	int i;
	int dx,dy;
	int sx,sy;
	int E;

	/* distance between two points */
	dx = ( x2 > x1 ) ? x2 - x1 : x1 - x2;
	dy = ( y2 > y1 ) ? y2 - y1 : y1 - y2;

	/* direction of two point */
	sx = ( x2 > x1 ) ? 1 : -1;
	sy = ( y2 > y1 ) ? 1 : -1;

	/* inclination < 1 */
	if ( dx > dy ) {
		E = -dx;
		for ( i = 0 ; i <= dx ; i++ ) {
			this->drawPixel(x1, y1, color);
			x1 += sx;
			E += 2 * dy;
			if ( E >= 0 ) {
			y1 += sy;
			E -= 2 * dx;
		}
	}

	/* inclination >= 1 */
	} else {
		E = -dy;
		for ( i = 0 ; i <= dy ; i++ ) {
			this->drawPixel(x1, y1, color);
			y1 += sy;
			E += 2 * dx;
			if ( E >= 0 ) {
				x1 += sx;
				E -= 2 * dy;
			}
		}
	}
}

// Draw rectangle
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// color:color
void LCD::drawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	this->drawLine(x1, y1, x2, y1, color);
	this->drawLine(x2, y1, x2, y2, color);
	this->drawLine(x2, y2, x1, y2, color);
	this->drawLine(x1, y2, x1, y1, color);
}

// Draw rectangle with angle
// xc:Center X coordinate
// yc:Center Y coordinate
// w:Width of rectangle
// h:Height of rectangle
// angle:Angle of rectangle
// color:color

//When the origin is (0, 0), the point (x1, y1) after rotating the point (x, y) by the angle is obtained by the following calculation.
// x1 = x * cos(angle) - y * sin(angle)
// y1 = x * sin(angle) + y * cos(angle)
void LCD::drawRectAngle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color) {
	double xd,yd,rd;
	int x1,y1;
	int x2,y2;
	int x3,y3;
	int x4,y4;
	rd = -angle * M_PI / 180.0;
	xd = 0.0 - w/2;
	yd = h/2;
	x1 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y1 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	yd = 0.0 - yd;
	x2 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y2 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = w/2;
	yd = h/2;
	x3 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y3 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	yd = 0.0 - yd;
	x4 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y4 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	this->drawLine(x1, y1, x2, y2, color);
	this->drawLine(x1, y1, x3, y3, color);
	this->drawLine(x2, y2, x4, y4, color);
	this->drawLine(x3, y3, x4, y4, color);
}

// Draw triangle
// xc:Center X coordinate
// yc:Center Y coordinate
// w:Width of triangle
// h:Height of triangle
// angle:Angle of triangle
// color:color

//When the origin is (0, 0), the point (x1, y1) after rotating the point (x, y) by the angle is obtained by the following calculation.
// x1 = x * cos(angle) - y * sin(angle)
// y1 = x * sin(angle) + y * cos(angle)
void LCD::drawTriangle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color) {
	double xd,yd,rd;
	int x1,y1;
	int x2,y2;
	int x3,y3;
	rd = -angle * M_PI / 180.0;
	xd = 0.0;
	yd = h/2;
	x1 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y1 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = w/2;
	yd = 0.0 - yd;
	x2 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y2 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = 0.0 - w/2;
	x3 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y3 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	this->drawLine(x1, y1, x2, y2, color);
	this->drawLine(x1, y1, x3, y3, color);
	this->drawLine(x2, y2, x3, y3, color);
}

// Draw circle
// x0:Central X coordinate
// y0:Central Y coordinate
// r:radius
// color:color
void LCD::drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
	int x;
	int y;
	int err;
	int old_err;

	x=0;
	y=-r;
	err=2-2*r;
	do{
		this->drawPixel(x0-x, y0+y, color); 
		this->drawPixel(x0-y, y0-x, color); 
		this->drawPixel(x0+x, y0-y, color); 
		this->drawPixel(x0+y, y0+x, color); 
		if ((old_err=err)<=x)	err+=++x*2+1;
		if (old_err>y || err>x) err+=++y*2+1;	 
	} while(y<0);
}

// Draw circle of filling
// x0:Central X coordinate
// y0:Central Y coordinate
// r:radius
// color:color
void LCD::fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
	int x;
	int y;
	int err;
	int old_err;
	int ChangeX;

	x=0;
	y=-r;
	err=2-2*r;
	ChangeX=1;
	do{
		if(ChangeX) {
			this->drawLine(x0-x, y0-y, x0-x, y0+y, color);
			this->drawLine(x0+x, y0-y, x0+x, y0+y, color);
		} // endif
		ChangeX=(old_err=err)<=x;
		if (ChangeX)			err+=++x*2+1;
		if (old_err>y || err>x) err+=++y*2+1;
	} while(y<=0);
} 

// Draw rectangle with round corner
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// r:radius
// color:color
void LCD::drawRoundRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t r, uint16_t color) {
	int x;
	int y;
	int err;
	int old_err;
	unsigned char temp;

	if(x1>x2) {
		temp=x1; x1=x2; x2=temp;
	} // endif
	  
	if(y1>y2) {
		temp=y1; y1=y2; y2=temp;
	} // endif

	if (x2-x1 < r) return; // Add 20190517
	if (y2-y1 < r) return; // Add 20190517

	x=0;
	y=-r;
	err=2-2*r;

	do{
		if(x) {
			this->drawPixel(x1+r-x, y1+r+y, color); 
			this->drawPixel(x2-r+x, y1+r+y, color); 
			this->drawPixel(x1+r-x, y2-r-y, color); 
			this->drawPixel(x2-r+x, y2-r-y, color);
		} // endif 
		if ((old_err=err)<=x)	err+=++x*2+1;
		if (old_err>y || err>x) err+=++y*2+1;	 
	} while(y<0);

	this->drawLine(x1+r,y1  ,x2-r,y1	,color);
	this->drawLine(x1+r,y2  ,x2-r,y2	,color);
	this->drawLine(x1  ,y1+r,x1  ,y2-r,color);
	this->drawLine(x2  ,y1+r,x2  ,y2-r,color);  
} 

// Draw arrow
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// w:Width of the botom
// color:color
// Thanks http://k-hiura.cocolog-nifty.com/blog/2010/11/post-2a62.html
void LCD::drawArrow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color) {
	double Vx= x1 - x0;
	double Vy= y1 - y0;
	double v = sqrt(Vx*Vx+Vy*Vy);
	//	 printf("v=%f\n",v);
	double Ux= Vx/v;
	double Uy= Vy/v;

	uint16_t L[2],R[2];
	L[0]= x1 - Uy*w - Ux*v;
	L[1]= y1 + Ux*w - Uy*v;
	R[0]= x1 + Uy*w - Ux*v;
	R[1]= y1 - Ux*w - Uy*v;
	//printf("L=%d-%d R=%d-%d\n",L[0],L[1],R[0],R[1]);

	//lcdDrawLine(x0,y0,x1,y1,color);
	this->drawLine(x1, y1, L[0], L[1], color);
	this->drawLine(x1, y1, R[0], R[1], color);
	this->drawLine(L[0], L[1], R[0], R[1], color);
}


// Draw arrow of filling
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// w:Width of the botom
// color:color
void LCD::fillArrow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color) {
	double Vx= x1 - x0;
	double Vy= y1 - y0;
	double v = sqrt(Vx*Vx+Vy*Vy);
	//printf("v=%f\n",v);
	double Ux= Vx/v;
	double Uy= Vy/v;

	uint16_t L[2],R[2];
	L[0]= x1 - Uy*w - Ux*v;
	L[1]= y1 + Ux*w - Uy*v;
	R[0]= x1 + Uy*w - Ux*v;
	R[1]= y1 - Ux*w - Uy*v;
	//printf("L=%d-%d R=%d-%d\n",L[0],L[1],R[0],R[1]);

	this->drawLine(x0, y0, x1, y1, color);
	this->drawLine(x1, y1, L[0], L[1], color);
	this->drawLine(x1, y1, R[0], R[1], color);
	this->drawLine(L[0], L[1], R[0], R[1], color);

	int ww;
	for(ww=w-1;ww>0;ww--) {
		L[0]= x1 - Uy*ww - Ux*v;
		L[1]= y1 + Ux*ww - Uy*v;
		R[0]= x1 + Uy*ww - Ux*v;
		R[1]= y1 - Ux*ww - Uy*v;
		//printf("Fill>L=%d-%d R=%d-%d\n",L[0],L[1],R[0],R[1]);
		this->drawLine(x1, y1, L[0], L[1], color);
		this->drawLine(x1, y1, R[0], R[1], color);
	}
}

#ifdef USE_LVGL
#define BUFFER_SIZE (16000) // 16000 color per time

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[BUFFER_SIZE];

static void disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  LCD * display = (LCD *)disp->user_data;
  display->drawBitmap(area->x1, area->y1, area->x2, area->y2, &color_p->full);

  lv_disp_flush_ready(disp);
}

unsigned long last_touch_on_display  = 0;

void display_inp_feedback(lv_indev_drv_t *indev_driver, uint8_t event) {
  if((event == LV_EVENT_CLICKED) || (event == LV_EVENT_KEY)) {
    last_touch_on_display = millis();
  }
}

void LCD::useLVGL() {
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, BUFFER_SIZE);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = this->lcd_width;
  disp_drv.ver_res = this->lcd_height;
  disp_drv.flush_cb = disp_flush;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.user_data = this;
  lv_disp_drv_register(&disp_drv);
}

void LCD::loop() {
  static unsigned long timer = 0;
  if ((millis() < timer) || (timer == 0) || ((millis() - timer) >= 5)) {
    timer = millis();
    lv_timer_handler();
  }

  if (this->auto_sleep_after_sec > 0) {
    static bool display_enter_to_sleep = false;
    if ((millis() - last_touch_on_display) > (this->auto_sleep_after_sec * 1000)) {
      if (!display_enter_to_sleep) {
        this->off();
        digitalWrite(LCD_BL_PIN, LOW);
        display_enter_to_sleep = true;
      }
    } else {
      if (display_enter_to_sleep) {
        lv_obj_invalidate(lv_scr_act());
        lv_timer_handler();
        this->on();
        digitalWrite(LCD_BL_PIN, HIGH);
        display_enter_to_sleep = false;
      }
    }
  }
}
#endif

void LCD::enableAutoSleep(uint32_t timeout_in_sec) {
  this->auto_sleep_after_sec = timeout_in_sec;
}

void LCD::disableAutoSleep() {
  this->auto_sleep_after_sec = 0;
}

LCD Display;
