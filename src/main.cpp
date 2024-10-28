#include <SPI.h>
#include <U8g2lib.h>
#define rotation U8G2_R2
#define RESET 20
#define DC    21
#define CS    22
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(rotation, /* cs=*/CS, /* dc=*/DC, /* reset=*/RESET);
uint8_t WID = 128;
uint8_t HGT = 64;

#include <Adafruit_AHTX0.h>
Adafruit_AHTX0 aht;

#include <BMx280I2C.h>
#define I2C_ADDRESS 0x77
BMx280I2C bmx280(I2C_ADDRESS);

// C O N S T A N T S
// ===========================================================================================
const uint8_t *font = u8g2_font_spleen16x32_me;
// u8g2.setFont(font);
// int8_t ascent = u8g2.getAscent();
// int8_t descent = u8g2.getDescent();
// int8_t mchh = u8g2.getMaxCharHeight();
// int8_t xoffs = u8g2.getXOffsetUTF8(" ");
// const uint8_t CHARWID = u8g2.getUTF8Width(" ");
const uint8_t CHARWID = 16;
const uint8_t CHARHGT = 26;
const uint8_t LINEOFFSET = 20;
const uint8_t SCROLLSPEED = 7; // scroll speed (number of characters per second)
const uint8_t SCROLLSTEP = 2;  // the number of columns (pixels) by which the image on the display
                               // is shifted during one scrolling step
                               // for higher speeds, it is necessary to set a larger step
const uint16_t DISPREFRESH = (SCROLLSTEP * 1000 / (SCROLLSPEED * CHARWID));
const uint8_t DISPWID = WID / CHARWID; // display width [characters]
const uint32_t TEXTCOLOR = 1;
const uint32_t BACKCOLOR = 0;
const uint8_t CONTRAST = 16;
const uint8_t ROWSNUM = 2;             // Number of lines with running text
const uint16_t BUFFLEN = 232;
const uint8_t MAXGAP = 32;
const char gaps[MAXGAP + 1] = "                                ";
const uint8_t SCROLLGAP = 7;           // the number of spaces to insert between two instances of text
const uint16_t MESSPERIOD = 30000;     // one measurement every 30 seconds
struct RowData
{
  uint8_t ypos;
  char input[BUFFLEN + MAXGAP + 1];
  char rowmap[BUFFLEN + MAXGAP + 1];
  uint8_t rowlen;
  uint8_t lock;
  uint8_t type = 0; // 0-none, 1-plane text, 2-scrolled text, 3-clear
  bool updated = false;
  int16_t scrolloffset; // current offset for the scrolling text
  uint16_t overjump;
};

// G L O B A L  V A R I A B L E S
// ===========================================================================================
struct RowData rows[ROWSNUM]; // If done this way, the declaration does NOT need to be completed in setup()
// struct RowData *rows;      // If done this way, the declaration must be completed in setup() using malloc
                              // including setting of initial values ​​!!!
unsigned long prevMillis = 0;
unsigned long prevMess = 0;
bool displayflag = false;
float oldbmxtemp = 0;
double oldbmxpress = 0;
float oldaht20temp = 0.0;
float oldaht20hum = 0.0;

// R O U T I N E S
// ===========================================================================================
void updateLine(uint8_t row, char *input)
{
  uint8_t k = 0;
  uint8_t val;
  uint8_t skip = 0;
  rows[row].lock = 1;

  char *p = input;
  for (uint8_t i = 0; i < strlen(input); i++)
  {
    if (skip > 0)
    {
      skip--;
    }
    else
    {
      rows[row].rowmap[k] = i;
      k++;
      val = (uint8_t)*p;
      if ((val & 0b10000000) == 0) // One-byte utf-8 char
      {
        p++;
        continue;
      }
      else if ((val & 0b11100000) == 0b11000000) // Two-bytes utf-8 char
      {
        skip = 1;
      }
      else if ((val & 0b11110000) == 0b11100000) // Three-bytes utf-8 char
      {
        skip = 2;
      }
      else if ((val & 0b11110000) == 0b11110000) // Four-bytes utf-8 char
      {
        skip = 3;
      }
    }
    p++;
  }

  rows[row].rowmap[k] = strlen(input);
  rows[row].rowlen = k;
  rows[row].input[0] = '\0';
  strncat(rows[row].input, gaps, SCROLLGAP);
  strncat(rows[row].input, input, BUFFLEN - 1);
  rows[row].overjump = CHARWID * (k + SCROLLGAP);
  strncat(rows[row].input, gaps, SCROLLGAP);

  if (SCROLLGAP <= DISPWID)
  {
    strncat(rows[row].input, input, rows[row].rowmap[DISPWID - SCROLLGAP]);
  }

  rows[row].type = 2; // In this demo application, it is given in advance 
                      // that the text is longer than the display and therefore must run
  rows[row].scrolloffset = 0;
  rows[row].lock = 0;
  rows[row].updated = true;
}

void setup()
{
  Serial.begin(115200);
  rows[0].ypos = 0;
  rows[1].ypos = 37;
  delay(3000);                // for serial console
  u8g2.setBusClock(32000000); // increase SPI speed !
  u8g2.setContrast(CONTRAST); // not working for my display ???
  u8g2.begin();
  u8g2.setFontMode(0);
  u8g2.setFontDirection(0);
  u8g2.setDrawColor(1);
  u8g2.enableUTF8Print();
  u8g2.setFont(u8g2_font_10x20_mf);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 20, "BMP280+AHT20");
  u8g2.drawStr(0, 50, "Demo");
  u8g2.sendBuffer();
  u8g2.setFont(font);

  delay(3000);
  u8g2.clearBuffer();
}

void clearLine(uint8_t line, uint8_t setnone = 0)
{
  if (rows[line].type != 0)
  {
    if (setnone == 1)
    {
      rows[line].type = 0;
    }
    u8g2.setDrawColor(BACKCOLOR);
    u8g2.drawBox(0, rows[line].ypos, WID, CHARHGT);
  }
  u8g2.setDrawColor(TEXTCOLOR);
}


void loop()
{
  unsigned long currmill = millis();
  if (currmill - prevMillis >= DISPREFRESH)
  {
    prevMillis = currmill;
    for (uint8_t row = 0; row < ROWSNUM; row++)
    {
      if ((rows[row].type == 2) && (rows[row].lock == 0))
      {
        u8g2.drawUTF8(rows[row].scrolloffset, rows[row].ypos + LINEOFFSET, rows[row].input);
        displayflag = true;

        rows[row].scrolloffset = rows[row].scrolloffset - SCROLLSTEP;
        if (rows[row].scrolloffset <= -(rows[row].overjump))
        {
          rows[row].scrolloffset += rows[row].overjump;
        }
      }
    }
  }
  // handle short lines (no scroll)
  //---------------------------------------------------------
  for (uint8_t row = 0; row < ROWSNUM; row++)
  {
    if ((rows[row].type == 1) && rows[row].updated)
    {
      clearLine(row, 0);
      u8g2.drawUTF8(0, rows[row].ypos + LINEOFFSET, rows[row].input + SCROLLGAP);
      displayflag = true;
      rows[row].updated = false;
    }
  }
  if (displayflag)
  {
    // u8g2.updateDisplay();
    u8g2.sendBuffer();
    displayflag = false;
  }
}

//===============================================================================================
// CORE 1
//===============================================================================================

int sensorpower = 6;
char buf[4];

void setup1()
{
  pinMode(sensorpower, OUTPUT);
}

sensors_event_t humidity, temp;
bool flag1 = false;
bool flag2 = false;
char buffer[64];
char dispbuffer[128];
float bmxtemp;
double bmxpress;

void loop1()
{
  unsigned long currentMillis = millis();
  if ((currentMillis - prevMess >= MESSPERIOD) || (prevMess == 0))
  {
    prevMess = currentMillis;

    digitalWrite(sensorpower, HIGH); // turn the sensor ON
    Wire.begin();
    delay(200);
    if (!aht.begin())
    {
      Serial.println("AHT20 not detected. Please check wiring. Freezing.");
      while (1)
        ;
    }
    Serial.println("AHT20 acknowledged");
    aht.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data

    if (!bmx280.begin())
    {
      Serial.println("BMX280 not detected. Please check wiring. Freezing.");
      while (1)
        ;
    }
    else
    {
      if (bmx280.isBME280())
        Serial.println("BME280 acknowledged");
      else
        Serial.println("BMP280 acknowledged");

      bmx280.resetToDefaults(); // reset sensor to default parameters
      bmx280.writePowerMode(bmx280.BMx280_MODE_FORCED);
      bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
      bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);
      bmx280.measure(); // Start a conversion
    }
    //
    delay(100);
    flag1 = true;
    flag2 = true;
  }
  if (flag1)
  {
    Serial.println("\nAHT20");
    sprintf(buffer, "Temperature     = %.2f *C", temp.temperature);
    Serial.println(buffer);
    sprintf(buffer, "Humidity        = %.2f RH %%", humidity.relative_humidity);
    Serial.println(buffer);
    if ((temp.temperature != oldaht20temp) || (humidity.relative_humidity != oldaht20hum))
    {
      oldaht20temp = temp.temperature;
      oldaht20hum = humidity.relative_humidity;
      sprintf(dispbuffer, "AHT20: Temperature = %.2f °C, Humidity = %.2f RH %%", temp.temperature, humidity.relative_humidity);
      updateLine(1, dispbuffer);
    }
    flag1 = false;
  }
  if (flag2)
  {
    if (bmx280.hasValue())
    {
      bmxtemp = bmx280.getTemperature();
      bmxpress = bmx280.getPressure64();
      Serial.println("\nBMP280");
      sprintf(buffer, "Temperature     = %.2f *C", bmxtemp);
      Serial.println(buffer);
      sprintf(buffer, "Pressure        = %.2f kPa\n", bmxpress / 1000.f);
      Serial.println(buffer);
      if ((bmxtemp != oldbmxtemp) || (bmxpress != oldbmxpress))
      {
        oldbmxpress = bmxpress;
        oldbmxtemp = bmxtemp;
        sprintf(dispbuffer, "BMP280: Temperature = %.2f °C, Pressure = %.2f kPa", bmxtemp, bmxpress / 1000.f);
        updateLine(0, dispbuffer);
      }

      Wire.end();
      digitalWrite(sensorpower, LOW); // turn the sensor OFF
      flag2 = false;
    }
  }
}