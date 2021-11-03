//*****************************************************************************
// Universidad del Valle de Guatemala
// BE3015: Electrónica Digital 2
// Sofía Salguero - 19236
// Proyecto # 2
// Código para Tiva
//*****************************************************************************

//*****************************************************************************
//Librerias
//*****************************************************************************

// Librería para SD
#include <SPI.h>
#include <SD.h>

// Librerías para Pantalla TFT
#include <stdint.h>
#include <stdbool.h>
#include <TM4C123GH6PM.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "bitmaps.h"
#include "font.h"
#include "lcd_registers.h"

//*****************************************************************************
//Definicion etiquetas
//*****************************************************************************

//Puertos Pantalla LCD
#define LCD_RST PD_0
#define LCD_CS PD_1
#define LCD_RS PD_2
#define LCD_WR PD_3
#define LCD_RD PE_1

//Variables para SPI con ESP32
#define MISO1 PF_0
#define MOSI1 PF_1
#define SCK1 PF_2
#define CS1 PF_3
//*****************************************************************************
//Varibles globales
//*****************************************************************************

//Pantalla TFT
int DPINS[] = {PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7}; //Pines
extern uint8_t fondo[]; //Fondo

//SD
File archivo;                                                   //Objeto de tipo archivo para escribir en micro SD

//Botones
int buttonState1;             // the current reading from the input pin
int lastButtonState1 = LOW;   // the previous reading from the input pin
long lastDebounceTime1 = 0;  // the last time the output pin was toggled
long debounceDelay1 = 50;    // the debounce time; increase if the output flickers

int buttonState2;             // the current reading from the input pin
int lastButtonState2 = LOW;   // the previous reading from the input pin
long lastDebounceTime2 = 0;  // the last time the output pin was toggled
long debounceDelay2 = 50;    // the debounce time; increase if the output flickers

byte distancia = 0;
int centena = 0;
int decena = 0;
int unidad = 0;

//***************************************************************************************************************************************
// Prototipo de funciones
//***************************************************************************************************************************************

//Funciones de SD
void writeSD(void);
void readSD(void);

//Funciones de pantalla TFT
void LCD_Init(void);
void LCD_CMD(uint8_t cmd);
void LCD_DATA(uint8_t data);
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);
void LCD_Clear(unsigned int c);
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void LCD_Print(String text, int x, int y, int fontSize, int color, int background);
void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]);
void LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[], int columns, int index, char flip, char offset);

void botones(void);

//***************************************************************************************************************************************
// Configuración
//***************************************************************************************************************************************
void setup()
{
  //Botones
  pinMode(PUSH1, INPUT_PULLUP);
  pinMode(PUSH2, INPUT_PULLUP);

  //Pantalla TFT
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  Serial.begin(115200);
  GPIOPadConfigSet(GPIO_PORTB_BASE, 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
  //Serial.println("Inicio");
  LCD_Init();
  LCD_Clear(0x00);
  LCD_Bitmap(0, 0, 320, 240, fondo);


  //Micro SD
  pinMode(PA_3, OUTPUT);
  SPI.setModule(0);

  // Estamos Inicializando la tarjeta SD
  if (!SD.begin(PA_3))
  {
    Serial.println("initialization failed!");
    return;
  }


  //UART con ESP32
  Serial.begin(115200);
  Serial2.begin(115200);


}
//***************************************************************************************************************************************
// Loop principal
//***************************************************************************************************************************************
void loop()
{
  botones();
  writeSD();
  Serial2.write(buttonState1); //Escribe en UART2
  distancia = Serial2.read(); //Lee en UART2

  int temp = distancia;
  centena = temp / 100.0;
  temp = temp - centena * 100.0;
  decena = temp / 10.0;
  temp = temp - decena * 10.0;
  unidad = temp;

  Serial.print("\nLa distancia medida es de: ");
  Serial.print(distancia);
  Serial.print(" cm");

  LCD_Print(String(centena), 150, 130, 2, 0x0000, 0xFF01);
  LCD_Print(String(decena), 165, 130, 2, 0x0000, 0xFF01);
  LCD_Print(String(unidad), 180, 130, 2, 0x0000, 0xFF01);
  LCD_Print("cm", 210, 130, 2, 0x0000, 0xFF01);


}

//***************************************************************************************************************************************
// Función para botones
//***************************************************************************************************************************************
void botones(void)
{
  // read the state of the switch into a local variable:
  int reading1 = digitalRead(PUSH1);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading1 != lastButtonState1) {
    // reset the debouncing timer
    lastDebounceTime1 = millis();
  }

  if ((millis() - lastDebounceTime1) > debounceDelay1) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    buttonState1 = reading1;
  }

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState1 = reading1;

  // read the state of the switch into a local variable:
  int reading2 = digitalRead(PUSH2);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading2 != lastButtonState2) {
    // reset the debouncing timer
    lastDebounceTime2 = millis();
  }

  if ((millis() - lastDebounceTime2) > debounceDelay2) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    buttonState2 = reading2;
  }

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState2 = reading2;
}
//***************************************************************************************************************************************
// Función para leer SD
//***************************************************************************************************************************************
void readSD(void)
{
  archivo = SD.open("DATA.CSV");
  if (archivo)
  {
    Serial.println("El archivo contiene lo siguiente:");

    // read from the file until there's nothing else in it:
    while (archivo.available())
    {
      Serial.write(archivo.read());
    }
    // close the file:
    archivo.close();
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

//***************************************************************************************************************************************
// Función para escribir en SD
//***************************************************************************************************************************************
void writeSD(void)
{

  archivo = SD.open("DATA.CSV", FILE_WRITE);

  // if the file opened okay, write to it:
  if (archivo)
  {
    if (buttonState2 == 0)
    {
      Serial.println("Escribiendo data");

      Serial.print("Distancia: ");
      Serial.print(distancia);

      archivo.print(distancia);
      archivo.println(",");

      // close the file:
      archivo.close();
      Serial.println("done.");
    }
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println("error opening data.csv");
  }
}

//***************************************************************************************************************************************
// Función para inicializar LCD
//***************************************************************************************************************************************
void LCD_Init(void)
{
  pinMode(LCD_RST, OUTPUT);
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_WR, OUTPUT);
  pinMode(LCD_RD, OUTPUT);
  for (uint8_t i = 0; i < 8; i++)
  {
    pinMode(DPINS[i], OUTPUT);
  }
  //****************************************
  // Secuencia de Inicialización
  //****************************************
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, HIGH);
  digitalWrite(LCD_RD, HIGH);
  digitalWrite(LCD_RST, HIGH);
  delay(5);
  digitalWrite(LCD_RST, LOW);
  delay(20);
  digitalWrite(LCD_RST, HIGH);
  delay(150);
  digitalWrite(LCD_CS, LOW);
  //****************************************
  LCD_CMD(0xE9); // SETPANELRELATED
  LCD_DATA(0x20);
  //****************************************
  LCD_CMD(0x11); // Exit Sleep SLEEP OUT (SLPOUT)
  delay(100);
  //****************************************
  LCD_CMD(0xD1); // (SETVCOM)
  LCD_DATA(0x00);
  LCD_DATA(0x71);
  LCD_DATA(0x19);
  //****************************************
  LCD_CMD(0xD0); // (SETPOWER)
  LCD_DATA(0x07);
  LCD_DATA(0x01);
  LCD_DATA(0x08);
  //****************************************
  LCD_CMD(0x36);                       // (MEMORYACCESS)
  LCD_DATA(0x40 | 0x80 | 0x20 | 0x08); // LCD_DATA(0x19);
  //****************************************
  LCD_CMD(0x3A);  // Set_pixel_format (PIXELFORMAT)
  LCD_DATA(0x05); // color setings, 05h - 16bit pixel, 11h - 3bit pixel
  //****************************************
  LCD_CMD(0xC1); // (POWERCONTROL2)
  LCD_DATA(0x10);
  LCD_DATA(0x10);
  LCD_DATA(0x02);
  LCD_DATA(0x02);
  //****************************************
  LCD_CMD(0xC0); // Set Default Gamma (POWERCONTROL1)
  LCD_DATA(0x00);
  LCD_DATA(0x35);
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0x02);
  //****************************************
  LCD_CMD(0xC5);  // Set Frame Rate (VCOMCONTROL1)
  LCD_DATA(0x04); // 72Hz
  //****************************************
  LCD_CMD(0xD2); // Power Settings  (SETPWRNORMAL)
  LCD_DATA(0x01);
  LCD_DATA(0x44);
  //****************************************
  LCD_CMD(0xC8); //Set Gamma  (GAMMASET)
  LCD_DATA(0x04);
  LCD_DATA(0x67);
  LCD_DATA(0x35);
  LCD_DATA(0x04);
  LCD_DATA(0x08);
  LCD_DATA(0x06);
  LCD_DATA(0x24);
  LCD_DATA(0x01);
  LCD_DATA(0x37);
  LCD_DATA(0x40);
  LCD_DATA(0x03);
  LCD_DATA(0x10);
  LCD_DATA(0x08);
  LCD_DATA(0x80);
  LCD_DATA(0x00);
  //****************************************
  LCD_CMD(0x2A); // Set_column_address 320px (CASET)
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0x3F);
  //****************************************
  LCD_CMD(0x2B); // Set_page_address 480px (PASET)
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0xE0);
  //  LCD_DATA(0x8F);
  LCD_CMD(0x29); //display on
  LCD_CMD(0x2C); //display on

  LCD_CMD(ILI9341_INVOFF); //Invert Off
  delay(120);
  LCD_CMD(ILI9341_SLPOUT); //Exit Sleep
  delay(120);
  LCD_CMD(ILI9341_DISPON); //Display on
  digitalWrite(LCD_CS, HIGH);
}

//***************************************************************************************************************************************
// Función para enviar comandos a la LCD - parámetro (comando)
//***************************************************************************************************************************************
void LCD_CMD(uint8_t cmd)
{
  digitalWrite(LCD_RS, LOW);
  digitalWrite(LCD_WR, LOW);
  GPIO_PORTB_DATA_R = cmd;
  digitalWrite(LCD_WR, HIGH);
}

//***************************************************************************************************************************************
// Función para enviar datos a la LCD - parámetro (dato)
//***************************************************************************************************************************************
void LCD_DATA(uint8_t data)
{
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, LOW);
  GPIO_PORTB_DATA_R = data;
  digitalWrite(LCD_WR, HIGH);
}

//***************************************************************************************************************************************
// Función para definir rango de direcciones de memoria con las cuales se trabajara (se define una ventana)
//***************************************************************************************************************************************
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2)
{
  LCD_CMD(0x2a); // Set_column_address 4 parameters
  LCD_DATA(x1 >> 8);
  LCD_DATA(x1);
  LCD_DATA(x2 >> 8);
  LCD_DATA(x2);
  LCD_CMD(0x2b); // Set_page_address 4 parameters
  LCD_DATA(y1 >> 8);
  LCD_DATA(y1);
  LCD_DATA(y2 >> 8);
  LCD_DATA(y2);
  LCD_CMD(0x2c); // Write_memory_start
}

//***************************************************************************************************************************************
// Función para borrar la pantalla - parámetros (color)
//***************************************************************************************************************************************
void LCD_Clear(unsigned int c)
{
  unsigned int x, y;
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  SetWindows(0, 0, 319, 239); // 479, 319);
  for (x = 0; x < 320; x++)
    for (y = 0; y < 240; y++)
    {
      LCD_DATA(c >> 8);
      LCD_DATA(c);
    }
  digitalWrite(LCD_CS, HIGH);
}

//***************************************************************************************************************************************
// Función para dibujar una línea horizontal - parámetros ( coordenada x, cordenada y, longitud, color)
//***************************************************************************************************************************************
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c)
{
  unsigned int i, j;
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  l = l + x;
  SetWindows(x, y, l, y);
  j = l; // * 2;
  for (i = 0; i < l; i++)
  {
    LCD_DATA(c >> 8);
    LCD_DATA(c);
  }
  digitalWrite(LCD_CS, HIGH);
}

//***************************************************************************************************************************************
// Función para dibujar una línea vertical - parámetros ( coordenada x, cordenada y, longitud, color)
//***************************************************************************************************************************************
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c)
{
  unsigned int i, j;
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  l = l + y;
  SetWindows(x, y, x, l);
  j = l; //* 2;
  for (i = 1; i <= j; i++)
  {
    LCD_DATA(c >> 8);
    LCD_DATA(c);
  }
  digitalWrite(LCD_CS, HIGH);
}

//***************************************************************************************************************************************
// Función para dibujar un rectángulo - parámetros ( coordenada x, cordenada y, ancho, alto, color)
//***************************************************************************************************************************************
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c)
{
  H_line(x, y, w, c);
  H_line(x, y + h, w, c);
  V_line(x, y, h, c);
  V_line(x + w, y, h, c);
}

//***************************************************************************************************************************************
// Función para dibujar un rectángulo relleno - parámetros ( coordenada x, cordenada y, ancho, alto, color)
//***************************************************************************************************************************************
void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c)
{
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 = x + w;
  y2 = y + h;
  SetWindows(x, y, x2 - 1, y2 - 1);
  unsigned int k = w * h * 2 - 1;
  unsigned int i, j;
  for (int i = 0; i < w; i++)
  {
    for (int j = 0; j < h; j++)
    {
      LCD_DATA(c >> 8);
      LCD_DATA(c);

      //LCD_DATA(bitmap[k]);
      k = k - 2;
    }
  }
  digitalWrite(LCD_CS, HIGH);
}

//***************************************************************************************************************************************
// Función para dibujar texto - parámetros ( texto, coordenada x, cordenada y, color, background)
//***************************************************************************************************************************************
void LCD_Print(String text, int x, int y, int fontSize, int color, int background)
{
  int fontXSize;
  int fontYSize;

  if (fontSize == 1)
  {
    fontXSize = fontXSizeSmal;
    fontYSize = fontYSizeSmal;
  }
  if (fontSize == 2)
  {
    fontXSize = fontXSizeBig;
    fontYSize = fontYSizeBig;
  }

  char charInput;
  int cLength = text.length();
  //Serial.println(cLength, DEC);
  int charDec;
  int c;
  int charHex;
  char char_array[cLength + 1];
  text.toCharArray(char_array, cLength + 1);
  for (int i = 0; i < cLength; i++)
  {
    charInput = char_array[i];
    //Serial.println(char_array[i]);
    charDec = int(charInput);
    digitalWrite(LCD_CS, LOW);
    SetWindows(x + (i * fontXSize), y, x + (i * fontXSize) + fontXSize - 1, y + fontYSize);
    long charHex1;
    for (int n = 0; n < fontYSize; n++)
    {
      if (fontSize == 1)
      {
        charHex1 = pgm_read_word_near(smallFont + ((charDec - 32) * fontYSize) + n);
      }
      if (fontSize == 2)
      {
        charHex1 = pgm_read_word_near(bigFont + ((charDec - 32) * fontYSize) + n);
      }
      for (int t = 1; t < fontXSize + 1; t++)
      {
        if ((charHex1 & (1 << (fontXSize - t))) > 0)
        {
          c = color;
        }
        else
        {
          c = background;
        }
        LCD_DATA(c >> 8);
        LCD_DATA(c);
      }
    }
    digitalWrite(LCD_CS, HIGH);
  }
}

//***************************************************************************************************************************************
// Función para dibujar una imagen a partir de un arreglo de colores (Bitmap) Formato (Color 16bit R 5bits G 6bits B 5bits)
//***************************************************************************************************************************************
void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[])
{
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 = x + width;
  y2 = y + height;
  SetWindows(x, y, x2 - 1, y2 - 1);
  unsigned int k = 0;
  unsigned int i, j;

  for (int i = 0; i < width; i++)
  {
    for (int j = 0; j < height; j++)
    {
      LCD_DATA(bitmap[k]);
      LCD_DATA(bitmap[k + 1]);
      //LCD_DATA(bitmap[k]);
      k = k + 2;
    }
  }
  digitalWrite(LCD_CS, HIGH);
}

//***************************************************************************************************************************************
// Función para dibujar una imagen sprite - los parámetros columns = número de imagenes en el sprite, index = cual desplegar, flip = darle vuelta
//***************************************************************************************************************************************
void LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[], int columns, int index, char flip, char offset)
{
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 = x + width;
  y2 = y + height;
  SetWindows(x, y, x2 - 1, y2 - 1);
  int k = 0;
  int ancho = ((width * columns));
  if (flip)
  {
    for (int j = 0; j < height; j++)
    {
      k = (j * (ancho) + index * width - 1 - offset) * 2;
      k = k + width * 2;
      for (int i = 0; i < width; i++)
      {
        LCD_DATA(bitmap[k]);
        LCD_DATA(bitmap[k + 1]);
        k = k - 2;
      }
    }
  }
  else
  {
    for (int j = 0; j < height; j++)
    {
      k = (j * (ancho) + index * width + 1 + offset) * 2;
      for (int i = 0; i < width; i++)
      {
        LCD_DATA(bitmap[k]);
        LCD_DATA(bitmap[k + 1]);
        k = k + 2;
      }
    }
  }
  digitalWrite(LCD_CS, HIGH);
}
