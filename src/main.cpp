//*****************************************************************************
// Universidad del Valle de Guatemala
// BE3015: Electrónica Digital 2
// Sofía Salguero - 19236
// Proyecto # 2
// Código para ESP 32
//*****************************************************************************

//*****************************************************************************
//Librerias
//*****************************************************************************
#include <Arduino.h>    //Librería de Arduino
#include <Ultrasonic.h> //Librería de sensor ultrasónico
//#include <SPI.h>        //Librería SPI

//*****************************************************************************
//Definicion etiquetas
//*****************************************************************************

//Pines para Sensor ultrasónico
#define PIN_TRIGGER 26
#define PIN_ECHO 27

//Pines para conexion SPI
#define MOSI 23
#define MISO 19
#define SCK 18
#define CS 5
//*****************************************************************************
//Prototipos de funcion
//*****************************************************************************
void getDistance(void);
void uart(void);
//*****************************************************************************
//Varibles globales
//*****************************************************************************
int distancia = 0; //Distancia tomada por el sensor
int bandera = 1;
Ultrasonic ultrasonic(PIN_TRIGGER, PIN_ECHO); //Sensor ultrasónico
//SPIClass SPI1(VSPI); //Tipo de SPI

//*****************************************************************************
//Configuracion
//*****************************************************************************
void setup()
{
  //Configuración Serial
  Serial.begin(115200);
  Serial2.begin(115200);
}

//*****************************************************************************
//Loop principal
//*****************************************************************************
void loop()
{
  getDistance();
  //Serial.print("\nLa distancia medida es de: ");
  //Serial.print(distancia);
  //Serial.print(" cm");
  //Serial.println(bandera);
  uart();
}

//*****************************************************************************
//Tomar distancia con sensor
//*****************************************************************************
void getDistance(void)
{
  if (bandera == 0)
  {
    distancia = ultrasonic.read();
  }
}

//****************************************************************
// Conexión UART con Tiva C
//****************************************************************
void uart(void)
{
  while (Serial2.available() > 0) //Mira si hay algo en el buffer
  {
    bandera = Serial2.read();

    Serial2.write(distancia); //Escribe en UART2 la distancia
  }
}
