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

//*****************************************************************************
//Definicion etiquetas
//*****************************************************************************

//Pines para Sensor ultrasónico
#define PIN_TRIGGER 26
#define PIN_ECHO 27

//Prescaler
#define prescaler 80

//Temporizadores
hw_timer_t *timer = NULL;


//*****************************************************************************
//Prototipos de funcion
//*****************************************************************************
void IRAM_ATTR ISRTimer0();
void getDistance(void);
void uart(void);
void configurarTimer(void);


//*****************************************************************************
//Varibles globales
//*****************************************************************************
int distancia = 0; //Distancia tomada por el sensor
int bandera = 1; //bandera para comunicación UART TivaC con Esp32
boolean banderaUART = false; //bandera para comunicacion UART ESP32 con computadora
Ultrasonic ultrasonic(PIN_TRIGGER, PIN_ECHO); //Sensor ultrasónico

//*****************************************************************************
//ISR: interrupciones
//*****************************************************************************
void IRAM_ATTR ISRTimer0() //interrupción para timer
{
  banderaUART = true;
}

//*****************************************************************************
//Configuracion
//*****************************************************************************
void setup()
{
  //Configuración Serial
  Serial.begin(115200);
  Serial2.begin(115200);

  //Temporizadores
  configurarTimer();
}

//*****************************************************************************
//Loop principal
//*****************************************************************************
void loop()
{
  getDistance(); //tomar distancia
  uart(); //UART con TivaC
  
  if (banderaUART == true) //Si ya pasaron 3 segundos, imprime la distancia
  {
    banderaUART = false;
    Serial.print("\nLa distancia medida es de: ");
    Serial.print(distancia);
    Serial.print(" cm");
  }
}

//******************************************************************************
// Configuración Timers
//******************************************************************************
void configurarTimer(void) //Timer para displays
{
  //Fosc = 80MHz = 80,000,000 Hz
  //Fosc / Prescaler = 80,000,000 / 80 = 1,000,000
  //Tosc = 1/Fosc = 1uS

  //Timer 0, prescaler = 80, flanco de subida
  timer = timerBegin(0, prescaler, true);

  //Handler de la interrupción
  timerAttachInterrupt(timer, &ISRTimer0, true);

  //Tic = 1uS    3s= 3000000uS
  timerAlarmWrite(timer, 3000000, true);

  //Inicia alarma
  timerAlarmEnable(timer);
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
