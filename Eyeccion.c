// Juan R. Robles Gómez
// Última vez: PWM ANGULOS
// Control del sistema de eyección del piloto para PIC18F4520

// Librerias
#include <p18f4520.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Declaracion de funciones
float conversor (unsigned int inputdig);
float ftemperatura (float inputan);
float fpresion (float inputan);
unsigned int fvelocidad (float presion, float presionpitot, float densidad);
unsigned int faltitud (float temperatura, float presion);
float fdensidad (unsigned int altitud, float temperatura);
int porcentaje (float inputan);
int fanguloroll (float inputan);
int factrolido (float inputan);
void eyeccion ();
void semaforo (unsigned int oxigeno);
unsigned long fnumbin (unsigned int numero);
unsigned char signo (int numero);
unsigned char barralimest (unsigned int limestructural);

// Declaracion de variables
int i;
unsigned char encendido, modo;
unsigned int inputdig;
unsigned char servomot;
unsigned char signroll, signact;
float inputan;
unsigned long numbin;
float temperatura, presion, presionpitot, densidad;
unsigned int velocidad, altitud, o2sangre, limestructural;
int actrolido, anguloroll;
unsigned int absactrolido, absanguloroll;
unsigned char palanca;
unsigned char motcinturon, cohetes;

// Interrupciones
void R_Int_Alta (void); //Declaracion de la subrutina de tratamiento de interrupciones de alta prioridad

#pragma code Vector_Int_Alta = 0x08 //Vectorizacion de las interrupciones de alta prioridad
void Int_Alta (void)
{
    _asm GOTO R_Int_Alta _endasm
}
#pragma code

#pragma interrupt R_Int_Alta //Rutina de tratamiento de las interrupciones de alta prioridad

void R_Int_Alta (void)
{
  if (INTCONbits.TMR0IF == 1)	// Se comprueba si la interrupción es por desbordamiento del temp. 0
  {
    INTCONbits.TMR0IF = 0;	// Se pone a 0 el flag de desbordamiento del temp. 0

    if (servomot == 1)
    {
      if (PORTDbits.RD6 == 0) // 180 grados
      {
        LATDbits.LATD6 = 1;
        TMR0H = 63036/256;
        TMR0L = 63036%256; // 2.5 ms
      }
      else
      {
        LATDbits.LATD6 = 0;
        TMR0H = 45536/256;
        TMR0L = 45536%256; // 20 ms
      }
    }
    else
    {
      if (PORTDbits.RD6 == 0) // 90 grados
      {
        LATDbits.LATD6 = 1;
        TMR0H = 64064/256;
        TMR0L = 64064%256; // 1.5 ms
      }
      else
      {
        LATDbits.LATD6 = 0;
        TMR0H = 45536/256;
        TMR0L = 45536%256; // 20 ms
      }
    }
  }
}

// Funcion principal

void main (void)
{
  // Bloque de inicializaciones
    // Entradas y salidas
  TRISA = 0xFF; // Entrada
  TRISB = 0x07; // RB0, RB1  y RB2 entradas
  TRISC = 0x00; // Salida
  TRISD = 0x00; // Salida

    // Entradas convertidor analógico
  ADCON2 = 0xBC;  // Justificacion a derechas. Tadq = 20*TADC. Señal reloj convetirdor = FOSC/4
  ADCON1 = 0x08;  // Entradas analágicas AN0 - AN6. Tensiones ref: Vref+=Vcc y Vref-=GND.
  ADCON0 = 0x01;

    // PWM
  T0CON = 0x88; // Timer 0 en 16-bit timer mode. Prescalar desactivado. Timer ON
  TMR0H = 45536/256; // Valor inicial correspondiente a 20 ms
  TMR0L = 45536%256; 
  INTCONbits.GIE = 1; // Interrupciones habilitadas globalmente
  INTCONbits.TMR0IE = 1; // Timer 0 interrupt activado
  LATDbits.LATD6 = 1;
  servomot = 0;

  // Loop
  while (1)
  {
    // Palanca de eyección
    palanca = PORTBbits.RB2;

    if (palanca == 1)
    {
      eyeccion();
      break;
    }

    // Lecturas
      // Encendido
    encendido = PORTBbits.RB0; // 1 = 0N

    if (encendido == 1) // Si no está encendido no seguimos
    {
        // Modo
      modo = PORTBbits.RB1; // 1 = automático, 0 = manual

        // Display
      LATCbits.LATC0 = 0; // STB off

        // Temperatura
      ADCON0 = 0x03;  // AN0
      while (ADCON0bits.GO == 1); // Esperamos a que termine
      inputdig = ADRESH*256 + ADRESL;
      inputan = conversor (inputdig);
      temperatura = ftemperatura (inputan);

        // Presion
      ADCON0 = 0X07;  // AN1
      while (ADCON0bits.GO == 1);
      inputdig = ADRESH*256 + ADRESL;
      inputan = conversor (inputdig);
      presion = fpresion (inputan);

        //  Presion pitot
      ADCON0 = 0X0B;  // AN2
      while (ADCON0bits.GO == 1);
      inputdig = ADRESH*256 + ADRESL;
      inputan = conversor (inputdig);
      presionpitot = fpresion (inputan);

        // Limite estructural
      ADCON0 = 0x0F;  // AN3
      while (ADCON0bits.GO == 1);
      inputdig = ADRESH*256 + ADRESL;
      inputan = conversor (inputdig);
      limestructural = porcentaje (inputan);

        // Actitud de rolido
      ADCON0 = 0x13;  // AN4
      while (ADCON0bits.GO == 1);
      inputdig = ADRESH*256 + ADRESL;
      inputan = conversor (inputdig);
      actrolido = factrolido (inputan);

        // Ángulo roll
      ADCON0 = 0x17;  // AN5
      while (ADCON0bits.GO == 1);
      inputdig = ADRESH*256 + ADRESL;
      inputan = conversor (inputdig);
      anguloroll = fanguloroll (inputan);

        // Oxígeno en o2sangre
      ADCON0 = 0x1B;  // AN6
      while (ADCON0bits.GO == 1);
      inputdig = ADRESH*256 + ADRESL;
      inputan = conversor (inputdig);
      o2sangre = porcentaje (inputan);

      // Proceso de datos
      altitud = faltitud (temperatura, presion);
      densidad = fdensidad (altitud, temperatura);
      velocidad = fvelocidad (presion, presionpitot, densidad);

      // Muestra en display
        // Altitud
      numbin = fnumbin (altitud);

      for (i=0; i<20; i++)
      {
        LATCbits.LATC1 = (numbin%2);  // D, altitud
        numbin = numbin/2;
        LATCbits.LATC2 = 1;
        LATCbits.LATC2 = 0; // CLK, altitud
      }

        // Velocidad
      numbin = fnumbin (velocidad);

      for (i=0; i<16; i++)
      {
        LATCbits.LATC3 = (numbin%2);  // D, velocidad
        numbin = numbin/2;
        LATCbits.LATC4 = 1;
        LATCbits.LATC4 = 0; // CLK, velocidad
      }

        // Angulo roll
      signroll = signo (anguloroll);
      absanguloroll = fabs (anguloroll);
      numbin = fnumbin (absanguloroll);

      for (i=0; i<12; i++)
      {
        LATCbits.LATC5 = (numbin%2);  // D, angulo roll
        numbin = numbin/2;
        LATCbits.LATC6 = 1;
        LATCbits.LATC6 = 0; // CLK, angulo roll
      }
      LATCbits.LATC7 = signroll;

        //  Actitud rolido
      signact = signo (actrolido);
      absactrolido = fabs (actrolido);
      numbin = fnumbin (absactrolido);

      for (i=0; i<8; i++)
      {
        LATBbits.LATB5 = (numbin%2);  // D, actitud rolido
        numbin = numbin/2;
        LATBbits.LATB6 = 1;
        LATBbits.LATB6 = 0; // CLK, actitud rolido
      }
      LATBbits.LATB7 = signact;

        // Límite estructural
      numbin = barralimest (limestructural);

      for (i=0; i<5; i++)
      {
        LATDbits.LATD4 = (numbin%2);  // D, límite estructural
        numbin = numbin/2;
        LATDbits.LATD5 = 1;
        LATDbits.LATD5 = 0; // CLK, límite estructural
      }

      // Actuadores

      semaforo (o2sangre);

      if (modo == 1)  // Automático
      {
        if (o2sangre < 60 || limestructural > 80) // Comprobamos condiciones eyeccion
        {
          if (altitud > 500 && altitud < 20000 && velocidad < 361)
          {
            if (actrolido > -20 && actrolido < 20)
            {
              eyeccion();
              break;
            }
          }
          else
          {
            if (altitud < 500 && velocidad < 361)
            {
              if (anguloroll > -45 && anguloroll < 45)
              {
                eyeccion();
                break;
              }
              else
              {
                servomot = 1;
              }
            }
          }
        }
      }

      LATCbits.LATC0 = 1; // Activamos los displays (STB)
    }
  }
}

// Funciones
unsigned long fnumbin (unsigned int numero)
{
  unsigned int decmil, mil, cent, dec, unid;
  unsigned long numbin;

  decmil = numero/10000;
  numero = numero%10000;
  mil = numero/1000;
  numero = numero%1000;
  cent = numero/100;
  numero = numero%100;
  dec = numero/10;
  unid = numero%10;

  numbin = 65536*decmil + 4096*mil + 256*cent + 16*dec + unid;
  return (numbin);
}

unsigned char signo (int numero)
{
  unsigned char signo = 0;

  if (numero < 0)
  {
    signo = 1;
  }
  return (signo);
}

void eyeccion ()
{
  LATBbits.LATB3 = 1; // Motor cinturones
  LATBbits.LATB4 = 1; // Eyeccion
  LATCbits.LATC0 = 1; // Muestra displays
  for (i=0; i<10000; i++);
}

void semaforo (unsigned int oxigeno)
{
  if (oxigeno > 70) // Verde
  {
    LATDbits.LATD0 = 0;
    LATDbits.LATD1 = 1;
    LATDbits.LATD2 = 0;
    LATDbits.LATD3 = 0;
  }
  else
  {
    if (oxigeno > 60)  // Naranja y alarma
    {
      LATDbits.LATD0 = 1;
      LATDbits.LATD1 = 0;
      LATDbits.LATD2 = 1;
      LATDbits.LATD3 = 0;
    }
    else  // Rojo y alarma
    {
      LATDbits.LATD0 = 1;
      LATDbits.LATD1 = 0;
      LATDbits.LATD2 = 0;
      LATDbits.LATD3 = 1;
    }
  }

}

float conversor (unsigned int inputdig)
{
  float inputan;
  inputan = 0.00488281*inputdig; // Convertir el valor digital de 0 a 1023 a un valor de 0 a 5V
  return (inputan);
}

unsigned char barralimest (unsigned int limestructural)
{
  if (limestructural > 80)
  {
    return (31); // 0b11111
  }
  if (limestructural > 60)
  {
    return (15); // 0b01111
  }
  if (limestructural > 40)
  {
    return (7); // 0b00111
  }
  if (limestructural > 20)
  {
    return (3); // 0b00011
  }
  if (limestructural > 20)
  {
    return (1); // 0b00001
  }
  return (0); // 0b00000
}

int porcentaje (float inputan)
{
  int porcentaje;
  porcentaje = inputan*20;
  return (porcentaje);
}

int factrolido (float inputan)
{
  int actrolido;
  actrolido = inputan*20 - 50;
  return (actrolido);
}

int fanguloroll (float inputan)
{
  int anguloroll;
  anguloroll = inputan*72 - 180;
  return (anguloroll);
}

float ftemperatura (float inputan)
{
  float temperatura, v2, rx;
  v2 = (inputan/7.1428) + 2.126436;
  rx = (100*v2)/(5-v2);
  temperatura = (rx-102)/0.39083;
  return (temperatura);
}

float fpresion (float inputan)
{
  float presion;
  presion = inputan*22.24964367 + 10.5105559;
  return (presion);
}

unsigned int faltitud (float temperatura, float presion)
{
  unsigned int altitud;
  float alt;
  if (presion > 101.32)
  {
      presion = 101.32;
  }
  
  alt = ((pow((101.32/presion),(1/5.256))-1)*(temperatura + 273.15))/0.0065;
  altitud = alt;
  
  if (altitud > 99999)
  {
      altitud = 99999;
  }
  return (altitud);
}

float fdensidad (unsigned int altitud, float temperatura)
{
  float densidad;
  densidad = 348.42 * (1 - altitud*0.000105)/(temperatura + 273);
  return (densidad);
}

unsigned int fvelocidad (float presion, float presionpitot, float densidad)
{
  unsigned int velocidad;
  velocidad = pow((2*(presion*1000 - presionpitot*1000))/densidad, 0.5);
  return (velocidad);
}
