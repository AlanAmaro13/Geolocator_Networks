/*  Name: Disposito Geolocalizador conectado a telecomunicaciones
    Date: 29 de mayo del 2023
    Author: Amaro, A.

    Description:
    El siguiente código tiene como finalidad incorporar un GPS que obtendrá la ubicación de manera progresiva,
    de esta manera, la ubicación podrá ser obtenida mediante un mensaje de texto.

    El disposito RECIBE un mensaje de texto y ENVIA al mismo contacto la ubicación.

    Incorpora un Bot de telegram conectado y encriptación de mensajes.

    V1.- FUNCIONAMIENTO DEL GPS en ARDNANO (Código de ejemplo) - EXITOSA
    V1.1 .- Funcionamiento en este código - EXITOSA
    V2 - Comprobar funcionamiento del módulo SIM800L. Enviar en la terminal a
    9600 el comenado AT y debemos recibir OK

    V2.1.- Incorporar funcionamiento del SIM800L, Recibir mensaje de texto - EXITOSA

    GPS (3.3, GND, TX4, RX3)
    SIM: +52.... - Utiliza una SIM Micro
    SIM (Alimentación externa de 3.7 V, GND unido con Arduino. TXD9, RXD2, RST 8)

    Usamos dos dispositos SoftwareSerial, GPS y SIM800L.

    Se recomienda usar baterías LiPo de 3.7 V y más de 500 mA
    Se recomienda usar una antena exterior a la dada por el Kit

    V2.2 - Incorpora el recibir sms y enviar un mensaje de respuesta sms (29/05/2023) - EXITOSA

    V3 - Incorpora la recepción GPS y reeenviar la ubicación cuando se encuentre lista. (EXITOSA)
    NOTA: No se puede utlizar dos dispositivos SS al mismo tiempo, por ello inicio primero SIM y después el GPS

    V4 - Incorpora funcionamiento del botón de pánico, utilizando interrupciones para enviar la ubicación a los números
    almacenados en la memoria del código - FALLIDA

    V5.- Incorpora la publicación de la ubicación en Telegram - FALLIDA

    ------------- V2 Código --------
    Se reincorpora todos los elementos principales del código anterior,
    se soluciona el problema del botón de pánico junto con la publicación
    de la ubicación en Telegram

    Se tiene mayor cuidado con las comunicaciones.

    V6.- Se incorpora interrupciones para solucionar el conflicto de la comunicación simultánea.

*/


// ---------------------- SIM ------------------
#include "Adafruit_FONA.h" // Importamos la libreria para uso del modulo SIM800L

#define FONA_RST 8  // Declaramos pin de reinicio

char replybuffer[255]; // Declaramos un buffer para las respuestas automaticas

#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
// Declaramos un if para uso
//de otros microcontroladores

#include <SoftwareSerial.h> // Importamos libreria para comunicación UART

#define FONA_RX 2 // Declaramos pines de recepción y transmisión. Colocamos RX para la interrupción.
#define FONA_TX 9

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX); // Declaramos nuestro objeto de comunicación
SoftwareSerial *fonaSerial = &fonaSS; // Declaramos nuestro pin de reinicio

#else
// En Leonardo/M0/etc, u otros con HardwareSerial usamos:
HardwareSerial *fonaSerial = &Serial1;

#endif


Adafruit_FONA fona = Adafruit_FONA(FONA_RST); // Inciamos pin de reinicio

// Inciamos características del bufffer.
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

char fonaNotificationBuffer[64];          //Para notificaciones del modulo
char smsBuffer[250];                      //  Para enviar sms

String emergencia_1 = "56" ;
String emergencia_2 = "55";  //Aquí almacenamos el número del que envío el sms de emergencia


// ------------------- GPS --------------
#include <TinyGPS.h> // Imporamos la libreria para el GPS
TinyGPS gps;  //  Declarmaos un objeto para su uso
SoftwareSerial gps_ss(4, 3);  // Iniciamos la configuración con puertos TX4 RX3
float flat, flon; // Declaramos una variable para almacenar la latitud y longitud
unsigned long age;
bool newData = false; // Declaramos una variable que indice cuando se tengan lecturas del sensor
String mensaje_gps; // En esta variable se almacena el mensaje la posición (Lat y lon)
String no_gps = "No es posible obtener la ubicación, intenta de nuevo en unos minutos";
#define LED1 13

// ------------------- Botón de Pánico -----------------
long timeCounter1 = 0;
int timeThreshold = 200;

unsigned long duracion_pausa = 35000;  // Duración de la pausa en milisegundos (en este caso, 2 segundos)
unsigned long tiempo_pasado = 0;
bool gps_datos = false;


// --------------- Setup() -------------------

void setup() {
  pinMode(LED1, OUTPUT); // Declaramos el LED que indica si el GPS está obteniendo datos

  attachInterrupt( digitalPinToInterrupt(FONA_RX), reenvio_gps , CHANGE ); // Declaramos la interrupción

  while (!Serial);

  Serial.begin(115200); // Inciamos comunicación Serial;
  // ------------- GPS ----------------------------
  Serial.println("Iniciando GPS...");
  gps_ss.begin(9600);
  Serial.println("GPS Iniciado en SS!");
  // --------------------------------------------

  Serial.println(F("Iniciando... (Puede tomar unos segundos)"));

  // -------- Funcionamiento módulo SIM ------------------
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("No se encuentra ningún módulo FONA"));
    while (1);
  }
  Serial.println(F("FONA está OK"));

  fonaSerial->print("AT+CNMI=2,1\r\n"); // Recibimos una notificación cada que se reciba un mensaje

  Serial.println("FONA listo!"); //

}





// ---------------------------- Loop() ---------------

void loop() {

  // Pimero obtendremos ubicación GPS y después la enviaremos:
  // --------------------------------- GPS INICIAL ----------------------------
  unsigned long tiempo_actual = millis();


  if ( tiempo_actual  >= duracion_pausa ) {

    gps_ss.listen();

    while ( gps_ss.available()) // En caso de que estemos recibiendo información del GPS
    {
      char c = gps_ss.read();
      if (gps.encode(c)) // En caso de recibir valores útiles
        newData = true;
        digitalWrite(LED1, HIGH); // Activamos el LED en caso de que se obtegan datos del GPS
    }

  }

  // ------------------------------ GPS CONTINUO -----------------------------

  if (newData) // Cuando recibamos información GPS comenzamos con el funcionamiento del SIM800L
  {

    gps.f_get_position(&flat, &flon, &age); // Obtenemos la posición

    (flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6); // Asignamos a sus variables.

    (flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);

  }


  if (gps_datos == true && millis() > duracion_pausa) {
    // ------------------------------- SIM ----------------------------------

    fonaSS.listen();

    Serial.println("Estamos dentro de reenvio_gps");

    char* bufPtr = fonaNotificationBuffer;    //Declaramos un pointer

    if ( fona.available() )      //Si ha recibido algún mensaje:
    {
      int slot = 0;            //Este es el slot del mensaje
      int charCount = 0;      // Este es nuestro contador char

      //Leemos la notificación dentro del bufferFona
      do  {
        *bufPtr = fona.read();
        Serial.write(*bufPtr);
        delay(1);
      } while ((*bufPtr++ != '\n') && (fona.available()) && (++charCount < (sizeof(fonaNotificationBuffer) - 1)));

      //Añadimos un terminal NULL a la notificación String
      *bufPtr = 0;

      //  Escaneamos la notificacion String buscando un mensaje:
      //  Si es un mensaje, lo almacenamos en slot
      if (1 == sscanf(fonaNotificationBuffer, "+CMTI: " FONA_PREF_SMS_STORAGE ",%d", &slot)) {
        Serial.print("slot: "); Serial.println(slot);

        char callerIDbuffer[32]; // Declaramos un buffer para almacenar el número del enviador

        // Imprimimos el número del enviador.
        if (! fona.getSMSSender(slot, callerIDbuffer, 31) ) {
          Serial.println("Didn't find SMS message in slot!");
        }
        Serial.print(F("FROM: ")); Serial.println(callerIDbuffer);

        // Imprimimos el contenido del mensaje:
        uint16_t smslen;
        if (fona.readSMS(slot, smsBuffer, 250, &smslen)) { // pass in buffer and max len!
          Serial.println(smsBuffer);
        }

        // Manda una respuesta automática
        Serial.println("Enviando respuesta...");
        mensaje_gps = "Ubicacion(Lat, Lon): \n" + String(flat, 6) + ", " + String(flon, 6);

        if (newData) {

          if (!fona.sendSMS(callerIDbuffer, mensaje_gps.c_str() )) {
            Serial.println(F("Failed"));
            Serial.println(callerIDbuffer);
          } else {
            Serial.println(F("Sent!"));
          }

        } else {

          if (!fona.sendSMS(callerIDbuffer, no_gps.c_str() )) {
            Serial.println(F("Failed"));
            Serial.println(callerIDbuffer);
          } else {
            Serial.println(F("Sent!"));
          }

        }

      } // FIN DE BUSQUEDA DEL SMS
    } // FIN DE FONA AVAILABLE
    gps_datos = false;
  } // FIN DEL PRIMER CONDICIONAL DEL SIM

} // FIN DEL LOOP


// --------------- REENVIO GPS ----------------------

void reenvio_gps() {

  gps_datos = true;

} // FIN DE LA FUNCION REENVIO.

/*
  void emergencia() {
  fonaSS.listen();

  if (millis() > timeCounter1 + timeThreshold) {
    Serial.println("Se ha presionado el botón de pánico");

    gps_ss.listen();
    String mensaje_emergencia = "He activado el botón de Pánico. Por favor, solicita mi ubicación" + mensaje_gps ;
    Serial.println(mensaje_emergencia);

    fonaSS.listen();
    if (!fona.sendSMS( emergencia_2.c_str(), mensaje_emergencia.c_str() )) {
      Serial.println(F("Imposible enviar mensaje de emergencia"));
    } else {
      Serial.println(F("Enviando mensaje de emergencia"));
    }

    timeCounter1 = millis();

  }
  }
*/
