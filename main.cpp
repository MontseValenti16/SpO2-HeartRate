#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
// añadimos las librerias para la conexion a la API
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// Configuración del WiFi

const char* serverUrl = "http://13.217.180.236:8081/max30102/"; 



// Instancia del sensor MAX30105
MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

// Definir los pines para los LEDs
byte pulseLED = 13; // LED de pulso (puedes cambiar el pin si lo deseas)
byte readLED = 2;   // LED para indicar la lectura de datos (puedes cambiar el pin)

#if defined(AVR_ATmega328P) || defined(AVR_ATmega168)
// Arduino Uno no tiene suficiente SRAM para almacenar 100 muestras de datos en formato de 32 bits
// Para solucionar esto, se truncará el 16-bit MSB de los datos muestreados. Las muestras se vuelven de 16 bits.
uint16_t irBuffer[100];  // Datos del LED infrarrojo
uint16_t redBuffer[100]; // Datos del LED rojo
#else
uint32_t irBuffer[100];  // Datos del LED infrarrojo
uint32_t redBuffer[100]; // Datos del LED rojo
#endif

int32_t bufferLength;   // Longitud del buffer
int32_t spo2;           // Valor de SpO2
int8_t validSPO2;       // Indicador para ver si el cálculo de SpO2 es válido
int32_t heartRate;      // Valor de la frecuencia cardíaca
int8_t validHeartRate;  // Indicador para ver si el cálculo de la frecuencia cardíaca es válido

void setup()
{
  Serial.begin(115200);

  //aqui hacemos la conexion a internet....
  WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConectado a WiFi");
    Serial.print("Dirección IP del ESP32: ");
    Serial.println(WiFi.localIP());


  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)){
    Serial.println(F("MAX30105 no fue encontrado. Verifica las conexiones y la alimentación."));
    while (1);
  }
  byte ledBrightness = 60;  // Opciones: 0 = Apagado, 255 = 50mA
  byte sampleAverage = 4;   // Opciones: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;         // Opciones: 1 = Solo Rojo, 2 = Rojo + IR, 3 = Rojo + IR + Verde
  byte sampleRate = 100;    // Opciones: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;     // Opciones: 69, 118, 215, 411
  int adcRange = 4096;      // Opciones: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configurar el sensor con estos parámetros
}

void loop()
{
  bufferLength = 100; // Longitud del buffer de 100 almacena 4 segundos de muestras a 25sps

  // Leer las primeras 100 muestras y determinar el rango de la señal
  for (byte i = 0; i < bufferLength; i++)
  {
    while (particleSensor.available() == false) // ¿Tenemos datos nuevos?
      particleSensor.check(); // Verifica el sensor para nuevos datos

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // Hemos terminado con esta muestra, pasa a la siguiente muestra

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  // Calcular la frecuencia cardíaca y SpO2 después de las primeras 100 muestras (primeros 4 segundos de muestras)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Tomando muestras continuamente del MAX30102. La frecuencia cardíaca y SpO2 se calculan cada 1 segundo
  while (1)
  {
    // Descartar los primeros 25 conjuntos de muestras en la memoria y mover los 75 conjuntos de muestras restantes al principio
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    // Tomar 25 conjuntos de muestras antes de calcular la frecuencia cardíaca.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) // ¿Tenemos datos nuevos?
        particleSensor.check(); // Verifica el sensor para nuevos datos

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();

      // Enviar las muestras y el resultado de los cálculos al programa terminal a través de UART

      //red Intensidad de luz reflejada del LED rojo.
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);

      //ir → Intensidad de luz reflejada del LED infrarrojo (IR).
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);
    
      //HR → Frecuencia cardíaca (Heart Rate, HR) en latidos por minuto (BPM).
      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      //HRvalido → Indica si la medición de la frecuencia cardíaca es válida.
      Serial.print(F(", HRvalido="));
      Serial.print(validHeartRate, DEC);

      //SPO2=93 → Saturación de oxígeno en sangre (SpO2) en %.
      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      //SPO2valido=1 → Indica si la medición de SpO2 es válida.
      Serial.print(F(", SPO2Valido="));
      Serial.println(validSPO2, DEC);
    }
    sendToApi(heartRate, spo2);
    // Después de recopilar 25 nuevas muestras, recalcular HR y SpO2
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}


void sendToApi ( int bpm, int spo2 ) { 
  Serial.println("Verificando conexión WiFi...");
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        Serial.println("Conectado a WiFi, intentando conectar a la API...");

        // Crear JSON con los datos del sensor
        StaticJsonDocument<200> jsonDoc;
        jsonDoc["spo2"] = spo2;
        jsonDoc["bpm"] = bpm;
  

        String requestBody;
        serializeJson(jsonDoc, requestBody);

        // Enviar la solicitud POST
        Serial.println("Enviando solicitud a la API...");
        Serial.println(requestBody);  // 📌 Verifica el JSON antes de enviarlo

        http.begin(serverUrl);
        http.addHeader("Content-Type", "application/json");

        // Configurar el tiempo de espera
        http.setTimeout(20000);  // Tiempo de espera de 20 segundos

        int httpResponseCode = http.POST(requestBody);

        if (httpResponseCode > 0) {
            Serial.print("Respuesta de la API: ");
            Serial.println(httpResponseCode);
            String response = http.getString();
            Serial.println("Respuesta del servidor:");
            Serial.println(response);  // 📌 Muestra la respuesta del servidor
        } else {
            Serial.print("Error en la solicitud: ");
            Serial.println(http.errorToString(httpResponseCode));
        }

        http.end();
    } else {
        Serial.println("Error: No hay conexión WiFi.");
    }
}