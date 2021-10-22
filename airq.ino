//===========================================================================================
// Project:     AIRQ
// Description: esp32 that sends co2, pm1, pm2.5, pm10, gamma radiation dosis, temperature, pressure, humidty and air quality to a mqtt broker
//
//===========================================================================================
// Libaries
#include "Adafruit_BME680.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include "MHZ19.h"
#include <PMserial.h>


// Constants
#define SEALEVELPRESSURE_HPA (1013.25)
const char* ssid = "FRITZ!Box";
const char* password = "8789032057293363";

char *mqttServer = "192.168.178.61";
int mqttPort = 1883;
const char* mqttDeviceId = "airq";

const int rx_pin = 19; //Serial rx pin
const int tx_pin = 18; //Serial tx pin 

const            float GMZ_factor_uSvph     = 1/12.2792   ; // for SI-22

int PIN_HV_FET_OUTPUT       =  26;  
int PIN_HV_CAP_FULL_INPUT   =  13;  // !! has to be capable of "interrupt on change"
int PIN_GMZ_count_INPUT     =  27;  // !! has to be capable of "interrupt on change"
// BME680 - END
// Wifi - BEGIN

// CO2 Sensor - BEGIN
MHZ19 *mhz19_uart = new MHZ19(rx_pin,tx_pin);
// CO2 Sensor - END
SerialPM pms(PMSx003, 16, 17);  // PMSx003, RX, TX

WiFiClient wifiClient;
PubSubClient client(wifiClient); 




// Parameters
bool debug                  = true; // false: no debug output will be printed
                                    // true : debug output will be printed
bool speaker_tick           = false;// false: no GMZ-Tick, no flickering LED
                                    // true : GMZ-Tick will be sounded, LED flickers every GMZ-Tick


                  
// Variables
volatile bool          GMZ_cap_full         = 0;
volatile unsigned char isr_GMZ_counts       = 0;
volatile unsigned long isr_count_timestamp  = millis(); 
         unsigned char GMZ_counts           = 0;
         unsigned long count_timestamp      = millis(); 
         unsigned long last_count_timestamp = millis(); 
         unsigned long time_difference      = 1000; 
         unsigned char last_GMZ_counts      = 0;
         unsigned char speaker_count        = 0;
              uint32_t HV_pulse_count       = 0;
         float         Count_Rate           = 0.0;
         float         Dose_Rate            = 0.0;

         
// ISRs
void isr_GMZ_capacitor_full() {
  GMZ_cap_full = 1;
  Serial.println("cap"); 
}

void isr_GMZ_count() {
  isr_GMZ_counts++;                                     // Count
  isr_count_timestamp = millis();                       // notice (System)-Time of the Count
  Serial.println("gmz"); 
}


// Sub-Functions
void reconnect()  // connecting to mqtt broker
{
  while (!client.connected()) 
  {
    Serial.print("connecting to mosquitto");
    Serial.println(client.state());
    if (client.connect(mqttDeviceId, "admin", "admin"))  // 
    {
      Serial.println("connected.");
     
    } else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
} 


int jb_HV_gen_charge__chargepules() {
  int  chargepules  = 0;
       GMZ_cap_full = 0;
  do {
    digitalWrite(PIN_HV_FET_OUTPUT, HIGH);              // turn the HV FET on
  delayMicroseconds(1500);                            // 5000 usec gives 1,3 times more charge, 500 usec gives 1/20 th of charge
    digitalWrite(PIN_HV_FET_OUTPUT, LOW);               // turn the HV FET off
    delayMicroseconds(1000);                          
  chargepules++;
  Serial.println(chargepules); 
                                        
  }                                                     
  while ( (chargepules < 1000) && !GMZ_cap_full);       // either Timeout or capacitor full stops this loop
  return chargepules;
}




void setup() {
  Serial.begin(115200);   // set and init Serial Communication
  while (!Serial) {};

  client.setServer(mqttServer, 1883); // setting mqtt broker 
  
  WiFi.mode(WIFI_STA); 
  if(WiFi.status() != WL_CONNECTED){
      Serial.print("Attempting to connect");  // connecting to wifi
      while(WiFi.status() != WL_CONNECTED){
        WiFi.begin(ssid, password); 
        delay(5000);     
      } 
      Serial.println("\nConnected.");
   }
    
   mhz19_uart->begin(rx_pin, tx_pin);   // stating co2 serical communication 
   mhz19_uart->setAutoCalibration(false); // turning off auto calibration (can cause errors)
   
   pms.init(); //stating fine particles sensor
   
   delay(3000); // waiting for sensor startup
   
   Serial.print("MH-Z19 now warming up...  status:"); // checking co2 sensor
   Serial.println(mhz19_uart->getStatus());
   if (!client.connected()) // connecting to mqtt broker
    {
      reconnect();
    }
  client.loop();
  
  sleep(20);
  
  // set IO-Pins  
  pinMode      (PIN_HV_FET_OUTPUT  , OUTPUT);    

  // set Interrupts (on pin change), attach interrupt handler
  attachInterrupt (digitalPinToInterrupt (PIN_HV_CAP_FULL_INPUT), isr_GMZ_capacitor_full, RISING);// Capacitor full
  attachInterrupt (digitalPinToInterrupt (PIN_GMZ_count_INPUT), isr_GMZ_count, FALLING);          // GMZ pulse detected
  

  
  noInterrupts();

  
  // Write Header of Table  
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("GMZ_counts\tTime_difference\tCount_Rate\tDose_Rate\tHV Pulses");
  Serial.println("[Counts]  \t[ms]           \t[cps]     \t[uSv/h]  \t[-]");
  Serial.println("----------------------------------------------------------------------------");
  interrupts();   
  
  
  jb_HV_gen_charge__chargepules(); // charging the capacitor
}
  
  
void loop() {
  


  // read out values from ISR
  noInterrupts();                                                  // disable Interrupts to be able to read the variables of the ISR correctly
  GMZ_counts           = isr_GMZ_counts     ;                    // copy values from ISR
  count_timestamp      = isr_count_timestamp;                    
  interrupts();                                                    // re-enable Interrupts  
  
  // Check if there are enough pules detected or if enough time has elapsed. If yes than its 
  // time to charge the HV-capacitor and calculate the pulse rate  
  if ((GMZ_counts>=100) || ((count_timestamp - last_count_timestamp)>=10000)) {  
    pms.read(); // getting fine particles data
    
    isr_GMZ_counts       = 0;                                      // initialize ISR values
    time_difference = count_timestamp - last_count_timestamp;      // Calculate all derived values
    last_count_timestamp = count_timestamp;                        // notice the old timestamp
    HV_pulse_count       = jb_HV_gen_charge__chargepules();        // Charge HV Capacitor    
    Count_Rate = (float)GMZ_counts*1000.0/(float)time_difference; 
    Dose_Rate = Count_Rate *GMZ_factor_uSvph;

    
    measurement_t m = mhz19_uart->getMeasurement(); //getting co2 data
    int co3 = m.co2_ppm;
    if (co3 < 400 ) //correcting for errors, the lowest value of the sensor is 400, everything below is an error
    {
      co3 = NULL;
    }
    float co2 = co3;
  
    // sending data to mqtt broker
    Serial.print("Temperature: ");
    Serial.println(m.temperature);
    client.publish("/airq/esp32/temp", String(m.temperature).c_str());

    Serial.print("CO2: ");
    Serial.println(co2);
    client.publish("/airq/esp32/co2", String(co2).c_str());
    
    client.publish("/airq/esp32/rate", String(Count_Rate).c_str());
    client.publish("/airq/esp32/dose", String(Dose_Rate).c_str());
    
    Serial.print("pm1: ");
    Serial.println(pms.pm01);
    client.publish("/airq/esp32/pm1", String(pms.pm01).c_str());
    
    Serial.print("pm2.5: ");
    Serial.println(pms.pm25);
    client.publish("/airq/esp32/pm2.5", String(pms.pm25).c_str());
    
    Serial.print("pm10: ");
    Serial.println(pms.pm10);
    client.publish("/airq/esp32/pm10", String(pms.pm10).c_str());




    
    // logging gamma radiaion
    Serial.print(" "); 
    Serial.print(GMZ_counts, DEC); 
    Serial.print("\t\t");    
    Serial.print(time_difference, DEC); 
    Serial.print("\t\t");    
    Serial.print(Count_Rate, 2); 
    Serial.print("\t\t");    
    Serial.print(Dose_Rate, 2); 
    Serial.print("\t\t");    
    Serial.println(HV_pulse_count, DEC); 
    
  }
}
