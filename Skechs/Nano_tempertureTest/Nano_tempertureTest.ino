#include <DHT.h>
#include <DHT_U.h>

#include "DHT.h"
#define DHT1STPIN 3
#define DHT2NDPIN 4
#define DHTTYPE DHT11

DHT dht1st(DHT1STPIN, DHTTYPE);
DHT dht2nd(DHT2NDPIN, DHTTYPE);
void setup() {
  Serial.begin(9600); 
  Serial.println("DHT11 test!"); 
  dht1st.begin();
  dht2nd.begin();
}

void loop() {
  delay(3000);

  float h1 = dht1st.readHumidity();
  float t1 = dht1st.readTemperature();
  float h2 = dht2nd.readHumidity();
  float t2 = dht2nd.readTemperature();
  
  if (isnan(h1) || isnan(t1)) {
    Serial.println("Failed to read from DHT1st sensor!");
    return;
  }
if (isnan(h2) || isnan(t2)) {
    Serial.println("Failed to read from DHT2nd sensor!");
    return;
  }
  
  Serial.print("Humidity: "); 
  Serial.print("1st " + (String)h1 + " %  2nd " + (String)h2);
  Serial.println(" %\t");
  Serial.print("Temperature: "); 
  Serial.print("1st " + (String)t1 + " *C  2nd " + (String)t2);
  Serial.println(" *C ");
}
