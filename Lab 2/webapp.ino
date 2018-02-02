#include <ESP8266WiFi.h>
//const char* ssid = "Verizon-SM-N930V-1D89";
//const char* password = "akju819!";
//const char* host = "192.168.43.215";  // IP serveur - Server IP
const char* ssid = "MySpectrumWiFi40-2G";
const char* password = "greensquirrel477";
const char* host = "192.168.1.11";  // IP serveur - Server IP
const int   port = 3000;            // Port serveur - Server Port
const int   watchdog = 10;        // Fréquence du watchdog - Watchdog frequency
unsigned long previousMillis = millis(); 
const short int ledPin = D2;
const short int ledPin2 = 16;

void blink(){
  for(int i = 0; i < 5; i++){
    digitalWrite(ledPin, HIGH);
    delay(150);
    digitalWrite(ledPin, LOW);
    delay(150);
  }
}

void decoder(String bin){
  if(bin == "ff"){
    digitalWrite(ledPin, LOW);
    digitalWrite(ledPin2, HIGH);
  }
  else if(bin == "ft"){
    digitalWrite(ledPin, LOW);
    digitalWrite(ledPin2, LOW);
  }
  else if(bin == "tf"){
    digitalWrite(ledPin, HIGH);
    digitalWrite(ledPin2, HIGH);
  }
  else if(bin == "tt"){
    digitalWrite(ledPin, HIGH);
    digitalWrite(ledPin2, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    //delay(500);
    Serial.print(".");
    blink();
  }
 
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  delay(100);
}

void loop() {
    WiFiClient client;
    if (!client.connect(host, port)) {
      Serial.println("connection failed");
      return;
    }
 
    String url = "/blink";
    //This will send the request to the server
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" + 
               "Connection: close\r\n\r\n");
    unsigned long timeout = millis();
    
    while (client.available() == 0) {
      if (millis() - timeout > 1000) {
        Serial.println(">>> Client Timeout !");
        client.stop();
        return;
      }
    }
  
    // Read all the lines of the reply from server and print them to Serial
    String line = "";
    while(client.available()){
      line = client.readStringUntil('\r');
      Serial.print(line);
    }
    Serial.println();
    String catLine = "";
    for(int i = 1; i < line.length(); i++){
      if(line[i] != '@')
        catLine += line[i];
      else
        break;
    }
    //Serial.print("printing catLine: ");
    //Serial.println(catLine);
    //Serial.print("length is: ");
    //Serial.println(catLine.length());
    //delay(100);
    decoder(catLine);
    /*
    if(catLine == "t"){
      digitalWrite(ledPin, HIGH);
    }
    else
      digitalWrite(ledPin, LOW);*/
}


