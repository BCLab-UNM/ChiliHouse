//ABN
#include <SPI.h>
#include <WiFiNINA.h>
#include <dht.h>
dht DHT;
#define DHT11_PIN 2
char Mymessage [10];

String postData;
String postVariable = "impedance=";

String postData1;
String postVariable1 = "temp=";


char ssid[] = "";  //  your network SSID (name)
char pass[] = ""; // your network password


int status = WL_IDLE_STATUS;
char servername[] = ""; 
WiFiClient client;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  Serial.println("Attempting to connect to WPA network...");
  Serial.print("SSID: ");
  Serial.println(ssid)
  status = WiFi.begin(ssid, pass);
  if ( status != WL_CONNECTED) {
    Serial.println("Couldn't get a wifi connection");
    // don't do anything else:
    while (true);
  }
  else {
    Serial.println("Connected to wifi");
    Serial.println("\nStarting connection...");

  }
}

void loop() {
  Serial1.readBytes(Mymessage, 10);
  delay(1000);
  /////////
  int reading = DHT.read11(DHT11_PIN);
  int readingtemp = DHT.temperature;
  delay(1500);

  String t_d;
  for (int i = 0; i < 10; i++)
  {
    if ((Mymessage[i] <= 57 && Mymessage[i] >= 48) || Mymessage[i] == 46)
      t_d = t_d + Mymessage[i];

  }


  postData = postVariable + t_d;
  postData1 = postVariable1 + readingtemp;

  // if you get a connection, report back via serial:
  if (client.connectSSL(servername, 443)) {
    Serial.println("connected");
    Serial.println(postData.length());
    Serial.println(postData);


    // Make a HTTP request:

    client.println("POST xxxx HTTP/1.1");
    client.println("Host: xxxx");
    client.println("Content-type: application/x-www-form-urlencoded"); 
    client.print("Content-Length: ");
    client.println(postData.length());
    client.println();
    client.print(postData);
    //client.println("Connection: close");
    client.println();

    delay(1000);

    Serial.println(postData1.length());
    Serial.println(postData1);
    client.println("POST xxxx HTTP/1.1");
    client.println("Host: xxxx");
    client.println("Content-type: application/x-www-form-urlencoded"); 
    client.print("Content-Length: ");
    client.println(postData1.length());
    client.println();
    client.print(postData1);
    client.println();

    delay(60000);
  }

  // if there are incoming bytes available
  //from the server, read them and print them:
  if (client.available()) {
    char c = client.read();
    Serial.print(c);
  }

  // if the server's disconnected, stop the client:
  if (!client.connected()) {
    Serial.println();
    Serial.println("disconnecting.");
    client.stop();

    // do nothing forevermore:
    for (;;)
      ;
  }
}
