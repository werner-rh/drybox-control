/***
 * Project: DryBox Control
 * File   : Secrets.h
 * Author : Jens Friedrich
 * Created: 20.10.2024
 * Board: Wemos D1 Mini
 * 
 * Description: Class secrets like user name, password
 * 
 */

#ifdef ESP8266
  // WiFi settings
  const char *ssid = "<your ssid>"; // Replace with your WiFi name
  const char *password = "<your password>"; // Replace with your WiFi password
  
  // MQTT Broker settings
  const char *mqtt_broker = "<MQTT broker host>";  // MQTT broker endpoint (ip address or hostname)
  const char *mqtt_topic = "DryBox";  // MQTT topic
  const char *mqtt_username = "";  // MQTT username for authentication or empty string for none
  const char *mqtt_password = "";  // MQTT password for authentication or empty string for none
  const int mqtt_port = 1883;  // MQTT port (TCP)
#endif
