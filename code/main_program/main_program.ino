TaskHandle_t DriveTask;
TaskHandle_t MQTTTask;
TaskHandle_t distSensTask;

#include <WiFi.h>
#include <PubSubClient.h>

// Options
int maxSpeed = 250; //max motor speed 0-255
int speedOffset = 100;
int line = 0; //does the sensor return 0 or 1 when it sees the line
int transmitDelay = 1000; // Set transmit interval in ms
int sonarDelay = 100; // Set distance ping interval in ms
int minimumDistance = 5; // Set minimum distance to obstacle in cm

// Configuration for wifi and mqtt
const char* ssid = "...";
const char* password = "...";
const char* mqttServer = "...";
const int mqttPort = 1883;
const char* mqttUser = "...";
const char* mqttPassword = "...";
const char* clientID = "...";  // MQTT client ID
const char* listenTopic = "iot_project/lineFLWR";
const char* drivingStatusTopic = "iot_project/status/driving";
const char* batteryTopic = "iot_project/status/battery";
const char* testTopic = "iot_project/status/test";
const char* stateTopic = "iot_project/status/state";

char* drivingStatus = "S"; // Initialize the driving status to "stopped"

int pingPin = 5; // Trigger Pin of Ultrasonic Sensor
int echoPin = 4; // Echo Pin of Ultrasonic Sensor

int button0Pin = 15;
int button1Pin = 2;

int button0State = 0;
int button1State = 0;

// Set transmit interval
int lastTransmitMillis;

// Global variable for obstacle distance
int sonarDistance = 0;
int obstacle = 0;

int IRRightPin = 34;
int IRLeftPin = 13;
int IRCenterPin = 35;

// Define pins for status LED's
int ledOkay = 18;
int ledObstacle = 19;
int ledLostLine = 23;

// Used to handle stopping and starting again
int lastStateIsStop = 0;
int lastStopMillis = 0;
int stopDelay = 5000;
int motorStopOverride = 0;
int stopped = 0;
int shouldContinue = 1;
int manualStop = 0;

// Used for detecting whether the robot has lost track of the line
int lastSeenLineMillis;
int lastSeenLineMaxDelay = 1500;
int lostState = 0;

int IRRightState, IRLeftState, IRCenterState = 0;

//Motor Right
int enA = 32;
int in1 = 33;
int in2 = 25;
//Motor Left
int enB = 26;
int in3 = 27;
int in4 = 14 ;


WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to WiFi network: ");
    Serial.println(ssid);
  }
  Serial.println("Connected to the WiFi network");

  //connect to broker
  client.setServer(mqttServer, mqttPort);

  //set a callback function for when data is received from broker
  client.setCallback(callback);

  // connect to broker
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect(clientID, mqttUser, mqttPassword)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  // subscribe to topic
  client.subscribe(listenTopic);

  // Set pinMode for sensors
  pinMode(IRRightPin, INPUT);
  pinMode(IRLeftPin, INPUT);
  pinMode(IRCenterPin, INPUT);
  // Set pinMode for buttons en distance sensor
  pinMode(button0Pin, INPUT);
  pinMode(button1Pin, INPUT);
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Set pinMode for LED's
  pinMode (ledOkay, OUTPUT);
  pinMode (ledObstacle, OUTPUT);
  pinMode (ledLostLine, OUTPUT);

  // Set mode for motor control pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Turn off LED's
  digitalWrite(ledOkay, LOW);
  digitalWrite(ledObstacle, LOW);
  digitalWrite(ledLostLine, LOW);

  // Indicate that the program has started
  Serial.println("Started");
  client.publish(testTopic, "Started");

  // Run a test sequence for the LED's
  checkLEDs();


  // TASKS
  // Create tasks for controlling the car and communicating over wifi.
  xTaskCreatePinnedToCore(
    DriveTaskCode,   /* Task function. */
    "DriveTask",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &DriveTask,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */

  xTaskCreatePinnedToCore(
    MQTTTaskCode,   /* Task function. */
    "MQTTTask",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &MQTTTask,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */

  // Create task for reading the distance sensor, this task has a higher priority than mqttTask. That means it will get priority. mqttTask will only run when this task is paused.
  xTaskCreatePinnedToCore(
    distSensTaskCode,   /* Task function. */
    "distSensTask",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    2,           /* priority of the task */
    &distSensTask,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */

  disableCore0WDT();  //Stop watchdog from interrupting the program
  disableCore1WDT();  //Stop watchdog from interrupting the program
}

// Define code to be executed by the driving task
void DriveTaskCode(void * pvParameters) {
  for (;;) {
    followLine();
  }
}

// Define code to be executed by the mqtt task
void MQTTTaskCode(void * pvParameters) {
  for (;;) {
    client.loop();
    doCheckWiFiAndMQTT();

    Serial.print("sending status...");
    client.publish(drivingStatusTopic, drivingStatus);
    Serial.print(drivingStatus);
    if (lostState) {
      client.publish(stateTopic, "Lost");
      Serial.println("..lost");
    }
    else if (stopped) {
      client.publish(stateTopic, "Stopped");
      Serial.println("..stopped");
    }
    else
    {
      client.publish(stateTopic, "OK");
      Serial.println("..ok");
    }


    vTaskDelay(transmitDelay);
  }
}

void distSensTaskCode(void * pvParameters) {
  for (;;) {

    long duration, inches, cm;

    digitalWrite(pingPin, LOW);
    ets_delay_us(2);
    digitalWrite(pingPin, HIGH);
    ets_delay_us(10);
    digitalWrite(pingPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    cm = microsecondsToCentimeters(duration);
    sonarDistance = cm;
    Serial.print(cm);
    Serial.println(" cm");
    vTaskDelay(sonarDelay);
  }
}

void loop() {
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

void followLine() {
  //read IR sensors
  IRRightState = digitalRead(IRRightPin);
  IRCenterState = digitalRead(IRCenterPin);
  IRLeftState = digitalRead(IRLeftPin);
  button0State = digitalRead(button0Pin);
  button1State = digitalRead(button1Pin);

  if (button0State) {
    manualStop = 0;
  }
  if (button1State) {
    manualStop = 1;
  }

  if (sonarDistance < minimumDistance) {
    stopMotors();
    Serial.println("Obstacle");
    drivingStatus = "O";
    digitalWrite(ledOkay, LOW);
    digitalWrite(ledObstacle, HIGH);
    digitalWrite(ledLostLine, LOW);
    obstacle = 1;
    lastSeenLineMillis = millis();
  }
  else if (! manualStop) {
    obstacle = 0;
    digitalWrite(ledObstacle, LOW);

    if ((IRRightState == line) && !(IRLeftState == line) && ((millis() - lastStopMillis) > (stopDelay + 500))) {
      // Turn right
      digitalWrite(ledOkay, HIGH);
      digitalWrite(ledObstacle, LOW);
      digitalWrite(ledLostLine, LOW);
      turnRight(maxSpeed);
      Serial.println("Right");
      drivingStatus = "R";
      lastStateIsStop = 0;

      lastSeenLineMillis = millis();
    }
    else if ((IRLeftState == line) && !(IRRightState == line) && ((millis() - lastStopMillis) > (stopDelay + 500))) {
      // Turn left
      digitalWrite(ledOkay, LOW);
      digitalWrite(ledObstacle, LOW);
      digitalWrite(ledLostLine, LOW);
      turnLeft(maxSpeed);
      Serial.println("Left");
      drivingStatus = "L";
      lastStateIsStop = 0;

      lastSeenLineMillis = millis();
    }
    else if ((IRCenterState == line) && !(IRLeftState == line || IRRightState == line)) {
      // Straight ahead
      digitalWrite(ledOkay, HIGH);
      digitalWrite(ledObstacle, LOW);
      digitalWrite(ledLostLine, LOW);
      goStraight((maxSpeed - speedOffset));
      Serial.println("Forward");
      drivingStatus = "F";
      lastStateIsStop = 0;

      lastSeenLineMillis = millis();
    }
  }
  else {
    stopMotors();
  }
  if ((IRCenterState == line) && (IRLeftState == line) && (IRRightState == line) && (! manualStop)) {
    // Stop
    if ((!lastStateIsStop) && ((millis() - lastStopMillis) > (stopDelay + 1000))) {
      shouldContinue = 0;
      stopMotors();
      Serial.println("Stop");
      drivingStatus = "S";
      stopped = 1;
      lastStateIsStop = 1;
      lastStopMillis = millis();
    }
    else if (((millis() - lastStopMillis) > stopDelay) || button0State) {
      shouldContinue = 1;
    }

    lastSeenLineMillis = millis();
    if (shouldContinue) {
      goStraight((maxSpeed - speedOffset));
      stopped = 0;
    }
  }

  if ((millis() - lastSeenLineMillis) > lastSeenLineMaxDelay) {
    stopMotors();
    digitalWrite(ledLostLine, HIGH);
    digitalWrite(ledOkay, LOW);
    lostState = 1;
  }
  else {
    digitalWrite(ledLostLine, LOW);
    digitalWrite(ledOkay, HIGH);
    lostState = 0;
  }
}

void turnLeft(int motorSpeed) {
  //Set motor speed 0-255
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);

  //Reverse motor left
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  //Forward motor right
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnRight(int motorSpeed) {
  //Set motor speed 0-255
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);

  //Forward motor left
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  //Reverse motor right
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void goStraight(int motorSpeed) {
  //Set motor speed 0-255
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);

  //Forward motor left
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  //Forward motor right
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void goBack (int motorSpeed) {
  //Set motor speed 0-255
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);

  //Reverse motor left
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  //Reverse motor right
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void stopMotors() {
  //Set motor speed 0-255
  analogWrite(enA, 50);
  analogWrite(enB, 50);

  //Stop motor left
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  //Stop motor right
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void doCheckWiFiAndMQTT() {
  //check wifi connection and reconnect if necessary
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("\n");
    Serial.println("WiFi disconnected!");
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print("Connecting to WiFi network: ");
      Serial.println(ssid);
    }
    Serial.println("Connected to the WiFi network");
  }

  // check MQTT connection and reconnect to broker if necessary
  if (!client.connected() && (WiFi.status() == WL_CONNECTED)) {
    Serial.print("\n");
    Serial.println("MQTT disconnected!");
    while (!client.connected()) {
      Serial.println("Connecting to MQTT...");
      if (client.connect(clientID, mqttUser, mqttPassword)) {
        Serial.println("connected");
      } else {
        Serial.print("failed with state ");
        Serial.println(client.state());
        delay(2000);
      }
    }
    // subscribe to topic
    client.subscribe(listenTopic);
  }
}

void checkLEDs() {
  digitalWrite(ledOkay, HIGH);
  delay(500);
  digitalWrite(ledObstacle, HIGH);
  delay(500);
  digitalWrite(ledLostLine, HIGH);
  delay(2000);
  digitalWrite(ledOkay, LOW);
  digitalWrite(ledObstacle, LOW);
  digitalWrite(ledLostLine, LOW);
}

void callback(char* topic, byte* payload, unsigned int length) {

  Serial.print("Message arrived with topic: ");
  Serial.println(topic);

  // Detect wether a start or stop signal has been received. 1 for start, 0 for stop.
  if (payload[0] == '1') {
    shouldContinue = 1;
    manualStop = 0;
  }
  else if (payload[0] == '0') {
    manualStop = 1;
  }

  Serial.println("Message: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println("");
}
