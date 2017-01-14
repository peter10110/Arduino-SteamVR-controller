// Button pinout
#define MENU_BUTTON_PIN 9
#define SYSTEM_BUTTON_PIN 7
#define GRIP_BUTTON_PIN 4
#define TRIGGER_BUTTON_PIN 2
#define TOUCHPAD_PRESS_PIN 3 
#define ANALOG_X_PIN A6
#define ANALOG_Y_PIN A7

// Analog correction values
#define ANALOG_DEAD 5
#define ANALOG_X_CENTER 494
#define ANALOG_X_MIN 0
#define ANALOG_X_MAX 1023
#define ANALOG_Y_CENTER 431
#define ANALOG_Y_MIN 0
#define ANALOG_Y_MAX 700

// Current button states
bool menuButtonDown = false;
bool systemButtonDown = false;
bool gripButtonDown = false;
bool triggerButtonDown = false;
bool touchpadDown = false;
int buttonStates = 0;
int lastButtonStates = 0;
float touchpadX = 0;
float touchpadY = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(ANALOG_X_PIN, INPUT);
  pinMode(ANALOG_Y_PIN, INPUT);
  
  pinMode(MENU_BUTTON_PIN, INPUT);
  digitalWrite(MENU_BUTTON_PIN, HIGH);
  
  pinMode(SYSTEM_BUTTON_PIN, INPUT);
  digitalWrite(SYSTEM_BUTTON_PIN, HIGH);
  
  pinMode(GRIP_BUTTON_PIN, INPUT);
  digitalWrite(GRIP_BUTTON_PIN, HIGH);
  
  pinMode(TRIGGER_BUTTON_PIN, INPUT);
  digitalWrite(TRIGGER_BUTTON_PIN, HIGH);

  pinMode(TOUCHPAD_PRESS_PIN, INPUT);
  digitalWrite(TOUCHPAD_PRESS_PIN, HIGH);
  
  Serial.begin(9600, SERIAL_8N1);
  Serial.print("#Program started");
}

// the loop function runs over and over again forever
void loop() {
  // Menu button
  if (!menuButtonDown && digitalRead(MENU_BUTTON_PIN) == LOW)
  {
    // Menu button pressed
    menuButtonDown = true;
    digitalWrite(LED_BUILTIN, HIGH);
    //Serial.print("Menu button down");
    buttonStates |= 4;
  }
  else if (menuButtonDown && digitalRead(MENU_BUTTON_PIN) == HIGH)
  {
    // Menu button released
    digitalWrite(LED_BUILTIN, LOW);
    menuButtonDown = false;
    buttonStates &= B11011;
  }

  // System button
  if (!systemButtonDown && digitalRead(SYSTEM_BUTTON_PIN) == LOW)
  {
    // System button pressed
    systemButtonDown = true;
    digitalWrite(LED_BUILTIN, HIGH);
    //Serial.print("System button down");
    buttonStates |= 2;
  }
  else if (systemButtonDown && digitalRead(SYSTEM_BUTTON_PIN) == HIGH)
  {
    // System button released
    digitalWrite(LED_BUILTIN, LOW);
    systemButtonDown = false;
    buttonStates &= B11101;
  }

  // Grip button
  if (!gripButtonDown && digitalRead(GRIP_BUTTON_PIN) == LOW)
  {
    // Grip button pressed
    gripButtonDown = true;
    digitalWrite(LED_BUILTIN, HIGH);
    //Serial.print("Grip button down");
    buttonStates |= 1;
  }
  else if (gripButtonDown && digitalRead(GRIP_BUTTON_PIN) == HIGH)
  {
    // Grip button released
    digitalWrite(LED_BUILTIN, LOW);
    gripButtonDown = false;
    buttonStates &= B11110;
  }

  // Trigger button
  if (!triggerButtonDown && digitalRead(TRIGGER_BUTTON_PIN) == LOW)
  {
    // Trigger button pressed
    triggerButtonDown = true;
    digitalWrite(LED_BUILTIN, HIGH);
    //Serial.print("Trigger button down");
    buttonStates |= 16;
  }
  else if (triggerButtonDown && digitalRead(TRIGGER_BUTTON_PIN) == HIGH)
  {
    // Trigger button released
    digitalWrite(LED_BUILTIN, LOW);
    triggerButtonDown = false;
    buttonStates &= B01111;
  }

  // Touchpad press
  if (!touchpadDown && digitalRead(TOUCHPAD_PRESS_PIN) == LOW)
  {
    // Touchpad pressed
    touchpadDown = true;
    digitalWrite(LED_BUILTIN, HIGH);
    //Serial.print("Touchpad press down");
    buttonStates |= 8;
  }
  else if (touchpadDown && digitalRead(TOUCHPAD_PRESS_PIN) == HIGH)
  {
    // Touchpad released
    digitalWrite(LED_BUILTIN, LOW);
    touchpadDown = false;
    buttonStates &= B10111;
  }

  // Touchpad
  touchpadX = analogRead(ANALOG_X_PIN);
  touchpadY = analogRead(ANALOG_Y_PIN);
  
  // Write only, if the state has changed
  if (lastButtonStates != buttonStates || true)
  {
    Serial.print(buttonStates);
    Serial.print("; X:");
    Serial.print(touchpadX);
    Serial.print("; Y: ");
    Serial.print(touchpadY);
    Serial.print('\n');
    lastButtonStates = buttonStates;  
  }


  delay(50);
}
