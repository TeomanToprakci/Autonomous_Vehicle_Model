/*
Simple Serial Motor Control for Arduino Leonardo
Communicates with ROS2 via serial protocol

Protocol: "L<pwm>,R<pwm>\n"
Example: "L150,R-100\n" (Left motor 150 PWM, Right motor -100 PWM)
*/

// Motor pins
const int ENA = 3, IN1 = 5, IN2 = 6;    // Left motors
const int ENB = 11, IN3 = 9, IN4 = 10;  // Right motors
const int LED_PIN = 13;
const int button1Pin = 7;
const int button2Pin = 8;
const int button3Pin = 2;
const int button4Pin = 4;

// Motor control variables
int left_pwm = 0;
int right_pwm = 0;
unsigned long last_command_time = 0;
const unsigned long TIMEOUT_MS = 1000; // Stop motors if no command for 1 second

// Button variables
int last_btn1_state = HIGH;
int last_btn2_state = HIGH;
int last_btn3_state = HIGH;
int last_btn4_state = HIGH;
unsigned long last_button_check = 0;
const unsigned long BUTTON_CHECK_INTERVAL = 50; // Check buttons every 50ms

// Serial communication
String input_string = "";
bool string_complete = false;

void setup() {
  // Initialize motor pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(button3Pin, INPUT_PULLUP);
  pinMode(button4Pin, INPUT_PULLUP);
  
  // Stop motors initially
  stopMotors();
  
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Arduino Motor Controller Ready");
  Serial.println("Protocol: L<pwm>,R<pwm>");
  Serial.println("Example: L150,R-100");
  
  // Reserve string space
  input_string.reserve(50);
  
  // LED on to show ready
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  // Handle serial input for motor commands
  while (Serial.available()) {
    char incoming_char = (char)Serial.read();
    
    if (incoming_char == '\n') {
      string_complete = true;
    } else {
      input_string += incoming_char;
    }
  }
  
  // Check for serial commands
  if (string_complete) {
    processCommand(input_string);
    input_string = "";
    string_complete = false;
  }
  
  // Safety timeout - stop motors if no command received
  if (millis() - last_command_time > TIMEOUT_MS) {
    if (left_pwm != 0 || right_pwm != 0) {
      stopMotors();
      left_pwm = 0;
      right_pwm = 0;
      // Silent timeout - don't print to avoid interfering with button data
    }
  }

  // Check buttons periodically and send only when state changes
  if (millis() - last_button_check > BUTTON_CHECK_INTERVAL) {
    checkButtons();
    last_button_check = millis();
  }

  // Heartbeat LED
  digitalWrite(LED_PIN, (millis() / 500) % 2);
  
  delay(5); // Reduced delay for better responsiveness
}

void checkButtons() {
  // Read current button states
  int btn1 = digitalRead(button1Pin);
  int btn2 = digitalRead(button2Pin);
  int btn3 = digitalRead(button3Pin);
  int btn4 = digitalRead(button4Pin);
  
  // Check if any button state changed
  if (btn1 != last_btn1_state || btn2 != last_btn2_state || 
      btn3 != last_btn3_state || btn4 != last_btn4_state) {
    
    // Send button status (LOW = pressed = 1, HIGH = not pressed = 0)
    Serial.print("BTN1:");
    Serial.print(btn1 == LOW ? 1 : 0);
    Serial.print(";BTN2:");
    Serial.print(btn2 == LOW ? 1 : 0);
    Serial.print(";BTN3:");
    Serial.print(btn3 == LOW ? 1 : 0);
    Serial.print(";BTN4:");
    Serial.print(btn4 == LOW ? 1 : 0);
    Serial.println(";");
    
    // Update last states
    last_btn1_state = btn1;
    last_btn2_state = btn2;
    last_btn3_state = btn3;
    last_btn4_state = btn4;
  }
}

void processCommand(String command) {
  command.trim();
  
  // Parse command: L<pwm>,R<pwm>
  int comma_index = command.indexOf(',');
  if (comma_index == -1) {
    Serial.println("ERROR: Invalid format. Use L<pwm>,R<pwm>");
    return;
  }
  
  String left_part = command.substring(0, comma_index);
  String right_part = command.substring(comma_index + 1);
  
  // Extract PWM values
  if (left_part.startsWith("L") && right_part.startsWith("R")) {
    int new_left_pwm = left_part.substring(1).toInt();
    int new_right_pwm = right_part.substring(1).toInt();
    
    // Constrain values
    new_left_pwm = constrain(new_left_pwm, -255, 255);
    new_right_pwm = constrain(new_right_pwm, -255, 255);
    
    // Apply motor commands
    left_pwm = new_left_pwm;
    right_pwm = new_right_pwm;
    
    controlLeftMotors(left_pwm);
    controlRightMotors(right_pwm);
    
    last_command_time = millis();
    
    // Send confirmation
    Serial.print("OK: L");
    Serial.print(left_pwm);
    Serial.print(",R");
    Serial.println(right_pwm);
  } else {
    Serial.println("ERROR: Invalid format. Use L<pwm>,R<pwm>");
  }
}

void controlLeftMotors(int pwm) {
  if (pwm > 0) {
    // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm);
  } else if (pwm < 0) {
    // Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -pwm);
  } else {
    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void controlRightMotors(int pwm) {
  if (pwm > 0) {
    // Forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm);
  } else if (pwm < 0) {
    // Backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -pwm);
  } else {
    // Stop
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
}
