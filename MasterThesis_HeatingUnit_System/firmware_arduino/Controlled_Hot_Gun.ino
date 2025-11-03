#include <SPI.h>
#include <MAX31855soft.h>
#include "MAX31865_NonBlocking.h"
#include <avr/wdt.h>

// Sensor setup
MAX31855soft dutThermocouple(45, 47, 49);
MAX31855soft myMAX31855(53, 44, 46);
MAX31865 rtd(48);
#define RREF 4301.0
#define RNOMINAL 1000.0

// Control pins
#define TRIAC_CONTROL_PIN 4
#define FAN_CONTROL_PIN 6
#define RELAY_PIN 7
#define LED_GREEN_PIN 13
#define LED_RED_PIN 12


// System variables
float setPoint = 20.0;
float currentTemperature = 0;
int fanSpeed = 0;
bool manualOverride = false;
bool setpointReached = false;
bool smallDifference = false;
bool systemRunning = false;
bool initializationComplete = false; // New flag to track initialization status
bool protectionConditionActive = false; // Flag to track if a protection condition is active

// Feedback selection
enum FeedbackSource { PT1000, THERMOCOUPLE };
FeedbackSource feedbackSource = PT1000;

// PID variables
float kp = 3.0, ki = 0.0, kd = 0.0;
float error = 0, lastError = 0;
float integral = 0;
float outputPWM = 0;
unsigned long lastMillis = 0;
float max_overshoot = 0;

// Safety monitoring
int overTempLimit;
int overTempCount = 0;
bool relayActivated = false;
bool thermocoupleInitialized = false;
float tempThreshold = 250.0;

// Serial communication
String serialBuffer = "";
bool newData = false;

void setup() {
  // Initialize hardware
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS12);
  OCR1A = 1250;
  TIMSK1 |= (1 << OCIE1A);

  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);  // Start with LEDs off
  digitalWrite(LED_RED_PIN, LOW);

  pinMode(TRIAC_CONTROL_PIN, OUTPUT);
  pinMode(FAN_CONTROL_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  
  // Important change: Force the relay to OPEN (HIGH signal) to disconnect power during initialization
  digitalWrite(RELAY_PIN, LOW);
  
  digitalWrite(TRIAC_CONTROL_PIN, LOW);
  digitalWrite(FAN_CONTROL_PIN, LOW);

  // Initialize sensors
  dutThermocouple.begin();
  while (dutThermocouple.getChipID() != MAX31855_ID) {
    delay(1000);
  }

  myMAX31855.begin();
  while (myMAX31855.getChipID() != MAX31855_ID) {
    delay(1000);
  }

  rtd.begin();
  rtd.setConvMode(MAX31865::CONV_MODE_CONTINUOUS);
  delay(1000);

  // Initialize Serial communication
  Serial.begin(115200); // Default USB port for Python GUI
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  Serial.setTimeout(50);
  Serial.println("SYSTEM:READY");
  Serial.println("ACK:FEEDBACK_SOURCE=PT1000");
  
  // Mark initialization as complete after the FEEDBACK_SOURCE message is sent
  initializationComplete = true;
  
  Serial3.begin(115200); // Communication with Arduino IDE via USB-to-TTL (COM26)
  Serial3.println("SYSTEM:READY ON SERIAL3");
}

void loop() {

  updateStatusLED();

  // Handle communication on Serial (COM25)
  receiveSerialData(Serial, serialBuffer, newData);
  if (newData) {
    parseInput(serialBuffer, Serial);
    serialBuffer = "";
    newData = false;
  }

   // Send real-time updates to Serial3 (COM26) for Arduino IDE
  static unsigned long lastReport = 0;
  if (millis() - lastReport >= 500) { // Report every 500ms
    reportSystemStatus();
    lastReport = millis();
  }

  // Add this check to ensure relay stays de-energized when system is not running
  if (!systemRunning) {
    digitalWrite(RELAY_PIN, LOW); // Keep relay de-energized (NO contact open)
  }

  // Read temperature based on selected source
  
  if (feedbackSource == PT1000) {
    currentTemperature = readPT1000();
  } else {
    currentTemperature = readDUTThermocouple();
  }
 

  static unsigned long lastThermocoupleRead2 = 0;
  if (millis() - lastThermocoupleRead2 >= 1500) {
    int32_t rawData = myMAX31855.readRawData();
    float thermocoupleTemp = myMAX31855.getTemperature(rawData);
    lastThermocoupleRead2 = millis();
    if (!thermocoupleInitialized && thermocoupleTemp < 1500) {
      thermocoupleInitialized = true;
    }
    
    // CRITICAL FIX: Always check for critical temperature regardless of feedback source
    if (thermocoupleTemp > 450.0) {
      // Emergency shutdown for extreme temperature
      digitalWrite(RELAY_PIN, LOW);
      setPoint = 20.0;
      systemRunning = false;
      Serial.println("ALARM:HOTGUN_OVERTEMP_CRITICAL");
      Serial.println("SYSTEM:STOPPED_PROTECTION");
      Serial.println("STATUS:System STOPPED - Press to Start");
    }
    
    // Only call full relay control function when using thermocouple feedback
    if (feedbackSource == THERMOCOUPLE)
      relayControl(); // Call relayControl() here
  }

  // Only run normal control logic if system is running and initialization is complete
  if (thermocoupleInitialized && systemRunning && initializationComplete) {
    updatePIDConstants();
    computePID();
    if (feedbackSource == PT1000) {
      relayControl();
    }
  } else {
    // System not running - turn off outputs
    analogWrite(TRIAC_CONTROL_PIN, 0);
  }

  // Only control the fan when system is running
  if (systemRunning) {
    analogWrite(FAN_CONTROL_PIN, map(fanSpeed, 0, 100, 255, 0));
  } else {
    analogWrite(FAN_CONTROL_PIN, 255); // Fan OFF
  }
}

void receiveSerialData(HardwareSerial &serialPort, String &buffer, bool &dataFlag) {
  while (serialPort.available() > 0) {
    char inChar = (char)serialPort.read();
    if (inChar == '\n') {
      dataFlag = true;
      break;
    }
    buffer += inChar;
  }
}

void forcePIDUpdate() {
  if (feedbackSource == PT1000) {
    setPIDConstants4PT1000(setPoint);
  } else {
    setPIDConstants4Thermocouple(setPoint);
  }
  // Reset PID calculation variables to avoid integral windup
  integral = 0;
  lastError = 0;
  // Reset control flags to ensure proper adaptation
  setpointReached = false;
  smallDifference = abs(setPoint - currentTemperature) < 20;
}

void parseInput(String input, HardwareSerial &serialPort) {
  input.trim();
  if (input.length() == 0) return;

  char command = input.charAt(0);
  String valueStr = input.substring(1);

  switch (command) {
    case 'S': // Setpoint
      setPoint = constrain(valueStr.toFloat(), 0, 180);
      serialPort.print("ACK:SETPOINT=");
      serialPort.println(setPoint);
      max_overshoot = 0;
      setpointReached = false;
      smallDifference = abs(setPoint - currentTemperature) < 20;
      break;

    case 'F': // Fan speed
      if (systemRunning) {
        int requestedSpeed = constrain(valueStr.toInt(), 30, 100);
        // Normalize to discrete steps (30, 40, 50, 60, 70, 80, 90, 100)
        int normalizedSpeed = ((requestedSpeed + 5) / 10) * 10;  // Round to nearest 10
        if (normalizedSpeed < 30) normalizedSpeed = 30;  // Ensure minimum is 30
        if (normalizedSpeed > 100) normalizedSpeed = 100;  // Ensure maximum is 100
        
        fanSpeed = normalizedSpeed;
        serialPort.print("ACK:FAN=");
        serialPort.println(fanSpeed);
      } else {
        serialPort.println("ERR:SYSTEM_NOT_RUNNING");
      }
      break;

    case 'T': // Temperature threshold
      tempThreshold = constrain(valueStr.toFloat(), 20, 300);
      serialPort.print("ACK:THRESHOLD=");
      serialPort.println(tempThreshold);
      break;

    case 'R': // System control
      if (valueStr == "START") {
        if (initializationComplete) {
          systemRunning = true;
          // CORRECTED: Comments now match actual behavior with NO contacts
          // HIGH energizes relay, closing NO contacts, allowing power to heater
          digitalWrite(RELAY_PIN, HIGH);
          fanSpeed = 100;
          serialPort.println("ACK:SYSTEM_STARTED");
        } else {
          serialPort.println("ERR:INITIALIZATION_NOT_COMPLETE");
        }
      } else if (valueStr == "STOP") {
        systemRunning = false;
        // CORRECTED: Comments now match actual behavior with NO contacts
        // LOW de-energizes relay, opening NO contacts, disconnecting power from heater
        digitalWrite(RELAY_PIN, LOW);
        fanSpeed = 0;
        serialPort.println("ACK:SYSTEM_STOPPED");
      }
      break;

    case 'B': // Feedback source
      if (valueStr == "1") {
        feedbackSource = PT1000;
        serialPort.println("ACK:FEEDBACK_SOURCE=PT1000");
        forcePIDUpdate(); // Force PID update for new feedback source
      } else if (valueStr == "2") {
        if (dutThermocouple.getChipID() == MAX31855_ID) {
          feedbackSource = THERMOCOUPLE;
          serialPort.println("ACK:FEEDBACK_SOURCE=THERMOCOUPLE");
          forcePIDUpdate(); // Force PID update for new feedback source
        } else {
          serialPort.println("ERR:THERMOCOUPLE_NOT_FOUND");
        }
      }
      break;

    default:
      serialPort.println("ERR:INVALID_CMD");
      break;
  }
}

// [Rest of the functions remain exactly the same as before...]
void handleFeedbackSourceSelection(String valueStr) {
  if (valueStr.length() != 1) {
    Serial.println("ERR:INVALID_SOURCE_FORMAT");
    return;
  }
 
  char selection = valueStr.charAt(0);
  switch(selection) {
    case '1':
      feedbackSource = PT1000;
      Serial.println("ACK:FEEDBACK_SOURCE=PT1000");
      forcePIDUpdate(); // Force PID update for new feedback source
      break;
    case '2':
      // Verify thermocouple is connected
      if (dutThermocouple.getChipID() != MAX31855_ID) {
        Serial.println("ERR:THERMOCOUPLE_NOT_FOUND");
        feedbackSource = PT1000; // Fall back to PT1000 if thermocouple not found
        Serial.println("ACK:FEEDBACK_SOURCE=PT1000");
      } else {
        feedbackSource = THERMOCOUPLE;
        Serial.println("ACK:FEEDBACK_SOURCE=THERMOCOUPLE");
        forcePIDUpdate(); // Force PID update for new feedback source
      }
      break;
    default:
      Serial.println("ERR:INVALID_SOURCE_SELECTION");
      break;
  }
}

void handleSetpointCommand(String valueStr) {
  float newSetpoint = valueStr.toFloat();
  if (newSetpoint >= 0 && newSetpoint <= 180) {
    setPoint = newSetpoint;
    Serial.print("ACK:SETPOINT=");
    Serial.println(setPoint);
    max_overshoot = 0;
    setpointReached = false;
    smallDifference = abs(setPoint - currentTemperature) < 20;
  } else {
    Serial.println("ERR:SETPOINT_RANGE");
  }
}

float readPT1000() {
  return rtd.getTemperature(RNOMINAL, RREF);
}

float readDUTThermocouple() {
  int32_t rawData = dutThermocouple.readRawData();
  return dutThermocouple.getTemperature(rawData);
}

float readRelayThermocouple() {
  int32_t rawData = myMAX31855.readRawData();
  return myMAX31855.getTemperature(rawData);
}


void relayControl() {
  // Read relay thermocouple temperature for protection condition 3
  float thermocoupleTemp = readRelayThermocouple(); // Using the dedicated function as specified
  
  // Protection alarm states for conditions 1 and 2
  static int alarmCycleCounter = 0;
  static bool alarmActive = false;
  static byte alarmType = 0; // 0 = no alarm, 1 = over temp, 2 = under temp, 3 = hotgun high temp
  
  // Special counters for condition 3
  static int overTempCounter = 0;  // Counts cycles when temperature > threshold
  static bool overTempDetected = false;  // Flag to track if we're in an overtemp condition
  static bool alarmSent = false;  // Track if alarm has been sent for current cycle
  static FeedbackSource overTempFeedbackSource = PT1000;  // Store the feedback source when overtemp is detected
  
  // ==================== Process active alarms for conditions 1 & 2 ====================
  if (alarmActive && (alarmType == 1 || alarmType == 2)) {
    // Set protection flag to true during alarm condition
    protectionConditionActive = true;
    
    // Calculate max alarm cycles based on feedback source
    // PT1000 requires more cycles for safety (25 cycles), Thermocouple keeps the original 15
    int maxAlarmCycles = (feedbackSource == PT1000) ? 40 : 15;
    
    // Continue alarm while under the max cycles
    if (alarmCycleCounter < maxAlarmCycles) {
      // Send the appropriate alarm message based on type
      if (alarmType == 1) {
        Serial.println("ALARM:OVERTEMP_THRESHOLD");
        // Extra warning specifically for PT1000 mode
        if (feedbackSource == PT1000 && (alarmCycleCounter % 5 == 0)) {
          Serial.println("WARNING:PT1000_OVERTEMP_EXTRA_CAUTION");
        }
      } else if (alarmType == 2) {
        Serial.println("ALARM:UNDERTEMP_PROTECTION");
        // Extra warning specifically for PT1000 mode
        if (feedbackSource == PT1000 && (alarmCycleCounter % 5 == 0)) {
          Serial.println("WARNING:PT1000_UNDERTEMP_EXTRA_CAUTION");
        }
      }
      
      // Ensure relay is activated (LOW = relay activated, disconnecting power)
      digitalWrite(RELAY_PIN, LOW);
      
      // Force setpoint to 20°C
      setPoint = 20.0;
      
      // Increment counter for this cycle
      alarmCycleCounter++;
    } 
    // After max cycles, stop the system
    else {
      // Stop the system
      systemRunning = false;
      alarmActive = false;
      alarmCycleCounter = 0;
      
      // KEY CHANGE: Reset protection flag to allow restart immediately
      protectionConditionActive = false;
      
      // Send messages to GUI
      Serial.println("SYSTEM:STOPPED_PROTECTION");
      Serial.println("SYSTEM:PROTECTION_CLEARED");  // Tell GUI protection is cleared
      
      // Different status message for PT1000 vs Thermocouple
      if (feedbackSource == PT1000) {
        Serial.println("STATUS:Protection triggered with PT1000 - System STOPPED - Press to Start");
      } else {
        Serial.println("STATUS:System STOPPED - Press to Start");
      }
    }
    
    return; // Exit function early as we're in alarm handling mode
  }
 
  // ==================== CONDITION 3: High-Temperature Final Protection using Hotgun thermocouple ====================
  
  // Check if temperature difference is small (<80°C) or large
  smallDifference = abs(setPoint - currentTemperature) < 80;

  // Determine cycle limit based on both feedback source and temperature difference
  // Increase cycle limit for PT1000 for extra safety
  int cycleLimit;
  if (smallDifference) {
    cycleLimit = (feedbackSource == PT1000) ? 180 : 2;  // Increased from 150 to 180 for PT1000
  } else {
    cycleLimit = (feedbackSource == PT1000) ? 450 : 5;  // Increased from 400 to 450 for PT1000
  }

  if (thermocoupleTemp > ((feedbackSource == PT1000) ? 350.0 : 350.0)) {
    // Set protection flag when temperature is above threshold
    protectionConditionActive = true;
    
    if (!overTempDetected) {
      overTempDetected = true; // Mark that we've detected an overtemp condition
      overTempFeedbackSource = feedbackSource; // Lock in the feedback source at detection time
      overTempCounter = 1; // Initialize counter at first detection

      // Debug output to verify feedback source and threshold used
      if (overTempFeedbackSource == PT1000) {
        Serial.println("DEBUG:OVERTEMP_PT1000_MODE_250C");
        // Additional warning for PT1000 mode
        Serial.println("WARNING:PT1000_HOTGUN_TEMP_RISING");
      } else {
        Serial.println("DEBUG:OVERTEMP_THERMOCOUPLE_MODE_200C");
      }
    } else {
      // Increment counter for subsequent cycles
      overTempCounter++;
      
      // Additional periodic warnings for PT1000 mode
      if (overTempFeedbackSource == PT1000 && (overTempCounter % 30 == 0)) {
        Serial.println("WARNING:PT1000_CONTINUOUS_OVERTEMP");
      }
    }

    // Recalculate the cycle limit based on the LOCKED feedback source and current temperature difference
    smallDifference = abs(setPoint - currentTemperature) < 80;
    if (smallDifference) {
      cycleLimit = (overTempFeedbackSource == PT1000) ? 180 : 2;
    } else {
      cycleLimit = (overTempFeedbackSource == PT1000) ? 450 : 5;
    }

    // For PT1000, start alarm messages earlier to provide more warning
    int alarmStartCycle;
    if (overTempFeedbackSource == PT1000) {
      alarmStartCycle = cycleLimit - 20;  // 20 cycles of alarm for PT1000 (more warning)
    } else {
      alarmStartCycle = cycleLimit - 10;  // 10 cycles for thermocouple
    }
    
    // Send alarm messages when counter exceeds the alarm start threshold
    if (overTempCounter > alarmStartCycle) {
      Serial.println("ALARM:HOTGUN_OVERTEMP");
      alarmSent = true;
      
      // For PT1000, provide additional detailed warning every 5 cycles
      if (overTempFeedbackSource == PT1000 && (overTempCounter % 5 == 0)) {
        Serial.println("WARNING:PT1000_APPROACHING_SHUTDOWN");
        Serial.print("DEBUG:REMAINING_CYCLES=");
        Serial.println(cycleLimit - overTempCounter);
      }
    }

    // Debug output to monitor counter progress
    if (overTempCounter % 25 == 0 || overTempCounter == 1 || overTempCounter == cycleLimit) {
      Serial.print("DEBUG:OVERTEMP_COUNT=");
      Serial.println(overTempCounter);
    }

    // Only take action when counter exceeds the cycle limit
    if (overTempCounter >= cycleLimit) {
      // Now activate relay to stop system operation
      digitalWrite(RELAY_PIN, LOW);
      // Set setpoint to 20°C
      setPoint = 20.0;
      // Stop the system
      systemRunning = false;
      
      // KEY CHANGE: Reset protection flag to allow restart immediately
      protectionConditionActive = false;
      
      // Send final system status messages
      Serial.println("SYSTEM:STOPPED_PROTECTION");
      Serial.println("SYSTEM:PROTECTION_CLEARED");  // Tell GUI protection is cleared
      
      // Different message based on feedback source
      if (overTempFeedbackSource == PT1000) {
        Serial.println("STATUS:Protection Complete - System STOPPED - Press to Start");
      } else {
        Serial.println("STATUS:System STOPPED - Press to Start");
      }

      // Reset condition flags for next time
      overTempDetected = false;
      overTempCounter = 0;
      alarmSent = false;
    }
  } else {
    // Temperature has dropped below threshold
    // Reset counters and flags if we haven't reached the cycle limit yet
    if (overTempDetected) {
      // Recalculate the cycle limit based on the LOCKED feedback source and current temperature difference
      smallDifference = abs(setPoint - currentTemperature) < 80;
      if (smallDifference) {
        cycleLimit = (overTempFeedbackSource == PT1000) ? 180 : 2;
      } else {
        cycleLimit = (overTempFeedbackSource == PT1000) ? 450 : 5;
      }

      if (overTempCounter < cycleLimit) {
        overTempDetected = false;
        overTempCounter = 0;
        alarmSent = false;
        
        // Clear protection flag when temperature drops back to normal
        protectionConditionActive = false;
        
        // Different debug message based on feedback source
        if (overTempFeedbackSource == PT1000) {
          Serial.println("DEBUG:PT1000_OVERTEMP_CONDITION_CLEARED");
        } else {
          Serial.println("DEBUG:OVERTEMP_CONDITION_CLEARED");
        }
      }
    }
  }
 
  // ==================== CONDITION 1: Over-Temperature Protection ====================
  // Only check condition 1 if we're not already handling condition 3
  if (!overTempDetected && currentTemperature > tempThreshold && !alarmActive) {
    // IMMEDIATE ACTIONS: Activate relay and set setpoint to 20°C
    digitalWrite(RELAY_PIN, LOW); // De-energize relay to disconnect power
    setPoint = 20.0;

    // Set protection flag to true when over-temperature condition detected
    protectionConditionActive = true;
    
    // Start alarm sequence
    alarmActive = true;
    alarmType = 1;
    alarmCycleCounter = 1; // Start at 1 since we're sending the first alarm now
    
    // Different initial alarm message based on feedback source
    if (feedbackSource == PT1000) {
      Serial.println("ALARM:PT1000_OVERTEMP_THRESHOLD");
    } else {
      Serial.println("ALARM:OVERTEMP_THRESHOLD");
    }
    
    return; // Exit function after triggering alarm
  }
  
  // ==================== CONDITION 2: Under-Temperature Protection ====================
  // Only check condition 2 if we're not already handling condition 3
  if (!overTempDetected && currentTemperature < 15.0 && !alarmActive) {
    // IMMEDIATE ACTIONS: Activate relay and set setpoint to 20°C
    digitalWrite(RELAY_PIN, LOW); // De-energize relay to disconnect power
    setPoint = 20.0;
    
    // Set protection flag to true when under-temperature condition detected
    protectionConditionActive = true;
    
    // Start alarm sequence
    alarmActive = true;
    alarmType = 2;
    alarmCycleCounter = 1; // Start at 1 since we're sending the first alarm now
    
    // Different initial alarm message based on feedback source
    if (feedbackSource == PT1000) {
      Serial.println("ALARM:PT1000_UNDERTEMP_PROTECTION");
    } else {
      Serial.println("ALARM:UNDERTEMP_PROTECTION");
    }
    
    return; // Exit function after triggering alarm
  }
  
  // If we get here, no new alarm conditions were triggered, so handle normal operation
  
  // Reset alarm state if we're no longer in an alarm condition
  if (!alarmActive) {
    alarmCycleCounter = 0;
    alarmType = 0;
    
    // Only reset protection flag if we're not in any protection condition
    if (!overTempDetected) {
      protectionConditionActive = false;
    }
  }
}


void updatePIDConstants() {
  smallDifference = abs(setPoint - currentTemperature) < 20;

  if (!manualOverride && !setpointReached) {
    if (smallDifference) {
      if (currentTemperature <= setPoint) {
        kp = 5.0; ki = 0.0; kd = 0.0;
      } else {
        if (feedbackSource == PT1000) {
          setPIDConstants4PT1000(setPoint);
        } else {
          setPIDConstants4Thermocouple(setPoint);
        }
        setpointReached = true;
      }
    } else {
      if (currentTemperature <= 0.80 * setPoint) {
        kp = 0.50; ki = 0.0; kd = 0.0;
      } else if (currentTemperature <= setPoint) {
        kp = 3.0; ki = 0.0; kd = 0.0;
      } else {
        if (feedbackSource == PT1000) {
          setPIDConstants4PT1000(setPoint);
        } else {
          setPIDConstants4Thermocouple(setPoint);
        }
        setpointReached = true;
      }
    }
  }
}

void setPIDConstants4PT1000(float sp) {
  if (sp >= 25 && sp <= 50) {
    kp = 5; ki = 1; kd = 3;
  } else if (sp <= 80) {
    kp = 6; ki = 1; kd = 3;
  } else if (sp <= 100) {
    kp = 8; ki = 5; kd =5;
  } else if (sp <= 155) {
    kp = 10; ki = 10; kd = 15;
  } else {
    kp = 15; ki = 20; kd = 15;
  }
}

void setPIDConstants4Thermocouple(float sp) {
  if (sp >= 25 && sp <= 50) {
    kp = 2; ki = 1; kd = 1;
  } else if (sp <= 80) {
    kp = 3.5; ki = 1; kd = 3;
  } else if (sp <= 100) {
    kp = 4; ki = 1; kd = 4;
  } else if (sp <= 155) {
    kp = 6; ki = 1; kd =5;
  } else {
    kp = 10; ki = 8; kd = 6;
  }
}

void computePID() {
  error = setPoint - currentTemperature;

  if (error < 0 && abs(error) > max_overshoot) {
    max_overshoot = abs(error);
  }

  float pTerm = kp * error;
  float derivative = 0;

  if (outputPWM > 0 && outputPWM < 255) {
    integral += ki * error * 0.02 * (1.0 - (outputPWM/255.0));
  } else {
    integral = 0;
  }

  unsigned long currentTime = millis();
  float dt = (currentTime - lastMillis) / 1000.0;
  if (dt > 0) {
    derivative = kd * (error - lastError) / dt;
  }

  outputPWM = pTerm + integral + derivative;
  outputPWM = constrain(outputPWM, 0, 255);

  lastError = error;
  lastMillis = currentTime;
}

void reportSystemStatus() {
  String status = "DATA:ST=" + String(setPoint, 2) +
                  ",CT=" + String(currentTemperature, 2) +
                  ",TT=" + String(readRelayThermocouple(), 2) +
                  ",DT=" + String(readDUTThermocouple(), 2) +
                  ",OV=" + String(max_overshoot, 2) +
                  ",FS=" + String(fanSpeed) +
                  ",KP=" + String(kp, 2) +
                  ",KI=" + String(ki, 2) +
                  ",KD=" + String(kd, 2) +
                  ",FB=" + String(feedbackSource == PT1000 ? "PT1000" : "THERMOCOUPLE");

  Serial.println(status);  // Send to Python GUI (COM25)
  Serial3.println(status); // Send to Arduino IDE (COM26)
}

void updateStatusLED() {
  bool heaterOn = systemRunning && initializationComplete;
  bool relayOn = digitalRead(RELAY_PIN) == HIGH;
  
  if (protectionConditionActive) {
    // Protection condition is active - red LED on, green LED off
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_RED_PIN, HIGH);
  } else if (heaterOn && relayOn) {
    // System running normally - green LED on, red LED off
    digitalWrite(LED_GREEN_PIN, HIGH);
    digitalWrite(LED_RED_PIN, LOW);
  } else if (systemRunning) {
    // System is supposed to be running but relay is off (not due to protection)
    // This might indicate some other issue - flash both LEDs alternately
    unsigned long currentMillis = millis();
    if ((currentMillis / 500) % 2 == 0) {
      digitalWrite(LED_GREEN_PIN, HIGH);
      digitalWrite(LED_RED_PIN, LOW);
    } else {
      digitalWrite(LED_GREEN_PIN, LOW);
      digitalWrite(LED_RED_PIN, HIGH);
    }
  } else {
    // System is stopped normally (not due to protection)
    // Green LED on, red LED off
    digitalWrite(LED_GREEN_PIN, HIGH);
    digitalWrite(LED_RED_PIN, LOW);
  }
}

ISR(TIMER1_COMPA_vect) {
  if (outputPWM > 0 && systemRunning && initializationComplete) {
    analogWrite(TRIAC_CONTROL_PIN, outputPWM);
  } else {
    analogWrite(TRIAC_CONTROL_PIN, 0);
  }
}
  




