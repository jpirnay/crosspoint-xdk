#include "InputManager.h"

const int InputManager::ADC_THRESHOLDS_1[] = {3470, 2655, 1470, 3};
const int InputManager::ADC_THRESHOLDS_2[] = {2205, 3};
const char* InputManager::BUTTON_NAMES[] = {"Back", "Confirm", "Left", "Right", "Up", "Down", "Power"};

InputManager::InputManager()
    : currentState(0),
      lastState(0),
      pressedEvents(0),
      releasedEvents(0),
      lastDebounceTime(0),
      powerButtonPressStart(0),
      powerButtonWasPressed(false) {}

void InputManager::begin() {
  pinMode(BUTTON_ADC_PIN_1, INPUT);
  pinMode(BUTTON_ADC_PIN_2, INPUT);
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);
  analogSetAttenuation(ADC_11db);
}

int InputManager::getButtonFromADC(int adcValue, const int thresholds[], int numButtons) {
  if (adcValue > ADC_NO_BUTTON) {
    return -1;
  }

  for (int i = 0; i < numButtons; i++) {
    if (abs(adcValue - thresholds[i]) < ADC_TOLERANCE) {
      return i;
    }
  }

  return -1;
}

uint8_t InputManager::getState() {
  uint8_t state = 0;

  // Read GPIO1 buttons
  int adcValue1 = analogRead(BUTTON_ADC_PIN_1);
  int button1 = getButtonFromADC(adcValue1, ADC_THRESHOLDS_1, NUM_BUTTONS_1);
  if (button1 >= 0) {
    state |= (1 << button1);
  }

  // Read GPIO2 buttons
  int adcValue2 = analogRead(BUTTON_ADC_PIN_2);
  int button2 = getButtonFromADC(adcValue2, ADC_THRESHOLDS_2, NUM_BUTTONS_2);
  if (button2 >= 0) {
    state |= (1 << (button2 + 4));
  }

  // Read power button (digital, active LOW)
  if (digitalRead(POWER_BUTTON_PIN) == LOW) {
    state |= (1 << BTN_POWER);
  }

  return state;
}

void InputManager::update() {
  unsigned long currentTime = millis();
  uint8_t state = getState();

  // Always clear events first
  pressedEvents = 0;
  releasedEvents = 0;

  // Debounce
  if (state != lastState) {
    lastDebounceTime = currentTime;
    lastState = state;
  }

  if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (state != currentState) {
      // Calculate pressed and released events
      pressedEvents = state & ~currentState;
      releasedEvents = currentState & ~state;
      currentState = state;

      // Track power button press timing
      if (pressedEvents & (1 << BTN_POWER)) {
        powerButtonPressStart = currentTime;
        powerButtonWasPressed = true;
      }
      if (releasedEvents & (1 << BTN_POWER)) {
        powerButtonWasPressed = false;
      }
    }
  }
}

bool InputManager::isPressed(uint8_t buttonIndex) {
  return currentState & (1 << buttonIndex);
}

bool InputManager::wasPressed(uint8_t buttonIndex) {
  return pressedEvents & (1 << buttonIndex);
}

bool InputManager::wasReleased(uint8_t buttonIndex) {
  return releasedEvents & (1 << buttonIndex);
}

const char* InputManager::getButtonName(uint8_t buttonIndex) {
  if (buttonIndex <= BTN_POWER) {
    return BUTTON_NAMES[buttonIndex];
  }
  return "Unknown";
}

bool InputManager::isPowerButtonPressed() {
  return isPressed(BTN_POWER);
}
