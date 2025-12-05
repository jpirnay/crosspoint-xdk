#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

#include <Arduino.h>

class InputManager {
 public:
  InputManager();
  void begin();
  uint8_t getState();
  void update();
  bool isPressed(uint8_t buttonIndex);
  bool wasPressed(uint8_t buttonIndex);
  bool wasReleased(uint8_t buttonIndex);

  // Button indices
  static const uint8_t BTN_BACK = 0;
  static const uint8_t BTN_CONFIRM = 1;
  static const uint8_t BTN_LEFT = 2;
  static const uint8_t BTN_RIGHT = 3;
  static const uint8_t BTN_UP = 4;
  static const uint8_t BTN_DOWN = 5;
  static const uint8_t BTN_POWER = 6;

  // Power button methods
  bool isPowerButtonPressed();

  // Button names
  static const char* getButtonName(uint8_t buttonIndex);

 private:
  int getButtonFromADC(int adcValue, const int thresholds[], int numButtons);

  uint8_t currentState;
  uint8_t lastState;
  uint8_t pressedEvents;
  uint8_t releasedEvents;
  unsigned long lastDebounceTime;
  unsigned long powerButtonPressStart;
  bool powerButtonWasPressed;

  static const int BUTTON_ADC_PIN_1 = 1;
  static const int NUM_BUTTONS_1 = 4;
  static const int ADC_THRESHOLDS_1[];

  static const int POWER_BUTTON_PIN = 3;

  static const int BUTTON_ADC_PIN_2 = 2;
  static const int NUM_BUTTONS_2 = 2;
  static const int ADC_THRESHOLDS_2[];

  static const int ADC_TOLERANCE = 200;
  static const int ADC_NO_BUTTON = 3800;
  static const unsigned long DEBOUNCE_DELAY = 5;

  static const char* BUTTON_NAMES[];
};

#endif
