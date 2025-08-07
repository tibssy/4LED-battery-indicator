// Define pin numbers for LEDs and the button.
#include <avr/sleep.h>
#include <avr/interrupt.h>

#define LED1 0
#define LED2 1
#define LED3 2
#define LED4 3
#define BUTTON_PIN 4

// Volatile variables are used because they are modified within an interrupt routine.
// The compiler needs to know that their values can change unexpectedly.
volatile bool wakeUpFlag = true;  // Flag to indicate the device should wake up and execute code.  Starts true to run on first boot.
volatile bool buttonPressed = false;  // Flag to indicate if the button has been pressed.

// Function to read the VCC (supply voltage) of the ATtiny85 using the internal voltage reference.
// Returns the voltage in millivolts (mV).
long readVcc() {
  ADCSRA |= _BV(ADEN);  // Enable the ADC (Analog-to-Digital Converter)
  ADMUX = _BV(MUX3) | _BV(MUX2);  // Select the internal voltage reference (1.1V) as the input to the ADC.  MUX3 and MUX2 select the proper channel for VCC measurement.
  delay(2);  // Small delay to allow the ADC to stabilize after selection of the channel.
  ADCSRA |= _BV(ADSC);  // Start the ADC conversion
  while (bit_is_set(ADCSRA, ADSC));  // Wait for the ADC conversion to complete.  ADSC (ADC Start Conversion) bit is cleared when conversion is finished.
  
  uint16_t result = ADC;  // Read the ADC result. This is a 10-bit value.
  return 1125300L / result;  // Calculate the VCC voltage in millivolts using the formula.  1125300 is a calibration constant derived from 1.1V internal reference.  This is calibrated based on the internal ref voltage.
}

void setup() {
  // Configure the LED pins as outputs.
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  // Configure the button pin as an input with the internal pull-up resistor enabled.
  // This means the pin will be HIGH unless the button is pressed, which connects it to ground (LOW).
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Enable pin change interrupts on the specified pin (BUTTON_PIN).
  GIMSK |= _BV(PCIE);  // Enable Pin Change Interrupt Enable bit. This enables interrupts for the whole pin change group.
  PCMSK |= _BV(PCINT4);  // Enable Pin Change Interrupt on pin PCINT4 (which corresponds to BUTTON_PIN = 4)
  
  sei();  // Enable global interrupts.  This is essential for the interrupt to work.
}

// Function to introduce a delay in milliseconds while also checking if the button has been pressed.
// If the button is pressed during the delay, the function returns 'true'. Otherwise, it returns 'false'.
bool delayAndCheckButton(unsigned int ms) {
  unsigned long start = millis();  // Record the starting time.
  while (millis() - start < ms) {  // Loop until the specified delay has elapsed.
    if (buttonPressed) {  // Check if the button has been pressed (set by the interrupt).
      return true;  // Return true if the button was pressed.
    }
    delay(1);  // Short delay to avoid excessive CPU usage within the loop.
  }
  return false;   // Return false if the button was not pressed during the delay.
}

// Function to light up the LEDs according to the 'ledStates' array.
// If the button is pressed during the animation, the function returns early.
void lightUpAnimation(bool ledStates[4]) {
  for (int i = 0; i < 4; i++) {  // Iterate through the LEDs.
    digitalWrite(i, ledStates[i] ? HIGH : LOW);  // Set the LED pin to HIGH or LOW based on the corresponding value in the 'ledStates' array.  If ledStates[i] is true, then HIGH is used; otherwise, LOW.
    if (delayAndCheckButton(75)) return;  // Delay briefly and check for button press.  If pressed, return immediately to stop the animation.
  }
}

// Function to turn off the LEDs, in reverse order, that are currently lit (as specified in 'ledStates').
// If the button is pressed during the animation, the function returns early.
void turnOffAnimation(bool ledStates[4]) {
  for (int i = 3; i >= 0; i--) {  // Iterate through the LEDs in reverse order.
    if (ledStates[i]) {  // Check if the LED at index 'i' should be on.
      digitalWrite(i, LOW);  // Turn off the LED.
      if (delayAndCheckButton(75)) return;  // Delay briefly and check for button press.  If pressed, return immediately to stop the animation.
    }
  }
}

// This function handles the core display logic based on the battery voltage.
void handleDisplay() {
  for (;;) {  // Infinite loop to continuously display the voltage level.  Exits under certain conditions (low voltage or button press).
    buttonPressed = false;  // Reset the button pressed flag at the beginning of each loop iteration.
    long vcc = readVcc();  // Read the VCC voltage in millivolts.
    
    // If the voltage is very low, indicate a critical condition and exit the loop.
    if (vcc < 2000) {
      for (int i = 0; i < 20; i++) {  // Blink LED1 rapidly to indicate very low voltage.
        digitalWrite(LED1, !digitalRead(LED1));  // Toggle LED1.
        if (delayAndCheckButton(50)) continue;  // Delay briefly and check for button press. Continue the loop if pressed.
      }
      digitalWrite(LED1, LOW);  // Turn off LED1 after blinking.
      break;  // Exit the infinite loop as the battery voltage is too low.
    }

    // Determine the voltage level based on the measured VCC value.
    uint8_t level = 0;
    if (vcc >= 2000) level = 1;
    if (vcc >= 2300) level = 2;
    if (vcc >= 2800) level = 3;
    if (vcc >= 3000) level = 4;
    if (vcc >= 3500) level = 5;
    if (vcc >= 3700) level = 6;
    if (vcc >= 4200) level = 7;
    if (vcc >= 4400) level = 8;
    
    // Determine which LEDs should be lit solid and which should blink, based on the voltage level.
    bool ledSolid[4] = {false, false, false, false};  // Array to store the state of each LED (solid on or off).
    int blinkIndex = -1;  // Index of the LED that should blink. -1 indicates no LED should blink.

    for (int i = 0; i < 4; i++) {  // Iterate through the LEDs.
      int lower = i * 2 + 1;  // Calculate the lower bound for the voltage range corresponding to this LED.
      int upper = i * 2 + 2;  // Calculate the upper bound for the voltage range corresponding to this LED.
      if (level >= upper) {  // If the voltage level is above the upper bound, the LED should be lit solid.
        ledSolid[i] = true;
      } else if (level == lower) {  // If the voltage level is equal to the lower bound, the LED should blink.
        blinkIndex = i;
      }
    }

    // Perform the lighting animation.
    lightUpAnimation(ledSolid);  // Light up the LEDs that should be solid.
    if (buttonPressed) continue;  // If the button was pressed, restart the loop from the beginning.

    // Blink the appropriate LED (if any) for a short duration.
    unsigned long blinkStartTime = millis();
    while(millis() - blinkStartTime < 4000) {  // Blink for 4 seconds
        if (blinkIndex >= 0) {  // If there's an LED to blink
            digitalWrite(blinkIndex, HIGH);  // Turn on the blinking LED
            if (delayAndCheckButton(250)) break;  // Delay for 250ms and check button
            digitalWrite(blinkIndex, LOW);  // Turn off the blinking LED
            if (delayAndCheckButton(250)) break;  // Delay for 250ms and check button
        } else {
            if (delayAndCheckButton(500)) break;  // No led to blink, just wait half a second and check for button
        }
    }
    if (buttonPressed) continue;  // If the button was pressed, restart the loop from the beginning.
    turnOffAnimation(ledSolid);  // Turn off the LEDs that were lit solid.
    if (buttonPressed) continue;  // If the button was pressed, restart the loop from the beginning.
    break;  // Exit the main display loop because the animation is complete and button was not pressed.
  }

  // Ensure all LEDs are turned off before exiting the function.
  for (int i=0; i<4; i++) {
    digitalWrite(i, LOW);
  }
}

void loop() {
  // If the wake-up flag is set, it indicates that the device needs to perform the display sequence.
  if (wakeUpFlag) {
    wakeUpFlag = false;  // Reset the wake-up flag.
    handleDisplay();  // Execute the display sequence.
  }

  // Prepare to enter sleep mode to conserve power.
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Set the sleep mode to Power-down mode (lowest power consumption).
  sleep_enable();  // Enable the sleep mode.
  ADCSRA &= ~bit(ADEN);  // Disable ADC before sleeping to reduce power consumption.
  sleep_cpu();  // Put the CPU to sleep. Execution resumes here when an interrupt occurs.
  sleep_disable();  // Disable sleep mode after waking up.
}

// Interrupt Service Routine (ISR) for pin change interrupt on PCINT0 (triggered by the button press).
ISR(PCINT0_vect) {
  delay(50);  // Debounce the button.  A short delay to ignore spurious signals caused by mechanical contact bouncing.
  if (digitalRead(BUTTON_PIN) == LOW) {  // Check if the button is actually pressed (LOW because of INPUT_PULLUP).
    wakeUpFlag = true;  // Set the wake-up flag, indicating the device should execute code in the main loop.
    buttonPressed = true;  // Set the buttonPressed flag to indicate button press to animation routines.
  }
}
