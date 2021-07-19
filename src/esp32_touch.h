
/** @file esp32_touch.h */
#ifndef ESP32_TOUCH_H
#define ESP32_TOUCH_H

#include <functional>
#include <driver/touch_pad.h>
#include <Ticker.h> // https://github.com/sstaub/Ticker.git
#include <map>

// Omit this line to disable debug print output
#define ENABLE_DEBUG_PRINT 1
#include "info_debug_error.h"

/** @brief User callback function type */
using CallbackT = std::function<void(void)>;

/******************************* ESP32Touch ********************************//**
 * @brief ESP32 touch button driver with async callback interface
 * 
 * This configures the hardware capacitive touch input pins on the Espressif
 * ESP32 platform for up to ten non-multiplexed buttons.
 * 
 * All of the operation takes place asynchronously via user-defined
 * callback functions, which can be any type compatible with a C++
 * std::function object, i.e. plain global functions, static and non-static
 * class or instance members or C++11 lambda expressions.
 * 
 * In contrast to the original Arduino touchRead() function,
 * this implementation works reliably with stable, filtered sensor readout
 * and without false triggers by random spikes/zeros from some hardware
 * or API failure (See: https://forum.arduino.cc/index.php?topic=629955.0)
 * 
 * This API uses the ESP-IDF touch sensor interface, but does not register
 * with the touch hardware ISR interface. Instead, this uses the continuous
 * output from the ESP-IDF touch IIR filter using the filter_read_cb() hook
 * from touch_pad.h.
 * A freeRTOS timer (via Ticker.h) is then set up periodicly calling an event
 * loop handler checking if any button threshold level is reached. If this is
 * the case, it then calls the respective user callback.
 * This has the advantage of not blocking the filter ISR for extended time.
 * 
 * The cycle time for the event checking loop can be configured in the header
 * via dispatch_cycle_time_ms setting, the default is 100 milliseconds.
 * 
 * For the sensor input pins, please again note that the touch API uses
 * a different numbering scheme than the standard GPIO numbers.
 * E.g. touch button no. 0 is GPIO 4.
 * 
 * Repository at: https://github.com/ul-gh/ESP32Touch
 * 2020-02-25 Ulrich Lukas
 */
class ESP32Touch
{
public:

    enum INSTANTANEOUS_BUTTON_STATE
    {
        PRESSED,
        NOT_PRESSED
    };

    enum BUTTON_STATE
    {
        NO_PRESS,
        SHORT_PRESSED,
        MEDIUM_PRESSED,
        LONG_PRESSED,
        NUM_STATES_DONT_USE
    };

    // RISING means the state change will trigger the callback function
    // when the button changes from not pressed to pressed. Falling means 
    // callback will be triggered when the button changes from pressed to 
    // not pressed. Using FALLING can be used to only have a single button 
    // event triggered per button press even if you have multiple button 
    // states configured on a single touch pad.
    enum TRIGGER_MODE
    {
        RISE,
        FALL
    };

    /** @brief Configure here the cycle time for the event loop/handler
     */
    uint32_t dispatch_cycle_time_ms = 20;

    /** @brief Configuration for the ESP-IDF API IIR filter, higher values mean
     *         more stable results, but also more time lag
     */
    int filter_period = 10;

    ESP32Touch();
    virtual ~ESP32Touch();

    void disableAllButtons();
    void disableButton(const int input_number);

    void disableEventTimer();
    void enableEventTimer();
    void updateButtons();

    /** @brief Get the time in ms since the last callback function was triggered.
     *         useful for detecting button inactivity. Will return -1 if a callback
     *         has never been triggered.
     */
    long getTimeSinceLastCallback_ms();

    /** @brief Configure input pin as a touch input, set threshold value and
     *         register the required user callback called when pin is touched.
     * @param input_number Touch input pin number
     *                     (!) different from GPIO numbering (!)
     * @param threshold_percent Touch button touch detection threshold in
     *                          percent of the calibration-time
     *                          (i.e. idle-state) sensor readout value.
     * @param callback User callback function, which can be any type compatible
     *                 with a C++ std::function object, i.e. plain global
     *                 functions, static an non-static class or instance members
     *                 or C++11 lambda expressions.
     *                 The callback must have a signature of void(void).
     * @param buttonState The state that the button must be in for the callback 
           *              to be triggered.
     * @param edgeTrigger Either RISE or FALL, determines if the callback should 
     *                    be triggered on the rising or falling edge.
     * @param waitForRelease Decide if a button should be released before the 
     *                       callback becomes active. This prevents button 
     *                       triggering if a button press causes button 
     *                       behavior to change.
     */
    void configure_input(const int input_number,
                         const uint8_t threshold_percent,
                         CallbackT callback = nullptr,
                         const BUTTON_STATE buttonState = SHORT_PRESSED,
                         const TRIGGER_MODE edgeTrigger = RISE,
                         const bool waitForRelease = true);
    
    
    /** @brief Force a sensor re-calibration.
     * 
     * This is called implicitly by ESP32Touch::begin(), but can be called
     * later at run-time to do a re-calibration.
     * 
     * The touch buttons must not be pressed down while the calibration
     * is running..
     */
    void calibrate_thresholds();

    /** @brief This must be called once after all the
     *         user callbacks have been set up.
     */
    void begin();

    /** @brief Call this periodicly to see the raw sensor readout values printed
     */
    void diagnostics();
    
private:
    // The ESP-IDF API threshold is not used in this code
    static constexpr int threshold_inactive = 0;

    // FreeRTOS timer
    Ticker event_timer;
    unsigned long timeOfLastCallback_ms = 0;
    // Static configuration and runtime state
    static uint8_t s_pad_threshold_percent[TOUCH_PAD_MAX];
    static bool s_pad_enabled[TOUCH_PAD_MAX];
    static bool s_pad_active[TOUCH_PAD_MAX][NUM_STATES_DONT_USE];
    static uint16_t s_pad_filtered_value[TOUCH_PAD_MAX];
    static uint16_t s_pad_threshold[TOUCH_PAD_MAX];
    static CallbackT s_pad_callback[TOUCH_PAD_MAX][NUM_STATES_DONT_USE];
    static BUTTON_STATE s_pad_state[TOUCH_PAD_MAX];
    static INSTANTANEOUS_BUTTON_STATE s_pad_instantaneous_state[TOUCH_PAD_MAX];
    static long s_pad_initial_press_time[TOUCH_PAD_MAX];
    static TRIGGER_MODE s_pad_trigger_mode[TOUCH_PAD_MAX];

    enum INSTANTANEOUS_BUTTON_STATE getInstantaneousButtonState(const int touch_pin);
    void updateButtonState(const int touch_pin);
    void initializeButtons();
    void initializeButton(const int touch_pin);

    std::map<BUTTON_STATE, float> BUTTON_THRESHOLD_TIMES_MS
    {
        {SHORT_PRESSED, 50},
        {MEDIUM_PRESSED, 300},
        {LONG_PRESSED, 2000}
    };

    // Filter output reading hook, see ESP-IDF file touch_pad.h
    static void filter_read_cb(uint16_t *raw_value, uint16_t *filtered_value);
    // Event loop/handling function
    void dispatch_callbacks();
}; // class ESP32Touch
/** @example esp32_touch_example.cpp
 */
#endif
