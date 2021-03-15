//#include "esp_log.h"
//#include "esp32-hal-log.h"

//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"

//#include "soc/rtc_cntl_reg.h"
//#include "soc/sens_reg.h"

#include "esp32_touch.h"

//////// ESP32Touch public:

ESP32Touch::ESP32Touch()
    : event_timer{}
{   
    // Initialize touch pad peripheral, it will start a timer to run a filter
    debug_print("Initializing touch pad");
    touch_pad_init();
    // If use interrupt trigger mode, should set touch sensor FSM mode at 'TOUCH_FSM_MODE_TIMER'.
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    // Set reference voltage for charging/discharging
    // For most usage scenarios, we recommend using the following combination:
    // the high reference valtage will be 2.7V - 1V = 1.7V, The low reference voltage will be 0.5V.
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    //init RTC IO and mode for touch pad.
    initializeButtons();
}

ESP32Touch::~ESP32Touch() {
    event_timer.detach();
}

void ESP32Touch::initializeButton(const int input_number)
{
    s_pad_enabled[input_number] = false;
    s_pad_is_pressed[input_number] = false;
    s_pad_threshold[input_number] = threshold_inactive;
    for(int i=0;i<NUM_STATES_DONT_USE;++i)
    {
        s_pad_callback[input_number][i] = {};
    }
    s_pad_state[input_number] = BUTTON_STATE::NO_PRESS;
}

void ESP32Touch::initializeButtons()
{
    for (int i=0; i<TOUCH_PAD_MAX; ++i) {
        initializeButton(i);
    }
}

void ESP32Touch::disableButton(const int input_number)
{
    s_pad_enabled[input_number] = false;
    s_pad_is_pressed[input_number] = false;
    for(int i=0;i<NUM_STATES_DONT_USE;++i)
    {
        s_pad_callback[input_number][i] = {};
    }
    s_pad_state[input_number] = BUTTON_STATE::NO_PRESS;

}

void ESP32Touch::disableAllButtons()
{
    for (int i=0; i<TOUCH_PAD_MAX; ++i) {
        disableButton(i);
    }
}

void ESP32Touch::configure_input(const int input_number,
                                 const uint8_t threshold_percent,
                                 CallbackT callback,
                                 const BUTTON_STATE buttonState,
                                 const TRIGGER_MODE edgeTrigger)
{
    debug_print_sv("Registering callback for touch button no.: ", input_number);
    //debug_print_hex("Callback address: ", (uint32_t)debug_get_address(&callback));
    s_pad_enabled[input_number] = true;
    s_pad_threshold_percent[input_number] = threshold_percent;
    s_pad_callback[input_number][buttonState] = callback;
    s_pad_state[input_number] = BUTTON_STATE::NO_PRESS;
    s_pad_trigger_mode[input_number] = edgeTrigger;
}

void ESP32Touch::calibrate_thresholds() {
    uint16_t touch_value;
    for (int i=0; i<TOUCH_PAD_MAX; ++i) {
        if (s_pad_enabled[i]) {
            //read filtered value
            touch_pad_read_filtered(static_cast<touch_pad_t>(i), &touch_value);
            debug_print_sv("Current touch input: ", i);
            debug_print_sv("touch pad val is: ", touch_value);
            s_pad_threshold[i] = static_cast<uint32_t>(touch_value)
                    * s_pad_threshold_percent[i] / 100;
            debug_print_sv("threshold value is: ", s_pad_threshold[i]);
        }
    }
}

void ESP32Touch::begin() {
    for (int i=0; i<TOUCH_PAD_MAX; ++i) {
        if (s_pad_enabled[i]) {
            touch_pad_config(static_cast<touch_pad_t>(i), threshold_inactive);
        }
    }
    // Initialize and start a software filter to detect slight change of capacitance.
    touch_pad_filter_start(filter_period);
    touch_pad_set_filter_read_cb(filter_read_cb);
    // Set threshold
    calibrate_thresholds();
    event_timer.attach_ms(dispatch_cycle_time_ms, dispatch_callbacks, this);
}

void ESP32Touch::diagnostics() {
    for (int i=0; i<TOUCH_PAD_MAX; ++i) {
        if (s_pad_enabled[i]) {
            Serial.print("Button no.: "); Serial.print(i);
            Serial.print(F("  Current sensor value: "));
            Serial.print(s_pad_filtered_value[i]);
            Serial.print(F("  Threshold: "));
            Serial.println(s_pad_threshold[i]);
        }

    }

}

//////// ESP32Touch private:

// Static members must be explicitly initialised
uint8_t ESP32Touch::s_pad_threshold_percent[TOUCH_PAD_MAX];
bool ESP32Touch::s_pad_enabled[TOUCH_PAD_MAX];
bool ESP32Touch::s_pad_is_pressed[TOUCH_PAD_MAX];
uint16_t ESP32Touch::s_pad_filtered_value[TOUCH_PAD_MAX];
uint16_t ESP32Touch::s_pad_threshold[TOUCH_PAD_MAX];
CallbackT ESP32Touch::s_pad_callback[TOUCH_PAD_MAX][NUM_STATES_DONT_USE];
ESP32Touch::BUTTON_STATE ESP32Touch::s_pad_state[TOUCH_PAD_MAX];
ESP32Touch::INSTANTANEOUS_BUTTON_STATE ESP32Touch::s_pad_instantaneous_state[TOUCH_PAD_MAX];
long ESP32Touch::s_pad_initial_press_time[TOUCH_PAD_MAX];
ESP32Touch::TRIGGER_MODE ESP32Touch::s_pad_trigger_mode[TOUCH_PAD_MAX];

void ESP32Touch::filter_read_cb(uint16_t *raw_value, uint16_t *filtered_value) {
    for (int i=0; i<TOUCH_PAD_MAX; ++i) {
        s_pad_filtered_value[i] = filtered_value[i];
    }
}

enum ESP32Touch::INSTANTANEOUS_BUTTON_STATE ESP32Touch::getInstantaneousButtonState(const int touch_pin)
{
    return s_pad_filtered_value[touch_pin] < s_pad_threshold[touch_pin] ? PRESSED : NOT_PRESSED;
}

void ESP32Touch::updateButtonState(const int touch_pin)
{
    INSTANTANEOUS_BUTTON_STATE lastButtonState = s_pad_instantaneous_state[touch_pin];
    INSTANTANEOUS_BUTTON_STATE currentButtonState = getInstantaneousButtonState(touch_pin);

    if(currentButtonState == PRESSED)
    {
        if(lastButtonState == NOT_PRESSED)
        {
            s_pad_initial_press_time[touch_pin] = millis();
        }
        else if(lastButtonState == PRESSED)
        {
            long timeDiff = millis() - s_pad_initial_press_time[touch_pin];
            debug_print_sv("Time difference ", timeDiff);
            if(timeDiff >= BUTTON_THRESHOLD_TIMES_MS[LONG_PRESSED])
            {
                s_pad_state[touch_pin] = LONG_PRESSED;
            }
            else if(timeDiff >= BUTTON_THRESHOLD_TIMES_MS[MEDIUM_PRESSED])
            {
                s_pad_state[touch_pin] = MEDIUM_PRESSED;
            }
            else if(timeDiff >= BUTTON_THRESHOLD_TIMES_MS[SHORT_PRESSED])
            {
                s_pad_state[touch_pin] = SHORT_PRESSED;
            }
        }
    }
    else
    {
        s_pad_state[touch_pin] = NO_PRESS;
    
    }
    s_pad_instantaneous_state[touch_pin] = currentButtonState;
}

void ESP32Touch::dispatch_callbacks(ESP32Touch* self) {
    for (int i=0; i<TOUCH_PAD_MAX; ++i) {
        if (s_pad_enabled[i]) {
            BUTTON_STATE lastButtonState = s_pad_state[i];
            self->updateButtonState(i);
            // bool new_button_state = s_pad_filtered_value[i] < s_pad_threshold[i];
            // Transition from off to on state detected.
            // The callbacks are only fired when a button state actually changes
            // in order to avoid repeat presses.
            // if (new_button_state && !s_pad_is_pressed[i]) {
            //     CallbackT cb = s_pad_callback[i][s_pad_state[i]];
            //     debug_print_sv("Dispatching callback for touch input no.: ", i);
            //     if (cb) {
            //         cb();
            //     }
            // }
            if(s_pad_trigger_mode[i] == RISE && s_pad_state[i] != NO_PRESS)
            {
                if(lastButtonState != s_pad_state[i])
                {
                    CallbackT cb = s_pad_callback[i][s_pad_state[i]];
                    if (cb)
                    {
                        debug_print_sv("Dispatching rising callback for touch input no.: ", i);
                        cb();
                    }
                }
            }
            else if(s_pad_trigger_mode[i] == FALL && s_pad_state[i] == NO_PRESS)
            {
                if(lastButtonState != s_pad_state[i])
                {
                    CallbackT cb = s_pad_callback[i][lastButtonState];
                    if (cb)
                    {
                        debug_print_sv("Dispatching falling callback for touch input no.: ", i);
                        cb();
                    }
                }
            }
            
            // s_pad_is_pressed[i] = new_button_state;
        }
    }
}
