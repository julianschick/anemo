#include <Arduino.h>
#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "soc/rtc.h"

SSD1306AsciiWire oled;


#define GPIO_CAP0_IN   5   //Set GPIO 23 as CAP0 input

//--- mcpwm variables
volatile uint32_t cap_count= 0;  //cycles between cap interrupts
volatile uint32_t last_count= 0; //last cap_count value
volatile uint32_t diff_usecs= 0;   //calculated frequency
volatile bool capture = false;

bool cap_ISR_cb(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_channel, const cap_event_data_t *edata,void *user_data){ //this function need to be in that format to be recognized as cap_isr_cb_t type

// cap_count = (uint32_t) edata->cap_value; //same as mcpwm_capture_signal_get_value()
cap_count = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);

// calculating the frequency: 
//cap_calc = 1/((cap_count - last_count)*0.0000000125); //Count freq = 80Mhz, Period= 0.0000000125
// cap_calc = cap_count - last_count; //Count freq = 80Mhz, Period= 0.0000000125
diff_usecs = (cap_count - last_count) / 80;

if (last_count != 0 && cap_count != 0) {
    capture = true;
}

last_count = cap_count;

return 0; //Whether a task switch is needed after the callback function returns, this is usually due to the callback wakes up some high priority task.

}

mcpwm_capture_config_t MCPWM_cap_config = { //Capture channel configuration
  .cap_edge = MCPWM_NEG_EDGE,               /*!<Set capture edge*/
  .cap_prescale = 1,                        /*!<Prescale of capture signal, ranging from 1 to 256 */
  .capture_cb = cap_ISR_cb,                 /*!<User defined capture event callback, running under interrupt context */
  .user_data = nullptr,                     /*!<User defined ISR callback function args*/
};


void setup() {
    Serial.begin(115200);
    Serial.println("Wind hello.");

    pinMode(5, INPUT);

    Wire.begin(22, 23);
    oled.begin(&Adafruit128x64, 0x3C);
    oled.setFont(System5x7);

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_IN);
    //gpio_pulldown_en((gpio_num_t)GPIO_CAP0_IN);             

    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, &MCPWM_cap_config));
}

uint32_t last_capture = 0;
double rps;
double kmh;
double ms;
uint32_t last_display = 0;
bool new_data = false;

void display();

void loop() {
    
    if (capture) {
        capture = false;
        uint32_t now = millis();
        if (now - last_capture < 2000) {
            rps = 1000000.0 / (double) diff_usecs;
            kmh = rps * 2.5;
            ms = kmh / 3.6;

            new_data = true;

            Serial.print("diff = ");
            Serial.print(diff_usecs);
            Serial.print("; rps = ");
            Serial.println(rps);
        }
        last_capture = now;        
    } else if (millis() - last_capture > 10000) {
        if (rps > 0) {
            rps = 0;
            kmh = 0;
            ms = 0;
            new_data = true;
        }
    }

    if (last_display == 0 || millis() - last_display >= 2000) {
        last_display = millis();
        if (new_data) {
            display();
            new_data = false;
        }
    }
    

}

void display() {
    oled.clear();
    oled.println("WIND"); oled.println("");
    oled.print(rps); oled.println(" u/min");
    oled.print(kmh); oled.println(" km/h");
    oled.print(ms); oled.println(" m/s");
}
