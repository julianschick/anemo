#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    pinMode(5, INPUT);
}

int val = 0;
long n1 = 0;
long n2 = 0;
long n3 = 0;

void loop() {

    int v = digitalRead(5);
    if (v != val) {
        val = v;
        n1 = n2;
        n2 = n3;
        n3 = millis();

        if (val == 0 && n3 != 0 && n2 != 0) {
            //Serial.println("!");
            //Serial.print("n3 - n2 = ");
            //Serial.println(n3 - n2);
            //Serial.print("n2 - n1 = ");
            //Serial.println(n2 - n1);
            int diff = n3 - n2;
            double u_sec = 1000.0 / (double) diff;
            Serial.print("diff = ");
            Serial.print(diff);
            Serial.print("; u_sec = ");
            Serial.println(u_sec);
        }
    }

    
}