#include <Servo.h>

// Arduino pin assignment

#define PIN_LED   9   // LED active-low

#define PIN_POTENTIOMETER 0 // Potentiometer at Pin A0
// Add IR Sensor Definition Here !!!
#define PIN_SERVO 10

#define _DIST_MIN 100     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300     // maximum distance to be measured (unit: mm)

#define _DUTY_MIN 553  // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counter-clockwise position (180 degree)


// Target Distance
#define _TARGET_LOW  100.0
#define _TARGET_HIGH 250.0

#define LOOP_INTERVAL 50   // Loop Interval (unit: msec)

#define _EMA_ALPHA 0.1    // EMA weight of new sample (range: 0 to 1)

float dist_ema, dist_prev = _DIST_MAX;
float dist;

Servo myservo;
unsigned long last_loop_time;   // unit: msec
unsigned long last_sampling_time_servo;  // unit: msec
unsigned long last_sampling_time_serial; // unit: msec



void setup()
{
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(1000000);
}

void loop()
{
  unsigned long time_curr = millis();
  int a_value, duty;

  // wait until next event time
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;


  // Remove Next line !!!
  a_value = analogRead(PIN_POTENTIOMETER);
  // Read IR Sensor value !!!
  dist = (6762.0/(a_value-9)-4.0)*10.0;
 
   
    if ((dist_ema < _DIST_MIN) || (dist_ema > _DIST_MAX))
        dist_ema = dist_prev;
    else
        dist_prev = dist_ema;
  
  // we need distance range filter here !!!

dist_ema = dist*_EMA_ALPHA + (1-_EMA_ALPHA)*dist_ema;
  // we need EMA filter here !!!

if (dist_ema < _DIST_MIN) {
    dist_ema = dist_prev;           // cut lower tha00n minimum
    digitalWrite(PIN_LED, 1);       // LED OFF

  } else if (dist_ema > _DIST_MAX) {
    dist_ema = dist_prev;           // Cut higher than maximum
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else {    // In desired Range
    digitalWrite(PIN_LED, 0);       // LED ON      
    dist_prev = dist_ema;
  }

  // map distance into duty
  duty = (dist_ema -10)*12;
  myservo.writeMicroseconds(duty);

  // print IR sensor value, distnace, duty !!!
  Serial.print("Min:");    Serial.print(_DIST_MIN);
    Serial.print(",IR:");   Serial.print(a_value);
    Serial.print(",dist:");   Serial.print(dist);
    Serial.print(",ema:"); Serial.print(dist_ema);  
    Serial.print(",servo:");  Serial.print(duty);
    Serial.print(",Max:");   Serial.print(_DIST_MAX);
    Serial.println("");
}
