#define PIN_LED 13
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while(!Serial){
    ;
  }
  Serial.println("Hello World!");
  count = toggle = 0;
  digitalWrite(PIN_LED,toggle);
  // put your setup code here, to run once:
}
void loop() {

  Serial.println(++count);
  toggle = toggle_state(toggle); //toggle LED value
  digitalWrite(PIN_LED,toggle); // update LED status.
  delay(1000);
  
  // put your main code here, to run repeatedly:
}

int toggle_state(int toggle){
  return toggle;
}
