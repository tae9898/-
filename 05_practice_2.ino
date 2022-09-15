int LED = 7;

void setup() {
  // put your setup code here, to run once:
  
  pinMode(LED, OUTPUT); 
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED, LOW); 
  delay(1000);            
  digitalWrite(LED, HIGH);  
  delay(200);  
   
  digitalWrite(LED, LOW); 
  delay(200);            
  digitalWrite(LED, HIGH);  
  delay(200);  
   
  digitalWrite(LED, LOW); 
  delay(200);            
  digitalWrite(LED, HIGH);  
  delay(200);  
   
  digitalWrite(LED, LOW); 
  delay(200);            
  digitalWrite(LED, HIGH);  
  delay(200);  
   
  digitalWrite(LED, LOW); 
  delay(200);            
  digitalWrite(LED, HIGH);  
  delay(200);  
   
  digitalWrite(LED, LOW); 
  delay(200);            
  digitalWrite(LED, HIGH);  
  delay(200);  
      
  while(1){ }
}
