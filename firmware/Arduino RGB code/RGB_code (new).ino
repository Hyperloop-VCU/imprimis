
int relay_1 = 3; //relay module 1(green)
int relay_3 = 5; //relay module 3(yellow)
int relay_7 = 7; //relay module 7(red)


//arduino pin 1
int pin = 1;



void setup() {

     Serial.begin(9600);

  pinMode(relay_1, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_7, OUTPUT);
  
}

void loop() {
   // If robot is controlled Autonomously; {make only the green light flash a few seconds}.
  if (pin == 6){

  digitalWrite(relay_1, HIGH);

  delay(500);

  digitalWrite(relay_1, LOW);
 

  delay(500);
}

// If robot is not autonomous then it's in manual mode. So "else" in this case means it has no choice but to be manual; { turn all lights on}.
else{
 if (pin == 7){
  digitalWrite(relay_1, HIGH);
  digitalWrite(relay_3, HIGH);
  digitalWrite(relay_7, HIGH);

  
}
  
}

  }
 

  
  
