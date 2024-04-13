


int relay_1 = 3;
//int relay_2 = 2;
//int relay_3 = 4;

int autoo = 1;





void setup() {

     Serial.begin(9600);

  pinMode(relay_1, OUTPUT);
  //pinMode(relay_2, OUTPUT);
 // pinMode(relay_3, OUTPUT);

}

void loop() {
   // If robot is controlled Autonomously; {make the all the lights flash}.
  if (autoo == 1){

  digitalWrite(relay_1, HIGH);
  //digitalWrite(relay_2, HIGH);
  //digitalWrite(relay_3, HIGH);
 

  Serial.println("All relays ON");
  

  delay(500);

  digitalWrite(relay_1, LOW);
  //digitalWrite(relay_2, LOW);
  //digitalWrite(relay_3, LOW);

  Serial.println("All relays OFF");

  delay(500);
}

// If robot is not autonomous then it's in manual mode. So "else" in this case means it has no choice but to be manual; {Turn only the green light on}.

else {
 // digitalWrite(relay_1, HIGH);
 // digitalWrite(relay_2, HIGH);
 // digitalWrite(relay_3, HIGH);

  Serial.println("All relays ON");
  

//  delay(1000);

 // digitalWrite(relay_1, LOW);
  //digitalWrite(relay_2, LOW);
  //digitalWrite(relay_3, LOW);

  Serial.println("Green realy is on");

  delay(1000);
  
}
  }
 

  
  
