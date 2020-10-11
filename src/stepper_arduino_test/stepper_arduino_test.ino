// Define pin connections & motor's steps per revolution
const int dirPin = 4;
const int stepPin = 6;
const int stepsPerRevolution = 800;
int speedd=300;
unsigned long int st=micros();
int count=0;
void setup()
{
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
   digitalWrite(dirPin, LOW);
   digitalWrite(stepPin, LOW);
   Serial.begin(9600);
}
void loop()
{
  if (micros()>st+speedd)
  {
    st=micros();
    digitalWrite(stepPin, !digitalRead(stepPin));
    count++;
  }
  if (count>=800)
  {
    count=0;
    speedd+=100;
    Serial.println(speedd);
    }

  
  // Set motor direction counterclockwise

  
  // Spin motor quickly
  
//  for(int x = 0; x < stepsPerRevolution; x++)
//  {
//    digitalWrite(stepPin, HIGH);
//    delayMicroseconds(speedd);
//  }
//  speedd+=100;
//  if (speedd>20000)
//  {
//  speedd=20000;
//  }
  //speedd=Serial.parseInt();
  //delay(5000); // Wait a second
}
