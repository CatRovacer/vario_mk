
#define LED_1 17  //R
#define LED_2 18  //G
#define LED_3 19  //B

// #define LED_1 27
// #define LED_2 28
// #define LED_3 29

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  digitalWrite(LED_1, LOW); digitalWrite(LED_2, LOW); digitalWrite(LED_3, LOW);

}

// the loop function runs over and over again forever
void loop() {
//  digitalWrite(LED_1, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(100);                      // wait for a second
//  digitalWrite(LED_2, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(100);                      // wait for a second
  digitalWrite(LED_3, HIGH);   // turn the LED off by making the voltage LOW
  delay(100);                      // wait for a second
//  digitalWrite(LED_1, LOW);  // turn the LED on (HIGH is the voltage level)
  delay(200);                      // wait for a second
//  digitalWrite(LED_2, LOW);   // turn the LED off by making the voltage LOW
  delay(100);                      // wait for a second
  digitalWrite(LED_3, LOW);   // turn the LED off by making the voltage LOW
  delay(100);                      // wait for a second
}
