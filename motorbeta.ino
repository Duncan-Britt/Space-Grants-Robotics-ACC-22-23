int motor = 9; //variables
int x = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  while(x < 255) {
    x = x + 15; //interval +15 every second until 255
    analogWrite(motor, x);
    delay(1000);
    analogWrite(motor, 0);
    delay(1000);
    Serial.println(x);
    //INCREASE INCREASE INCREASEEEEE
  }

}
