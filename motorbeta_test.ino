//inputs
int motorRforward = 6;
int motorRreverse = 7;
int motorLforward = 8;
int motorLreverse = 8;


void setup() {
//output to board, connections
plrMode(motorRforward, OUTPUT);
plrMode(motorRreverse, OUTPUT);
plrMode(motorLforward, OUTPUT);
plrMode(motorLreverse, OUTPUT);
}

void loop() {
//writes to board, on or off
digitalWrite(motorRforward, HIGH);
digitalWrite(motorRreverse, LOW);
digitalWrite(motorLforward, HIGH);
digitalWrite(motorLreverse, LOW);
}
