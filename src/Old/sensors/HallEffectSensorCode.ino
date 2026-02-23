const int hallPin = 2;
const int maxCnt = 1;

volatile int cnt = 0;

int INDEX = 0;
float VALUE = 0;
float SUM = 0;
const int WINDOW_SIZE = 3;
float READINGS[WINDOW_SIZE];
float AVERAGED = 0;

void count() {
  cnt++;
}

void setup() {
  Serial.begin(9600);
  pinMode(hallPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(hallPin), count, FALLING);
}

void loop() {
  unsigned long start = micros();
  while (cnt < maxCnt) ;
  float seconds = (micros() - start) / 1000000.0;
  float rpm = (cnt / seconds * 60.0) / 3;

  SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
  VALUE = rpm;        // Read the next sensor value
  READINGS[INDEX] = VALUE;           // Add the newest reading to the window
  SUM = SUM + VALUE;                 // Add the newest reading to the sum
  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size

  AVERAGED = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result

  //Serial.print("");
  Serial.println(AVERAGED);
  cnt = 0;
}
