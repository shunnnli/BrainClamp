const int numSamples = 1000;  // Number of samples to average

void setup() {
  Serial.begin(115200);
}

void loop() {
  unsigned long startTime, endTime, totalTime = 0;
  
  // Take a series of analog readings
  for (int i = 0; i < numSamples; i++) {
    startTime = micros();
    analogRead(A0);  // Replace with your desired analog pin
    endTime = micros();
    totalTime += (endTime - startTime);
  }
  
  // Calculate average time per sample in microseconds
  float avgTime = totalTime / (float)numSamples;
  
  // Convert average time to frequency (samples per second)
  float samplingFrequency = 1000000.0 / avgTime;
  
  Serial.print("Average conversion time: ");
  Serial.print(avgTime);
  Serial.print(" us, which corresponds to ~");
  Serial.print(samplingFrequency);
  Serial.println(" Hz sampling frequency.");
  
  // Wait a bit before running the test again
  delay(1000);
}
