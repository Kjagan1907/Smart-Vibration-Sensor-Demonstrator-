/* 
   --------------------------------Description------------------------------
   This is rough code and under testing
*/

#include <Arduino_LSM9DS1.h>
#include "arduinoFFT.h"

arduinoFFT FFT;

const uint16_t samples = 2048; // This value MUST ALWAYS be a power of 2 for optimized implementation
double samplingFrequency;

double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

bool newDataAvailable = false; // Flag to track new data

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  IMU.setAccelODR(5);
  samplingFrequency = IMU.getAccelODR(); // Assign the value in setup
}
double getRMS_TIME(double* vData, uint16_t bufferSize) {
  double sumOfSquares = 0;

  // Start the loop from the specified number of samples to omit
  for (uint16_t i = 0; i < bufferSize; i++) {
    sumOfSquares += vData[i] * vData[i];
  }

  double meanOfSquares = sumOfSquares / bufferSize;
  return sqrt(meanOfSquares);
}
double getRMS_FFT(double* vData, uint16_t bufferSize, uint16_t numSamplesToOmit = 0) {
  double sumOfSquares = 0;

  // Start the loop from the specified number of samples to omit
  for (uint16_t i = numSamplesToOmit; i < bufferSize; i++) {
    sumOfSquares += vData[i] * vData[i];
  }

  double meanOfSquares = sumOfSquares / (bufferSize - numSamplesToOmit);
  return sqrt(meanOfSquares) / sqrt(bufferSize - numSamplesToOmit);
}

void loop() {
  double sum = 0;
  Serial.println(samplingFrequency);
  for (uint16_t i = 0; i < samples;) {
    if (IMU.accelerationAvailable()) {
      float x, y, z;
      IMU.readAcceleration(x, y, z);

      // You can choose which axis to use for FFT, for example, x-axis:
      vReal[i] = z;
      vImag[i] = 0.0;

      Serial.print(String(x));
      Serial.print(",");
      Serial.print(String(y));
      Serial.print(",");
      Serial.println(String(z));

      i++;
      newDataAvailable = true; // Set the flag to indicate new data is available
    } else {
      newDataAvailable = false; // No new data is available
    }
  }
  // Calculate RMS on the accelerometer data
  double rmsAcceleration = getRMS_TIME(vReal, samples);



  FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */

  /* Print the results of the simulated sampling according to time */
  Serial.println("Data:");
  PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
  Serial.println("Weighed data:");
  PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(FFT_FORWARD); /* Compute FFT */
  Serial.println("Computed Real values:");
  PrintVector(vReal, samples, SCL_INDEX);
  Serial.println("Computed Imaginary values:");
  PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(); /* Compute magnitudes */
  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);

  // Print RMS value
  Serial.print("RMS Acceleration: ");
  Serial.println(rmsAcceleration);

  // Calculate RMS on the magnitudes of FFT values, omitting the first two magnitudes
  double rmsFFT = getRMS_FFT(vReal, samples >> 1, 2);

  // Print RMS value of FFT magnitudes
  Serial.print("RMS FFT Magnitudes (Omitting first two samples): ");
  Serial.println(rmsFFT);

  double pk = FFT.MajorPeak();
  double normalizedMagnitudeAtPeak = GetNormalizedMagnitudeAtFrequency(vReal, vImag, samples, samplingFrequency, pk);
  Serial.print(pk, 2);
  Serial.print("\t");
  Serial.println(normalizedMagnitudeAtPeak, 2);

  // Delay or any additional processing you need
  delay(2000); /* Repeat after delay */
}


double GetNormalizedMagnitudeAtFrequency(double *vReal, double *vImag, uint16_t samples, double samplingFrequency, double targetFrequency) {
  double frequencyResolution = samplingFrequency / samples;
  uint16_t targetIndex = static_cast<uint16_t>(round(targetFrequency / frequencyResolution));
  double magnitude = sqrt(vReal[targetIndex] * vReal[targetIndex] + vImag[targetIndex] * vImag[targetIndex]);
  double normalizedMagnitude = (2.0 / samples) * magnitude;
  return normalizedMagnitude;
}
void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType) {
  for (uint16_t i = 0; i < bufferSize; i++) {
    double abscissa;

    /* Print abscissa value */
    switch (scaleType) {
      case SCL_INDEX:
        abscissa = (i * 1.0);
        break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
        break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
        break;
    }
    Serial.print(abscissa, 2);
    if (scaleType == SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 2);
  }
  Serial.println();
}
