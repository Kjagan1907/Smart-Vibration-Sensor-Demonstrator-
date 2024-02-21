/*
  Smart Vibration Sensor Demonstrator
  Author: Jaganmohan Karumanchi

  Description:
  This code is for performing FFT on acceleration data of Nano board. This is to understand the smart capabilities accelerometer in the board.
  I have done this to realize the maximum possible sampling rate and samples Nano can perform the FFT.
  I have included the timestamps for the sake of comparing the computational power of this board with the Nicla board.
  Here I have included RMS calculation of both time series and FFT values to check the code by giving known sine and random excitations with an external shaker.

  Credits:
  - Portions of this code are based on the Arduino FFT library:
    https://github.com/kosme/arduinoFFT
    Copyright (C) [2023] [kosme]

  - Parts of the code are derived from Arduino_BHY2 library:
    Copyright (C)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <Arduino_LSM9DS1.h>
#include "arduinoFFT.h"

arduinoFFT FFT;

const uint16_t samples = 512; // This value MUST ALWAYS be a power of 2 for optimized implementation
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
  IMU.setAccelODR(4);
  samplingFrequency = IMU.getAccelODR(); // Assign the value in setup
}
double calculateKurtosis(double *vData, uint16_t bufferSize) {
  double mean = 0.0;
  double sumSquaredDeviation = 0.0;
  double sumFourthMoment = 0.0;

  // Calculate mean
  for (uint16_t i = 0; i < bufferSize; ++i) {
    mean += vData[i];
  }
  mean /= bufferSize;

  // Calculate sum of squared deviations and sum of fourth moments
  for (uint16_t i = 0; i < bufferSize; ++i) {
    double deviation = vData[i] - mean;
    sumSquaredDeviation += deviation * deviation;
    sumFourthMoment += deviation * deviation * deviation * deviation;
  }

  // Calculate kurtosis
  double variance = sumSquaredDeviation / bufferSize;
  double kurtosis = (sumFourthMoment / bufferSize) / (variance * variance);

  return kurtosis;
}

double getRMS_TIME(double* vData, uint16_t bufferSize) {
  double sumOfSquares = 0;

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
  unsigned long startTime = millis();
  double peakValue = 0.0;
  for (uint16_t i = 0; i < samples;) {
    if (IMU.accelerationAvailable()) {
      float x, y, z;
      IMU.readAcceleration(x, y, z);

      // You can choose which axis to use for FFT, for example, x-axis:
      z = z -1.0;
      vReal[i] = z;
      vImag[i] = 0.0;

      // Update peakValue
      if (abs(z) > peakValue) {
        peakValue = abs(z);
      }

      i++;
      newDataAvailable = true; // Set the flag to indicate new data is available
    } else {
      newDataAvailable = false; // No new data is available
    }
  }

  // Calculate RMS on the accelerometer data
  double rmsAcceleration = getRMS_TIME(vReal, samples);
  // Calculate kurtosis on the accelerometer data
  double kurtosisValue = calculateKurtosis(vReal, samples);

  FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */

  //FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
  FFT.Compute(FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(); /* Compute magnitudes */


  // Calculate RMS on the magnitudes of FFT values, omitting the first two magnitudes
  double rmsFFT = getRMS_FFT(vReal, samples >> 1, 2);

  double pk = FFT.MajorPeak();
  double normalizedMagnitudeAtPeak = GetNormalizedMagnitudeAtFrequency(vReal, vImag, samples, samplingFrequency, pk);
  //Serial.println(kurtosisValue);
  // Determine whether the input is a sine wave or random noise
  if (kurtosisValue < 2.5) {
    // Signal is a sine wave
    Serial.print("Sine, ");
    Serial.print(pk, 2);
    Serial.print("Hz;");
    Serial.print(" Amp: ");
    Serial.print(normalizedMagnitudeAtPeak, 2);
    Serial.print("g");
  } //else if(kurtosisValue = 3) {
    else {
    // Signal is random noise
    Serial.print("Random, ");
    //Serial.print("RMSvalue: ");
    Serial.print(rmsAcceleration, 2);
    Serial.print("grms");
    double threshold = 5.0;
    if (rmsAcceleration > threshold) {
      //Serial.print(" and\t");
      Serial.print(", limit Exceeded");

    }
  }

  // Check if the peak value exceeds a limit
  double peakLimit = 2; // Adjust this threshold as needed
  if (peakValue > peakLimit) {
    Serial.print(", PkLimit of ");
    Serial.print(peakLimit);
    Serial.print("g crossed, Peak: ");
    Serial.print(peakValue, 2);
    Serial.print("g");
  }

  Serial.println(); // Print newline
  unsigned long computationalTime = millis() - startTime;
  //Serial.println(computationalTime);
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
