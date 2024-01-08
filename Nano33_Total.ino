/*
  Smart Vibration Sensor Demonstrator
  Author: Jaganmohan Karumanchi

  Description:
  This code is for performing FFT on acceleration data of Nano board. This is to understand the smart capabilities accelerometer in the board.
  I have done this to realize the maximum possible sampling rate and samples Nano can perform the FFT.
  Here I have included RMS calculation of both time series and FFT values to check the code by giving known sine and random excitations with an external shaker.

  Credits:
  - Portions of this code are based on the Arduino FFT library:
    https://github.com/kosme/arduinoFFT
    Copyright (C) [2023] [kosme]

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

const uint16_t samples = 512; // Samples taken to perform FFT, should be a power of 2 for FFT
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
  IMU.setAccelODR(5); // Max sampling rate at which it can perform the computation correctly
  samplingFrequency = IMU.getAccelODR();
}

// To calculate the RMS value of the time series data
double getRMS_TIME(double* vData, uint16_t bufferSize) {
  double sumOfSquares = 0;

  for (uint16_t i = 0; i < bufferSize; i++) {
    sumOfSquares += vData[i] * vData[i];
  }

  double meanOfSquares = sumOfSquares / bufferSize;
  return sqrt(meanOfSquares);
}

// To calculate the RMS value of the magnitudes of FFT. first two samples are omitted to avoid the DC offset or edge effect
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
    if (IMU.accelerationAvailable()) {  // To ensure getting the updated values based on the sampling frequency
      float x, y, z;
      IMU.readAcceleration(x, y, z);

      // You can choose which axis to use for FFT, for example
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

  double rmsAcceleration = getRMS_TIME(vReal, samples);

  FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */

  Serial.println("Data:");
  PrintVector(vReal, samples, SCL_TIME); // to print the data taken for performing FFT
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
  Serial.println("Weighed data:");
  PrintVector(vReal, samples, SCL_TIME); // Hamming windowed data
  FFT.Compute(FFT_FORWARD); /* Compute FFT */
  Serial.println("Computed Real values:");
  PrintVector(vReal, samples, SCL_INDEX);
  Serial.println("Computed Imaginary values:");
  PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(); /* Compute magnitudes */
  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);

  // Print RMS value of time series data
  Serial.print("RMS Acceleration: ");
  Serial.println(rmsAcceleration);

  // Calculate RMS on the magnitudes of FFT values, omitting the first two magnitudes
  double rmsFFT = getRMS_FFT(vReal, samples >> 1, 2);

  // Print RMS value of FFT magnitudes
  Serial.print("RMS FFT Magnitudes (Omitting first two samples): ");
  Serial.println(rmsFFT);

  double resonantFrequency = FFT.MajorPeak();
  double normalizedMagAtPeakFrequency = GetNormalizedMagnitudeAtPeakFrequency(vReal, vImag, samples, samplingFrequency, resonantFrequency);
  Serial.print(resonantFrequency, 2);
  Serial.print("\t");
  Serial.println(normalizedMagAtPeakFrequency, 2);

  delay(2000); // Delay to check the result
}

// Normalized magnitude at resonant frequency, to compare and check with the exciting amplitude
double GetNormalizedMagnitudeAtPeakFrequency(double *vReal, double *vImag, uint16_t samples, double samplingFrequency, double targetFrequency) {
  double frequencyResolution = samplingFrequency / samples;
  uint16_t targetIndex = static_cast<uint16_t>(round(targetFrequency / frequencyResolution));
  double magnitude = sqrt(vReal[targetIndex] * vReal[targetIndex] + vImag[targetIndex] * vImag[targetIndex]);
  double normalizedMagnitude = (2.0 / samples) * magnitude;
  return normalizedMagnitude;
}

// To perform and print the FFT results
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
