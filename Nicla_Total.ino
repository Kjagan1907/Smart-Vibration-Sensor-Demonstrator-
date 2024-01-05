/*
  Smart Vibration Sensor Demonstrator
  Author: Jaganmohan Karumanchi

  Description:
  This code is for performing FFT on acceleration data of Nicla. This is to understand the smart capabilities accelerometer in the board.
  I have done this to realize the maximum possible sampling rate and samples Nicla can perform the FFT.
  Here I have included RMS calculation of both time series and fft values to check the code by giving known sine and random excitations with external shaker.

  Credits:
  - Portions of this code are based on the Arduino FFT library:
    https://github.com/kosme/arduinoFFT
    Copyright (C) [2023] [kosme]

  - Parts of the code are derived from Arduino_BHY2 library:
    Copyright (C)

  - Code contribution from Moechl in Bosch Sensortech Community:
    Moechl's posting link: https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/Arduino-Nicla-BHI260AP-Timestamp-and-Data-Logging-to-SPI-Flash/td-p/55728

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


#include "Arduino.h"
#include "Arduino_BHY2.h"
#include "Nicla_System.h"
#include "arduinoFFT.h"

arduinoFFT FFT;

const uint16_t samples = 512; // Samples taken to perform FFT, should be a power of 2 for FFT
int datalines[samples][3];
int sampleInterval = 5000; // In microseconds, derived from sampling frequency, max possible is 200Hz, 1/200 = 5000 microseconds
int samplingFrequency;

SensorXYZ accel(SENSOR_ID_ACC);
double vReal[samples]; 
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

void setup() {
  Serial.begin(115200);
  while (!Serial);

  BHY2.begin(NICLA_STANDALONE); // NICLA_STANDALONE to increase the samples it can take to perform FFT
  accel.begin(200, 0); // Sampling rate of 200Hz and latency of 0 ms

  SensorConfig cfg = accel.getConfiguration();
  samplingFrequency = cfg.sample_rate;
  FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency);

  nicla::begin();
  nicla::leds.begin();

  BHY2.delay(50);
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
  static auto sampleTime = micros();

  for (int i = 0; i < samples; i++) {
    sampleTime = micros();
    while (micros() - sampleTime < sampleInterval) // To ensure getting the updated values, based on the sampling frequency
    {  
      BHY2.update();
    }
    datalines[i][0] = accel.x();
    datalines[i][1] = accel.y();
    datalines[i][2] = accel.z();
  }

  // To write the data PC over port
  for (int k = 0; k < samples; k++) {
    vReal[k] = datalines[k][2] / 4096; // Only Z axis data is taken for performing FFT
    vImag[k] = 0.0; // no phase angle or delay
    Serial.println(String(datalines[k][0]) + "," + String(datalines[k][1]) + "," + String(datalines[k][2]) + ";");
    delay(30);
  }

  double rmsAcceleration = getRMS_TIME(vReal, samples);
  Serial.println("Data:");
  PrintVector(vReal, samples, SCL_TIME); // to print the data taken for performing FFT
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  Serial.println("Weighed data:");
  PrintVector(vReal, samples, SCL_TIME); // Hamming windowed data
  FFT.Compute(FFT_FORWARD);
  Serial.println("Computed Real values:");
  PrintVector(vReal, samples, SCL_INDEX);
  Serial.println("Computed Imaginary values:");
  PrintVector(vImag, samples, SCL_INDEX);

  FFT.ComplexToMagnitude();
  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY); 
  // Print RMS value of time series data
  Serial.print("RMS Acceleration: ");
  Serial.println(rmsAcceleration);

  // Calculate RMS on the magnitudes of FFT values, omitting the first two magnitudes
  double rmsFFT = getRMS_FFT(vReal, samples >> 1, 2);
  Serial.print("RMS FFT Magnitudes (Omitting first two samples): ");
  Serial.println(rmsFFT);

  double resonantFrequency = FFT.MajorPeak();
  double normalizedMagAtPeakFrequency = GetNormalizedMagnitudeAtPeakFrequency(vReal, vImag, samples, samplingFrequency, resonantFrequency); 
  Serial.print(resonantFrequency, 2);
  Serial.print("\t");
  Serial.println(normalizedMagAtPeakFrequency, 2);

  delay(1000);  // Delay to check the result
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

    Serial.print(abscissa, 6);

    if (scaleType == SCL_FREQUENCY)
      Serial.print("Hz");

    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
