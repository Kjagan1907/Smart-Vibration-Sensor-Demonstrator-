#include "Arduino.h"
#include "Arduino_BHY2.h"
#include "Nicla_System.h"
#include "arduinoFFT.h"

arduinoFFT FFT;

const uint16_t samples = 512;
int datalines[samples][4];
int sampleInterval = 2500;
int samplingFrequency;

SensorXYZ accel(SENSOR_ID_ACC);
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

void setup()
{
   Serial.begin(115200);
   while (!Serial);

   BHY2.begin(NICLA_STANDALONE);
   accel.begin(400, 0);

   SensorConfig cfg = accel.getConfiguration();
   samplingFrequency = cfg.sample_rate;
   FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency);

   nicla::begin();
   nicla::leds.begin();

   BHY2.delay(50);
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
void loop()
{
   static auto sampleTime = micros();
   static auto startTime = millis();

   /* record values to datalines */
   for (int i = 0; i < samples; i++) {
      sampleTime = micros();
      while (micros() - sampleTime < sampleInterval) {
         BHY2.update();
      }
      datalines[i][0] = millis() - startTime;
      datalines[i][1] = accel.x();
      datalines[i][2] = accel.y();
      datalines[i][3] = accel.z();
   }

   /* write the data to PC over serial */
   for (int k = 0; k < samples; k++) {
      vReal[k] = datalines[k][3] / 4096;
      vImag[k] = 0.0;
      Serial.println(String(datalines[k][0]) + "," + String(datalines[k][1]) + "," + String(datalines[k][2]) + "," + String(datalines[k][3]) + ";");
      delay(30);
   }
// Calculate RMS on the accelerometer data
  double rmsAcceleration = getRMS_TIME(vReal, samples);
   Serial.println("Data:");
   PrintVector(vReal, samples, SCL_TIME);
   FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
   Serial.println("Weighed data:");
   PrintVector(vReal, samples, SCL_TIME);
   FFT.Compute(FFT_FORWARD);
   Serial.println("Computed Real values:");
   PrintVector(vReal, samples, SCL_INDEX);
   Serial.println("Computed Imaginary values:");
   PrintVector(vImag, samples, SCL_INDEX);

   FFT.ComplexToMagnitude();
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

   // Additional loop code...

   delay(1000);  // Adjust the delay as needed
}

double GetNormalizedMagnitudeAtFrequency(double *vReal, double *vImag, uint16_t samples, double samplingFrequency, double targetFrequency)
{
   double frequencyResolution = samplingFrequency / samples;
   uint16_t targetIndex = static_cast<uint16_t>(round(targetFrequency / frequencyResolution));
   double magnitude = sqrt(vReal[targetIndex] * vReal[targetIndex] + vImag[targetIndex] * vImag[targetIndex]);
   double normalizedMagnitude = (2.0 / samples) * magnitude;
   return normalizedMagnitude;
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
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
