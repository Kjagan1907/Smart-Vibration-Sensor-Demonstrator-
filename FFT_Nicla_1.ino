#include "Arduino_BHY2.h"
#include "arduinoFFT.h"

arduinoFFT FFT;
SensorXYZ accel(SENSOR_ID_ACC);
const uint16_t samples = 1024; // This value MUST ALWAYS be a power of 2
int samplingFrequency;
double vReal[samples];
double vImag[samples];
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

void setup() {
  Serial.begin(115200);
  while (!Serial);

  BHY2.begin(NICLA_STANDALONE);
  accel.begin(400);
  SensorConfig cfg = accel.getConfiguration();
  samplingFrequency = cfg.sample_rate;
  FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); // Initialize the FFT object
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
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = micros();
  float x, y, z;
  static auto i = 0;
  

  // Update accelerometer data at a reduced frequency (e.g., every 10ms)
  while (currentTime - lastUpdateTime >= 2500 && i < samples) {
    
    lastUpdateTime = currentTime;

    //unsigned long startTime = micros(); // Record start time

    BHY2.update();
    //accel.configure(200, 0); // Configures the accelerometer with a sampling rate and latency in ms
    x = accel.x();
    y = accel.y();
    z = accel.z();

    //unsigned long endTime = micros(); // Record end time
    //unsigned long totalLatency = endTime - startTime;
      SensorConfig cfg = accel.getConfiguration();
      Serial.println(cfg.sample_rate);
    // Print the accelerometer data
    Serial.print(String(x));
    Serial.print(",");
    Serial.print(String(y));
    Serial.print(",");
    Serial.println(String(z));
    //Serial.print(",");
    //Serial.println(totalLatency);

    vReal[i] = z / 4096; // Divide with respective binary to 'g' value #for better visualization of magnitude values
    vImag[i] = 0.0;
    //sum += vReal[i]*vReal[i];
    i++;
  }

  if (i == samples) {
      // Calculate RMS on the accelerometer data
    double rmsAcceleration = getRMS_TIME(vReal, samples);
    /* Print the results of the simulated sampling according to time */
    Serial.println("Data:");
    PrintVector(vReal, samples, SCL_TIME);
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
    Serial.println("Weighed data:");
    PrintVector(vReal, samples, SCL_TIME);
    FFT.Compute(FFT_FORWARD); /* Compute FFT */
    // Print the results of the FFT
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
    // Serial.println(magnitudeAtPeak_test, 4);
    // Calculate RMS values for time series data and FFT magnitudes

    i = 0; // Reset the sample counter

    //while(1);
    delay(2000); // Repeat after a delay
  }
}

double GetNormalizedMagnitudeAtFrequency(double *vReal, double *vImag, uint16_t samples, double samplingFrequency, double targetFrequency) {
  double frequencyResolution = samplingFrequency / samples;
  uint16_t targetIndex = static_cast<uint16_t>(round(targetFrequency / frequencyResolution));
  double magnitude = sqrt(vReal[targetIndex] * vReal[targetIndex] + vImag[targetIndex] * vImag[targetIndex]);
  double normalizedMagnitude = (2.0 / samples) * magnitude;
  return normalizedMagnitude;
}

// double GetMagnitudeAtFrequency_test(double *vReal, double *vImag, uint16_t samples, double samplingFrequency, double targetFrequency) {
//   // Calculate the frequency resolution
//   double frequencyResolution = samplingFrequency / samples;

//   // Find the index of the target frequency
//   uint16_t targetIndex = static_cast<uint16_t>(round(targetFrequency / frequencyResolution));

//   // Calculate the magnitude as the absolute value of the complex number at the target index
//   double magnitude = vReal[targetIndex];

//   return magnitude;
// }

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
