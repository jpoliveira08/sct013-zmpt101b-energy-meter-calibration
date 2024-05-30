#include <Arduino.h>

#define CLOCKRATE 80000000 /* Hz */
#define TIMERDIVIDER 4

#define OVER_SAMPLE_RATIO (16)
#define CYCLES (20)
#define NSAMPLES (OVER_SAMPLE_RATIO * CYCLES)

#define ADC_BITS 12
#define ADC_COUNTS (1 << ADC_BITS) // 4096


#define VOLTAGE_ADC_PIN (34)
#define CURRENT_ADC_PIN (35)

volatile int sampleCount = NSAMPLES;
volatile int voltageSamples[NSAMPLES];
volatile int currentSamples[NSAMPLES];

const uint8_t pinVoltageAdc = 34;
const uint8_t pinCurrentAdc = 35;

hw_timer_t *My_timer = NULL;

struct measurements {
  float Vrms;
  float Irms;
};


void IRAM_ATTR onTimer() {
  if ((sampleCount >= 0) && (sampleCount < NSAMPLES)) {
    sampleCount = sampleCount + 1;
    voltageSamples[sampleCount] = analogRead(VOLTAGE_ADC_PIN);
    currentSamples[sampleCount] = analogRead(CURRENT_ADC_PIN);
  }
}


void setupMeasurement() {
  My_timer = timerBegin(1, TIMERDIVIDER, true);

  timerAttachInterrupt(My_timer, &onTimer, true);

  float measureRatePerInterval = 1.0 / ( 60.0 * OVER_SAMPLE_RATIO);

  int amountTimeBetweenInterruption = (int)( measureRatePerInterval * CLOCKRATE / TIMERDIVIDER + 0.5);

  timerAlarmWrite(My_timer, amountTimeBetweenInterruption, true);
  timerAlarmEnable(My_timer);
}

void readAnalogSamples() {
  int waitDelay = 17 * CYCLES;
  sampleCount = 0;

  delay(waitDelay);

  if (sampleCount != NSAMPLES) {
    Serial.print("ADC processing is not working.");
  }

  timerWrite(My_timer, 0);
}

struct measurements measureRms(int* voltageSamples, int* currentSamples, int nsamples) {
  struct measurements eletricMeasurements;
  int32_t sumVoltageSamples = 0;
  int32_t sumCurrentSamples = 0;

  for (int i = 0; i < nsamples; i++) {
    sumVoltageSamples += voltageSamples[i];
    sumCurrentSamples += currentSamples[i];
  }

  int voltageMean = (int)(sumVoltageSamples / (int32_t)(nsamples));
  int currentMean = (int)(sumCurrentSamples / (int32_t)(nsamples));

  int32_t sumVoltage = 0;
  int32_t sumCurrent = 0;
  for (int i = 0; i < nsamples; i++) {
    int32_t y_voltage = (voltageSamples[i] - voltageMean);
    int32_t y_current = (currentSamples[i] - currentMean);
  
    sumVoltage += y_voltage * y_voltage;
    sumCurrent += y_current * y_current;
  }

  float ym_voltage = (float) sumVoltage / (float) nsamples;
  float ym_current = (float) sumCurrent / (float) nsamples;

  float Vrms = sqrt(ym_voltage);
  float Irms = sqrt(ym_current);

  eletricMeasurements.Vrms = Vrms * 3.3/4096.0;
  eletricMeasurements.Irms = Irms * 3.3/4096.0;

  return eletricMeasurements;
}

struct measurements makeMeasurement() {
  struct measurements eletricMeasurements;

  readAnalogSamples();
  if (sampleCount == NSAMPLES) {
    eletricMeasurements = measureRms((int*) voltageSamples, (int*) currentSamples, NSAMPLES);
  }

  return eletricMeasurements;
}

void setup() {
  Serial.begin(115200);

  setupMeasurement();

  struct measurements eletricMeasurements;

  eletricMeasurements = makeMeasurement();

  Serial.print("Vrms_esp32: ");
  Serial.print(eletricMeasurements.Vrms, 5);
  Serial.print(" Irms_esp32: ");
  Serial.println(eletricMeasurements.Irms, 5);
}

void loop() {
  
}