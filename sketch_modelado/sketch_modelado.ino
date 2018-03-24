#include <DueTimer.h>

#define ENCODER_PIN_A 3
#define ENCODER_PIN_B 7

#define MOTOR_PWM_FORWARD 6
#define MOTOR_PWM_BACKWARD 8
#define BRIDGE_ENABLE 2
#define COUNTS_PER_REVOLUTION 3591.84

// 20Khz
#define PWM_FREQ 20000
// 8 bits de resolucion del PWM.
#define PWM_MAX_DUTY 255
#define VMAX 12

// Cogemos datos cada segundo
#define SAMPLING_TIME_MS 1000
#define SAMPLES 1200
#define TIMES 10

#define SPEED 12

#define SERIAL_BPS 112500

int32_t iterations;
int32_t p;
int32_t encoderPosition;
double data[SAMPLES+1];
uint8_t lastState;


void pwmConf();
void initBridge();
void initEncoder();
void setPWMValue(int pin, double percentage);
void setSpeed(double speed);

void sampling();
void writeDataInSerial();

void interruptionPin();
int32_t getCurrentPinState();

void setup() {
  initEncoder();
  initBridge();

  iterations = 0;
  p = 0;
  encoderPosition = 0;
  lastState = 0;
  memset(data, 0, sizeof(data));
  
  
  pwmConf(MOTOR_PWM_FORWARD, PWM_FREQ);
  pwmConf(MOTOR_PWM_BACKWARD, PWM_FREQ);

  // Check encoder
  lastState = getCurrentPinState();  
    
  // Interrupciones del encoder.
  attachInterrupt(ENCODER_PIN_A, interruptionPin, CHANGE); 
  attachInterrupt(ENCODER_PIN_B, interruptionPin, CHANGE);

  Timer1.attachInterrupt(sampling).start(SAMPLING_TIME_MS); // Calls every 1ms
  
  Serial.begin(SERIAL_BPS);
  Serial.println("Init finished");
  
  setSpeed(SPEED);
}

void loop() {
  if (iterations == (SAMPLES+1) && p < TIMES) {
   writeDataInSerial();
   iterations = 0;
   p++;
   setSpeed(SPEED);
   encoderPosition = 0;
  }
}

/**
 * Pone los pines del encoder como input para poder leerlos con digitalWrite
 */
void initEncoder() {
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);
}

/*
 * Configura el puente H
 */
void initBridge() {
  pinMode (MOTOR_PWM_FORWARD, OUTPUT);
  pinMode (MOTOR_PWM_BACKWARD, OUTPUT);
  pinMode (BRIDGE_ENABLE, OUTPUT);
  digitalWrite (BRIDGE_ENABLE, HIGH);
}

/**
 * Funcion para configurar un pwm.
 */
void pwmConf(int pin, int freq) {

    PIO_Configure(g_APinDescription[pin].pPort,
    PIO_PERIPH_B,
    g_APinDescription[pin].ulPin,
    g_APinDescription[pin].ulPinConfiguration);

    int channel = g_APinDescription[pin].ulPWMChannel;
    
    pmc_enable_periph_clk(PWM_INTERFACE_ID);

    // VARIANT_MCK is the micro clock frequency
    PWMC_ConfigureClocks(freq * PWM_MAX_DUTY, freq * PWM_MAX_DUTY, VARIANT_MCK);
   
    PWMC_ConfigureChannel(
      PWM_INTERFACE,
      channel,
      PWM_CMR_CPRE_CLKA, // prescaler
      0, // alignment, 0 = left aligned
      0 // polarity
    );
    
    PWMC_SetPeriod(PWM_INTERFACE,
      channel,
      PWM_MAX_DUTY // period
    );

    // Active PWM
    PWMC_EnableChannel(PWM_INTERFACE, channel);
}
 
/**
 * Pone un valor en el PWM entre 0 y 255, 0 para parado, 255 para maximo. (Ya que la resolucion es de 8 bits)
 * percentage tiene que venir entre 0 1 y 1
 */
void setPWMValue(int const pin, double const percentage) {
  int channel = g_APinDescription[pin].ulPWMChannel;

  double value = (double)((double)percentage/(double)VMAX)*PWM_MAX_DUTY;
  
  PWMC_SetDutyCycle(PWM_INTERFACE, channel, value);
}

/**
 * Speed puede venir desde -12V a 12V
 */
void setSpeed(double speed) {
  if (speed >= 0) {
    setPWMValue(MOTOR_PWM_FORWARD, speed);
    setPWMValue(MOTOR_PWM_BACKWARD, 0);
  } else {
    speed = -speed;
    setPWMValue(MOTOR_PWM_FORWARD, 0);
    setPWMValue(MOTOR_PWM_BACKWARD, speed);
  }
}

/**
 * Retorna el estado actual de los pines del encoder
 */
int32_t getCurrentPinState() {
  bool pinA = digitalRead(ENCODER_PIN_A);
  bool pinB = digitalRead(ENCODER_PIN_B);

  if (!pinA && !pinB) return 0;
  if (!pinA && pinB) return 1;
  if (pinA && !pinB) return 2;
  if (pinA && pinB) return 3;
}

/**
 * Maneja las interrupciones de los pines actualizando el posicion del encoder
 */
void interruptionPin() {
  int32_t state = getCurrentPinState();

  switch(lastState) {
    case 0:
      if (state == 1) encoderPosition--;
      else if (state == 2) encoderPosition++;
      break;
    case 1:
      if (state == 3) encoderPosition--;
      else if (state == 0) encoderPosition++;
      break;
    case 2:
      if (state == 0) encoderPosition--;
      else if (state == 3) encoderPosition++;
      break;
    case 3:
      if (state == 2) encoderPosition--;
      else if (state == 1) encoderPosition++;
      break;
    default:
      break;
  }

  lastState = state;
}

void sampling() {

  if (iterations <= SAMPLES) {
    data[iterations] = encoderPosition;
    iterations++;
  }

  if (iterations == 600)
    setSpeed(0);
}

void writeDataInSerial() {
 for (int i = 0; i <= SAMPLES; i++) {
   Serial.print(i);
   Serial.print(" ");
   Serial.println(data[i]);
 }
}

