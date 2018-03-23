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

#define SERIAL_BPS 115200

#define VALUE_TO_MOVE_TO 1796

/**
 * Valores de KP (antiguos):
 * 1 -> 0.2154
 * 0.707 -> 0.304
 * 0.3 -> 0.7181
 * 0.1 -> 2.1544
 * 
 * Valores de KP (nuevos):
 * 1 -> 0.18846
 * 0.707 -> 0.26656
 * 0.3 -> 0.628201
 * 0.1 -> 1.8846
 */
#define KP 1.8846

int32_t encoderPosition;
int32_t data[SAMPLES+1];
int32_t iterations;
int32_t p;

//volatile bool firstPositionA = false;
//volatile bool firstPositionB = false;
int32_t lastState;

void pwmConf();
void initEncoder();
void setPWMValue(int pin, double percentage);
//void interruptionPinA();
//void interruptionPinB();
void interruptionPin();
int32_t getCurrentPinState();
void controller();
void setSpeed(double speed);
void writeData();

void setup() {
  initEncoder();
  initBridge();
  
  pwmConf(MOTOR_PWM_FORWARD, PWM_FREQ);
  pwmConf(MOTOR_PWM_BACKWARD, PWM_FREQ);

  encoderPosition = 0;
  iterations = 0;
  p = 0;
  
   // Check encoder
  lastState = getCurrentPinState();
  
  // Interrupciones del encoder.
  attachInterrupt(ENCODER_PIN_A, interruptionPin, CHANGE); 
  attachInterrupt(ENCODER_PIN_B, interruptionPin, CHANGE);
  
  Timer1.attachInterrupt(controller).start(SAMPLING_TIME_MS); // Calls every 1ms
  Timer2.attachInterrupt(sampling).start(SAMPLING_TIME_MS);
  
  Serial.begin(SERIAL_BPS);
  Serial.println("Init finished");
}

void loop() {
  // Nothing
  if (iterations == (SAMPLES+1) && p < TIMES) {
     writeDataInSerial();
     iterations = 0;
     p++;
     encoderPosition = 0;
     lastState = getCurrentPinState();
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

void sampling() {
  if (iterations <= SAMPLES) {
    data[iterations] = encoderPosition;
    iterations++;
  }
}

void controller() {
    // Get position
    double positionRadian = encoderPosition;
    
    // Error signal. 
    double error = VALUE_TO_MOVE_TO - positionRadian;
    
    // Señal de Control
    double u = error * KP;
    
    // Saturacion
    if(u > 12) {
      u = 12;
    } else if (u < -12) {
      u = -12;
    }

    setSpeed(u);
}


/**
 * Pone un valor en el PWM entre 0 y 255, 0 para parado, 255 para maximo. (Ya que la resolucion es de 8 bits)
 * percentage tiene que venir entre 0 y 12V
 */
void setPWMValue(int const pin, double const percentage) {
  int channel = g_APinDescription[pin].ulPWMChannel;

  double value = (uint16_t)((double)percentage/(double)VMAX)*PWM_MAX_DUTY;
  
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
 * Interupcion asociada al pin A
 */
/*void interruptionPinA() {
  if (!firstPositionA) {
     if (digitalRead(ENCODER_PIN_A) == digitalRead(ENCODER_PIN_B))
      encoderPosition++;
     else
      encoderPosition--;
  }
  
  firstPositionA = false;
}*/

/**
 * Interupcion asociada al pin B
 */
/*void interruptionPinB() {
  if (!firstPositionB) {
     if (digitalRead(ENCODER_PIN_A) == digitalRead(ENCODER_PIN_B))
      encoderPosition--;
     else
      encoderPosition++;
  }
  
  firstPositionB = false;
}*/

int32_t getCurrentPinState() {
  bool pinA = digitalRead(ENCODER_PIN_A);
  bool pinB = digitalRead(ENCODER_PIN_B);

  if (!pinA && !pinB) return 0;
  if (!pinA && pinB) return 1;
  if (pinA && !pinB) return 2;
  if (pinA && pinB) return 3;
}

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

void writeDataInSerial() {
 for (int32_t i = 0; i <= SAMPLES; i++) {
   Serial.print(i);
   Serial.print(" ");
   Serial.println(data[i]);
 }
}

