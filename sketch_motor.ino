#define ENCODER_PIN_A 3
#define ENCODER_PIN_B 7

#define MOTOR_PWM_FORWARD 6
#define MOTOR_PWM_BACKWARD 8
#define BRIDGE_ENABLE 2
#define COUNTS_PER_REVOLUTION 3591.84

// 30Khz para no molestar a Alvaro
#define PWM_FREQ 20000
// 8 bits de resolucion del PWM.
#define PWM_MAX_DUTY 255
#define VMAX 12


#define MAX_ITERATIONS 50
// Cogemos datos cada segundo
#define SAMPLING_TIME_MS 1000

#define SERIAL_BPS 112500

int iterations = 0;
volatile int encoderPosition = 0;

void pwmConf();
void initEncoder();
void setPWMValue(int pin, double percentage);
void interruptionPinA();
void interruptionPinB();
void interruptionTime();
void writeDataInSerial();
void setSpeed(double speed);

void setup() {
  initEncoder();
  initBridge();
  
  pwmConf(MOTOR_PWM_FORWARD, PWM_FREQ);
  pwmConf(MOTOR_PWM_BACKWARD, PWM_FREQ);
  
  // setPWMValue(MOTOR_PWM_FORWARD, 0.5);
  
  // INTERRUPCIONES DEL ENCODER: en los flancos de A y  en los flancos de B
  attachInterrupt(ENCODER_PIN_A, interruptionPinA, CHANGE); 
  attachInterrupt(ENCODER_PIN_B, interruptionPinB, CHANGE);

  Serial.begin(SERIAL_BPS);
  Serial.println("Init finished");
}

void loop() {
  interruptionTimer();
  delay(SAMPLING_TIME_MS);
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
    PWMC_ConfigureClocks(freq * PWM_MAX_DUTY, 0, VARIANT_MCK);
    
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
  
  Serial.print("Value: ");
  Serial.println(value);
  
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
void interruptionPinA() {
  if (digitalRead(ENCODER_PIN_A) == digitalRead(ENCODER_PIN_B))
    encoderPosition--;
  else
    encoderPosition++;
}

/**
 * Interupcion asociada al pin B
 */
void interruptionPinB() {
  if (digitalRead(ENCODER_PIN_A) == digitalRead(ENCODER_PIN_B))
    encoderPosition++;
  else
    encoderPosition--;
}

void interruptionTimer() {
  writeDataInSerial();
  static int speed = 0;
  setSpeed(speed);
  speed++;

  if (speed == 12)
    speed = -12;
}

void writeDataInSerial() {
  // Sacar cosas por pantalla
 Serial.print("Iteration ");
 Serial.print(iterations);
 Serial.print(" Encoder pos ");
 Serial.print(encoderPosition);
 Serial.print(" en radianes ");
 Serial.println(2*PI*encoderPosition/COUNTS_PER_REVOLUTION);
}

