#define PI 3.141592

#define ENCODER_PIN_A 3
#define ENCODER_PIN_B 7


#define MOTOR_PWM_FORWARD 8
#define MOTOR_PWM_BACKWARD 9
#define BRIDGE_ENABLE 10
#define COUNTS_PER_REVOLUTION 3591.84

// 40Khz para no molestar a Alvaro
#define PWM_FREQ 40000
// 8 bits de resolucion del PWM. Suficiente para el motor tan malo que tenemos. La diferencia entre 1V casi es imperceptible, aumentar la resolucion no sirve de nada.
#define PWM_MAX_DUTY 256
#define VMAX 9


#define MAX_ITERATIONS 50
// Cogemos datos cada segundo
#define SAMPLING_TIME_MS 1000

int iterations = 0;
volatile int encoderPosition = 0;

void pwmConf();
void initEncoder();
void setPWMValue(int pin, double percentage);
void interruptionPinA();
void interruptionPinB();
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
  
  Serial.begin(112500);
  Serial.println("Init finished");
}

void loop() {
  if (iterations < 50) {
    writeDataInSerial();
    delay(SAMPLING_TIME_MS);
    // Testing
    static int speed = 0;
    setSpeed(speed);
    speed++;
    if (speed > 9)
      speed = -9;    
  }
  
  iterations++;
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
    PWMC_ConfigureClocks(freq * PWM_MAX_DUTY, 0, VARIANT_MCK);
    PWMC_ConfigureChannel(PWM_INTERFACE, channel, PWM_CMR_CPRE_CLKA, 0, 0);
    PWMC_SetPeriod(PWM_INTERFACE, channel, PWM_MAX_DUTY); 
    PWMC_EnableChannel(PWM_INTERFACE, channel);
}

/**
 * Pone un valor en el PWM entre 0 y 255, 0 para parado, 255 para maximo. (Ya que la resolucion es de 8 bits)
 * percentage tiene que venir entre 0 1 y 1
 */
void setPWMValue(int const pin, double const percentage) {
  int channel = g_APinDescription[pin].ulPWMChannel;

  int value = percentage*PWM_MAX_DUTY;
  
  PWMC_SetDutyCycle(PWM_INTERFACE, channel, value);
}

/**
 * Speed puede venir desde -9V a 9V
 */
void setSpeed(double speed) {
  double pwmToWrite = 0;
  if (speed >= 0) {
    pwmToWrite = (double)speed/(double)VMAX;
    setPWMValue(MOTOR_PWM_FORWARD, pwmToWrite);
    setPWMValue(MOTOR_PWM_BACKWARD, 0);
  } else {
    speed = -speed;
    pwmToWrite = (double)speed/(double)VMAX;
    setPWMValue(MOTOR_PWM_FORWARD, 0);
    setPWMValue(MOTOR_PWM_BACKWARD, pwmToWrite);
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

void writeDataInSerial() {
  // Sacar cosas por pantalla
 Serial.print("Iteration ");
 Serial.print(iterations);
 Serial.print(" Encoder pos ");
 Serial.print(encoderPosition);
 Serial.print(" en radianes ");
 Serial.println(2*PI*encoderPosition/COUNTS_PER_REVOLUTION);
}

