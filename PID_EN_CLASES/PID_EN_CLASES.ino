// --- DEFINICIÓN DE PINES ---
const int ENA = 9;  
const int IN1 = 8;  
const int IN2 = 7;  

const int ENB = 10; 
const int IN3 = 12; 
const int IN4 = 13; 

const int S1_PIN = A0; 
const int S2_PIN = A1; 
const int S3_PIN = A2; 
const int S4_PIN = A3; 
const int S5_PIN = A4; 

// --- VARIABLES PID (Ajustadas para velocidad baja) ---
float Kp = 18.0; // Subimos Kp para que el giro sea más agresivo
float Ki = 0.0;  
float Kd = 8.0;  

int P, I, D, PID_value;
int error = 0;
int previous_error = 0;

// --- VELOCIDADES BAJAS ---
const int VEL_RECTA = 130; // Velocidad moderada en recta
const int VEL_CURVA = 100;  // Velocidad base en curva (No bajar de 50)

void setup() {
  Serial.begin(9600);
  
  pinMode(S1_PIN, INPUT);
  pinMode(S2_PIN, INPUT);
  pinMode(S3_PIN, INPUT);
  pinMode(S4_PIN, INPUT);
  pinMode(S5_PIN, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  leerSensores();
  calcularPID();
  controlarMotores();
}

void leerSensores() {
  int s1 = digitalRead(S1_PIN) == LOW ? 1 : 0;
  int s2 = digitalRead(S2_PIN) == LOW ? 1 : 0;
  int s3 = digitalRead(S3_PIN) == LOW ? 1 : 0;
  int s4 = digitalRead(S4_PIN) == LOW ? 1 : 0;
  int s5 = digitalRead(S5_PIN) == LOW ? 1 : 0;

  if (s1 == 1 && s5 == 1) {
    error = 0; 
    return;
  }

  // Ponderación de errores
  if      (s1==1 && s2==0 && s3==0 && s4==0 && s5==0) error = -4; 
  else if (s1==1 && s2==1 && s3==0 && s4==0 && s5==0) error = -3;
  else if (s1==0 && s2==1 && s3==0 && s4==0 && s5==0) error = -2;
  else if (s1==0 && s2==1 && s3==1 && s4==0 && s5==0) error = -1;
  else if (s1==0 && s2==0 && s3==1 && s4==0 && s5==0) error = 0;  
  else if (s1==0 && s2==0 && s3==1 && s4==1 && s5==0) error = 1;
  else if (s1==0 && s2==0 && s3==0 && s4==1 && s5==0) error = 2;
  else if (s1==0 && s2==0 && s3==0 && s4==1 && s5==1) error = 3;
  else if (s1==0 && s2==0 && s3==0 && s4==0 && s5==1) error = 4;  
}

void calcularPID() {
  P = error;
  I = I + error;
  D = error - previous_error;
  
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  
  previous_error = error;
}

void controlarMotores() {
  int velocidadBase = VEL_CURVA;
  
  if (error == 0) {
    velocidadBase = VEL_RECTA;
  }

  int velocidadMotorIzq = velocidadBase + PID_value;
  int velocidadMotorDer = velocidadBase - PID_value;

  // --- LÓGICA DE GIRO PIVOTE PARA VELOCIDAD BAJA ---
  // Si el valor cae por debajo de 0, lo forzamos a 0 eléctrico.
  // Esto hace que una rueda se detenga por completo y la otra empuje con fuerza.
  if (velocidadMotorIzq < 0) velocidadMotorIzq = 0;
  if (velocidadMotorDer < 0) velocidadMotorDer = 0;

  // Aseguramos máximo PWM de 255
  if (velocidadMotorIzq > 255) velocidadMotorIzq = 255;
  if (velocidadMotorDer > 255) velocidadMotorDer = 255;

  // Ambos motores siempre van hacia adelante, el giro lo da la diferencia de PWM
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Aplicar potencias corregidas
  analogWrite(ENA, velocidadMotorIzq);
  analogWrite(ENB, velocidadMotorDer);
}
