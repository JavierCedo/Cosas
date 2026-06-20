// --- DEFINICIÓN DE PINES ---
// Motores (Puente H)
const int ENA = 9;  // PWM Motor Izquierdo
const int IN1 = 8;  // Dirección Izquierda 1
const int IN2 = 7;  // Dirección Izquierda 2

const int ENB = 10; // PWM Motor Derecho
const int IN3 = 12; // Dirección Derecha 1
const int IN4 = 13; // Dirección Derecha 2

// Sensores TCRT5000 (Usamos A0-A4 como entradas digitales para evitar conflictos)
const int S1_PIN = A0; // Extremo Izquierdo
const int S2_PIN = A1; // Centro Izquierdo
const int S3_PIN = A2; // Centro
const int S4_PIN = A3; // Centro Derecho
const int S5_PIN = A4; // Extremo Derecho

// --- VARIABLES PID ---
float Kp = 15.0; // Constante Proporcional (Ajustar)
float Ki = 0.0;  // Constante Integral (Usualmente 0 en seguidores de línea)
float Kd = 10.0; // Constante Derivativa (Ajustar)

int P, I, D, PID_value;
int error = 0;
int previous_error = 0;

// --- VELOCIDADES ---
const int VEL_RECTA = 200; // Velocidad en rectas (máx 255)
const int VEL_CURVA = 120; // Velocidad base en curvas

void setup() {
  Serial.begin(9600);
  
  // Configurar pines de sensores
  pinMode(S1_PIN, INPUT);
  pinMode(S2_PIN, INPUT);
  pinMode(S3_PIN, INPUT);
  pinMode(S4_PIN, INPUT);
  pinMode(S5_PIN, INPUT);

  // Configurar pines de motores
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
  // Asumimos que LOW es línea (NEGRO) y HIGH es fondo (BLANCO). 
  // Si tu sensor funciona al revés, cambia los LOW por HIGH aquí.
  int s1 = digitalRead(S1_PIN) == LOW ? 1 : 0;
  int s2 = digitalRead(S2_PIN) == LOW ? 1 : 0;
  int s3 = digitalRead(S3_PIN) == LOW ? 1 : 0;
  int s4 = digitalRead(S4_PIN) == LOW ? 1 : 0;
  int s5 = digitalRead(S5_PIN) == LOW ? 1 : 0;

  // Detección de intersección (T o Cruz)
  if (s1 == 1 && s5 == 1) {
    // Si los extremos leen negro, es una intersección.
    // Estrategia: Seguir recto por un momento corto para cruzarla.
    error = 0; 
    return;
  }

  // Cálculo de Error ponderado
  if      (s1==1 && s2==0 && s3==0 && s4==0 && s5==0) error = -4; // Muy a la derecha
  else if (s1==1 && s2==1 && s3==0 && s4==0 && s5==0) error = -3;
  else if (s1==0 && s2==1 && s3==0 && s4==0 && s5==0) error = -2;
  else if (s1==0 && s2==1 && s3==1 && s4==0 && s5==0) error = -1;
  else if (s1==0 && s2==0 && s3==1 && s4==0 && s5==0) error = 0;  // Centrado perfecto
  else if (s1==0 && s2==0 && s3==1 && s4==1 && s5==0) error = 1;
  else if (s1==0 && s2==0 && s3==0 && s4==1 && s5==0) error = 2;
  else if (s1==0 && s2==0 && s3==0 && s4==1 && s5==1) error = 3;
  else if (s1==0 && s2==0 && s3==0 && s4==0 && s5==1) error = 4;  // Muy a la izquierda
  // Si pierde la línea (todos en 0), mantiene el error anterior para seguir girando
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
  
  // Velocidad adaptativa: Si está centrado, acelera en la recta
  if (error == 0) {
    velocidadBase = VEL_RECTA;
  }

  // Calculamos la velocidad final de cada rueda
  int velocidadMotorIzq = velocidadBase + PID_value;
  int velocidadMotorDer = velocidadBase - PID_value;

  // Limitamos las velocidades entre 0 y 255 (PWM max)
  velocidadMotorIzq = constrain(velocidadMotorIzq, 0, 255);
  velocidadMotorDer = constrain(velocidadMotorDer, 0, 255);

  // Mover motores hacia adelante
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Aplicar potencia
  analogWrite(ENA, velocidadMotorIzq);
  analogWrite(ENB, velocidadMotorDer);
}