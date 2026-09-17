// --------- Motor Pins (PWM) ---------
#define M3A 2
#define M3B 3
#define M4A 44
#define M4B 45
#define M1A 11
#define M1B 12
#define M2A 4
#define M2B 5

// --------- Sensors, Buzzer & LED ---------
#define MAG_SENSOR_PIN 40    // حساس المغناطيس
#define BUZZER_PIN 48        // البازر
#define SMOKE_SENSOR_PIN 49  // حساس الغاز
#define LED_PIN 50           // الليد الخاص بالغاز على بنة 50

char bt;
int carSpeed = 80;

void setup() {
  Serial.begin(9600);

  // إعداد دبابيس المواتير
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  pinMode(M3A, OUTPUT);
  pinMode(M3B, OUTPUT);
  pinMode(M4A, OUTPUT);
  pinMode(M4B, OUTPUT);

  // إعداد الحساسات والبازر والليد
  pinMode(MAG_SENSOR_PIN, INPUT_PULLUP);
  pinMode(SMOKE_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // البداية أمان: قفل البازر وتعديل إشارة الليد المبدئية عشان يطفي
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, HIGH);

  stopMotor();
}

void loop() {
  // قراءة الحساسات
  int smokeState = digitalRead(SMOKE_SENSOR_PIN);
  int magState = digitalRead(MAG_SENSOR_PIN);

  // --------------------------------------------------
  // 1. التحكم في البازر (يزمر لو في غاز أو مغناطيس)
  // --------------------------------------------------
  if (smokeState == LOW || magState == LOW) {
    digitalWrite(BUZZER_PIN, HIGH);  // شغل البازر
  } else {
    digitalWrite(BUZZER_PIN, LOW);   // افصل البازر
  }

  // --------------------------------------------------
  // 2. التحكم
  // --------------------------------------------------
  if (smokeState == LOW) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }

  // --------------------------------------------------
  // 3. استقبال أوامر البلوتوث (التحكم في الحركة)
  // --------------------------------------------------
  if (Serial.available()) {
    bt = Serial.read();
    if (bt == 'F') forward();
    else if (bt == 'B') backward();
    else if (bt == 'L') left();
    else if (bt == 'R') right();
    else if (bt == 'A') rotateLeft();
    else if (bt == 'D') rotateRight();
    else if (bt == 'S') stopMotor();
  }
}

// --------- Mecanum Movements ---------
void forward() {
  motor(M1A, M1B, 1);
  motor(M2A, M2B, 1);
  motor(M3A, M3B, 1);
  motor(M4A, M4B, 1);
}

void backward() {
  motor(M1A, M1B, 0);
  motor(M2A, M2B, 0);
  motor(M3A, M3B, 0);
  motor(M4A, M4B, 0);
}

void left() {
  motor(M1A, M1B, 0);
  motor(M2A, M2B, 1);
  motor(M3A, M3B, 1);
  motor(M4A, M4B, 0);
}

void right() {
  motor(M1A, M1B, 1);
  motor(M2A, M2B, 0);
  motor(M3A, M3B, 0);
  motor(M4A, M4B, 1);
}

void rotateLeft() {
  motor(M1A, M1B, 0);
  motor(M2A, M2B, 1);
  motor(M3A, M3B, 0);
  motor(M4A, M4B, 1);
}

void rotateRight() {
  motor(M1A, M1B, 1);
  motor(M2A, M2B, 0);
  motor(M3A, M3B, 1);
  motor(M4A, M4B, 0);
}

void stopMotor() {
  analogWrite(M1A, 0);
  analogWrite(M1B, 0);
  analogWrite(M2A, 0);
  analogWrite(M2B, 0);
  analogWrite(M3A, 0);
  analogWrite(M3B, 0);
  analogWrite(M4A, 0);
  analogWrite(M4B, 0);
}

void motor(int pinA, int pinB, bool dir) {
  if (dir) {
    analogWrite(pinA, carSpeed);
    analogWrite(pinB, 0);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, carSpeed);
  }
}
