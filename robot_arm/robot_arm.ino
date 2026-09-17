#include <Servo.h>

// تعريف محركات السيرفو للذراع
Servo servo_M1;  // القاعدة (Theta 1) -> Pin 3
Servo servo_M2;  // الكتف (Theta 2) -> Pin 4
Servo servo_M3;  // الكوع (Theta 3) -> Pin 5
Servo servo_M4;  // المعصم (Theta 4) -> Pin 6

// تعريف بن الليزر
const int LASER_PIN = 2;  // الليزر -> Pin 2

// --- متغيرات نظام الحماية Watchdog ---
unsigned long last_data_time = 0;
const unsigned long TIMEOUT_LIMIT = 2000;  // ثانيتين

// --- إعدادات بطء حركة السيرفو ---
const unsigned long SERVO_STEP_INTERVAL = 80;
const int SERVO_STEP_SIZE = 1;  // درجة واحدة كل خطوة
unsigned long last_servo_step_time = 0;

// --- إعدادات تصحيح الخطأ البصري (Visual Servoing) ---
// الراسبيري باي بيبعت قيمتين بس: error_x, error_y (فرق البكسل بين مركز الهدف ومنتصف الكاميرا)
// مش 4 زوايا مفاصل، فالذراع بيتحرك في اتجاهين بس: يمين/شمال (Pan) وفوق/تحت (Tilt)
const int PIXEL_DEADBAND = 15;           // منطقة أمان بالبكسل عشان نمنع اهتزاز السيرفو (زي ما مذكور في تقرير المشروع)
const float ERROR_TO_DEGREE_DIVISOR = 4.0;  // معامل تحويل بكسل->درجة، يحتاج ضبط عملي على الهاردوير الحقيقي

// الزوايا الحالية الفعلية للموتورات
int current_M1 = 90;
int current_M2 = 90;
int current_M3 = 90;
int current_M4 = 90;

// الزوايا المطلوبة
int target_M1 = 90;
int target_M2 = 90;
int target_M3 = 90;
int target_M4 = 90;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  servo_M1.attach(3);
  servo_M2.attach(4);
  servo_M3.attach(5);
  servo_M4.attach(6);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, HIGH);

  // وضعية البداية الآمنة
  servo_M1.write(current_M1);
  servo_M2.write(current_M2);
  servo_M3.write(current_M3);
  servo_M4.write(current_M4);

  last_data_time = millis();
  last_servo_step_time = millis();
}

void loop() {
  // استقبال بيانات الراسبيري
  if (Serial.available() > 0) {
    String data_stream = Serial.readStringUntil('\n');
    data_stream.trim();

    if (data_stream.startsWith("STOP")) {
      // الهدف اتفقد: نطفي الليزر بس، ونسيب الذراع في مكانها الحالي (من غير رجوع مفاجئ للنص)
      digitalWrite(LASER_PIN, LOW);
      last_data_time = millis();
    } else {
      // error_x: فرق البكسل الأفقي (يمين/شمال) -- error_y: فرق البكسل الرأسي (فوق/تحت)
      float error_x = parser_float(data_stream, ',', 0);
      float error_y = parser_float(data_stream, ',', 1);

      last_data_time = millis();
      digitalWrite(LASER_PIN, HIGH);

      // Pan (يمين/شمال) عن طريق سيرفو القاعدة M1
      if (abs(error_x) > PIXEL_DEADBAND) {
        target_M1 = constrain(target_M1 - (int)(error_x / ERROR_TO_DEGREE_DIVISOR), 0, 180);
      }
      // Tilt (فوق/تحت) عن طريق سيرفو الكتف M2
      if (abs(error_y) > PIXEL_DEADBAND) {
        target_M2 = constrain(target_M2 + (int)(error_y / ERROR_TO_DEGREE_DIVISOR), 0, 180);
      }
      // الكوع والمعصم (M3, M4) مفيش ليهم إحداثيات مبعوتة من الراسبيري، فبيفضلوا ثابتين على وضع محايد
    }
  }

  // نظام الحماية لو البيانات وقفت
  if (millis() - last_data_time > TIMEOUT_LIMIT) {
    target_M1 = current_M1;  // فضل في مكانك، متطلعش فجأة للنص
    target_M2 = current_M2;
    target_M3 = 90;
    target_M4 = 97;
    digitalWrite(LASER_PIN, LOW);
  }

  // تحريك السيرفو تدريجياً بدون delay
  move_servos_slowly();
}

void move_servos_slowly() {
  if (millis() - last_servo_step_time >= SERVO_STEP_INTERVAL) {
    last_servo_step_time = millis();

    current_M1 = move_towards(current_M1, target_M1, SERVO_STEP_SIZE);
    current_M2 = move_towards(current_M2, target_M2, SERVO_STEP_SIZE);
    current_M3 = move_towards(current_M3, target_M3, SERVO_STEP_SIZE);
    current_M4 = move_towards(current_M4, target_M4, SERVO_STEP_SIZE);

    servo_M1.write(current_M1);
    servo_M2.write(current_M2);
    servo_M3.write(current_M3);
    servo_M4.write(current_M4);
  }
}

int move_towards(int current_pos, int target_pos, int step_size) {
  if (current_pos < target_pos) {
    current_pos += step_size;
    if (current_pos > target_pos) {
      current_pos = target_pos;
    }
  } else if (current_pos > target_pos) {
    current_pos -= step_size;
    if (current_pos < target_pos) {
      current_pos = target_pos;
    }
  }
  return current_pos;
}

float parser_float(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]).toFloat() : 0.00;
}
