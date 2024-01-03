#define AIN1 6  // 定義AIN1為數位腳位6，用於電機A的控制
#define AIN2 5  // 定義AIN2為數位腳位5，用於電機A的控制
#define BIN1 10 // 定義BIN1為數位腳位10，用於電機B的控制
#define BIN2 9  // 定義BIN2為數位腳位9，用於電機B的控制
#define PWMA 4  // 定義PWMA為數位腳位4，用於控制電機A的PWM（脈衝寬度調制）
#define PWMB 8  // 定義PWMB為數位腳位8，用於控制電機B的PWM
#define STBY 7  // 定義STBY為數位腳位7，用於電機驅動板的待機控制

int const trigPin= 12;
int const echoPin= 11;
int Duration;
int Distance;

// 初始化電機函式
void initMotor(){
  // 將電機控制腳位設定為輸出模式
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // 初始化電機狀態為停止
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  digitalWrite(STBY, HIGH);  // 啟動電機驅動板（非待機狀態）
  analogWrite(PWMA, 0);     // 設定電機A的速度為0
  analogWrite(PWMB, 0);     // 設定電機B的速度為0
}

// 設定特定電機的PWM值以控制速度和方向
void SetPWM(int motor, int pwm) {
  boolean inReverse = pwm < 0;  // 判斷是否為倒轉
  int speed = inReverse ? -pwm : pwm;  // 取絕對值作為速度

  // 根據指定的電機來設定控制腳位和速度
  if (motor == 1) {  // 控制電機A
    digitalWrite(AIN1, !inReverse);
    digitalWrite(AIN2, inReverse);
    analogWrite(PWMA, speed);
  } else if (motor == 2) {  // 控制電機B
    digitalWrite(BIN1, inReverse);
    digitalWrite(BIN2, !inReverse);
    analogWrite(PWMB, speed);
  }
}

// 以下函式用於控制電機的不同動作（前進、後退、左轉、右轉、停止）
void forward(int speed) {
  SetPWM(1, speed); // 電機A和B同時前進
  SetPWM(2, -(speed-5));
}

void back(int speed) {
  SetPWM(1, -speed); // 電機A和B同時後退
  SetPWM(2, -speed);
}

void left(int speed) {
  SetPWM(1, speed-17); // 電機A減速或後退，電機B正常速度前進，實現左轉
  SetPWM(2, -speed-10);//10);
}

void right(int speed) {
  SetPWM(1, speed+24);//18);//30);  // 電機A正常速度前進，電機B減速或後退，實現右轉
  SetPWM(2, -speed+8);//15);
}

void stopp() {
  SetPWM(1, 0); // 電機A和B停止
  SetPWM(2, 0);
}


void setup() {
  Serial.begin(57600); // 开启串行通信，波特率设置为57600
  initMotor(); // 初始化電機
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  digitalWrite(trigPin,LOW);
}
long duration, cm;
unsigned long previousMillis = 0;  // stores the last time the ultrasonic sensor was updated
const long interval = 300;        // interval at which to run the ultrasonic sensor (milliseconds)

void loop() {
  unsigned long currentMillis = millis();

  // Check if three seconds have passed; if so, run the ultrasonic sensor code
  if (currentMillis - previousMillis >= interval) {
    // save the last time the ultrasonic sensor was updated
    previousMillis = currentMillis;

    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);
    cm = (duration/2) / 29.1;
    if (cm < 34){
      Serial.print("c");
    }
  }

  // Rest of your code goes here (handling commands from the serial port)
  char command = Serial.read(); // 读取接收到的命令

  // Handling commands (G, L, R, H)
    if (command == 'G') {
      //Serial.print("go straight");
      forward(25);//50);
    }else if (command == 'L'){
      //Serial.print("go left");
      left(25);//38);
      //delay(100);
      //stopp();
      //delay(100);
      //left(60);
    }else if (command == 'R'){
      //Serial.print("go right");
      right(25);//38);
      //delay(100);
      //stopp();
      //delay(100);
      //right(60);
    }else if (command == 'H'){
      //Serial.print("stop");
      stopp();
    }
}
