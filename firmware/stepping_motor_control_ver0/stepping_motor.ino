#define PIN_A 10   // 스텝 모터의 A 코일 연결 핀
#define PIN_NA 11  // 스텝 모터의 B 코일 연결 핀
#define PIN_B 12   // 스텝 모터의 A' 코일 연결 핀
#define PIN_NB 13  // 스텝 모터의 B' 코일 연결 핀

volatile int stepNumber = 0;  // 현재 스텝 번호

// a, b, a_, b_
int step_info[8][4] = {
  { HIGH, HIGH, LOW, LOW },
  { LOW, HIGH, HIGH, LOW },
  { LOW, LOW, HIGH, HIGH },
  { HIGH, LOW, LOW, HIGH },
};

void doStep() {
  static int step = 0;
  digitalWrite(PIN_A, step_info[step][0]);
  digitalWrite(PIN_B, step_info[step][1]);
  digitalWrite(PIN_NA, step_info[step][2]);
  digitalWrite(PIN_NB, step_info[step][3]);
  step++;
  if (step > 3) step=0;
}

void setup() {
  // 스텝 모터 핀을 출력으로 설정
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_NA, OUTPUT);
  pinMode(PIN_NB, OUTPUT);

  noInterrupts();  // 인터럽트 비활성화

  // 타이머 설정 (예: Timer1 사용)
  TCCR1A = 0;  // TCCR1A 레지스터 초기화
  TCCR1B = 0;  // TCCR1B 레지스터 초기화

  // 타이머 비교 매치 레지스터 설정 (16MHz 클럭, 분주비 64)
  OCR1A = 300;

  // CTC 모드 설정 (Clear Timer on Compare Match)
  TCCR1B |= (1 << WGM12);

  // 분주비 64 설정
  TCCR1B |= (1 << CS11) | (1 << CS10);

  // 타이머 비교 매치 인터럽트 활성화
  TIMSK1 |= (1 << OCIE1A);

  interrupts();  // 인터럽트 활성화
}

ISR(TIMER1_COMPA_vect) {
  doStep();
}


void loop() {

}
