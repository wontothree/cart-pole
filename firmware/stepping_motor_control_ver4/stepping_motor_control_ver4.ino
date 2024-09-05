#define PIN_A 10   
#define PIN_NA 11  
#define PIN_B 12   
#define PIN_NB 13  

// Register address definitions (for Arduino Uno)
#define UDR0 (*(volatile uint8_t *)(0xC6))   // USART Data Register
#define UCSRA (*(volatile uint8_t *)(0xC0))  // USART Control and Status Register A
#define RXC0 7                               // USART Receive Complete bit position

volatile int direction = 1;
volatile bool directionChanged = false;
float vel_target = 1;

// A, B, A_, B_
int step_info[8][4] = {
  { HIGH, LOW, LOW, LOW },
  { HIGH, HIGH, LOW, LOW },
  { LOW, HIGH, LOW, LOW },
  { LOW, HIGH, HIGH, LOW },
  { LOW, LOW, HIGH, LOW },
  { LOW, LOW, HIGH, HIGH },
  { LOW, LOW, LOW, HIGH },
  { HIGH, LOW, LOW, HIGH }
};

void doStep() {
  static int step = 0;
  digitalWrite(PIN_A, step_info[step][0]);
  digitalWrite(PIN_B, step_info[step][1]);
  digitalWrite(PIN_NA, step_info[step][2]);
  digitalWrite(PIN_NB, step_info[step][3]);
  step += direction;
  if (step > 7) step = 0;
  if (step < 0) step = 7;
}

void UART_init(unsigned int baud) {
    unsigned int ubrr = F_CPU/16/baud - 1;  // UBRR 계산 (F_CPU는 CPU 주파수)

    // Baud rate 설정
    UBRR0H = (unsigned char)(ubrr >> 8);   // 상위 8비트 설정
    UBRR0L = (unsigned char)ubrr;          // 하위 8비트 설정

    // 송신 및 수신 활성화, 8비트 데이터 크기 설정
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);  // RXEN0: 수신 활성화, TXEN0: 송신 활성화, RXCIE0: 수신 인터럽트 활성화
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // UCSZ01, UCSZ00: 8비트 데이터 프레임 설정
}

ISR(USART_RX_vect) {
    uint8_t receivedByte = UDR0;
    if ((receivedByte >= '0') && (receivedByte <= '9')) {
        vel_target = (receivedByte - '0') * 1.5f;
    } else if (receivedByte == ' ') { // 스페이스바 입력
        directionChanged = true; // 방향 전환 플래그 설정
    }
}

void setup() {
  UART_init(9600); 

  // Set stepper motor pins as outputs
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_NA, OUTPUT);
  pinMode(PIN_NB, OUTPUT);

  // Disable interrupts
  noInterrupts();

  // 타이머 설정 (예: Timer1 사용)
  TCCR1A = 0;  // TCCR1A 레지스터 초기화
  TCCR1B = 0;  // TCCR1B 레지스터 초기화
  TCNT1 = 0;   // 타이머 1 카운터를 0으로 설정
  TCCR1B |= (1 << CS11) | (1 << CS10);

  // enable interrupts
  interrupts();

  float vel_current = 0;
  uint16_t interval = 500;
  uint16_t time_last_step = 0;
  uint16_t time_last_controller_ctrl = 0;
  const uint16_t PERIOD_MOTOR_CTRL = 30; // ms, 모터 제어 주기 설정

  while (true) {
    uint16_t time_current = (uint16_t)TCNT1;

    if ((time_current - time_last_step) > interval) {
      doStep();
      time_last_step = time_current;
    }

    if ((time_current - time_last_controller_ctrl) > PERIOD_MOTOR_CTRL) {
      // 속도 조절
      if (vel_current < vel_target) {
        vel_current += 0.001f;
        // if (vel_current > vel_target) vel_current = vel_target;
      } else if (vel_current > vel_target) {
        vel_current -= 0.001f;
        // if (vel_current < vel_target) vel_current = vel_target;
      }

      interval = (uint16_t)(500.f / vel_current);
      
      time_last_controller_ctrl = time_current;
    }

    // 방향 전환 처리
    if (directionChanged) {
      noInterrupts();
      delay(50);  // 잠시 멈추고
      direction = (direction == 1) ? -1 : 1;
      directionChanged = false;
      interrupts();
    }
  }
}

void loop() {}
