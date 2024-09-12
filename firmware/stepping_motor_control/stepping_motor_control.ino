# define pin numbers
#define PIN_A 10   
#define PIN_NA 11  
#define PIN_B 12   
#define PIN_NB 13  

// define register addresses
#define UDR0 (*(volatile uint8_t *)(0xC6))   // USART Data Register
#define UCSRA (*(volatile uint8_t *)(0xC0))  // USART Control and Status Register A
#define RXC0 7                               // USART Receive Complete bit position

void init_UART(unsigned int baud) {
    unsigned int ubrr = F_CPU/16/baud - 1;  // UBRR 계산 (F_CPU는 CPU 주파수)

    // Baud rate 설정
    UBRR0H = (unsigned char)(ubrr >> 8);   // 상위 8비트 설정
    UBRR0L = (unsigned char)ubrr;          // 하위 8비트 설정

    // 송신 및 수신 활성화, 8비트 데이터 크기 설정
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);  // RXEN0: 수신 활성화, TXEN0: 송신 활성화, RXCIE0: 수신 인터럽트 활성화
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // UCSZ01, UCSZ00: 8비트 데이터 프레임 설정
}

volatile int direction = 1; // 1 : clockwise, -1 : counter-clockwise
volatile bool isDirectionChanged = false;
float target_velcocity = 1;

// UART 수신 인터럽트가 발생할 때 호출되는 ISD
ISR(USART_RX_vect) {
    uint8_t receivedByte = UDR0;
    if ((receivedByte >= '0') && (receivedByte <= '9')) {
        target_velcocity = (receivedByte - '0') * 0.4f;
    } else if (receivedByte == ' ') { // 스페이스바 입력
        isDirectionChanged = true; // 방향 전환 플래그 설정
    }
}

// voltage pattern to be applied to each coil of a stepper motor - A, B, A_, B_
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

// apply voltage to each coils A, B, A_, B_
void moveOneStep() {
  static int step = 0;
  digitalWrite(PIN_A, step_info[step][0]);
  digitalWrite(PIN_B, step_info[step][1]);
  digitalWrite(PIN_NA, step_info[step][2]);
  digitalWrite(PIN_NB, step_info[step][3]);
  step += direction;
  if (step > 7) step = 0;
  if (step < 0) step = 7;
}

void setup() {
  init_UART(9600); 

  // set stepper motor pins as outputs
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_NA, OUTPUT);
  pinMode(PIN_NB, OUTPUT);

  // disable interrupts
  noInterrupts();

  // set timer (Timer1)
  TCCR1A = 0;  // initialize TCCR1A register
  TCCR1B = 0;  // initialize TCCR1B register
  TCNT1 = 0;   // set timer 1 count to 0
  // TCCR1B |= (1 << CS11) | (1 << CS10); // 64분주 mode
  TCCR1B = (1 << CS11); // 8 분주

  // enable interrupts
  interrupts();

  // core logic
  float current_velocity = 1;
  uint16_t last_step_count = 0;
  uint16_t last_control_count = 0;
  uint16_t step_interval_counts = 314;
  const uint16_t MOTOR_CONTROL_COUNTS = 200;

  while (true) {
    // clock count
    uint16_t current_count = (uint16_t)TCNT1;

    // step motor
    if ((current_count - last_step_count) > step_interval_counts) {
      moveOneStep();
      last_step_count = current_count;
    }

    // set d
    if ((current_count - last_control_count) > MOTOR_CONTROL_COUNTS) {
      if (current_velocity < target_velcocity) {
        current_velocity += 0.0005f;
      } else if (current_velocity > target_velcocity) {
        current_velocity -= 0.0005f;
      }

      step_interval_counts = (uint16_t)(314.f / current_velocity);
      
      last_control_count = current_count;
    }

    // set direction
    if (isDirectionChanged) {
      noInterrupts();
      delay(50);  // 잠시 멈추고
      direction = (direction == 1) ? -1 : 1;
      isDirectionChanged = false;
      interrupts();
    }
  }
}

void loop() {}
