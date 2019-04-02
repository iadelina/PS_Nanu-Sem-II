void setup() {
//  pinMode(3, OUTPUT);
//  pinMode(11, OUTPUT);
//  TCCR1B |= (1 << WGM11) | (1 << WGM10);
     
//  TCCR1B |= (1 << CS11) | (1 << CS10);
//  TIMSK1 |= (1 << OCIE1A);
//  OCR1A = 128;
    
//  sei();
 pinMode(4, OUTPUT);
  pinMode (2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode (7, OUTPUT);
  pinMode (8, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  TCCR2A = (1<<COM2A1) | (1<<COM2B1) | (1<<WGM21) | (1<<WGM20);
  TCCR2B = (1<<CS22);
  OCR2A = 0; //11
  OCR2B = 0; //3
  TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM12) | (1<<WGM10);
  TCCR1B = (1<<CS11) | (1<<CS10);
  OCR1A = 180; //9 sau 10
  OCR1B = 180; //9 sau 10

}
ISR(TIMER1_COMPA_vect){
  
}


void loop() {
  digitalWrite(7,HIGH);
  digitalWrite(8,LOW);
   digitalWrite(4,HIGH);
  digitalWrite(2,LOW);
}



