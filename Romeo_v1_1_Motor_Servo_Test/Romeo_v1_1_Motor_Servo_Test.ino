//Standard PWM DC control
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 4;    //M1 Direction Control
int M2 = 7;    //M1 Direction Control
 
void stop (void) {
  digitalWrite(E1, LOW);  
  digitalWrite(E2, LOW);     
}

void forward (char a, char b) {
  analogWrite (E1, a);      //PWM Speed Control
  digitalWrite(M1, HIGH);   
  analogWrite (E2, b);   
  digitalWrite(M2, HIGH);
} 

void reverse (char a, char b) {
  analogWrite (E1, a);
  digitalWrite(M1, LOW);  
  analogWrite (E2, b);   
  digitalWrite(M2, LOW);
}

void turnLeft (char a, char b) {
  analogWrite (E1, a);
  digitalWrite(M1, LOW);   
  analogWrite (E2, b);   
  digitalWrite(M2, HIGH);
}

void turnRight (char a, char b) {
  analogWrite (E1, a);
  digitalWrite(M1, HIGH);   
  analogWrite (E2, b);   
  digitalWrite(M2, LOW);
}

void setup (void) {
  int i;

  //  Set all the motor control pins to OUTPUT
  for(i = M1; i <= M2; i++) {
    pinMode(i, OUTPUT); 
  }

  Serial.begin(9600);
  Serial.println(F("Run keyboard control.."));
}

void loop(void) {
  char val;

  if (Serial.available()) {
    val = Serial.read();

    if (val != -1) {
      switch(val) {
        case 'w':
        case 'W':
          forward(100, 100);
          break;

        case 's':
        case 'S':
          reverse(100, 100);
          break;

        case 'a':
        case 'A':
          turnLeft(100, 100);
          break;      

        case 'd':
        case 'D':
          turnRight(100, 100);
          break;

        case 'z':
        case 'Z':
          Serial.println("Hello");
          break;

        case 'x':
        case 'X':
          stop();
          break;

        default:
          Serial.println(F("Invalid command received!"));
          break;
      }
    }
      else stop(); 
  }
}
