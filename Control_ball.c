#include<PID_v1.h>                                                     //biblioteca do PID
#include<Servo.h>                                                      //biblioteca do servo
#include <Ultrasonic.h>                                                //biblioteca do sensor ultrassom
#define N 6                                                            //numero do vetor media movel sensor 
#define M 5                                                            //numero do vetor media movel motor

  const int servoPin = 11;
  const int pingPin = 9; //saida(trig pin)
  const int pingPin2 = 8; //entrada (echo  pin)
  Ultrasonic ultrasonic(9,8);                                            //sensor ultrasom, trig(9) e o echo(8) respectivamente
  Servo myServo;       //Inicializa o servo
  unsigned long now;
  long duration, cm;

  float Kp = 2.5;                                                        //Ganho Proporcional Inicial
  float Ki = 0;                                                          //Ganho Integral Inicial
  float Kd = 1.1;                                                        //Ganho Derivativo Inicial 
  double Filtrado, Filtrado1, Setpoint, Input, Input1, Output, ServoOutput;

  PID myPID(&Filtrado, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);             //Inicializa o objeto PID, que está na classe PID.
  
  int Valores[N], Valores1[M];
  
void setup() {
  
  Serial.begin(9600);                                                  //liga a porta serial 
  myServo.attach(servoPin);                                            //aonde o servo está conectado
  Input = readPosition();                                              //chama a função readPosition() e seta o objeto/posição de entrada do algorítmo PID                                                                     
  myPID.SetMode(AUTOMATIC);                                            //seta PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(-75,75);                                       //seta os limites do Output de -80 a 80 graus 
}

void loop(){
  
  Input = readPosition();
  for(int i = N - 1; i > 0; i-- ){
    Valores[i] = Valores[i-1];    
  }
  Valores[0] = Input;
 
  long soma = 0;
  
  for(int i = 0; i < N; i++){
    soma = soma + Valores[i];  
  }
  Filtrado = soma/N;
   
  if(Filtrado >= 13 && Filtrado <= 16){
    Setpoint = Filtrado;
  }
  else
    Setpoint = 15;
                                             
  myPID.Compute();                                                    //computa o Output entre -80 to 80 graus  
    
  Input1 = Output;
  for(int j = M - 1; j > 0; j-- ){
    Valores1[j] = Valores1[j-1];    
  }
  Valores1[0] = Input1;
  long soma1 = 0;
  for(int j = 0; j < M; j++){
    soma1 = soma1 + Valores1[j];  
  }
  Filtrado1 = soma1/M;
  ServoOutput = 80 + Filtrado1;
  myServo.write(ServoOutput);                                           //manda o valor para o servo
}
      
float readPosition(){
  
  delay(40);
  now = millis();
  
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin2, INPUT);
  
  duration = pulseIn(pingPin2, HIGH);
  cm = duration/(29*2);
  
  
  if(cm > 30)     // maxima posição da bola
  cm=30;
  Serial.print("Cm: ");
  Serial.println(cm);
  Serial.print("Filtrado: ");
  Serial.println(Filtrado);
  Serial.print("Angulo: ");
  Serial.println(ServoOutput);
  return (float)cm;                                          // retorna distancia em cm
}
