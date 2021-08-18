//70 pulsos = 45cm
//angulo negativo - direita
//angulo positivo - esquerda

/*
 *  VARIÁVEIS PARA MAPEAMENTO FLOOD FILL
 */
//parede
int pd = -1;
//caminho livre
int cl = 10;
//destino
int fi = -3;
//inicio
int in = 11;
//labirinto
int mapa[7][7] = {{pd,pd,pd,pd,pd,pd,pd},
                  {pd,cl,cl,cl,cl,cl,pd},
                  {pd,cl,cl,cl,cl,cl,pd},
                  {pd,fi,cl,cl,cl,cl,pd},
                  {pd,cl,cl,cl,cl,cl,pd},
                  {pd,in,cl,cl,cl,cl,pd},
                  {pd,pd,pd,pd,pd,pd,pd},};


/*          
 *  VARIÁVEIS PARA O MOTOR
 */
//velocidade motor1
int           rpm;
//veloidade motor2
int           rpm2;
//leitura de pulsos do primeiro encoder
volatile long pulsos;
//leitura de pulsos do segundo encoder
volatile long pulsos2;
unsigned long timeold;
unsigned long timeold2;
//somador de pulsos par cm
int pulsosParaCm;
//definicao da quantidade de pulsos para dar uma volta
unsigned int pulsos_por_volta = 20;
float rad;
double propA;
double propB;
// Entrada dos motores
#define pinMot1A 6
#define pinMot2A 7
#define pinMot1B 8
#define pinMot2B 9

// Leitura HC-SR04
//Escolhendo as portas
const uint8_t trig_pin2OESTE = 52;
const uint8_t echo_pin2OESTE = 53;
const uint8_t trig_pin3LESTE = 50;
const uint8_t echo_pin3LESTE = 51;
uint32_t pulse_time2;
uint32_t pulse_time3;
double distanceOESTEUltra;
double distanceLESTEUltra;
/*
 * Variaveis temporarias 
 */

int validador;
float InicioanguloZ;

/*
 *  VARIÁVEIS PARA O SENSOR INFRAVERMELHO
 */
//Pinos do sensor infravermelho 
#define pinoSharp1 A0          
#define pinoSharp2 A1
#define pinoSharp3 A2
double valorVoltsSharp1;
double valorVoltsSharp2;
double valorVoltsSharp3;
double distanciaSharp1;
double distanciaSharp2;
double distanciaSharp3;
double distancia1;
double distancia2;
double distancia3;
double potencia1, potencia2;
double A = 118.67443195;
double t = 0.53449301;
double y0 = 8.76683547;
double exp1;

//VARIAVÉL PARA LOOP OPCIONAL
bool loop1 = true;

/*
 *  VARIAVEIS PARA O PID
 */
//variaveis do PID
float    diferenca,                            
           kp = 0.5,                              
           ki = 0.0005,                             
           kd = -19.0,                              
           proporcional,                          
           integral,                              
           derivativo,                            
           PID,                                  
           ideal_value = 0,                  
           ultimaMedida;

float    diferencaN,                            
           kpN = 1.0,                              
           kiN = 0.0001,                             
           kdN = 20.0,                              
           proporcionalN,                          
           integralN,                              
           derivativoN,                            
           PIDN,                                  
           ideal_valueN = 15,                    
           ultimaMedidaN;

float    diferencaL,                            
           kpL = 1.0,                              
           kiL = 0.0,                             
           kdL = -15.0,                              
           proporcionalL,                          
           integralL,                              
           derivativoL,                            
           PIDL,                                  
           ideal_valueL = 15.88,                    
           ultimaMedidaL;

float    diferencaLD,                            
           kpLD = 1.0,                              
           kiLD = 0.0001,                             
           kdLD = -10.0,                              
           proporcionalLD,                          
           integralLD,                              
           derivativoLD,                            
           PIDLD,                                  
           ideal_valueLD = 40,                    
           ultimaMedidaLD;

float    diferencaLE,                            
           kpLE = 1.0,                              
           kiLE = 0.0001,                             
           kdLE = -10.0,                              
           proporcionalLE,                          
           integralLE,                              
           derivativoLE,                            
           PIDLE,                                  
           ideal_valueLE = 40,                    
           ultimaMedidaLE;

float direita, esquerda;

float orientacao = 0;



//CLASSE MICROMOUSE
class Micromouse{
  public:
  //O - 4 /L - 2/ S - 3/ N - 1 
  int direcao;
  int posL = 5;
  int posC = 1;
  int distanciaPercorrida;

  void setPosL(int posL){
    this->posL = posL;
  }
  int getPosL(){
    return this->posL;
  }
  void setPosC(int posC){
    this->posC = posC;
  }
  int getPosC(){
    return this->posC;
  }
  
};

#define led 4
#define led2 5

//incluindo as bibliotecas necessarias
#include <MPU6050_tockn.h>
#include <Wire.h>

// DEFINIÇÕES
#define MPU6050_ADDR         0x68 // ENDEREÇO QUANDO O PINO AD0 ESTIVER LIGADO AO GND

#define DEBUG
// INSTANCIANDO OBJETOS
MPU6050 mpu6050(Wire);

// DECLARAÇÃO DE VARIÁVEIS  
float anguloX;
float anguloY;
float anguloZ;

int motor2 = 0;

float kpAngulo = 1.5;
float kiAnguloDireita = 0.009;
float kiAnguloEsquerda = 0.009;
float kdAnguloDireita = 30.0;
float kdAnguloEsquerda = 30.0;

float erroDireita;
float erroEsquerda;

float propAnguloDireita;
float propAnguloEsquerda;
float integralAnguloDireita = 0;
float integralAnguloEsquerda = 0;
float PIDdireita;
float PIDesquerda;
float ultimaMedidaAngulo;
float derivadaAnguloDireita;
float derivadaAnguloEsquerda;


void contador()
{
  pulsos = pulsos + 1;

}

void contador2()
{
  pulsos2 = pulsos2 + 1;
}

unsigned long controleTempo;

uint32_t print_timer;
#define led1 12


int ladoObstaculo;

#define velocidade 100
Micromouse micromouse;


int norte = mapa[micromouse.getPosL() - 1][micromouse.getPosC()];
int sul =   mapa[micromouse.getPosL() + 1][micromouse.getPosC()];
int leste = mapa[micromouse.getPosL()][micromouse.getPosC() + 1];
int oeste = mapa[micromouse.getPosL()][micromouse.getPosC() - 1];


bool direitaParede(){
  if(micromouse.direcao == 1){
    if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] == pd){
      return 1;
    }
  }else
  
  if(micromouse.direcao == 2){
    if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] == pd){
      return 1;
    }
  }else

  if(micromouse.direcao == 3){
    if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] == pd){
      return 1;
    }
  }else

  if(micromouse.direcao == 4){
    if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] == pd){
      return 1;
    }
  }
  
  if(distanceLESTEUltra < 30){
    return 1;
  }
  else{
    return 0;
  }
}

bool esquerdaParede(){
  if(micromouse.direcao == 1){
    if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] == pd){
      return 1;
    }
  }else
  
  if(micromouse.direcao == 2){
    if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] == pd){
      return 1;
    }
  }else

  if(micromouse.direcao == 3){
    if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] == pd){
      return 1;
    }
  }else

  if(micromouse.direcao == 4){
    if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] == pd){
      return 1;
    }
  }
  
  if(distanceOESTEUltra < 30){
    return 1;
  }
  else{
    return 0;
  }
}

bool frenteParede(){
  if(micromouse.direcao == 1){
    if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] == pd){
      return 1;
    }
  }else
  
  if(micromouse.direcao == 2){
    if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] == pd){
      return 1;
    }
  }else

  if(micromouse.direcao == 3){
    if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] == pd){
      return 1;
    }
  }else

  if(micromouse.direcao == 4){
    if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] == pd){
      return 1;
    }
  }
  
  if(distancia2 < 30){
    return 1;
  }
  
  else{
    return 0;
  }
}

bool direitaLivre(){
  if(distanceLESTEUltra >= 31){
    return 1;
  }
  else{
    return 0;
  }
}
bool esquerdaLivre(){
  if(distanceOESTEUltra >= 31){
    return 1;
  }
  else{
    return 0;
  }
}
bool frenteLivre(){
  if(distancia2 >= 50){
    return 1;
  }
  else{
    return 0;
  }
}

bool diagonalDireitaLivre(){
  if(distancia3 >= 50){
    return 1;
  }
  else{
    return 0;
  }
}

bool diagonalEsquerdaLivre(){
  if(distancia1 >= 50){
    return 1;
  }
  else{
    return 0;
  }
}

bool diagonalDireitaParede(){
  if(distancia3 <= 43){
    return 1;
  }
  else{
    return 0;
  }
}

bool diagonalEsquerdaParede(){
  if(distancia1 <= 43){
    return 1;
  }
  else{
    return 0;
  }
}

void atualizarVariaveis(){

  /*
   *  AQUI DENTRO TODAS AS VARIÁVEIS QUE FUNCIONAM DENTRO DE ALGUM LOOP SÃO ATUALIZADAS
   *  LEITURA DO GIROSCÓPIO
   *  LEITURA SENSOR INFRAVERMELHO
   *  PID
   *  ESSA FUNÇÃO DEVE ESTAR DENTRO DE TODAS FUNÇÕES QUE TRABALHAM COM MOVIMENTO OU LEITURA,
   *  CASO VOCE ESTIVER TRABALHANDO NO VOID LOOP ESSA FUNÇÃO DEVE SER COLOCADA!
   */
  //Atualizando o giroscpio
  mpu6050.update();


  // GUARDA NA MEMÓRIA OS VALORES ENVIADOS PELO GIROSCOPIO
  anguloX = mpu6050.getAngleX();
  anguloY = mpu6050.getAngleY();
  anguloZ = mpu6050.getAngleZ();

  integral = 0;
  integralN = 0;
  integralL = 0;
  integralLD = 0;
  integralLE = 0;
  integralAnguloDireita = 0;
  integralAnguloEsquerda = 0;
  
  //Atualizando variaveis do PID
  diferenca = anguloZ - InicioanguloZ;

  proporcional = diferenca * kp;

  integral += diferenca * ki;

  derivativo = (ultimaMedida - anguloZ) * kd;

  ultimaMedida = anguloZ;

  PID = proporcional + integral + derivativo;
 
  
  diferencaN = distancia2 - ideal_valueN;

  proporcionalN = diferencaN * kpN;

  integralN += diferencaN * kiN;

  derivativoN = (distancia2 - ultimaMedidaN) * kdN;

  ultimaMedidaN = distancia2;

  PIDN = proporcionalN + integralN + derivativoN;

  
  if(distanceLESTEUltra > 40) distanceLESTEUltra = 40;
  diferencaL = distanceLESTEUltra - ideal_valueL;

  proporcionalL = diferencaL * kpL;

  integralL += diferencaL * kiL;

  derivativoL = (ultimaMedidaL - distanceLESTEUltra) * kdL;

  ultimaMedidaL = distanceLESTEUltra;

  PIDL = proporcionalL + integralL + derivativoL;
  

  diferencaLD = distancia3 - ideal_valueLD;

  proporcionalLD = diferencaLD * kpLD;

  integralLD += diferencaLD * kiLD;

  derivativoLD = (ultimaMedidaLD - distancia3) * kdLD;

  ultimaMedidaLD = anguloZ;

  PIDLD = proporcionalLD + integralLD + derivativoLD;
  

  diferencaLE = distancia1 - ideal_valueLE;

  proporcionalLE = diferencaLE * kpLE;

  integralLE += diferencaLE * kiLE;

  derivativoLE = (ultimaMedidaLE - distancia1) * kdLE;

  ultimaMedidaLE = anguloZ;

  PIDLE = proporcionalLE + integralLE + derivativoLE;


  direitaParede();
  esquerdaParede();
  frenteParede();
  direitaLivre();
  esquerdaLivre();
  frenteLivre();

  if(micromouse.direcao > 4){
    micromouse.direcao = 1;
  }
  if(micromouse.direcao < 1){
    micromouse.direcao = 4;
  }

  norte = mapa[micromouse.getPosL() - 1][micromouse.getPosC()];
  sul = mapa[micromouse.getPosL() + 1][micromouse.getPosC()];
  leste = mapa[micromouse.getPosL()][micromouse.getPosC() + 1];
  oeste = mapa[micromouse.getPosL()][micromouse.getPosC() - 1];

  norteMenorValor();
  lesteMenorValor();
  oesteMenorValor();
  sulMenorValor();
  
}

void atualizarUltrassonico(){
  digitalWrite(trig_pin2OESTE, HIGH);
  delayMicroseconds(11600);
  digitalWrite(trig_pin2OESTE, LOW);
  pulse_time2 = pulseIn(echo_pin2OESTE, HIGH);
  digitalWrite(trig_pin3LESTE, HIGH);
  delayMicroseconds(11600);
  digitalWrite(trig_pin3LESTE, LOW);
  pulse_time3 = pulseIn(echo_pin3LESTE, HIGH);
  distanceOESTEUltra = 0.01715 * pulse_time2 + 2.3;
  distanceLESTEUltra = 0.01715 * pulse_time3;
}


void zerarPulsos(){
  /*
   *  ESSA FUNCAO ZERA OS PULSOS DO ENCODER
   */
    pulsos = 0;
    pulsos2 = 0;
}

void desligarMotores(){
  /*
   *  ESSA FUNCAO DESLIGA TODOS OS MOTORES, COLOQUE ELA FORA DE ALGUM LOOP QUE ENVOLVA MOTORES,
   *  ASSIM QUE ACABAR A MOVIMENTAÇÃO, OS MOTORES DEVEM SER DESLIGADOS
   */
  analogWrite(pinMot1A, 0);             
   analogWrite(pinMot1B, 0);
    analogWrite(pinMot2A, 0);             
    analogWrite(pinMot2B, 0);
}

void corrigirDirecao(){
  //Serial.println(diferenca);
  atualizarVariaveis();
  //Andando reto
  if(PID < -1)
    {
      PID = map(PID, 0, -30, 25, 140);
      if (PID > 140) motor2 = 140;
      analogWrite(pinMot2A, PID);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, 0);
      //direita
      analogWrite(pinMot1B, PID);
    }

  else if(PID > 1)
  {
    PID = map(PID, 0, 30, 25, 140);
    if(PID > 140) PID = 140;
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, PID);  
    //esquerda   
    analogWrite(pinMot1A, PID); 
    //direita                
    analogWrite(pinMot1B, 0);
  }
  else
  {
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0); 
    //esquerda    
    analogWrite(pinMot1A, 0);
    //direita                 
    analogWrite(pinMot1B, 0);
  }
}

void corrigirCentroCelula(){
  atualizarVariaveis();
  leituraSensorInfravermelho();
  //Serial.println(PIDN);
  if(PIDN < 0){
    PIDN = map(PIDN, 0, -5, 0, 170);
      if(PIDN > 170) PIDN = 170;
    //Andando reto
  if(PID < -3)
    {
      PID = map(PID, 0, -30, 0, 30);
      if(PID > 30) PID = 30;
      analogWrite(pinMot2A, PIDN + PID);
      analogWrite(pinMot2B, PIDN);
      //esquerda
      analogWrite(pinMot1A, 0);
      //direita
      analogWrite(pinMot1B, 0);
    }

  else if(PID > 3)
  {
    PID = map(PID, 0, 30, 0, 30);
    if(PID > 30) PID = 30;
    analogWrite(pinMot2A, PIDN);
    analogWrite(pinMot2B, PIDN + PID);  
    //esquerda   
    analogWrite(pinMot1A, 0); 
    //direita                
    analogWrite(pinMot1B, 0);
  }
  else
  {
    analogWrite(pinMot2A, PIDN);
    analogWrite(pinMot2B, PIDN); 
    //esquerda    
    analogWrite(pinMot1A, 0);
    //direita                 
    analogWrite(pinMot1B, 0);
  }
  }
  else if(PIDN > 0){
  //Andando reto
  PIDN = map(PIDN, 0, 15, 0, 100);
      if(PIDN > 100) PIDN = 100;

      if(PID < -3)
    {
      PID = map(PID, 0, -30, 0, 20);
      if (motor2 > 20) motor2 = 20;
      analogWrite(pinMot2A, 0);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, PIDN);
      //direita
      analogWrite(pinMot1B, PIDN + PID);
    }

  else if(PID > 3)
  {
    PID = map(PID, 0, 30, 0, 20);
    if(PID > 20) PID = 20;
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0);  
    //esquerda   
    analogWrite(pinMot1A, PIDN + PID); 
    //direita                
    analogWrite(pinMot1B, PIDN);
  }
  else
  {
     analogWrite(pinMot2A, 0);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, PIDN);
      //direita
      analogWrite(pinMot1B, PIDN);
  }
  
  atualizarVariaveis();
  leituraSensorInfravermelho();   
     
  }
  else{
    desligarMotores();
  }
}

void irParaFrentePIDlateralEangulo(int distancia){
  atualizarUltrassonico();
  atualizarVariaveis();
  leituraSensorInfravermelho();
  integralL = 0;
  while((pulsos2)/1 < distancia){
    if(direitaParede() && esquerdaParede()){
    atualizarVariaveis();
  leituraSensorInfravermelho();
  //Serial.println(PIDL);
  atualizarVariaveis();
  //Andando reto
  if(PIDL < -3)
    {
      PIDL = map(PIDL, 0, -30, 30, 80);
      if(PIDL > 80) PIDL = 80;
      motor2 = map(PIDL, 30, 80, 0, 40);
      if (motor2 > 40) motor2 = 40;
      analogWrite(pinMot2A, 0);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, motor2);
      //direita
      analogWrite(pinMot1B, PIDL);
    }

  else if(PIDL > 3)
  {
    PIDL = map(PIDL, 0, 30, 30, 80);
    if(PID > 80) PIDL = 80;
    motor2 = map(PIDL, 30, 150, 0, 40);
    if (motor2 > 40) motor2 = 40;
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0);  
    //esquerda   
    analogWrite(pinMot1A, PIDL); 
    //direita                
    analogWrite(pinMot1B, motor2);
  }
  else
  {
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0); 
    //esquerda    
    analogWrite(pinMot1A, 50);
    //direita                 
    analogWrite(pinMot1B, 50);
  } 
  }else
  if(direitaLivre() || esquerdaLivre()){
    atualizarVariaveis();
  Serial.println(diferenca);
  atualizarVariaveis();
  //Andando reto
  if(PID < -3)
    {
      PID = map(PID, 0, -30, 25, 100);
      if(PID > 100) PID = 100;
      motor2 = map(PID, 25, 100, 0, 60);
      if (motor2 > 60) motor2 = 60;
      analogWrite(pinMot2A, 0);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, motor2);
      //direita
      analogWrite(pinMot1B, PID);
    }

  else if(PID > 3)
  {
    PID = map(PID, 0, 30, 25, 100);
    if(PID > 100) PID = 100;
    motor2 = map(PID, 25, 100, 0, 60);
    if (motor2 > 60) motor2 = 60;
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0);  
    //esquerda   
    analogWrite(pinMot1A, PID); 
    //direita                
    analogWrite(pinMot1B, motor2);
  }
  else
  {
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0); 
    //esquerda    
    analogWrite(pinMot1A, 100);
    //direita                 
    analogWrite(pinMot1B, 100);
  }
  }
  }
  
}

void irParaFrente(){
  /*
   *  ESSA FUNCAO TRABALHA JUNTO COM O PID, ELA SEMPRE ANDA PARA FRENTE. A FRENTE NÃO É FIXA, ELE 
   *  SE MODIFICA SEMPRE QUE O ROBÔ VIRA PARA DIREITA OU PARA ESQUERDA!!! CASO O ROBÔ VIRE PARA DIREITA
   *  SEM SER DE PROPÓSITO A ROTA É CORRIGIDA!
   */
  atualizarVariaveis();
  //Serial.println(diferenca);
  atualizarVariaveis();
  //Andando reto
  if(PID < -3)
    {
      PID = map(PID, 0, -30, 25, 100);
      if(PID > 100) PID = 100;
      motor2 = map(PID, 25, 100, 0, 60);
      if (motor2 > 60) motor2 = 60;
      analogWrite(pinMot2A, 0);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, motor2);
      //direita
      analogWrite(pinMot1B, PID);
    }

  else if(PID > 3)
  {
    PID = map(PID, 0, 30, 25, 100);
    if(PID > 100) PID = 100;
    motor2 = map(PID, 25, 100, 0, 60);
    if (motor2 > 60) motor2 = 60;
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0);  
    //esquerda   
    analogWrite(pinMot1A, PID); 
    //direita                
    analogWrite(pinMot1B, motor2);
  }
  else
  {
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0); 
    //esquerda    
    analogWrite(pinMot1A, 50);
    //direita                 
    analogWrite(pinMot1B, 50);
  } 
}

void irParaFrenteDistancia(int pulsosParaCmRecebido){
  /*
   *  ESSA FUNCAO TRABALHA JUNTO COM O PID, ELA SEMPRE ANDA PARA FRENTE. A FRENTE NÃO É FIXA, ELE 
   *  SE MODIFICA SEMPRE QUE O ROBÔ VIRA PARA DIREITA OU PARA ESQUERDA!!! CASO O ROBÔ VIRE PARA DIREITA
   *  SEM SER DE PROPÓSITO A ROTA É CORRIGIDA!
   *  NESSA FUNÇÃO É ESCOLHIDO QUANTOS PULSOS O ROBÔ DEVE SE MOVIMENTAR
   */
  zerarPulsos();
  integral = 0;
  while((pulsos2)/1 < pulsosParaCmRecebido){
    atualizarVariaveis();
    atualizarVariaveis();
  //Andando reto
  if(PID < -2)
    {
      PID = map(PID, 0, -30, 25, 100);
      if(PID > 100) PID = 100;
      motor2 = map(PID, 25, 100, 0, 60);
      if (motor2 > 60) motor2 = 60;
      analogWrite(pinMot2A, 0);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, motor2);
      //direita
      analogWrite(pinMot1B, PID);
    }

  else if(PID > 2)
  {
    PID = map(PID, 0, 30, 25, 100);
    if(PID > 100) PID = 100;
    motor2 = map(PID, 25, 100, 0, 60);
    if (motor2 > 60) motor2 = 60;
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0);  
    //esquerda   
    analogWrite(pinMot1A, PID); 
    //direita                
    analogWrite(pinMot1B, motor2);
  }
  else
  {
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0); 
    //esquerda    
    analogWrite(pinMot1A, 100);
    //direita                 
    analogWrite(pinMot1B, 100);
  }
  }
  desligarMotores();  
}

void irParaFrentePIDLateral(){
  /*
   *  ESSA FUNCAO TRABALHA JUNTO COM O PID, ELA SEMPRE ANDA PARA FRENTE. A FRENTE NÃO É FIXA, ELE 
   *  SE MODIFICA SEMPRE QUE O ROBÔ VIRA PARA DIREITA OU PARA ESQUERDA!!! CASO O ROBÔ VIRE PARA DIREITA
   *  SEM SER DE PROPÓSITO A ROTA É CORRIGIDA!
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  atualizarUltrassonico();
  integralL = 0;
  //Serial.println(PIDL);
  //Andando reto
  if(PIDL < -1)
    {
      PIDL = map(PIDL, 0, -15, 0, 20);
      if(PIDL > 20) PIDL = 20;
      analogWrite(pinMot2A, 0);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, 50 - PIDL);
      //direita
      analogWrite(pinMot1B, 50 + PIDL);
    }

  else if(PIDL > 1)
  {
    PIDL = map(PIDL, 0, 15, 0, 20);
    if(PIDL > 20) PIDL = 20;
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0);  
    //esquerda   
    analogWrite(pinMot1A, 50 + PIDL); 
    //direita                
    analogWrite(pinMot1B, 50 - PIDL);
  }
  else
  {
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0); 
    //esquerda    
    analogWrite(pinMot1A, 50);
    //direita                 
    analogWrite(pinMot1B, 50);
  } 
}


void irParaFrentePIDLateralDireita(){
  /*
   *  ESSA FUNCAO TRABALHA JUNTO COM O PID, ELA SEMPRE ANDA PARA FRENTE. A FRENTE NÃO É FIXA, ELE 
   *  SE MODIFICA SEMPRE QUE O ROBÔ VIRA PARA DIREITA OU PARA ESQUERDA!!! CASO O ROBÔ VIRE PARA DIREITA
   *  SEM SER DE PROPÓSITO A ROTA É CORRIGIDA!
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  integralLD = 0;
  //Serial.println(PIDLD);
  atualizarVariaveis();
  //Andando reto
  if(PIDLD < -7)
    {
      PIDLD = map(PIDLD, 0, -20, 0, 60);
      if(PIDLD > 60) PIDLD = 60;
      motor2 = map(PIDLD, 0, 60, 0, 40);
      if (motor2 > 40) motor2 = 40;
      analogWrite(pinMot2A, 0);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, motor2);
      //direita
      analogWrite(pinMot1B, PIDLD);
    }

  else if(PIDLD > 7)
  {
    PIDLD = map(PIDLD, 0, 20, 0, 60);
    if(PIDLD > 60) PIDLD = 60;
    motor2 = map(PIDLD, 0, 60, 0, 40);
    if (motor2 > 40) motor2 = 40;
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0);  
    //esquerda   
    analogWrite(pinMot1A, PIDLD); 
    //direita                
    analogWrite(pinMot1B, motor2);
  }
  else
  {
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0); 
    //esquerda    
    analogWrite(pinMot1A, 60);
    //direita                 
    analogWrite(pinMot1B, 60);
  } 
}

void irParaFrentePIDLateralDistancia(int distancia){
  /*
   *  ESSA FUNCAO TRABALHA JUNTO COM O PID, ELA SEMPRE ANDA PARA FRENTE. A FRENTE NÃO É FIXA, ELE 
   *  SE MODIFICA SEMPRE QUE O ROBÔ VIRA PARA DIREITA OU PARA ESQUERDA!!! CASO O ROBÔ VIRE PARA DIREITA
   *  SEM SER DE PROPÓSITO A ROTA É CORRIGIDA!
   */
  zerarPulsos();
  integralL = 0;
  while((pulsos2)/1 < distancia){
  atualizarVariaveis();
  leituraSensorInfravermelho();
  //Serial.println(PIDL);
  atualizarVariaveis();
  //Andando reto
  if(PIDL < -7)
    {
      PIDL = map(PIDL, 0, -20, 0, 60);
      if(PIDL > 60) PIDL = 60;
      motor2 = map(PIDL, 0, 60, 0, 40);
      if (motor2 > 40) motor2 = 40;
      analogWrite(pinMot2A, 0);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, motor2);
      //direita
      analogWrite(pinMot1B, PIDL);
    }

  else if(PIDL > 7)
  {
    PIDL = map(PIDL, 0, 20, 0, 60);
    if(PID > 60) PIDL = 60;
    motor2 = map(PIDL, 0, 60, 0, 40);
    if (motor2 > 40) motor2 = 40;
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0);  
    //esquerda   
    analogWrite(pinMot1A, PIDL); 
    //direita                
    analogWrite(pinMot1B, motor2);
  }
  else
  {
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0); 
    //esquerda    
    analogWrite(pinMot1A, 60);
    //direita                 
    analogWrite(pinMot1B, 60);
  } 
  }
  desligarMotores(); 
}

void irParaTras(){
  /*
   *  ESSA FUNCAO TRABALHA JUNTO COM O PID, ELA SEMPRE ANDA PARA TRÁS. A "TRÁS" NÃO É FIXA, ELE 
   *  SE MODIFICA SEMPRE QUE O ROBÔ VIRA PARA DIREITA OU PARA ESQUERDA!!! CASO O ROBÔ VIRE PARA DIREITA
   *  SEM SER DE PROPÓSITO A ROTA É CORRIGIDA!
   */
  atualizarVariaveis();
    //Andando reto
  if(PID < -10)
    {
      PID = map(PID, 0, -30, 60, 80);
      if(PID > 80) PID = 80;
      motor2 = map(PID, 60, 80, 0, 40);
      if (motor2 > 40) motor2 = 40;
      analogWrite(pinMot2A, PID);
      analogWrite(pinMot2B, motor2);
      //esquerda
      analogWrite(pinMot1A, 0);
      //direita
      analogWrite(pinMot1B, 0);
    }

  else if(PID > 10)
  {
    PID = map(PID, 0, 30, 60, 80);
    if(PID > 80) PID = 80;
    motor2 = map(PID, 60, 80, 0, 40);
    if (motor2 > 40) motor2 = 40;
    analogWrite(pinMot2A, motor2);
    analogWrite(pinMot2B, PID);  
    //esquerda   
    analogWrite(pinMot1A, 0); 
    //direita                
    analogWrite(pinMot1B, 0);
  }
  else
  {
    analogWrite(pinMot2A, 40);
    analogWrite(pinMot2B, 40); 
    //esquerda    
    analogWrite(pinMot1A, 0);
    //direita                 
    analogWrite(pinMot1B, 0);
  }
}

void virarParaDireita(){
  /*
   *  NESSA FUNCAO O ROBÔ VIRA 90 GRAUS PARA DIREITA!!!
   */
  atualizarVariaveis();
  for(int i=0; i<100; i++){
    atualizarVariaveis();
    corrigirDirecao();
  }
  InicioanguloZ = mpu6050.getAngleZ() - 84.3;
  ideal_value = InicioanguloZ;
  integral = 0;
  
  for(int i=0; i<100; i++){
    atualizarVariaveis();
    corrigirDirecao();
  }
  integral = 0;
  for(int i=0; i<50; i++){
    atualizarVariaveis();
    desligarMotores();
    zerarPulsos();
  }
}

void virarParaEsquerda(){
  /*
   *  NESSA FUNÇÃO O ROBÔ VIRA 90 GRAUS PARA ESQUERDA!!!
   */
  atualizarVariaveis();
  for(int i=0; i<100; i++){
    atualizarVariaveis();
    corrigirDirecao();
  }
  InicioanguloZ = mpu6050.getAngleZ() + 84.3;
  ideal_value = InicioanguloZ;
  integral = 0;
  
  for(int i=0; i<100; i++){
    atualizarVariaveis();
    corrigirDirecao();
  }
  integral = 0;
  for(int i=0; i<50; i++){
    atualizarVariaveis();
    desligarMotores();
    zerarPulsos();
  }
}



void leituraSensorInfravermelho(){

  /*
   * AQUI SÃO ATUALIZADAS AS LEITURAS DOS SENSORES INFRAVERMELHOS!!! ALÉM DISSO É FEITO O CÁLCULO DE INTERPOLAÇÃO DA DISTÂNCIA
   */
  
   //Obtenção do valor em Volts através da formula ( valorAnalógico * (5/1023) )
   valorVoltsSharp1 = analogRead(pinoSharp1) * 0.0048828125;
   // Obtenção do valor em Volts através da formula ( valorAnalógico * (5/1023) )
   valorVoltsSharp2 = analogRead(pinoSharp2) * 0.0048828125; 
   // Obtenção do valor em Volts através da formula ( valorAnalógico * (5/1023) )
   valorVoltsSharp3 = analogRead(pinoSharp3) * 0.0048828125; 
   
   //media das distancias do sensor 1
   for(int i=0;i<20;i++){
    // Obtenção do valor em Volts através da formula ( valorAnalógico * (5/1023) )
    double valorVoltsSharp1 = analogRead(pinoSharp1) * 0.0048828125; 
    // Formula para cálculo da distância levando em consideração o valor em volts
    exp1 = (-1*valorVoltsSharp1)/t;
    distanciaSharp1 = y0 + A*exp(exp1);
    if(distanciaSharp1 > 80) distanciaSharp1 = 80;
    if(distanciaSharp1 < 0) distanciaSharp1 = 0;
    distancia1 = (distancia1 + distanciaSharp1);
   }
   distancia1 = distancia1/20;
   
   //media das distancias do sensor 2
   for(int i=0;i<20;i++){
    // Obtenção do valor em Volts através da formula ( valorAnalógico * (5/1023) )
    double valorVoltsSharp2 = analogRead(pinoSharp2) * 0.0048828125; 
    // Formula para cálculo da distância levando em consideração o valor em volts
    exp1 = (-1*valorVoltsSharp2)/t;
    distanciaSharp2 = y0 + A*exp(exp1);
    if(distanciaSharp2 > 80) distanciaSharp2 = 80;
    if(distanciaSharp2 < 0) distanciaSharp2 = 0;
    distancia2 = (distancia2 + distanciaSharp2);
   }
   distancia2 = distancia2/20;
   
   ///media das distancias do sensor 3
   for(int i=0;i<20;i++){
    // Obtenção do valor em Volts através da formula ( valorAnalógico * (5/1023) )
    double valorVoltsSharp3 = analogRead(pinoSharp3) * 0.0048828125; 
    // Formula para cálculo da distância levando em consideração o valor em volts
    exp1 = (-1*valorVoltsSharp3)/t;
    distanciaSharp3 = y0 + A*exp(exp1);
    if(distanciaSharp3 > 80) distanciaSharp3 = 80;
    if(distanciaSharp3 < 0) distanciaSharp3 = 0;
    distancia3 = (distancia3 + distanciaSharp3);
   }
   distancia3 = distancia3/20;

}


bool norteMenorValor(){
  if((norte < leste || leste == pd) && (norte < oeste || oeste == pd) && (norte < sul || sul == pd) && norte != pd){
    return 1;
  }
  else{
    return 0;
  }
}

bool lesteMenorValor(){
  if((leste < norte || norte == pd) && (leste < oeste || oeste == pd) && (leste < sul || sul == pd) && leste != pd){
    return 1;
  }
  else{
    return 0;
  }
}

bool oesteMenorValor(){
  if((oeste < norte || norte == pd) && (oeste < leste || leste == pd) && (oeste < sul || sul == pd) && oeste != pd){
    return 1;
  }
  else{
    return 0;
  }
}

bool sulMenorValor(){
  if((sul < norte || norte == pd) && (sul < leste || leste == pd) && (sul < oeste || oeste == pd) && sul != pd){
    return 1;
  }
  else{
    return 0;
  }
}


bool frenteUnicoCaminhoLivre(){
  /*
   *  CASO O ROBÔ ESTEJA EM UM CORREDOR (PAREDE DOS DOIS LADOS E FRENTE LIVRE) É RETORNADO O VALOR TRUE
   */

   //((distancia1 - distancia3) >= -10 && (distancia1 - distancia3) <= 10)
  atualizarVariaveis();
  atualizarUltrassonico();
  leituraSensorInfravermelho();
  if(esquerdaParede() && direitaParede() && frenteLivre()){
    return 1;
  }
  else{
    return 0;
  }
}

bool direitaUnicoCaminhoLivre(){
  /*
   *  CASO APENAS O LADO DIREITO DO ROBÔ POSSUA CAMINHO LIVRE
   */
  atualizarVariaveis();
  atualizarUltrassonico();
  leituraSensorInfravermelho();
  if(esquerdaParede() && frenteParede() && direitaLivre()){
    return 1;
  }
  else{
    return 0;
  }
}

bool esquerdaUnicoCaminhoLivre(){
  /*
   *  CASO APENAS O LADO ESQUERDO DO ROBÔ POSSUA CAMINHO LIVRE
   */
  atualizarVariaveis();
  atualizarUltrassonico();
  leituraSensorInfravermelho();
  if(esquerdaLivre() && frenteParede() && direitaParede()){
    return 1;
  }
  else{
    return 0;
  }
}

bool trasUnicoCaminhoLivre(){
  /*
   *  CASO APENAS A TRÁS ESQUERDO DO ROBÔ POSSUA CAMINHO LIVRE
   */
  atualizarVariaveis();
  atualizarUltrassonico();
  leituraSensorInfravermelho();
  if(esquerdaParede() && frenteParede() && direitaParede()){
    return 1;
  }
  else{
    return 0;
  }
}

bool frenteEDireitaLivre(){
  atualizarVariaveis();
  atualizarUltrassonico();
  leituraSensorInfravermelho();
  if(esquerdaParede() && frenteLivre() && direitaLivre()){
    return 1;
  }
  else{
    return 0;
  }
}

bool frenteEEsquerdaLivre(){
  atualizarVariaveis();
  atualizarUltrassonico();
  leituraSensorInfravermelho();
  if(direitaParede() && frenteLivre() && esquerdaLivre()){
    return 1;
  }
  else{
    return 0;
  }
}

bool frenteEDireitaEEsquerdaLivre(){
  atualizarVariaveis();
  atualizarUltrassonico();
  leituraSensorInfravermelho();
  if(esquerdaLivre() && frenteLivre() && esquerdaLivre()){
    return 1;
  }
  else{
    return 0;
  }
}

bool direitaEEsquerdaLivre(){
  atualizarVariaveis();
  atualizarUltrassonico();
  leituraSensorInfravermelho();
  if(direitaLivre() && frenteParede() && esquerdaLivre()){
    return 1;
  }
  else{
    return 0;
  }
}


void corrigirCentro(){
  leituraSensorInfravermelho();
  atualizarVariaveis();
  desligarMotores();
  if(distancia2 < 40){
    for(int i=0;i<70;i++){
      digitalWrite(led2, HIGH);
      leituraSensorInfravermelho();
      atualizarVariaveis();
      corrigirCentroCelula();
      digitalWrite(led2, LOW);
    }
  }
    for(int i=0;i<20;i++){
      digitalWrite(led2, HIGH);
      atualizarVariaveis();
      corrigirDirecao();
      digitalWrite(led2, LOW);
    }
    zerarPulsos();
    desligarMotores();
}

void imprimirLabirinto(){
  Serial.println("////////////////////////");
  for(int x=0; x<7; x++){
    for(int y=0; y<7; y++){
      Serial.print(mapa[x][y]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println("////////////////////////");
}
  

void imprimirLeituraInfra(){
  /*
   *  IMPRIMI AS LEITURAS DOS 3 SENSORES INFRAVERMELHOS
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  //Serial.print("d1 ");
  //Serial.print(distancia1);
  Serial.print(" d2 ");
  Serial.println(distancia2);
  //Serial.print(" d3 ");
  //Serial.println(distancia3);
  //delay(500);
}

void imprimirLeituraAnguloZ(){
  /*
   *  IMPRIMI A LEITURA DO ANGULO DO GIROSCÓPIO
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  Serial.print("angulo Z ");
  Serial.println(anguloZ);
}

void imprimirEncoders(){
  /*
   *  IMPRIMI O VALOR DOS ENCODERS
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  //Serial.print("pulso 1 ");
  //Serial.print(pulsos);
  Serial.print(" pulso 2 ");
  Serial.println(pulsos2);
}

void imprimirUltrassonico(){
  atualizarUltrassonico();
  Serial.print("Distancia oeste: ");
  Serial.println(distanceOESTEUltra);
  Serial.print("Distancia leste: ");
  Serial.println(distanceLESTEUltra);
  delay(500);
 
  
}

bool detectouObstaculo(){
  /*
   *  CASO TENHA ALGUM OBSTÁCULO NA FRENTE DENTRO DE UMA DISTÂNCIA FIXA É RETORNADO TRUE
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  if(distancia2 <=20){
    desligarMotores();
    return true;
  }
  else{
    return false;
  }
}


void andarUmaCelula2(){
  zerarPulsos();
  while(pulsos2 < 120){
     atualizarVariaveis();
     leituraSensorInfravermelho();
     atualizarUltrassonico();
     if(distanceOESTEUltra < 27 && distanceLESTEUltra < 27){
      irParaFrentePIDLateral();
     }else{
      irParaFrente();
     }
  }
  for(int i=0; i<100; i++){
    atualizarVariaveis();
    corrigirDirecao();
  }
  zerarPulsos();
  desligarMotores();
}
void andarUmaCelula(){
  zerarPulsos();
  desligarMotores();
  atualizarVariaveis();
  irParaFrenteDistancia(65);
  zerarPulsos();
  desligarMotores();
  atualizarVariaveis();
}
void andarUmaCelula3(){
  zerarPulsos();
  desligarMotores();
  atualizarVariaveis();
  irParaFrenteDistancia(66);
  zerarPulsos();
  desligarMotores();
  atualizarVariaveis();
  
}

void andarUmaCelulaLateral(){
  zerarPulsos();
  irParaFrentePIDLateralDistancia(70);
}

void zerarPulsosParaCm(){
  pulsosParaCm = 0;
}

bool floodFillFinalizado = false;

bool chegouDestino(){
  if(mapa[micromouse.getPosL()][micromouse.getPosC()] == fi){
    return 1;
  }
  else{
    return 0;
  }
}

void desenharQuadrado(){
  andarUmaCelula2();
  virarParaDireita();
  andarUmaCelula2();
  virarParaDireita();
  andarUmaCelula2();
  virarParaDireita();
  andarUmaCelula2();
  virarParaDireita();
}

void desenharQuadrado2(){
  andarUmaCelula2();
  virarParaEsquerda();
  andarUmaCelula2();
  virarParaEsquerda();
  andarUmaCelula2();
  virarParaEsquerda();
  andarUmaCelula2();
  virarParaEsquerda();
}

void floodFill(){
  if(!chegouDestino()){
    for(int i=0; i<10; i++){
      digitalWrite(led, HIGH);
      digitalWrite(led2, HIGH);
      delay(100);
      digitalWrite(led, LOW);
      digitalWrite(led2, LOW);
      delay(100);
    }
  }
  
  while(!chegouDestino()){
    int atual = mapa[micromouse.getPosL()][micromouse.getPosC()];
    int linha = micromouse.getPosL();
    int coluna = micromouse.getPosC();
    chegouDestino();
    atualizarVariaveis();
    leituraSensorInfravermelho();
    
    /////////////////////////////////////////////////
    
    //Inicio frenteUnicoCaminhoLivre()
    if(frenteUnicoCaminhoLivre() && !chegouDestino()){
      digitalWrite(led, HIGH);
      andarUmaCelula2();

      /////////////////////////////////////////////////////////////////
      
      //CASO a direcao seja NORTE
      if(micromouse.direcao == 1){
        //Aplicando o somatario das celulas
        //se norte for diferente de final
        if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
          //norte = atual + 1
          mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
        }
        //se norte 2 celulas for diferente de final
        if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] != fi){
          //norte 2 celulas = atual + 2
          mapa[micromouse.getPosL() - 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
        }
        //Definindo as paredes
        //leste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
        //oeste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
        //leste 2 = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC() + 1] = pd;
        //oeste 2 = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC() - 1] = pd;
        //Definindo a nova posicao
        linha = linha - 2;
        micromouse.setPosL(linha);
      }else

      /////////////////////////////////////////////////////////////////
      
      //CASO a direcao seja LESTE
      if(micromouse.direcao == 2){
        //Aplicando o somatario das celulas
        //se leste for diferente de final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
          //leste = atual + 1
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
        }
        //se leste 2 celulas for diferente de final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] != fi){
          //leste 2 celulas = atual + 2
          mapa[micromouse.getPosL()][micromouse.getPosC() + 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
        }
        //Definindo as paredes
        //norte = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
        //sul = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
        //norte + leste = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC() + 1] = pd;
        //sul + leste = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC() + 1] = pd;
        //Definindo a nova posicao
        coluna = coluna + 2;
        micromouse.setPosC(coluna);
      }else

      /////////////////////////////////////////////////////////////////
      
      //CASO a direcao seja SUL
      if(micromouse.direcao == 3){
        //Aplicando o somatario das celulas
        //se sul for diferente de final
        if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
          //sul = atual + 1
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
        }
        //se sul duas celulas for diferente de final
        if(mapa[micromouse.getPosL() + 2][micromouse.getPosC()] != fi){
          //sul 2 celulas = atual + 2
          mapa[micromouse.getPosL() + 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
        }
        //Definindo as paredes
        //leste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
        //oeste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
        //sul + leste = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC() + 1] = pd;
        //sul + oeste = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC() - 1] = pd;
        //Definindo a nova posicao
        linha = linha + 2;
        micromouse.setPosL(linha);
      }else

      /////////////////////////////////////////////////////////////////
      
      //CASO a direcao seja OESTE
      if(micromouse.direcao == 4){
        //Aplicando o somatario das celulas
        //se oeste for diferente de final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
          //oeste = atual + 1
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
        }
        //se oeste 2 celulas for diferente de final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] != fi){
          //oeste 2 celulas = atual + 2
          mapa[micromouse.getPosL()][micromouse.getPosC() - 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
        }
        //Definindo as paredes
        //norte = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
        //sul = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
        //norte + oeste = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC() - 1] = pd;
        //sul + oeste = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC() - 1] = pd;
        //Definindo a nova posicao
        coluna = coluna - 2;
        micromouse.setPosC(coluna);
      }
      digitalWrite(led, LOW);
      atualizarVariaveis();
    }else
    //Fim frenteUnicoCaminhoLivre()
    
    /////////////////////////////////////////////////
    
    //Inicio esquerdaUnicoCaminhoLivre()
    if(esquerdaUnicoCaminhoLivre() && !chegouDestino()){
      digitalWrite(led, HIGH);
      corrigirCentro();
      virarParaEsquerda();
      andarUmaCelula2();

      /////////////////////////////////////////////////////////////////
      //caso a direcao seja norte
      if(micromouse.direcao == 1){
        //Aplicando o somatario das celulas
        //se oeste for diferente de final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
          //oeste = atual + 1
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
        }
        //se oeste 2 celulas for diferente de final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] != fi){
          //oeste 2 celulas = atual + 2
          mapa[micromouse.getPosL()][micromouse.getPosC() - 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
        }
        //Definindo as paredes
        //norte = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
        //leste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
        //norte + oeste = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC() - 1] = pd;
        //sul + oeste = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC() - 1] = pd;
        //Definindo a nova posicao
        coluna = coluna - 2;
        micromouse.setPosC(coluna);
      }else

      /////////////////////////////////////////////////////////////////
      //se a direcao for igual a leste
      if(micromouse.direcao == 2){
        //Aplicando o somatario das celulas
        //se norte for diferente de parede
        if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
          //norte = atual + 1
          mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
        }
        //se norte 2 celulas for diferente de final
        if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] != fi){
          //norte 2 celulas = atual + 2
          mapa[micromouse.getPosL() - 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
        }
        //Definindo as paredes
        //sul = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
        //leste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
        //norte + leste = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC() + 1] = pd;
        //norte + oeste = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC() - 1] = pd;
        //Definindo a nova posicao
        linha = linha - 2;
        micromouse.setPosL(linha);
      }else

      /////////////////////////////////////////////////////////////////
      //se a direcao for sul
      if(micromouse.direcao == 3){
        //Aplicando o somatario das celulas
        //se leste for diferente de final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
          //leste = atual + 1
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
        }
        //se leste 2 celulas for diferente de final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] != fi){
          //leste 2 celulas = atual + 2
          mapa[micromouse.getPosL()][micromouse.getPosC() + 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
        }
        //Definindo as paredes
        //sul = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
        //oeste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
        //norte + leste = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC() + 1] = pd;
        //sul + leste = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC() + 1] = pd;
        //Definindo a nova posicao
        coluna = coluna + 2;
        micromouse.setPosC(coluna);
      }else

      /////////////////////////////////////////////////////////////////
      //se a direcao for oeste
      if(micromouse.direcao == 4){
        //Aplicando o somatario das celulas
        //se sul for diferente de final
        if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
          //sul = atual + 1
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
        }
        //se sul 2 celulas for diferente de final
        if(mapa[micromouse.getPosL() + 2][micromouse.getPosC()] != fi){
          //sul 2 celulas = atual + 2
          mapa[micromouse.getPosL() + 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
        }
        //Definindo as paredes
        //norte = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
        //oeste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
        //sul + leste = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC() + 1] = pd;
        //sul + oeste = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC() - 1] = pd;
        //Definindo a nova posicao
        linha = linha + 2;
        micromouse.setPosL(linha);
      }
      micromouse.direcao--;
      digitalWrite(led, LOW);
      atualizarVariaveis();
    } else

    //Fim esquerdaUnicoCaminhoLivre()
    
    /////////////////////////////////////////////////
    
    //Inicio direitaUnicoCaminhoLivre()
    if(direitaUnicoCaminhoLivre() && !chegouDestino()){
      digitalWrite(led, HIGH);
      corrigirCentro();
      virarParaDireita();
      andarUmaCelula2();

      /////////////////////////////////////////////////////////////////
      //se a direcao for norte 
      if(micromouse.direcao == 1){
        //Aplicando o somatario das celulas
        //se leste for diferente de final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
          //leste = atual + 1
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
        }
        //se leste 2 celulas for diferente de final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] != fi){
          //leste 2 celulas = atual + 2
          mapa[micromouse.getPosL()][micromouse.getPosC() + 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
        }
        //Definindo as paredes
        //norte = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
        //oeste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
        //sul = leste = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC() + 1] = pd;
        //norte = leste = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC() + 1] = pd;
        //Definindo a nova posicao
        coluna = coluna + 2;
        micromouse.setPosC(coluna);
      }else

      /////////////////////////////////////////////////////////////////
      //se a direcao for leste
      if(micromouse.direcao == 2){
        //Aplicando o somatario das celulas
        //se sul for diferente de final
        if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
          //sul = atual + 1
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
        }
        //se sul 2 celulas for diferente de final
        if(mapa[micromouse.getPosL() + 2][micromouse.getPosC()] != fi){
          //sul 2 celulas = atual + 2
          mapa[micromouse.getPosL() + 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
        }
        //Definindo as paredes
        //norte = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
        //leste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
        //sul + leste = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC() + 1] = pd;
        //sul + oeste = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC() - 1] = pd;
        //Definindo a nova posicao
        linha = linha + 2;
        micromouse.setPosL(linha);
      }else

      /////////////////////////////////////////////////////////////////
      //se a direcao for sul
      if(micromouse.direcao == 3){
        //Aplicando o somatario das celulas
        //se oeste for diferente de final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
          //oeste = atual + 1
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
        }
        //se oeste 2 celulas for diferente de final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] != fi){
          //oeste 2 celulas = atual + 2
          mapa[micromouse.getPosL()][micromouse.getPosC() - 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
        }
        //Definindo as paredes
        // sul = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
        //leste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
        //sul + oeste = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC() - 1] = pd;
        //norte + oeste = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC() - 1] = pd;
        //Definindo a nova posicao
        coluna = coluna - 2;
        micromouse.setPosC(coluna);
      }else

      /////////////////////////////////////////////////////////////////
      //se a direcao for oeste 
      if(micromouse.direcao == 4){
        //Aplicando o somatario das celulas
        //se norte for diferente de final
        if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
          //norte = atual + 1
          mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
        }
        //se norte 2 celulas for diferente de final
        if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] != fi){
          //norte 2 celulas = atual + 2
          mapa[micromouse.getPosL() - 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
        }
        //Definindo as paredes
        //sul = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
        //oeste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
        //norte + leste = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC() + 1] = pd;
        //norte + oeste = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC() - 1] = pd;
        //Definindo a nova posicao
        linha = linha - 2;
        micromouse.setPosL(linha);
      }

      /////////////////////////////////////////////////////////////////
      
      micromouse.direcao++;
      digitalWrite(led, LOW);
      atualizarVariaveis();
    } else
    //Fim direitaUnicoCaminhoLivre()
    
    /////////////////////////////////////////////////
    
    //Inicio trasUnicoCaminhoLivre()
    if(trasUnicoCaminhoLivre() && !chegouDestino()){
      digitalWrite(led, HIGH);
      //Aplicando o somatario das celulas
      //se atual for diferente de final
      if(mapa[micromouse.getPosL()][micromouse.getPosC()] != fi){
        //atual = atual + 1
        mapa[micromouse.getPosL()][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
      }

      /////////////////////////////////////////////////////////////////
      //se a direcao for norte
      if(micromouse.direcao == 1){
        //Definindo as paredes
        //norte = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
        //leste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
        //oeste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
      }else

      /////////////////////////////////////////////////////////////////
      //se a direcao for leste
      if(micromouse.direcao == 2){
        //Definindo as paredes
        //leste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
        //norte = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
        //sul = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
      }else

      /////////////////////////////////////////////////////////////////
      //se a direcao for sul
      if(micromouse.direcao == 3){
        //Definindo as paredes
        //sul = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
        //leste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
        //oeste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
      }else

      /////////////////////////////////////////////////////////////////
      //se a direcao for oeste
      if(micromouse.direcao == 4){
        //Definindo as paredes
        //norte = parede
        mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
        //sul = parede
        mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
        //oeste = parede
        mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
      }

      /////////////////////////////////////////////////////////////////
      corrigirCentro();
      virarParaDireita();
      micromouse.direcao++;
      corrigirCentro();
      virarParaDireita();
      micromouse.direcao++;
      digitalWrite(led, LOW);
      atualizarVariaveis();
    }
    //Fim trasUnicoCaminhoLivre()
    
    /////////////////////////////////////////////////

    //Inicio frenteEDireitaLivre()
    if(frenteEDireitaLivre()){
      digitalWrite(led, HIGH);
      
      /////////////////////////////////////////////////////////////////
      //caso direcao igual a norte
      if(micromouse.direcao == 1){
        //caso norte 2 celulas seja o destino
        if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] == fi){
          //Aplicando o somatario das celulas
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = posicao atual + 1
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = posicao atual + 1
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          //Definindo as paredes
          //oeste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
          //Definindo a nova posicao
          linha = linha - 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
          
        }else //se nao
        //se leste 2 celulas for igual a destino
        if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] == fi){
          //Aplicando o somatario das celulas
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = posicao atual + 1
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = posicao atual + 1
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = cl;
          }
          //Definindo as paredes
          //oeste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna + 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }else
        //se leste for o menor valor
        if(lesteMenorValor()){
          //Aplicando o somatario das celulas
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = posicao atual + 1
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se leste 2 celulas for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] != fi){
            //leste = posicao atual + 2
            mapa[micromouse.getPosL()][micromouse.getPosC() + 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = posicao atual + 1
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = cl;
          }
          //Definindo as paredes
          //oeste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna + 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }
        else{
          //Aplicando o somatario das celulas
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = posicao atual + 1
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se norte 2 celulas for diferente de final
          if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] != fi){
            //norte = posicao atual + 2
            mapa[micromouse.getPosL() - 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = posicao atual + 1
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          atualizarVariaveis();
          //Definindo as paredes
          //oeste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
          //Definindo a nova posicao
          linha = linha - 2;
          micromouse.setPosL(linha);
        }
        
      }else
      /////////////////////////////////////////////////////////////////
      //direcao igual a leste
      if(micromouse.direcao == 2){
        //Se leste 2 celulas for igual a final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] == fi){
          //aplicando somatario das celulas
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          atualizarVariaveis();
          //Definindo as paredes
          //norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
          //Definindo a nova posicao
          coluna = coluna + 2;
          micromouse.setPosC(coluna);
        }else
        //se sul 2 celulas for igual final
        if(mapa[micromouse.getPosL() + 2][micromouse.getPosC()] == fi){
          //aplicando somatario das celulas
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = cl;
          }
          //Definindo as paredes
          //norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha + 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }else
        if(sulMenorValor()){
          //aplicando somatario das celulas
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se sul 2 celulas for diferente de final
          if(mapa[micromouse.getPosL() + 2][micromouse.getPosC()] != fi){
            //sul 2 celulas = atual + 2 
            mapa[micromouse.getPosL() + 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = cl;
          }
          //Definindo as paredes
          //norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha + 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }
        else{
          //aplicando somatario das celulas
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se leste 2 celulas for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] != fi){
            //leste 2 celulas = atual + 2 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          atualizarVariaveis();
          //Definindo as paredes
          //norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
          //Definindo a nova posicao
          coluna = coluna + 2;
          micromouse.setPosC(coluna);
        }
        
      }else
      /////////////////////////////////////////////////////////////////
      //se a direcao for igual a sul
      if(micromouse.direcao == 3){
        //se sul 2 celulas for igual final
        if(mapa[micromouse.getPosL() + 2][micromouse.getPosC()] == fi){
          //aplicando somatorio dos celulas
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          //Definindo as paredes
          //leste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
          //Definindo a nova posicao
          linha = linha + 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }else
        //se oeste 2 celulas for igual final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] == fi){
          //aplicando somatorio dos celulas
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = cl;
          }
          //Definindo as paredes
          //leste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna - 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        } else
        if(oesteMenorValor()){
          //aplicando somatorio dos celulas
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se oeste 2 celulas for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] != fi){
            //oeste = atual + 2 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = cl;
          }
          //Definindo as paredes
          //leste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;          
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna - 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }
        else{
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se sul 2 celulas for diferente de final
          if(mapa[micromouse.getPosL() + 2][micromouse.getPosC()] != fi){
            //sul = atual + 2 
            mapa[micromouse.getPosL() + 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          //Definindo as paredes
          //leste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
          //Definindo a nova posicao
          linha = linha + 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }
        
      }else
      /////////////////////////////////////////////////////////////////
      //se direcao for igual a oeste
      if(micromouse.direcao == 4){
        //se oeste 2 celulas for igual a final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] == fi){
          //aplicando somatoria das celulas
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          //Definindo as paredes
          //sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
          //Definindo a nova posicao
          coluna = coluna - 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }
        else
        //se norte 2 celulas for igual a final
        if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] == fi){
          //aplicando somatoria das celulas
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = cl;
          }
          //Definindo as paredes
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;          
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha - 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }else
        if(norteMenorValor()){
          //aplicando somatoria das celulas
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se norte 2 celulas for diferente de final
          if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] != fi){
            //norte = atual + 2 
            mapa[micromouse.getPosL() - 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = cl;
          }
          //Definindo as paredes
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha - 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }
        else{
          //aplicando somatoria das celulas
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se oeste 2 celulas for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] != fi){
            //oeste = atual + 2 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          //Definindo as paredes
          //sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
          //Definindo a nova posicao
          coluna = coluna - 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }
      /////////////////////////////////////////////////////////////////
      }
      digitalWrite(led, LOW);
      atualizarVariaveis();
    } else
    //Fim frenteEDireitaLivre()

    /////////////////////////////////////////////////

    //Inicio frenteEEsquerdaLivre()
    if(frenteEEsquerdaLivre()){
      digitalWrite(led, HIGH);
      //se a direcao for norte
      if(micromouse.direcao == 1){
        //se norte 2 celulas for igual a final
        if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] == fi){
          //aplicando somatoria das celulas
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          //Definindo as paredes
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
          //Definindo a nova posicao
          linha = linha - 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        } else
        //se oeste 2 celulas for igual a final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] == fi){
          //aplicando somatoria das celulas
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = cl;
          }
          //Definindo as paredes
          //leste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna - 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }else
        if(oesteMenorValor()){
          //aplicando somatoria das celulas
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se oeste 2 celulas for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] != fi){
            //oeste = atual + 2 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = cl;
          }
          //Definindo as paredes
          //leste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna - 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }
        else{
          //aplicando somatoria das celulas
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se norte 2 celulas for diferente de final
          if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] != fi){
            //norte 2 celulas = atual + 2 
            mapa[micromouse.getPosL() - 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          //Definindo as paredes
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
          //Definindo a nova posicao
          linha = linha - 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }
      } else
      
      /////////////////////////////////////////////////////////////////
      //se a direcao for igual a leste
      if(micromouse.direcao == 2){
        //se leste 2 celulas for igual a final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] == fi){
          //aplicando somatoria das celulas
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          //Definindo as paredes
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
          //Definindo a nova posicao
          coluna = coluna + 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        } else
        //se norte 2 celulas for igual a final
        if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] == fi){
          //aplicando somatoria das celulas
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = cl;
          }
          //Definindo as paredes
          //sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha - 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }else
        if(norteMenorValor()){
          //aplicando somatoria das celulas
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se norte 2 celulas for diferente de final
          if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] != fi){
            //norte 2 celulas = atual + 2 
            mapa[micromouse.getPosL() - 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = cl;
          }
          //Definindo as paredes
          //sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha - 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }
        else{
          //aplicando somatoria das celulas
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se leste 2 celulas for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] != fi){
            //leste 2 celulas = atual + 2 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          //Definindo as paredes
          //sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
          //Definindo a nova posicao
          coluna = coluna + 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }
      } else
      
      /////////////////////////////////////////////////////////////////
      //se a direcao for sul
      if(micromouse.direcao == 3){
        //se sul 2 celulas for igual a final
        if(mapa[micromouse.getPosL() + 2][micromouse.getPosC()] == fi){
          //aplicando somatorio dos celulas
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          //Definindo as paredes
          //oeste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
          //Definindo a nova posicao
          linha = linha + 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        } else
        //se leste 2 celulas for igual a final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] == fi){
          //aplicando somatoria das celulas
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = cl;
          }
          //Definindo as paredes
          //oeste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna + 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }else
        if(lesteMenorValor()){
          //aplicando somatoria das celulas
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se leste 2 celulas for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] != fi){
            //leste = atual + 2 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = cl;
          }
          //Definindo as paredes
          //oeste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna + 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }
        else{
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se sul 2 celulas for diferente de final
          if(mapa[micromouse.getPosL() + 2][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 2][micromouse.getPosC()] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          //Definindo as paredes
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
          //Definindo a nova posicao
          linha = linha + 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }
      } else
      
      /////////////////////////////////////////////////////////////////
      //se direcao for igual a oeste
      if(micromouse.direcao == 4){
        //se oeste 2 celulas for igual a final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] == fi){
          //aplicando somatoria das celulas
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          //Definindo as paredes
          //norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
          //Definindo a nova posicao
          coluna = coluna - 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        } else
        //se sul for igual a final
        if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] == fi){
          //aplicando somatorio dos celulas
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = cl;
          }
          //Definindo as paredes
          //norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha + 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }else
        if(sulMenorValor()){
          //aplicando somatorio dos celulas
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se sul 2 celulas for diferente de final
          if(mapa[micromouse.getPosL() + 2][micromouse.getPosC()] != fi){
            //sul 2 celulas = atual + 2 
            mapa[micromouse.getPosL() + 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = cl;
          }
          //Definindo as paredes
          //norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha + 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }
        else{
          //aplicando somatoria das celulas
          //se oeste for diferente de final
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se oeste 2 celulas for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] != fi){
            //oeste 2 celulas = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 2] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          //Definindo as paredes
          //norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = pd;
          //Definindo a nova posicao
          coluna = coluna - 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }
      }
      digitalWrite(led, LOW);
      /////////////////////////////////////////////////////////////////
    }
    //Fim frenteEEsquerdaLivre()

    /////////////////////////////////////////////////
    else
    
    //Inicio direitaEEsquerdaLivre()
    if(direitaEEsquerdaLivre()){
      digitalWrite(led, HIGH);
      /////////////////////////////////////////////////////////////////
      
      if(micromouse.direcao == 1){
      //se leste 2 celulas for igual a final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] == fi){
          //aplicando somatoria das celulas
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //Definindo as paredes
          //norte = parede
          mapa[micromouse.getPosL() -1][micromouse.getPosC()] = pd;
          //norte e leste = parede
          mapa[micromouse.getPosL() -1][micromouse.getPosC() + 1] = pd;
          //norte e oeste = parede
          mapa[micromouse.getPosL() -1][micromouse.getPosC() - 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna + 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        } else
        //se oeste 2 celulas for igual a final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] == fi){
          //aplicando somatoria das celulas
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //Definindo as paredes
          //norte = parede
          mapa[micromouse.getPosL() -1][micromouse.getPosC()] = pd;
          //norte e leste = parede
          mapa[micromouse.getPosL() -1][micromouse.getPosC() + 1] = pd;
          //norte e oeste = parede
          mapa[micromouse.getPosL() -1][micromouse.getPosC() - 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna - 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        } else
        //se leste for o menor valor
        if(lesteMenorValor()){
          //Aplicando o somatario das celulas
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = posicao atual + 1
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se leste 2 celulas for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] != fi){
            //leste = posicao atual + 2
            mapa[micromouse.getPosL()][micromouse.getPosC() + 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //Definindo as paredes
          //norte = parede
          mapa[micromouse.getPosL() -1][micromouse.getPosC()] = pd;
          //norte e leste = parede
          mapa[micromouse.getPosL() -1][micromouse.getPosC() + 1] = pd;
          //norte e oeste = parede
          mapa[micromouse.getPosL() -1][micromouse.getPosC() - 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna + 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        } else{
          //aplicando somatoria das celulas
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se oeste 2 celulas for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] != fi){
            //oeste = atual + 2 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //Definindo as paredes
          //norte = parede
          mapa[micromouse.getPosL() -1][micromouse.getPosC()] = pd;
          //norte e leste = parede
          mapa[micromouse.getPosL() -1][micromouse.getPosC() + 1] = pd;
          //norte e oeste = parede
          mapa[micromouse.getPosL() -1][micromouse.getPosC() - 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna - 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }
        
      } else
      
       /////////////////////////////////////////////////////////////////
      //Se a direcao for leste
      if(micromouse.direcao == 2){
        //se norte 2 celulas for igual a final
        if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] == fi){
          //aplicando somatoria das celulas
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //Definindo as paredes
          //leste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
          //leste e norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC() + 1] = pd;
          //leste e sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() + 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha - 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        } else
        //se sul for igual a final
        if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] == fi){
          //aplicando somatorio dos celulas
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //Definindo as paredes
          //leste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
          //leste e norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC() + 1] = pd;
          //leste e sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() + 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha + 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        } else
        //Se sul for o menor valor
        if(sulMenorValor()){
          //aplicando somatorio dos celulas
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se sul 2 celulas for diferente de final
          if(mapa[micromouse.getPosL() + 2][micromouse.getPosC()] != fi){
            //sul 2 celulas = atual + 2 
            mapa[micromouse.getPosL() + 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //Definindo as paredes
          //leste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
          //leste e norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC() + 1] = pd;
          //leste e sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() + 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha + 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }
        else {
          //aplicando somatoria das celulas
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se norte 2 celulas for diferente de final
          if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] != fi){
            //norte 2 celulas = atual + 2 
            mapa[micromouse.getPosL() - 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //Definindo as paredes
          //leste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = pd;
          //leste e norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC() + 1] = pd;
          //leste e sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() + 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha - 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }
      }else

      /////////////////////////////////////////////////////////////////
      //Se a direcao for Sul
      if(micromouse.direcao == 3){
        //se leste 2 celulas for igual a final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] == fi){
          //aplicando somatoria das celulas
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //Definindo as paredes
          //sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
          //sul e leste = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() + 1] = pd;
          //sul e oeste = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() - 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna + 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        } else
        //se oeste 2 celulas for igual a final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] == fi){
          //aplicando somatoria das celulas
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //Definindo as paredes
          //sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
          //sul e leste = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() + 1] = pd;
          //sul e oeste = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() - 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna - 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        } else
        //se leste for o menor valor
        if(lesteMenorValor()){
          //Aplicando o somatario das celulas
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = posicao atual + 1
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se leste 2 celulas for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] != fi){
            //leste = posicao atual + 2
            mapa[micromouse.getPosL()][micromouse.getPosC() + 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //Definindo as paredes
          //sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
          //sul e leste = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() + 1] = pd;
          //sul e oeste = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() - 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna + 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        } else{
          //aplicando somatoria das celulas
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se oeste 2 celulas for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] != fi){
            //oeste = atual + 2 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //Definindo as paredes
          //sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = pd;
          //sul e leste = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() + 1] = pd;
          //sul e oeste = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() - 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna - 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }
      }else

      /////////////////////////////////////////////////////////////////
      //Se a direcao for Oeste
      if(micromouse.direcao == 4){
        //se norte 2 celulas for igual a final
        if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] == fi){
          //aplicando somatoria das celulas
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //Definindo as paredes
          //oeste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
          //oeste e norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC() - 1] = pd;
          //oeste e sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() - 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha - 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        } else
        //se sul for igual a final
        if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] == fi){
          //aplicando somatorio dos celulas
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //Definindo as paredes
          //oeste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
          //oeste e norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC() - 1] = pd;
          //oeste e sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() - 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha + 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        } else
        //Se sul for o menor valor
        if(sulMenorValor()){
          //aplicando somatorio dos celulas
          //se sul for diferente de final
          if(mapa[micromouse.getPosL() + 1][micromouse.getPosC()] != fi){
            //sul = atual + 1 
            mapa[micromouse.getPosL() + 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se sul 2 celulas for diferente de final
          if(mapa[micromouse.getPosL() + 2][micromouse.getPosC()] != fi){
            //sul 2 celulas = atual + 2 
            mapa[micromouse.getPosL() + 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //Definindo as paredes
          //oeste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
          //oeste e norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC() - 1] = pd;
          //oeste e sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() - 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha + 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }
        else {
          //aplicando somatoria das celulas
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se norte 2 celulas for diferente de final
          if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] != fi){
            //norte 2 celulas = atual + 2 
            mapa[micromouse.getPosL() - 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //Definindo as paredes
          //oeste = parede
          mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = pd;
          //oeste e norte = parede
          mapa[micromouse.getPosL() - 1][micromouse.getPosC() - 1] = pd;
          //oeste e sul = parede
          mapa[micromouse.getPosL() + 1][micromouse.getPosC() - 1] = pd;
          //Corrigir centro
          corrigirCentro();
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;  
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha + 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        }
      }
      digitalWrite(led, LOW);
      /////////////////////////////////////////////////////////////////
    } 
    //Fim direitaEEsquerdaLivre()

    /////////////////////////////////////////////////////////////////
    
    //Inicio frenteEDireitaEEsquerdaLivre()
    if(frenteEDireitaEEsquerdaLivre()){
      digitalWrite(led, HIGH);
      /////////////////////////////////////////////////////////////////
      //Se a direcao for Norte
      if(micromouse.direcao == 1){
        //se norte 2 celulas for igual a final
        if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] == fi){
          //aplicando somatoria das celulas
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = cl;
          }
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = cl;
          }
          //ir para frente
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha - 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        } else
        //se leste 2 celulas for igual a final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] == fi){
          //aplicando somatoria das celulas
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = cl;
          }
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = cl;
          }
          //Corrigir centro
          corrigirCentro();
          //ir para esquerda
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna + 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        } else
        //se oeste 2 celulas for igual a final
        if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] == fi){
          //aplicando somatoria das celulas
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = cl;
          }
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = cl;
          }
          //Corrigir centro
          corrigirCentro();
          //ir para direita
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna - 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        } else
        if(norteMenorValor()){
          //aplicando somatoria das celulas
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se norte 2 celulas for diferente de final
          if(mapa[micromouse.getPosL() - 2][micromouse.getPosC()] != fi){
            //norte 2 celulas = atual + 2 
            mapa[micromouse.getPosL() - 2][micromouse.getPosC()] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = cl;
          }
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = atual + 1 
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = cl;
          }
          //ir para esquerda
          virarParaEsquerda();
          micromouse.direcao--;
          andarUmaCelula2();
          //Definindo a nova posicao
          linha = linha - 2;
          micromouse.setPosL(linha);
          atualizarVariaveis();
        } else
        //se leste for o menor valor
        if(lesteMenorValor()){
          //Aplicando o somatario das celulas
          //se leste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 1] != fi){
            //leste = posicao atual + 1
            mapa[micromouse.getPosL()][micromouse.getPosC() + 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se leste 2 celulas for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() + 2] != fi){
            //leste = posicao atual + 2
            mapa[micromouse.getPosL()][micromouse.getPosC() + 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //se norte for diferente de final
          if(mapa[micromouse.getPosL() - 1][micromouse.getPosC()] != fi){
            //norte = posicao atual + 1
            mapa[micromouse.getPosL() - 1][micromouse.getPosC()] = cl;
          }
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = cl;
          }
          //ir para direita
          virarParaDireita();
          micromouse.direcao++;
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna + 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }else {
          //aplicando somatoria das celulas
          //se oeste for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 1] != fi){
            //oeste = atual + 1 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 1] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 1;
          }
          //se oeste 2 celulas for diferente de final
          if(mapa[micromouse.getPosL()][micromouse.getPosC() - 2] != fi){
            //oeste = atual + 2 
            mapa[micromouse.getPosL()][micromouse.getPosC() - 2] = mapa[micromouse.getPosL()][micromouse.getPosC()] + 2;
          }
          //Corrigir centro
          corrigirCentro();
          //ir para direita
          virarParaEsquerda();
          micromouse.direca--+; 
          andarUmaCelula2();
          //Definindo a nova posicao
          coluna = coluna - 2;
          micromouse.setPosC(coluna);
          atualizarVariaveis();
        }
      }else

      /////////////////////////////////////////////////////////////////
      //Se a direcao for Leste
      if(micromouse.direcao == 2){
        
      }else

      /////////////////////////////////////////////////////////////////
      //Se a direcao for Sul
      if(micromouse.direcao == 3){
        
      }else

      /////////////////////////////////////////////////////////////////
      //Se a direcao for Oeste
      if(micromouse.direcao == 4){
        
      }

      /////////////////////////////////////////////////////////////////
      digitalWrite(led, LOW);
    }
    //Fim frenteEDireitaEEsquerdaLivre()
    
    
    //Caso nenhuma das opcoes seja satisfeita
    else{
      desligarMotores();
    }

    imprimirLabirinto();
  }
  chegouDestino();
  digitalWrite(led2, HIGH);
  //corrigirCentro();
}


void setup() {
  
  attachInterrupt(digitalPinToInterrupt(3), contador, RISING);
  attachInterrupt(digitalPinToInterrupt(2), contador2, RISING);
  pulsos  = 0;
  rpm     = 0;
  rpm2     = 0;
  timeold = 0;
  timeold2 = 0;
  pinMode(led1, OUTPUT);
  digitalWrite(led1, HIGH);
  Serial.begin(9600);
  pinMode(pinMot1A, OUTPUT);
  pinMode(pinMot1B, OUTPUT);
  pinMode(pinMot2A, OUTPUT);
  pinMode(pinMot2B, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(led2, OUTPUT);
  digitalWrite(led, LOW);
  digitalWrite(led2, LOW);
  pinMode(trig_pin2OESTE, OUTPUT);
  pinMode(echo_pin2OESTE, INPUT);
  pinMode(trig_pin3LESTE, OUTPUT);
  pinMode(echo_pin3LESTE, INPUT);
  digitalWrite(trig_pin2OESTE, LOW);
  digitalWrite(trig_pin3LESTE, LOW);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);// MUDAR PARA "true" SE QUISER VISUALIZAR INFORMAÇÕES DE CALIBRAÇÃO NO MONITOR SERIAL
  
  #ifdef DEBUG
  Serial.println("Fim Setup");
  #endif  
  digitalWrite(led1, LOW);

  loop1 = true;
  //Serial.print(mapa[micromouse.getPosL()][micromouse.getPosC()]);

  micromouse.direcao = 1;
  mpu6050.update();
  InicioanguloZ = anguloZ;
  //andarUmaCelula2();
  //desenharQuadrado();
  
}

void loop() {
  
  /*  
   *   NÃO TEM PROBLEMA DEIXAR O VOID LOOP VAZIO!!!
   */
  //imprimirLeituraAnguloZ();
  //irParaFrente();
   floodFill();
  //irParaFrentePIDLateral();
  //corrigirCentro();
  //corrigirDirecao();
  //imprimirUltrassonico();
  //imprimirLeituraInfra();
  //imprimirEncoders();
}
