#define MAX_ENC (8192)
#define INTERVAL (1000)


void handler_tim4(void){
 
 }








//pini
// A0 - ENC A
// A1 - ENC B

//ccmr1/2 : 68 - pwm(+); 48 - force inactive(-)
//ccer : F - activ(+/-); A - inactiv (Z)

//trapez                                      -Z+     -+Z     Z+-     +Z-     +-Z     Z-+
//uint16_t COM_CCER  [COMMUTATION_LENGTH] = {0x0FAF, 0x0FFA, 0x0AFF, 0x0FAF, 0x0FFA, 0x0AFF};
//uint16_t COM_CCMR1 [COMMUTATION_LENGTH] = {0x6868, 0x6868, 0x6848, 0x6848, 0x4868, 0x4868};
//uint16_t COM_CCMR2 [COMMUTATION_LENGTH] = {0x0048, 0x0048, 0x0068, 0x0068, 0x0068, 0x0068};
//uint8_t tabel_magic[6] = {4,0,5,2,3,1};
//volatile uint8_t offset = 4;  //4 - forward; 1 -  pt cealalta directie
uint8_t tabel_magic_fw[6] = {0,4,5,2,1,3};
uint8_t tabel_magic_rv[6] = {3,1,2,5,4,0};




// pentru pid cascada (motor samsung CSMT-04BB1ANT3) //8 mag 2000 enc
float para[7] = {0    //para[0] - Kp - cuplu
                ,0    //para[1] - Ki - cuplu (x20000 ts)
                ,5.85    //para[2] - Kp - viteza
                ,1.59      //para[3] - Ki - viteza (x1000 ts)
                ,0.1    //para[4] - Kp - pozitie
                ,0.0    //para[5] - Kd - pozitie
                ,0.5     // para[6] - Kv - constanta BEMF motor
                };


                

float lim[3]  = {350   //lim[0] - limita pwm (hw 1000)
                ,600   //lim[1] - limita cuplu (adc*16)
                ,100   //lim[2] - limita viteza (enc/ms)
                };                

uint8_t mod = 0;
volatile float innput = 0;
volatile uint16_t trimis = 0;


float serv=150;




uint8_t old_sec = 6;
volatile uint32_t sumasus = 0;
volatile uint8_t curentN = 0;
volatile uint16_t curentM = 0;
int previousMillis = 0;
int previousMillisC = 0;

volatile uint16_t ad1,ad2,ad3,ad4;


volatile uint32_t nrcomut = 0;
uint32_t rpm=0;
uint16_t putere_out = 0;
uint32_t limita_crt = 300;

float integrala=0;
float serv2=0;

char received = 0;
volatile uint16_t putere = 0;

// the setup function runs once when you press reset or power the board
void setup() {

Serial2.begin(56700);
  digitalWrite(PB12,1); //pullup
  GPIOA->regs->CRH = 0x44444A3A;//alternate function
  GPIOA->regs->CRL = 0x4A444444;
  GPIOB->regs->CRH = 0x3AA84444;
  GPIOB->regs->CRL = 0x88444444;

  pinMode(PA5, INPUT_PULLUP);

  pinMode(PA7, INPUT_PULLUP);


  digitalWrite(PB13,1);  //output low
  digitalWrite(PB14,1);
  digitalWrite(PB15,1);
  digitalWrite(PA8,1);
  digitalWrite(PA9,1);
  digitalWrite(PA10,1);



  RCC_BASE->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
  delay(1);
  RCC_BASE->APB1RSTR = 0;  

  //Build_Commutation_Table();



  //init tim enc

  (TIMER2->regs).gen->CCMR1 = TIMER_CCMR1_CC2S_INPUT_TI1  | TIMER_CCMR1_CC1S_INPUT_TI1 | (0x9<<12) | (0x9<<0);  //filtre
  (TIMER2->regs).gen->SMCR = TIMER_SMCR_SMS_ENCODER3;
  (TIMER2->regs).gen->CCMR2 = TIMER_CCMR2_CC3S_INPUT_TI3;  //captura
  (TIMER2->regs).gen->CCER  = TIMER_CCER_CC3E | TIMER_CCER_CC3P; //captura

  //(TIMER2->regs).gen->PSC  = 3;  //prescaler

  
  (TIMER2->regs).gen->CR1 = TIMER_CR1_CEN;   
  (TIMER2->regs).gen->CNT = 0x0000;  
  //(TIMER2->regs).gen->ARR = MAX_ENC-1;

  //init timer1 motor 
  (TIMER1->regs).gen->ARR = 1050;
  (TIMER1->regs).gen->CNT = 0;
  (TIMER1->regs).gen->CCR1 = 10;
  (TIMER1->regs).gen->CCR2 = 10;
  (TIMER1->regs).gen->CCR3 = 10;

  (TIMER1->regs).gen->CR1 = TIMER_CR1_CKD_CMS_CENTER3; //(1<<CMS1) | (1<<CMS0);
  (TIMER1->regs).gen->CR2 = TIMER_CR2_MMS_UPDATE;
  //CCMR1, CCMR2 si CCER se initializeaza mai incolo
  (TIMER1->regs).adv->BDTR = TIMER_BDTR_MOE | 0x0f; //(1<<MOE)+ 16 DT;
  (TIMER1->regs).gen->CR1 |= TIMER_CR1_CEN;//(1 << CEN);
  (TIMER1->regs).gen->EGR = TIMER_EGR_UG;//(1<<UG);

  (TIMER1->regs).gen->CCMR1 = 0x0068;
  (TIMER1->regs).gen->CCMR2 = 0;
  (TIMER1->regs).gen->CCER  = 0x0001;

  (TIMER1->regs).adv->CCR1 = (TIMER1->regs).adv->CCR2 = (TIMER1->regs).adv->CCR3 = 0;

//servo timer
  (TIMER3->regs).gen->PSC=71;
  (TIMER3->regs).gen->ARR = 20000;
  (TIMER3->regs).gen->CNT = 0;
  (TIMER3->regs).gen->CCR1 = 10;
  (TIMER3->regs).gen->CCR2 = 10;
  (TIMER3->regs).gen->CCR3 = 10;

  (TIMER3->regs).gen->CR2 = TIMER_CR2_MMS_UPDATE;
  //CCMR1, CCMR2 si CCER se initializeaza mai incolo
  (TIMER3->regs).gen->CR1 |= TIMER_CR1_CEN;//(1 << CEN);
  (TIMER3->regs).gen->EGR = TIMER_EGR_UG;//(1<<UG);

  (TIMER3->regs).gen->CCMR1 = 0x0068;
  (TIMER3->regs).gen->CCMR2 = 0;
  (TIMER3->regs).gen->CCER  = 0x0001;

  (TIMER3->regs).gen->CCR1 = 1500;
  
 (TIMER4->regs).gen->PSC=72;
 (TIMER4->regs).gen->ARR=20000;
 (TIMER4->regs).gen->CNT=0;
 (TIMER4->regs).gen->DIER=0x0001;
 (TIMER4->regs).gen->EGR=0x0001;
 (TIMER4->regs).gen->SR=0;
 timer_attach_interrupt(TIMER4, 0, handler_tim4);
 (TIMER4->regs).gen->CR1 |= 0x00000084;
pinMode(PA10,INPUT);
//Serial2.begin(57600);
Serial.begin(57600);
  pinMode(PC13,OUTPUT);  
  pinMode(PB0,OUTPUT);
  //digitalWrite(PC13, 1);

}

uint8_t sector=0;

uint8_t primit=0;


uint32_t lastPoz = 0,crtPoz;
int16_t crtViteza;
int32_t crtPozitie = 0;

float viteza_c;
int32_t poz_c;

uint8_t pozViteza=0;
volatile int32_t sumaViteza=0;
int16_t sirViteze[18]={0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0};

void encoder1(void){
  crtPoz = 0xffff - (TIMER2->regs).gen->CNT;
  crtViteza = (int16_t)(crtPoz - lastPoz);
  crtPozitie += crtViteza;


  /*//viteza acumulata pe ultimele 16
  sumaViteza -= sirViteze[pozViteza];
  sirViteze[pozViteza] = crtViteza;
  sumaViteza += crtViteza;
  if (++pozViteza > 15) pozViteza = 0;*/

  viteza_c = crtViteza;
  poz_c = crtPozitie;
  lastPoz = crtPoz; 
}




void stop_intersectie(void){
  mod=0;
  innput=0;
  delay(3000);
  innput=60;
  delay(500);
  }

uint16_t curent;
uint8_t curentF=0;

int16_t val_adc[256] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile float curent_cerut=500;
//volatile int32_t sumasus=0;
//uint8_t curentN=0;
float pidout=0;
float itemp=0,ptemp,etemp;

void adc_int(void) {
  ADC1->regs->SR = 0;
  if (!(((TIMER1->regs).gen->CR1) & (1<<4))) {
  /*
    // masoara curent
    sumasus -= val_adc[curentN];
    val_adc[curentN] = (ADC1->regs->JDR1+ADC1->regs->JDR1+ADC1->regs->JDR1+ADC1->regs->JDR1)/4;
    sumasus += val_adc[curentN];
    if (++curentN>16) curentN=0;

    etemp=curent_cerut-sumasus;
    itemp+=0.01*(etemp);
    itemp=constrain(itemp,0,1000);
    pidout=0.002*(etemp)+itemp;
    pidout=constrain(pidout,0,1000);
    
    (TIMER1->regs).adv->CCR1 = (TIMER1->regs).adv->CCR2 = (TIMER1->regs).adv->CCR3 = pidout;
    
  */


   curentF++;
  }
  //digitalWrite(PC13, 0);
}



float integ,input;

uint8_t citeste(void) {
  uint8_t index;
  if (Serial.available()) {
    char c=Serial.read();
    switch (c){
    case 'i':
      innput=Serial.parseFloat();    
      while (Serial.available()) Serial.read(); //Serial.flush();
      break;
    case 'm':
      index=Serial.read()-uint8_t('0');
      mod = (index<=3)?index:mod;
      crtPozitie = 0;  //reset pozitie ax
      while (Serial.available()) Serial.read(); //Serial.flush(); 
      break;      
    case 'p':
      index = Serial.read() - uint8_t('0');
      para[index]=Serial.parseFloat();    
      while (Serial.available()) Serial.read(); //Serial.flush(); 
      break;
    case 'l':
      index = Serial.read() - uint8_t('0');
      lim[index]=Serial.parseFloat();    
      while (Serial.available()) Serial.read(); //Serial.flush(); 
      break;
      case 's':
      //index = Serial.read();
      serv=Serial.parseFloat();    
      while (Serial.available()) Serial.read(); //Serial.flush();
      break; 
    default:
      while (Serial.available()) Serial.read(); //Serial.flush();      
    }
    return 1;        
  }
  else {
    return 0;
  }
}
uint8_t citeste2(void) {
  uint8_t index;
  if (Serial2.available()) {

    char c=Serial2.read();
    switch (c){
    case 'i':   //input
      innput=Serial2.parseFloat();    
      while (Serial2.available()) Serial2.read(); //Serial.flush();
      break;
    case 'm':  //mod motor
      index=Serial2.read()-uint8_t('0');
      mod = (index<=3)?index:mod;
      crtPozitie = 0;  //reset pozitie ax
      while (Serial2.available()) Serial2.read(); //Serial.flush(); 
      break;      
    case 'p':    //paramaetrii
      index = Serial2.read() - uint8_t('0');
      para[index]=Serial2.parseFloat();    
      while (Serial2.available()) Serial2.read(); //Serial.flush(); 
      break;
    case 'l':  //limitari
      index = Serial2.read() - uint8_t('0');
      lim[index]=Serial2.parseFloat();    
      while (Serial2.available()) Serial2.read(); //Serial.flush(); 
      break;
      case 's'://servo
      serv=(Serial2.parseFloat());   
    //  if(serv<(-560)) serv=230;
      break;
      while (Serial2.available()) Serial2.read(); //Serial.flush(); 
     case 'o'://stop
     innput=0;
      (TIMER4->regs).gen->CR1 |= 0x0001;
    // delay(3000);
     //innput=25;
     while (Serial2.available()) Serial2.read(); //Serial.flush();
      default:
      while (Serial2.available()) Serial2.read(); //Serial.flush();      
    }
    return 1;        
  }
  else {
    return 0;
  }
}

void scrie(void){
  Serial.write(255); Serial.write(255);
  Serial.print("mod= "); Serial.println(mod);
  Serial.print("input= "); Serial.println(innput);
  for (uint8_t i=0;i<7; i++){
      Serial.print("para["); Serial.print(i);  Serial.print("]= "); Serial.println(para[i],6);
  }
  for (uint8_t i=0;i<3; i++){
      Serial.print("lim["); Serial.print(i);  Serial.print("]= "); Serial.println(lim[i]);
  }
  Serial.print("servo");Serial.print(serv);
  Serial.println(" ");
  Serial.write(255); Serial.write(254);
}

float olderr=0;


void loop() {

   
if (citeste()) scrie();


citeste2();
  //bucla viteza
  
  if (micros() - previousMillis > INTERVAL) {
    previousMillis = micros();
    encoder1();
    
    input = innput;
    float err;   



    //bucla pozitie
    //input = (input-poz_c)*para[4]-viteza_c*para[5];            //kp pozitie (p4)
    input = (input-poz_c)*para[4]-float(sumaViteza)*para[5];            //kp pozitie (p4)
    
    
    //bucla viteza
    if (mod==2) 
      input = innput;
    input = constrain(input,-lim[2],lim[2]);        //limitare viteza (l2)
    err = (input - viteza_c);                       //coeficient viteza curenta 
    //err = (input - float(sumaViteza));                       //coeficient viteza curenta 
    integ += para[3]*err;                                   //ki viteza (p3)
    integ = constrain(integ,-lim[1],lim[1]);               //limitare integrala viteza = limita cuplu (l1)
    input = err*para[2] + integ;                          //kp viteza (p2)


    
    
    //bucla curent
    if (mod==1)
    
      input=innput;

    input = constrain(input,-lim[1],lim[1]); //limitare curent (l1)

    pidout = input+float(viteza_c)*para[6];
    //pidout = input+float(sumaViteza)*para[6];

    //fara bucla
    
    pidout = (mod==0)?innput:pidout;

    
    pidout=constrain(pidout,-lim[0],lim[0]);  //de modificat cu dublu sens    

    if (pidout>=0) {
      digitalWrite(PA9,1);
      digitalWrite(PB15,0);
    } else {
      pidout=-pidout;
      digitalWrite(PA9,0);
      digitalWrite(PB15,1);
    }
    //if(serv<(-550)) serv=250;
    serv=constrain(serv,-200,200);//    
    serv2=map(serv,-200,200,700,1500);
    (TIMER3->regs).gen->CCR1 = serv2;
    (TIMER1->regs).adv->CCR1 = (TIMER1->regs).adv->CCR2 = (TIMER1->regs).adv->CCR3 = abs(pidout);
    if (mod==3) trimis=poz_c;
    if (mod==2) trimis=abs(crtViteza);
    //if (mod==2) trimis=abs(sumaViteza);
    if (mod==0) trimis=crtViteza+1000;//crtPoz%MAX_ENC;//sector*100;//crtViteza+1000;
    //if (mod==0) trimis=sumaViteza+1000;
    if (mod==1) trimis=(pidout)+1000;

    //trimis=(TIMER2->regs).gen->CCR3%MAX_ENC;

    
 /*   for usb debug grafic deplhi
    Serial.write(((trimis>>7) & 0x7f)+0x80);
    Serial.write(((trimis) & 0x7f)+0x00);*/

  }    
    

  
}
void timer4_handler(void)
{
  innput=30;
}

