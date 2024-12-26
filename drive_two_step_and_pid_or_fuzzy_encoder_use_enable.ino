#include <AutoPID.h>
#include <Fuzzy.h>
#include<TimerOne.h>
#include<TimerThree.h>
#include<TimerFour.h>
#include<TimerFive.h>

Fuzzy *fuzzy = new Fuzzy();
// FuzzyInput THE
FuzzySet *NB1          = new FuzzySet(-1, -1, -0.1, -0.01);
FuzzySet *NS1          = new FuzzySet(-0.1, -0.01, 0, 0);
FuzzySet *Z1           = new FuzzySet(-0.01, 0, 0, 0.01);
FuzzySet *PS1          = new FuzzySet(0, 0, 0.01, 0.1);
FuzzySet *PB11          = new FuzzySet(0.01, 0.1, 1, 1);

// FuzzyInput THE_DOT
FuzzySet *NB2          = new FuzzySet(-1, -1, -0.3, -0.075);
FuzzySet *NS2          = new FuzzySet(-0.3, -0.075, 0, 0);
FuzzySet *Z2           = new FuzzySet(-0.075, 0, 0, 0.075);
FuzzySet *PS2          = new FuzzySet(0, 0, 0.075, 0.3);
FuzzySet *PB22          = new FuzzySet(0.075, 0.3, 1, 1);

// FuzzyOutput OUT
FuzzySet *NBO          = new FuzzySet(-0.67, -0.67, -0.67, -0.67);
FuzzySet *NSO          = new FuzzySet(-0.3, -0.3, -0.3, -0.3);
FuzzySet *ZO           = new FuzzySet(0, 0, 0, 0);
FuzzySet *PSO          = new FuzzySet(0.3, 0.3, 0.3, 0.3);
FuzzySet *PBO          = new FuzzySet(0.67, 0.67, 0.67, 0.67);


// 2 steps
#define DIR_1 11
#define STEP_1 10
#define DIR_2 13
#define STEP_2 12

//DC encoder
#define quay_duong 8
#define quay_am 9
#define outNL 7
#define encoder 4
#define OUTPUT_MIN -250
#define OUTPUT_MAX 250
//#define KP 1.7
//#define KI .04
//#define KD .8           // modify for optimal performance
//
//#define KP 1.62
//#define KI .08
//#define KD 1.05

#define KP 10.4
#define KI .024
#define KD 2.4

//#define KP 5.7
//#define KI .024
//#define KD 1.26


//PID
volatile int mode_PF=0;
volatile float theta0_present , theta0_target;
volatile int vitri_coder  , vitri_target ;
double input , outputPID , setpoint ;  
AutoPID myPID(&input, &setpoint, &outputPID, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

// information động cơ
const double vi_do = 1.8/16; 
const long full_step = 200*16;
//2step
volatile float theta1_target , theta1_change , theta1_present;
volatile float theta2_target , theta2_change , theta2_present;

volatile float delta_theta1,delta_theta2;
volatile int  sai_so_theta1 , sai_so_theta2 ; 

volatile float a1,b1,c1,d1;
volatile float a2,b2,c2,d2;

volatile double  time_change , time_dat;
volatile float time_xung_theta1 ,time_xung_theta2 ;

volatile int xung_dat_theta1 , xung_dem_theta1 ; 
volatile int xung_dat_theta2 , xung_dem_theta2 ; 

volatile bool mode_theta1 , mode_theta2;

//Fuzzy
unsigned long time1_fuzzy , time2_fuzzy , time_lay_mau , time_pause_fuzzy, time_run_fuzzy , time_duthua_fuzzy ;
volatile float giatoc_theta;
volatile int vitri_old , vitri_new ;
volatile float delta_old , delta_new , delta_dot , theta0_dot;
volatile float input1,input2,output;


//Read vitri encoder
void read_encoder()
{
if(digitalRead(encoder)== LOW)
     vitri_coder--;
else vitri_coder++;
//Serial.println(vitri_coder);
theta0_present=int(vitri_coder*1.1); //330
//theta0_present=int(vitri_coder*1.1077);
Serial.println(theta0_present);
}

//Function put energy
void pwmOut(int nangluong)
{  
    if(nangluong>0)
      {
        digitalWrite(quay_am,LOW);
        digitalWrite(quay_duong,HIGH);
        analogWrite(outNL,abs(nangluong));
       }
    if(nangluong<0)
      {
        digitalWrite(quay_am,HIGH);
        digitalWrite(quay_duong,LOW);
        analogWrite(outNL,abs(nangluong));
        }
     if(nangluong==0)
      {
        digitalWrite(quay_am,LOW );
        digitalWrite(quay_duong,LOW);
        analogWrite(outNL,abs(nangluong));
        }
}

void setup()
{
  //-------------------------------------
  Serial.begin(115200);
  pinMode(STEP_1, OUTPUT); //chân xuất xung pwm
  pinMode(DIR_1, OUTPUT); // chân đảo chiều động cơ 
  digitalWrite(STEP_1, LOW); // tạo f=0hz , cấm chạy
  pinMode(STEP_2, OUTPUT); //chân xuất xung pwm
  pinMode(DIR_2, OUTPUT); // chân đảo chiều động cơ 
  digitalWrite(STEP_2, LOW); // tạo f=0hz , cấm chạy
  pinMode(quay_duong, OUTPUT); 
  pinMode(quay_am, OUTPUT);
  pinMode(outNL, OUTPUT);
  
  theta1_present=90;
  theta2_present=-90;
  vitri_target=0;
  vitri_coder=0;
  mode_PF=0;
  time_duthua_fuzzy=0;
  //--------------------------------------
  //PID
  myPID.setTimeStep(100);

  pinMode(encoder, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
//  attachInterrupt (0, read_encoder, RISING);
attachInterrupt (0, read_encoder, FALLING);
//  Timer3.initialize(10000);
  Timer3.attachInterrupt(bo_dieu_khien_mix);
  Timer3.stop();
  Timer4.attachInterrupt(phat_xung_theta2);
  Timer4.stop();
  Timer5.attachInterrupt(phat_xung_theta1);
  Timer5.stop();
  Timer1.attachInterrupt(quy_hoach_theta1);
  Timer1.stop();

  //--------------------------------------

  // FuzzyInput
  FuzzyInput *THE = new FuzzyInput(1);

  THE->addFuzzySet(NB1);
  THE->addFuzzySet(NS1);
  THE->addFuzzySet(Z1);
  THE->addFuzzySet(PS1);
  THE->addFuzzySet(PB11);
  fuzzy->addFuzzyInput(THE);



  // FuzzyInput
  FuzzyInput *THE_DOT = new FuzzyInput(2);

  THE_DOT->addFuzzySet(NB2);
  THE_DOT->addFuzzySet(NS2);
  THE_DOT->addFuzzySet(Z2);
  THE_DOT->addFuzzySet(PS2);
  THE_DOT->addFuzzySet(PB22);
  fuzzy->addFuzzyInput(THE_DOT);



  // FuzzyOutput
  FuzzyOutput *OUT = new FuzzyOutput(1);

  OUT->addFuzzySet(NBO);
  OUT->addFuzzySet(NSO);
  OUT->addFuzzySet(ZO);
  OUT->addFuzzySet(PSO);
  OUT->addFuzzySet(PBO);
  fuzzy->addFuzzyOutput(OUT);
  //--------------------------------------




  // 25 RULE

    // Building FuzzyRule//////////////////////////////////////////////////////////////////1
    FuzzyRuleAntecedent *NB1_NB2 = new FuzzyRuleAntecedent();
    NB1_NB2->joinWithAND(NB1, NB2);
  
    FuzzyRuleConsequent *NBO1 = new FuzzyRuleConsequent();
    NBO1->addOutput(NBO);
  
    FuzzyRule *fuzzyRule1 = new FuzzyRule(1, NB1_NB2, NBO1);
    fuzzy->addFuzzyRule(fuzzyRule1);
  
    // Building FuzzyRule//////////////////////////////////////////////////////////////////2
    FuzzyRuleAntecedent *NB1_NS2 = new FuzzyRuleAntecedent();
    NB1_NS2->joinWithAND(NB1, NS2);
  
    FuzzyRuleConsequent *NBO2 = new FuzzyRuleConsequent();
    NBO2->addOutput(NBO);
  
    FuzzyRule *fuzzyRule2 = new FuzzyRule(2, NB1_NS2, NBO2);
    fuzzy->addFuzzyRule(fuzzyRule2);
  
    // Building FuzzyRule//////////////////////////////////////////////////////////////////3
    FuzzyRuleAntecedent *NB1_Z2 = new FuzzyRuleAntecedent();
    NB1_Z2->joinWithAND(NB1, Z2);
  
    FuzzyRuleConsequent *NSO3 = new FuzzyRuleConsequent();
    NSO3->addOutput(NSO);
  
    FuzzyRule *fuzzyRule3 = new FuzzyRule(3, NB1_Z2, NSO3);
    fuzzy->addFuzzyRule(fuzzyRule3);
  
    // Building FuzzyRule//////////////////////////////////////////////////////////////////4
//    FuzzyRuleAntecedent *NB1_PS2 = new FuzzyRuleAntecedent();
//    NB1_PS2->joinWithAND(NB1, PS2);
//  
//    FuzzyRuleConsequent *ZO4 = new FuzzyRuleConsequent();
//    ZO4->addOutput(ZO);
//  
//    FuzzyRule *fuzzyRule4 = new FuzzyRule(4, NB1_PS2, ZO4);
//    fuzzy->addFuzzyRule(fuzzyRule4);
  
    // Building FuzzyRule//////////////////////////////////////////////////////////////////5
    FuzzyRuleAntecedent *NB1_PB22 = new FuzzyRuleAntecedent();
    NB1_PB22->joinWithAND(NB1, PB22);
  
    FuzzyRuleConsequent *ZO5 = new FuzzyRuleConsequent();
    ZO5->addOutput(ZO);
  
    FuzzyRule *fuzzyRule5 = new FuzzyRule(5, NB1_PB22, ZO5);
    fuzzy->addFuzzyRule(fuzzyRule5);
  
    // Building FuzzyRule//////////////////////////////////////////////////////////////////6
    FuzzyRuleAntecedent *NS1_NB2 = new FuzzyRuleAntecedent();
    NS1_NB2->joinWithAND(NS1, NB2);
  
    FuzzyRuleConsequent *NBO6 = new FuzzyRuleConsequent();
    NBO6->addOutput(NBO);
  
    FuzzyRule *fuzzyRule6 = new FuzzyRule(6, NS1_NB2, NBO6);
    fuzzy->addFuzzyRule(fuzzyRule6);
  
    // Building FuzzyRule//////////////////////////////////////////////////////////////////7
//    FuzzyRuleAntecedent *NS1_NS2 = new FuzzyRuleAntecedent();
//    NS1_NS2->joinWithAND(NS1, NS2);
//  
//    FuzzyRuleConsequent *NBO7 = new FuzzyRuleConsequent();
//    NBO7->addOutput(NBO);
//  
//    FuzzyRule *fuzzyRule7 = new FuzzyRule(7, NS1_NS2, NBO7);
//    fuzzy->addFuzzyRule(fuzzyRule7);

  // Building FuzzyRule//////////////////////////////////////////////////////////////////8
  FuzzyRuleAntecedent *NS1_Z2 = new FuzzyRuleAntecedent();
  NS1_Z2->joinWithAND(NS1, Z2);

  FuzzyRuleConsequent *NSO8 = new FuzzyRuleConsequent();
  NSO8->addOutput(NSO);

  FuzzyRule *fuzzyRule8 = new FuzzyRule(8, NS1_Z2, NSO8);
  fuzzy->addFuzzyRule(fuzzyRule8);

  // Building FuzzyRule//////////////////////////////////////////////////////////////////9
  FuzzyRuleAntecedent *NS1_PS2 = new FuzzyRuleAntecedent();
  NS1_PS2->joinWithAND(NS1, PS2);

  FuzzyRuleConsequent *ZO9 = new FuzzyRuleConsequent();
  ZO9->addOutput(ZO);

  FuzzyRule *fuzzyRule9 = new FuzzyRule(9, NS1_PS2, ZO9);
  fuzzy->addFuzzyRule(fuzzyRule9);
//  
    // Building FuzzyRule//////////////////////////////////////////////////////////////////10
//    FuzzyRuleAntecedent *NS1_PB22 = new FuzzyRuleAntecedent();
//    NS1_PB22->joinWithAND(NS1, PB22);
//  
//    FuzzyRuleConsequent *PSO10 = new FuzzyRuleConsequent();
//    PSO10->addOutput(PSO);
//  
//    FuzzyRule *fuzzyRule10 = new FuzzyRule(10, NS1_PB22, PSO10);
//    fuzzy->addFuzzyRule(fuzzyRule10);

  // Building FuzzyRule//////////////////////////////////////////////////////////////////11
  FuzzyRuleAntecedent *Z1_NB2 = new FuzzyRuleAntecedent();
  Z1_NB2->joinWithAND(Z1, NB2);

  FuzzyRuleConsequent *NSO11 = new FuzzyRuleConsequent();
  NSO11->addOutput(NSO);

  FuzzyRule *fuzzyRule11 = new FuzzyRule(11, Z1_NB2, NSO11);
  fuzzy->addFuzzyRule(fuzzyRule11);

  // Building FuzzyRule//////////////////////////////////////////////////////////////////12
  FuzzyRuleAntecedent *Z1_NS2 = new FuzzyRuleAntecedent();
  Z1_NS2->joinWithAND(Z1, NS2);

  FuzzyRuleConsequent *NSO12 = new FuzzyRuleConsequent();
  NSO12->addOutput(NSO);

  FuzzyRule *fuzzyRule12 = new FuzzyRule(12, Z1_NS2, NSO12);
  fuzzy->addFuzzyRule(fuzzyRule12);

  // Building FuzzyRule//////////////////////////////////////////////////////////////////13
  FuzzyRuleAntecedent *Z1_Z2 = new FuzzyRuleAntecedent();
  Z1_Z2->joinWithAND(Z1, Z2);

  FuzzyRuleConsequent *ZO13 = new FuzzyRuleConsequent();
  ZO13->addOutput(ZO);

  FuzzyRule *fuzzyRule13 = new FuzzyRule(13, Z1_Z2, ZO13);
  fuzzy->addFuzzyRule(fuzzyRule13);

  // Building FuzzyRule//////////////////////////////////////////////////////////////////14
  FuzzyRuleAntecedent *Z1_PS2 = new FuzzyRuleAntecedent();
  Z1_PS2->joinWithAND(Z1, PS2);

  FuzzyRuleConsequent *PSO14 = new FuzzyRuleConsequent();
  PSO14->addOutput(PSO);

  FuzzyRule *fuzzyRule14 = new FuzzyRule(14, Z1_PS2, PSO14);
  fuzzy->addFuzzyRule(fuzzyRule14);

  // Building FuzzyRule//////////////////////////////////////////////////////////////////15
  FuzzyRuleAntecedent *Z1_PB22 = new FuzzyRuleAntecedent();
  Z1_PB22->joinWithAND(Z1, PB22);

  FuzzyRuleConsequent *PSO15 = new FuzzyRuleConsequent();
  PSO15->addOutput(PSO);

  FuzzyRule *fuzzyRule15 = new FuzzyRule(15, Z1_PB22, PSO15);
  fuzzy->addFuzzyRule(fuzzyRule15);

//  // Building FuzzyRule//////////////////////////////////////////////////////////////////16
//  FuzzyRuleAntecedent *PS1_NB2 = new FuzzyRuleAntecedent();
//  PS1_NB2->joinWithAND(PS1, NB2);
//
//  FuzzyRuleConsequent *NSO16 = new FuzzyRuleConsequent();
//  NSO16->addOutput(NSO);
//
//  FuzzyRule *fuzzyRule16 = new FuzzyRule(16, PS1_NB2, NSO16);
//  fuzzy->addFuzzyRule(fuzzyRule16);

  // Building FuzzyRule//////////////////////////////////////////////////////////////////17
  FuzzyRuleAntecedent *PS1_NS2 = new FuzzyRuleAntecedent();
  PS1_NS2->joinWithAND(PS1, NS2);

  FuzzyRuleConsequent *ZO17 = new FuzzyRuleConsequent();
  ZO17->addOutput(ZO);

  FuzzyRule *fuzzyRule17 = new FuzzyRule(17, PS1_NS2, ZO17);
  fuzzy->addFuzzyRule(fuzzyRule17);

  // Building FuzzyRule//////////////////////////////////////////////////////////////////18
  FuzzyRuleAntecedent *PS1_Z2 = new FuzzyRuleAntecedent();
  PS1_Z2->joinWithAND(PS1, Z2);

  FuzzyRuleConsequent *PSO18 = new FuzzyRuleConsequent();
  PSO18->addOutput(PSO);

  FuzzyRule *fuzzyRule18 = new FuzzyRule(18, PS1_Z2, PSO18);
  fuzzy->addFuzzyRule(fuzzyRule18);

//  // Building FuzzyRule//////////////////////////////////////////////////////////////////19
//  FuzzyRuleAntecedent *PS1_PS2 = new FuzzyRuleAntecedent();
//  PS1_PS2->joinWithAND(PS1, PS2);
//
//  FuzzyRuleConsequent *PBO19 = new FuzzyRuleConsequent();
//  PBO19->addOutput(PBO);
//
//  FuzzyRule *fuzzyRule19 = new FuzzyRule(19, PS1_PS2, PBO19);
//  fuzzy->addFuzzyRule(fuzzyRule19);

  // Building FuzzyRule//////////////////////////////////////////////////////////////////20
  FuzzyRuleAntecedent *PS1_PB22 = new FuzzyRuleAntecedent();
  PS1_PB22->joinWithAND(PS1, PB22);

  FuzzyRuleConsequent *PBO20 = new FuzzyRuleConsequent();
  PBO20->addOutput(PBO);

  FuzzyRule *fuzzyRule20 = new FuzzyRule(20, PS1_PB22, PBO20);
  fuzzy->addFuzzyRule(fuzzyRule20);

  // Building FuzzyRule//////////////////////////////////////////////////////////////////21
  FuzzyRuleAntecedent *PB11_NB2 = new FuzzyRuleAntecedent();
  PB11_NB2->joinWithAND(PB11, NB2);

  FuzzyRuleConsequent *ZO21 = new FuzzyRuleConsequent();
  ZO21->addOutput(ZO);

  FuzzyRule *fuzzyRule21 = new FuzzyRule(21, PB11_NB2, ZO21);
  fuzzy->addFuzzyRule(fuzzyRule21);

  //  // Building FuzzyRule//////////////////////////////////////////////////////////////////22
//    FuzzyRuleAntecedent *PB11_NS2 = new FuzzyRuleAntecedent();
//    PB11_NS2->joinWithAND(PB11, NS2);
//  
//    FuzzyRuleConsequent *ZO22 = new FuzzyRuleConsequent();
//    ZO22->addOutput(ZO);
//  
//    FuzzyRule *fuzzyRule22 = new FuzzyRule(22, PB11_NS2, ZO22);
//    fuzzy->addFuzzyRule(fuzzyRule22);

  // Building FuzzyRule//////////////////////////////////////////////////////////////////23
  FuzzyRuleAntecedent *PB11_Z2 = new FuzzyRuleAntecedent();
  PB11_Z2->joinWithAND(PB11, Z2);

  FuzzyRuleConsequent *PSO23 = new FuzzyRuleConsequent();
  PSO23->addOutput(PSO);

  FuzzyRule *fuzzyRule23 = new FuzzyRule(23, PB11_Z2, PSO23);
  fuzzy->addFuzzyRule(fuzzyRule23);

  // Building FuzzyRule//////////////////////////////////////////////////////////////////24
  FuzzyRuleAntecedent *PB11_PS2 = new FuzzyRuleAntecedent();
  PB11_PS2->joinWithAND(PB11, PS2);

  FuzzyRuleConsequent *PBO24 = new FuzzyRuleConsequent();
  PBO24->addOutput(PBO);

  FuzzyRule *fuzzyRule24 = new FuzzyRule(24, PB11_PS2, PBO24);
  fuzzy->addFuzzyRule(fuzzyRule24);

  // Building FuzzyRule//////////////////////////////////////////////////////////////////25
  FuzzyRuleAntecedent *PB11_PB22 = new FuzzyRuleAntecedent();
  PB11_PB22->joinWithAND(PB11, PB22);

  FuzzyRuleConsequent *PBO25 = new FuzzyRuleConsequent();
  PBO25->addOutput(PBO);

  FuzzyRule *fuzzyRule25 = new FuzzyRule(25, PB11_PB22, PBO25);
  fuzzy->addFuzzyRule(fuzzyRule25);
}
  

void serialEvent()
{
  if(Serial.available()>2)
  {
    theta0_target=Serial.parseFloat();
    theta1_target=Serial.parseFloat();
    theta2_target=Serial.parseFloat();
    time_dat = Serial.parseFloat();
    mode_PF  = Serial.parseFloat();

    vitri_target    = int(theta0_target*0.9167);        //330
    if(round(mode_PF)==1)
        Timer3.initialize(100000);
    else Timer3.stop();
    time_duthua_fuzzy=0;

    
        time_change=0;
        a1 = theta1_present;
        b1 = 0;
        c1 = 3*(theta1_target - theta1_present)/(time_dat*time_dat);
        d1 = -2*(theta1_target - theta1_present)/(time_dat*time_dat*time_dat);

        a2 = theta2_present;
        b2 = 0;
        c2 = 3*(theta2_target - theta2_present)/(time_dat*time_dat);
        d2 = -2*(theta2_target - theta2_present)/(time_dat*time_dat*time_dat);

        

        mode_theta1=true;
        mode_theta2=true;
        Timer1.initialize(100000);
    
  }
}


// hàm chọn chiều quay 
void forward(unsigned int chan_chieu)
{
  digitalWrite(chan_chieu, HIGH);
}

void reverse(unsigned int chan_chieu)
{
  digitalWrite(chan_chieu, LOW);
}

// hàm bù
void bu_theta1()
{
  sai_so_theta1= int((theta1_target - theta1_present)*80/9);
  // chọn chiều
  if (sai_so_theta1 >0)
  {
    forward(DIR_1);
    digitalWrite(STEP_1, HIGH);
    theta1_present+=vi_do;

    digitalWrite(STEP_1, LOW);
  }
  else if (sai_so_theta1 <0)
  {
    reverse(DIR_1);
    digitalWrite(STEP_1, HIGH);
    theta1_present-=vi_do;
    digitalWrite(STEP_1, LOW);
  }
  else
  {
  mode_theta1=false;
  digitalWrite(STEP_1, LOW);
  }
}

void bu_theta2()
{
  sai_so_theta2= int((theta2_target - theta2_present)*80/9);
  // chọn chiều
  if (sai_so_theta2 >0)
  {
    forward(DIR_2);
    digitalWrite(STEP_2, HIGH);
    theta2_present+=vi_do;

    digitalWrite(STEP_2, LOW);
  }
  else if (sai_so_theta2 <0)
  {
    reverse(DIR_2);
    digitalWrite(STEP_2, HIGH);
    theta2_present-=vi_do;
    digitalWrite(STEP_2, LOW);
  }
  else
  {
  mode_theta2=false;
  digitalWrite(STEP_2, LOW);
  }
}

//Quy hoạch
void quy_hoach_theta1()
{
  if (time_change > time_dat)
  {
//    time_duthua_fuzzy=0;
    time_dat =0;
    if ((mode_theta1==false)&(mode_theta2==false))
      Timer1.stop();
     else
     {
      bu_theta1(); 
      bu_theta2(); 
     }
  }
  else if ( time_change <= time_dat)
  {
    time_change += 0.1; 
    theta1_change = a1 + b1*time_change + c1*(time_change*time_change) + d1*(time_change*time_change*time_change);
    theta2_change = a2 + b2*time_change + c2*(time_change*time_change) + d2*(time_change*time_change*time_change);

    delta_theta1 = theta1_change-theta1_present;
    delta_theta2 = theta2_change-theta2_present;
    
    xung_dat_theta1=round(abs(delta_theta1)*full_step/360);
    xung_dat_theta2=round(abs(delta_theta2)*full_step/360);
    
    if(xung_dat_theta1!=0)
    {
      //chọn chiều quay
      if(delta_theta1>0) forward(DIR_1);
      else reverse(DIR_1);
      
      time_xung_theta1 = round(100000/xung_dat_theta1);
      xung_dem_theta1=0;
      Timer5.initialize(time_xung_theta1);
    }

        if(xung_dat_theta2!=0)
    {
      //chọn chiều quay
      if(delta_theta2>0) forward(DIR_2);
      else reverse(DIR_2);
      
      time_xung_theta2 = round(100000/xung_dat_theta2);
      xung_dem_theta2=0;
      Timer4.initialize(time_xung_theta2);
    }
 
  }
}



//Phát xung
void phat_xung_theta1()
{
  if(xung_dem_theta1 >= xung_dat_theta1)
  {
    Timer5.stop();
  }
  else
  {
    digitalWrite(STEP_1, HIGH);
    xung_dem_theta1++;
    
    if(delta_theta1 > 0)
      theta1_present += vi_do;
    else 
      theta1_present -= vi_do;
    digitalWrite(STEP_1, LOW);
  }
}

void phat_xung_theta2()
{
  if(xung_dem_theta2 >= xung_dat_theta2)
  {
    Timer4.stop();
  }
  else
  {
    digitalWrite(STEP_2, HIGH);
    xung_dem_theta2++;
    
    if(delta_theta2 > 0)
      theta2_present += vi_do;
    else 
      theta2_present -= vi_do;
    digitalWrite(STEP_2, LOW);
  }
}

//PID
void bo_dieu_khien_mix()
{
  input=vitri_coder;
  setpoint=vitri_target;
  myPID.run();
  pwmOut(outputPID); 
}

void loop() {
    if(round(mode_PF)==1)
        Timer3.initialize(10000);
    else Timer3.stop();
       Serial.print(theta0_present);
       Serial.print(",");
       Serial.print(theta1_present);
       Serial.print(",");
       Serial.print(theta2_present);
       Serial.print(",");
       Serial.println(time_change);

       if(mode_PF==int(2))
       {
          vitri_coder=vitri_coder;
        time_lay_mau=millis();
        while((unsigned long)(millis()-time_lay_mau) <10)
        {
      time1_fuzzy=millis();
      
    delta_old= float(theta0_target)-float(vitri_coder)*1.1;
    vitri_old=vitri_coder;
    delay(1);
      time2_fuzzy=millis();
    delta_new= float(theta0_target)-float(vitri_coder)*1.1;
    vitri_new = vitri_coder;
    delta_dot=(delta_new-delta_old)/(time2_fuzzy-time1_fuzzy);
    theta0_dot=(vitri_new-vitri_old)*1.1/(time2_fuzzy-time1_fuzzy);
    giatoc_theta=2*c1*((millis()-time_duthua_fuzzy)/1000.0) + 3*d1*pow((millis()-time_duthua_fuzzy)/1000.0,2);

          input1=delta_new-theta0_dot;
          input2=delta_dot-giatoc_theta;
          if(input1>=180)  input1=180;
          if(input1<=-180) input1=-180;
          if(input2>=180)  input2=180;
          if(input2<=-180) input2=-180;
          input1=input1/180;
          input2=input2/1500;
          
          fuzzy->setInput(1, input1);
          fuzzy->setInput(2, input2);
          fuzzy->fuzzify();
          output = fuzzy->defuzzify(1);  
          output = output*500;
          pwmOut(output);
        }
       }
       
       if(round(mode_PF)!=2) delay(35);
       if(round(mode_PF)==2) {
        time_pause_fuzzy=millis();
        delay(30);
      time_run_fuzzy=millis();
      time_duthua_fuzzy=time_duthua_fuzzy+(time_run_fuzzy-time_pause_fuzzy);
       }
     
}
