/*
PID速度跟隨加入
雙馬達 個別PID獨立控制 測試速度同步收斂情況
*/


#include "variant.h"
#include <due_can.h>
#include "PID_v1.h"
#include "DueTimer.h"



//***************************基本馬達操控腳位*********
//AB交換
#define DIR2 11
#define DIR1 8
#define A2 12
#define B2 13
#define A1 10
#define B1 9


//DAC 0 link to 2
//DAC 1 link to 1
//****************************************************

//***************************速度讀取用***************
unsigned long old1;
unsigned long old2;
unsigned long old3;
unsigned long old4;

long int M1_counts=0;
long int M2_counts=0;
int M1_speed;
int M2_speed;

long int M1_lastcounts;					//未來考慮溢位
long int M2_lastcounts;					//未來考慮溢位
int samplef=200;
double M1_Filter1,M1_Filter2,M1_Filter1_p,M1_Filter2_p;
double M2_Filter1, M2_Filter2, M2_Filter1_p, M2_Filter2_p;
float M1_Outputspeed;
float M2_Outputspeed;
//****************************************************

//***************************PID參數調整用************
double M1_Kp=3.5,M1_Ki=2.5,M1_Kd=0.6;
//double M1_Kp=6.5,M1_Ki=3,M1_Kd=1;
double M2_Kp = 3.5, M2_Ki = 2.5, M2_Kd = 0.6;
//double M2_Kp = 4, M2_Ki = 1.6, M2_Kd = 0.8;
double ST=100;

double P_gain,I_gain,D_gain;
String receive_string;
String msg;
boolean income=false;
//****************************************************

//***************************PID宣告******************
double M1_Setpoint, M1_Input, M1_Output;
PID M1_PID(&M1_Input, &M1_Output, &M1_Setpoint, M1_Kp, M1_Ki, M1_Kd, DIRECT);

double M2_Setpoint, M2_Input, M2_Output;
PID M2_PID(&M2_Input, &M2_Output, &M2_Setpoint, M2_Kp, M2_Ki, M2_Kd, DIRECT);
//****************************************************

//***************************其他*********************
//int i;
int M1_Vcommand=0;
int M2_Vcommand = 0;
byte command=0;
//****************************************************

void setup()
{
	
  analogWriteResolution(12);
  pinMode(DIR1, OUTPUT);      
  pinMode(DIR2, OUTPUT); 
  pinMode(A1, INPUT);      
  pinMode(B1, INPUT);    
  pinMode(A2, INPUT);     
  pinMode(B2, INPUT);    
  
  digitalWrite(DIR1,LOW);
  digitalWrite(DIR2,HIGH);
 // digitalWrite(DIR1,HIGH);
 // digitalWrite(DIR2,LOW);
  
  delay(1000);
  attachInterrupt(A1,interA1, CHANGE);      //外部中斷設定 腳位A1設定 正負緣觸發  
  attachInterrupt(B1,interB1, CHANGE);      //同上
  attachInterrupt(A2,interA2, CHANGE);      
  attachInterrupt(B2,interB2, CHANGE); 

  Serial.begin(57600);
  Serial3.begin(57600);
  Timer2.attachInterrupt(speedreading).setFrequency(samplef).start();   //數度讀取程式設定頻率    
  Timer3.attachInterrupt(compute).setFrequency(200).start(); 
 // Timer4.attachInterrupt(sending).setFrequency(50).start(); 
  Timer5.attachInterrupt(Filtering).setFrequency(150).start(); 
  M1_PID.SetMode(AUTOMATIC);                                        //設定PID模式
  M1_PID.SetSampleTime(50);                                        //設定PID取樣時間
  M2_PID.SetMode(AUTOMATIC);                                        //設定PID模式
  M2_PID.SetSampleTime(50);     
 // M1_PID.SetControllerDirection(DIRECT);
 // M1_PID.SetTunings(17.0, 45.0, 1.0, 0.0, 1.2);
                                                    //設定延時避免驅動器進操控保護
}


void loop(){ 

      delay(10);                             //若無延遲timer計時器會失效

      Serial3.print("M1_Kp：");  
      Serial3.print(M1_Kp);
      Serial3.print("  ");  
      Serial3.print("M1_Ki：");  
      Serial3.print(M1_Ki);
      Serial3.print("  ");  
      Serial3.print("M1_Kd：");  
      Serial3.print(M1_Kd);
      Serial3.print("  ");  
      Serial3.print("COMMAND："); 
      Serial3.println(M1_Vcommand);

      Serial.print(ST);               //輸出繪圖用
      //Serial.print(" ");
     // Serial.print(M2_Outputspeed);
     // Serial.print(M1_counts);
      Serial.print(" ");
      Serial.print(M1_Outputspeed);
     //Serial.print(M1_speed);
      Serial.print(" ");
      Serial.print(M2_Outputspeed);
      //Serial.print(M1_speed);
      Serial.print(" ");
      //Serial.print(M2_speed*(-1));
      Serial.println(" ");
          
      if (income==true)Set_parameters();      //接收旗標推動新參數設定機制

}

/*
void DIRset()
{
  if(M1_speed!=0)
  {
     digitalWrite(DIR1,LOW);
     digitalWrite(DIR2,HIGH);
    
    }
  
  }*/

void compute()
{
		//共用ST setpoint

      M1_Input=M1_Outputspeed;               //輸入PID速度
      M1_Setpoint = ST;                         //PID運算之到位點
      M1_PID.Compute();                         //執行PID運算                               
	    M1_Vcommand =(M1_Output/0.27)+1425;      //取出PID運算值，單位同化漸進線擬和
      M1_Vcommand=constrain(M1_Vcommand,0,4000);
      analogWrite(DAC1, M1_Vcommand);
      //analogWrite(DAC1, 1750);           //命令送出
     
  	  M2_Input = M2_Outputspeed;               
  	  M2_Setpoint = ST;                      
  	  M2_PID.Compute();                                            
  	  M2_Vcommand = (M2_Output / 0.27) + 1425;    
      M2_Vcommand=constrain(M2_Vcommand,0,4000);
  	  analogWrite(DAC0, M2_Vcommand);     
      //analogWrite(DAC0, 1750);           
      

  }

void Set_parameters()                              //挑出與轉換參數資料
{
  if (receive_string.charAt(0)=='P')               //辨識P字元  若string第0為P
  {
    receive_string.setCharAt(0,'0');               //清除陣列辨識字元
    P_gain=((float)receive_string.toInt())/100;    //轉換陣列為int後轉為float後除100
    receive_string="";                             //清除接收陣列
    M1_Kp=P_gain;                                     //寫入參數值
    M2_Kp=P_gain;
    }
  if (receive_string.charAt(0)=='I')
  {
    receive_string.setCharAt(0,'0');
    I_gain=((float)receive_string.toInt())/100;
    receive_string="";
    M1_Ki=I_gain;
    M2_Ki=I_gain;
    }
  if (receive_string.charAt(0)=='D')
  {
    receive_string.setCharAt(0,'0');
    D_gain=((float)receive_string.toInt())/100;
    receive_string="";
    M1_Kd=D_gain;
    M2_Kd=D_gain;
    }
    if (receive_string.charAt(0)=='S')
  {
    receive_string.setCharAt(0,'0');
    ST=((float)receive_string.toInt());
    receive_string="";

    }

    M1_PID.SetTunings(M1_Kp,M1_Ki,M1_Kd);         //寫入新PID值
    M2_PID.SetTunings(M2_Kp,M2_Ki,M2_Kd);
    income=false;                       //接收旗標歸位
  }

void serialEvent3()
{
    msg = Serial3.readString();    //接收新PID參數
    receive_string=msg;          //字串陣列轉存
    msg="";                      //接收字串清除
    income=true;                 //接收資料旗標
  }
void Filtering()
{
   /************************************************M1****************************************************/
  
  float M1_inspeed=M1_speed;                        //單位速度*取樣率/常數(避免範圍過大)，得真實速度
  float FOLF1=exp(-3.14*1/75);                              //一階濾波擬合常數 自然指數^(-3.14*頻寬/一半取樣率)
  M1_Filter1 = FOLF1*M1_Filter1_p+(1-FOLF1)*M1_inspeed;        //一階濾波輸出=濾波常數*上一次輸出 + (1-濾波常數)*輸入值
  M1_Filter1_p=M1_Filter1;
  
  double FOLF2=exp(-3.14*1/75);
  M1_Filter2 = FOLF2*M1_Filter2_p+(1-FOLF2)*M1_Filter1;     //疊代第二次一階濾波器
  M1_Filter2_p=M1_Filter2;
  M1_Outputspeed=M1_Filter2;                                //輸出值

  /************************************************M2****************************************************/

  float M2_inspeed = M2_speed;                
  //float FOLF3 = exp(-3.14 * 1 / 75);         
  M2_Filter1 = FOLF1*M2_Filter1_p + (1 - FOLF1)*M2_inspeed;     
  M2_Filter1_p = M2_Filter1;

  //double FOLF4 = exp(-3.14 * 1 / 75);
  M2_Filter2 = FOLF2*M2_Filter2_p + (1 - FOLF2)*M2_Filter1;       
  M2_Filter2_p = M2_Filter2;
  M2_Outputspeed = M2_Filter2;                             //輸出值

  
  }

void speedreading()
{

  M1_speed=M1_counts-M1_lastcounts;                 //M1取得速度 單位時間count 
  M1_lastcounts=M1_counts;
//  M1_speed=M1_speed*samplef/20;
  M1_speed=M1_speed*samplef/10;

  M2_speed = M2_counts - M2_lastcounts;             //M2取得速度 單位時間count 
  M2_lastcounts = M2_counts;
//  M2_speed=M2_speed*samplef/20;
  M2_speed=M2_speed*samplef/10;
 
  }
  
void interA1()                   //encoder1 A相
{
  unsigned long T=micros();
  byte lastB;
  if ((old1-T)>200&&lastB!=digitalRead(B1))
//  if (lastB!=digitalRead(B1))
  {
  if(digitalRead(A1)==1&&digitalRead(B1)==1)
  M1_counts=M1_counts-1;
  if(digitalRead(A1)==0&&digitalRead(B1)==0)
  M1_counts=M1_counts-1;
  
  if(digitalRead(A1)==0&&digitalRead(B1)==1)
  M1_counts=M1_counts+1;
  if(digitalRead(A1)==1&&digitalRead(B1)==0)
  M1_counts=M1_counts+1;
}
old1=micros();
lastB=digitalRead(B1);
}

void interB1()                   //encoder1 B相
{
  unsigned long T=micros();
  byte lastA;
  if ((old2-T)>200&&lastA!=digitalRead(A1))
//  if (lastA!=digitalRead(A1))
  {
  if(digitalRead(A1)==1&&digitalRead(B1)==1)
  M1_counts=M1_counts+1;
  if(digitalRead(A1)==0&&digitalRead(B1)==0)
  M1_counts=M1_counts+1;
  
  if(digitalRead(A1)==0&&digitalRead(B1)==1)
  M1_counts=M1_counts-1;
  if(digitalRead(A1)==1&&digitalRead(B1)==0)
  M1_counts=M1_counts-1;
}
old2=micros();
lastA=digitalRead(A1);
}

void interA2()                   //encoder2 A相
{
  unsigned long T=micros();
  byte lastB;
  if ((old3-T)>200&&lastB!=digitalRead(B2))
  {
  if(digitalRead(A2)==1&&digitalRead(B2)==1)
  M2_counts=M2_counts-1;
  if(digitalRead(A2)==0&&digitalRead(B2)==0)
  M2_counts=M2_counts-1;
  
  if(digitalRead(A2)==0&&digitalRead(B2)==1)
  M2_counts=M2_counts+1;
  if(digitalRead(A2)==1&&digitalRead(B2)==0)
  M2_counts=M2_counts+1;
  }
  old1=micros();
  lastB=digitalRead(B2);
}
void interB2()                   //encoder2 B相
{
  unsigned long T=micros();
  byte lastA;
  if ((old4-T)>200&&lastA!=digitalRead(A2))
  {
  if(digitalRead(A2)==1&&digitalRead(B2)==1)
  M2_counts=M2_counts+1;
  if(digitalRead(A2)==0&&digitalRead(B2)==0)
  M2_counts=M2_counts+1;
  
  if(digitalRead(A2)==0&&digitalRead(B2)==1)
  M2_counts=M2_counts-1;
  if(digitalRead(A2)==1&&digitalRead(B2)==0)
  M2_counts=M2_counts-1;
  }
  old4=micros();
  lastA=digitalRead(A2);
}
