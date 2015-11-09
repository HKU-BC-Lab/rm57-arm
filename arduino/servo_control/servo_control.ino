#include <Servo.h>
#define test 1
const double pi=3.1415926;

Servo servo[6];
float q[6];              //solved joints
float currentPos[6];     //current Cartisian position after interpolation
char form=1;               //choosing interpolation method. 1 stands for linear, and 2 stands for cubic.
int  currentIndex=0;     //current startframe Index
int  i=0,k;
unsigned long tic,toc;           // current time in ms
float t;
float duration;    // durtion between succeeding via_frames;
float reltime;
float r,p,y;
float via_breaks[11]={0,12,24,36,48,50,62,74,86,98,120};  // designated time breaks
float via_frames[6][11]={{ 0.14 ,    0.140,     0.1400,     0.1400,     0.1400,    0.1400,    0.1400,     0.1400,     0.1400,    0.1400  ,    0.1400   },
                          { 0,        0.11,     0.11 ,       0      ,   -0.11  ,    -0.11  ,     0 ,       0.11 ,    0.11  ,     0  ,  -0.11000  },
                          {0.1,      0.1 ,    0.15,         0.15 ,       0.15   ,     0.1,     0.1,       0.1      ,0.15,       0.15 ,     0.15},
                          {   0,      0     ,    0,          0 ,         0,           0,        0 ,         0  ,        0  ,     0  ,     0 },
                       //   {-pi/2 ,    -pi/2  , -pi/2  ,     -pi/2  ,    -pi/2 ,     -pi/2     ,-pi/2  ,  -pi/2  ,      -pi/2  , -pi/2  , -pi/2  },
                          {pi/2 ,    pi/2  , pi/2  ,     pi/2  ,    pi/2 ,     pi/2     ,pi/2  ,  pi/2  ,      pi/2  , pi/2  , pi/2  },
                          {   0,       0     ,   0,            0 ,       0,           0,        0 ,        0  ,        0  ,     0  ,     0  }};
                         // designated frames at the corresponding time breaks

void writeJoints(float q0,float q1,float q2,float q3,float q4,float q5);                          //write the 6 angless values to the servo motor
void solveJoints(float x,float y,float z,float roll,float pitch, float yaw);
void getCurrentPos(char form);      //calculate the current frame using interpolation;
void rotmat2rpy(float *R[3]);
void testJoint(int num);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  for(i=0;i<6;i++)
    servo[i].attach(i+8);      //servos attached to pin 8 9 10 11 12 13

  t = millis();
  currentIndex=0;
  duration = via_breaks[currentIndex+1]-via_breaks[currentIndex];

}

void loop() {
  // put your main code here, to run repeatedly:
   t = (float)millis()/1000;
#ifdef test
   Serial.println(t);
#endif
   if (currentIndex<11 && via_breaks[currentIndex+1]<=t){
         currentIndex = currentIndex+1;
         duration = via_breaks[currentIndex+1]-via_breaks[currentIndex];
     }

    form=1;
    getCurrentPos(form);
#ifdef test
     Serial.print("Pos[0]=");Serial.println(currentPos[0]);
      Serial.print("Pos[1]=");Serial.println(currentPos[1]);
       Serial.print("Pos[2]=");Serial.println(currentPos[2]);
        Serial.print("Pos[3]=");Serial.println(currentPos[3]);
         Serial.print("Pos[4]=");Serial.println(currentPos[4]);
          Serial.print("Pos[5]=");Serial.println(currentPos[5]);
 #endif
  //       tic=millis();
  //     for(k=1;k<1000;k++)
    solveJoints(currentPos[0],currentPos[1],currentPos[2],currentPos[3],currentPos[4],currentPos[5]);
//    toc=millis();
//   Serial.print("1000 times InverseKinematics takes ");Serial.print(toc-tic);Serial.println("ms");
   writeJoints(q[0],q[1],q[2],q[3],q[4],q[5]);

//  for(i=5;i>=0;i--)
//    testJoint(i);

   Serial.println(millis());

}



void solveJoints(float x,float y,float z,float roll,float pitch, float yaw) {
    double r11,r12,r13,r21,r22,r23,r31,r32,r33,cos_r,sin_r,cos_p,sin_p,cos_y,sin_y;
    double c0,s0,c1,c2,c3,c4,c5,c6,s1,s2,s3,s4,s5,s6,c123,s123,c12,s12,t0,tmp;
    float th0,th1,th2,th3,th4,th5,th12,th123;
    double a=0.078,b=0.105,m,n;
    cos_r=cos(roll);
    sin_r=sin(roll);
    cos_p=cos(pitch);
    sin_p=sin(pitch);
    cos_y=cos(yaw);
    sin_y=sin(yaw);

    if (abs(cos_p)<0.0000000001)
        cos_p = cos_p + 0.000000001;

    r11=cos_y*cos_p;
    r12=cos_y*sin_p*sin_r-sin_y*cos_r;
    r13=cos_y*sin_p*cos_r+sin_y*sin_r;
    r21=sin_y*cos_p;
    r22=sin_y*sin_p*sin_r+cos_y*cos_r;
    r23=sin_y*sin_p*cos_r-cos_y*sin_r;
    r31=-sin_p;
    r32=cos_p*sin_r;
    r33=cos_p*cos_r;

    t0=y/x;
    tmp=sqrt(x*x+y*y);
    s0=y/tmp;
    c0=x/tmp;
    th0=atan2(s0,c0);

    s4=r13*s0-r23*c0;
    th4=asin(s4);
    c4=cos(th4);;

    c5=(r22*c0-r12*s0)/c4;
    s5=(r21*c0-r11*s0)/c4;
    th5=atan2(s5,c5);

    s123=(r13*c0+r23*s0)/c4;
    c123=r33/c4;
    th123=atan2(s123,c123);
    if (!s123 && c123<0) th123=pi;



    m =x*c0+y*s0-0.075*s123-0.01;
    n =(z-0.075*c123);
    c2=(m*m+n*n-a*a-b*b)/(2*a*b);
    th2=acos(c2);
    s2=sin(th2);

    s1=(m*a*c2-n*a*s2+m*b)/(m*m+n*n);
    th1=asin(s1);
    c1=cos(th1);

    th3=th123-th1-th2;

       q[0]=th0;
       q[1]=th1;
       q[2]=th2;
       q[3]=th3;
       q[4]=th4;
       q[5]=th5;

 #ifdef test
    Serial.print("s2= ");Serial.println(s2);
    Serial.print("c2= ");Serial.println(c2);
    Serial.print("th2= ");Serial.println(th2);

    Serial.print("s1= ");Serial.println(s1);
    Serial.print("c1= ");Serial.println(c1);
    Serial.print("th1= ");Serial.println(th1);

     Serial.print("m= ");Serial.println(m);
     Serial.print("n= ");Serial.println(n);

    Serial.print("s123= ");Serial.println(s123);
    Serial.print("c123= ");Serial.println(c123);
    Serial.print("th123= ");Serial.println(th123);

    Serial.print("s4= ");Serial.println(s4);
    Serial.print("c4= ");Serial.println(c4);
    Serial.print("th4= ");Serial.println(th4);
#endif


}

void writeJoints(float q0,float q1,float q2,float q3,float q4,float q5)
{
    if (q0<=pi/2 && q0>=-pi/2)
        { servo[0].write(q0/pi*180+90);
        #ifdef test
        Serial.print("q0=");Serial.print(q0);Serial.print(" rad, and is written as ");Serial.println(q0/pi*180+90);
        #endif
        }
    else {
        Serial.print("q0 is out of range, q0= ");
        Serial.println(q0);}

    if (q1<=pi/2 && q1>=-pi/2)
        {servo[1].write(90-q1/pi*180);
        #ifdef test
        Serial.print(" q1=");Serial.print(q1);Serial.print(" rad and is written as ");Serial.println(90-q1/pi*180);
        #endif
        }
    else {
        Serial.print("q1 is out of range, q1= ");
        Serial.println(q1);}

    if (q2<=pi && q2>=0)
        {servo[2].write(90+q2/pi*180);
        #ifdef test
        Serial.print(" q2=");Serial.print(q2);Serial.print(" rad and is written as ");Serial.println(90+q2/pi*180);
        #endif
        }
    else {
        Serial.print("q2 is out of range, q2= ");
        Serial.println(q2);}

    if (q3<=pi && q3>=-pi)
        {servo[3].write(q3/pi*180+90);
        #ifdef test
        Serial.print("  q3=");Serial.print(q3);Serial.print(" rad and is written as ");Serial.println(q3/pi*180+90);
        #endif
        }
    else {
        Serial.print("q3 is out of range, q3= ");
        Serial.println(q3);}

    if (q4<=pi && q4>=-pi)
        {servo[4].write(q4/pi*180+90);
        #ifdef test
        Serial.print("   q4=");Serial.print(q4);Serial.print(" rad and is written as ");Serial.println(q4/pi*180+90);
        #endif
        }
    else {
        Serial.print("q4 is out of range, q4= ");
        Serial.println(q4);}

    if (q5<=pi && q5>=-pi)
        {servo[5].write(q5/pi*180+90);
        #ifdef test
        Serial.print("    q5=");Serial.print(q5);Serial.print(" rad and is written as ");Serial.println(q5/pi*180+90);
        #endif
        }
    else {
        Serial.print("q5 is out of range, q5= ");
        Serial.println(q5);}
}

void getCurrentPos(char form){
  double a0,a1,a2,a3;

  reltime = t-via_breaks[currentIndex];
#ifdef test
  Serial.print("reltime=");Serial.println(reltime);
   Serial.print("currentIndex=");Serial.println(currentIndex);
    Serial.print("duration=");Serial.println(duration);
#endif
  if (form==1)
     for(i=0;i<6;i++)
        currentPos[i]= (via_breaks[currentIndex+1]-t)/duration*via_frames[i][currentIndex] +  reltime/duration*via_frames[i][currentIndex+1];
  if (form==2)
     for(i=0;i<6;i++){
      a0=via_frames[i][currentIndex];
      a1=0;
      a2=3.0/(duration*duration)*(via_frames[i][currentIndex+1]-via_frames[i][currentIndex]);
      a3=-2.0/(duration*duration*duration)*(via_frames[i][currentIndex+1]-via_frames[i][currentIndex]);
      currentPos[i]=a3*reltime*reltime*reltime+a2*reltime*reltime+a0;
#ifdef test
      Serial.print("currentPos[");Serial.print(i);Serial.print("]=");Serial.println(currentPos[i]);
#endif
     }

}

void rotmat2rpy(float *R[3]){
  r =atan2(R[2][1],R[2][2]);
  p=  atan2(-R[2][0],sqrt(R[2][1]*R[2][1] + R[2][2]*R[2][2]));
  y=  atan2(R[1][0],R[0][0]);
}


void testJoint(int num){
      float tmp[11]={0,-pi/4,-pi/2,-pi/4,0,pi/4,pi/2,pi/4,0,-pi/4,-pi/2};

      int j=0;
      if (num==0)
           for(j=0;j<11;j++){
            writeJoints(tmp[j],0,0,0,0,0);
            delay(1000);
           }
       else if (num==1)
           for(j=0;j<11;j++){
            writeJoints(0,tmp[j],-tmp[j],0,0,0);
            delay(1000);
           }
           else if (num==2)
           for(j=0;j<11;j++){
            writeJoints(0,0,tmp[j]+pi/2,0,0,0);
            delay(1000);
           }
           else if (num==3)
           for(j=0;j<11;j++){
            writeJoints(0,0,0,tmp[j],0,0);
            delay(1000);
           }
           else if (num==4)
           for(j=0;j<11;j++){
            writeJoints(0,0,0,0,tmp[j],0);
            delay(1000);
           }
           else if (num==5)
           for(j=0;j<11;j++){
            writeJoints(0,0,0,0,0,tmp[j]);
            delay(1000);
           }

}

