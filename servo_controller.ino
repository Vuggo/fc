
#include <Wire.h>
#include <Servo.h>

Servo elevator;
Servo ail;

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, time, timePrev;
int i;
int Control;
int Value;

float elangle;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float P_control=0;
float I_control=0;
float D_control=0;

/////////////////PID CONSTANTS/////////////////
//these values represent the degree to which our output is affected by each value, higher = more weight

double kp=1.2; //3.55
double ki=.004;//0.003
double kd=.1;//2.05
///////////////////////////////////////////////

double Servoang_0 =90; //initial value of servo
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady
void setup() {
  // put your setup code here, to run once:
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  elevator.attach(10);   //attach elevator servo to pin 9 (** Check if true **)
  ail.write(11);
  //sensor.attach(9)

  time = millis();      //Start Counting time in milliseconds

  pinMode(8,INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  /////////////////////////////I M U/////////////////////////////////////

    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; // Divided by 1000 to convert into seconds

  /*The tiemStep is the time that elapsed since the previous loop.
   * This is the value that we will use in the formulas as "elapsedTime"
   * in seconds. We work in ms so we haveto divide the value by 1000
   to obtain seconds*/

  /*Reed the values that the accelerometre gives.
   * We know that the slave adress for this IMU is 0x68 in
   * hexadecimal. For that in the RequestFrom and the
   * begin functions we have to put this value.*/

     Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true);

   /*We have asked for the 0x3B register. The IMU will send a brust of register.
    * The amount of register to read is specify in the requestFrom function.
    * In this case we request 6 registers. Each value of acceleration is made out of
    * two 8bits registers, low values and high values. For that we request the 6 of them
    * and just make then sum of each pair. For that we shift to the left the high values
    * register (<<) and make an or (|) operation to add the low values.*/

     Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();
     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;

   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers

   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   Gyr_rawY=Wire.read()<<8|Wire.read();
   
   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0;
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;

   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];

   Value = pulseIn(8,LOW,9600);

   /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
   //Serial.println(Total_angle[0]);


   elangle = (Total_angle[0] - 90) ;


   Serial.println(Total_angle[0]);

   Serial.print(" ");

   //Serial.println(time);


   previous_error = error;

   error = desired_angle - elangle;

   P_control =  kp * error;

   D_control = kd*((error - previous_error)/elapsedTime);


   //I_control = kd * error;

   //D_control = kd * error;

   Control = P_control + I_control + D_control;


   //Serial.println(elangle);
   elevator.write(Control);
   ail.write(Control);

}
