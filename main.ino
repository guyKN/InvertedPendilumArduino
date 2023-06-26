#include <Encoder.h>
#include "ArduPID.h"

#include <ArduinoQueue.h>

double sign(double a){
  if (a>0) return 1;
  else if (a<0) return -1;
  else return 0;
}

int motorPwmPin = 9;
int motorControl1Pin = 7;
int motorControl2Pin = 8;

int blackPin1 = 2;
int whitePin1 = 10;
int orangePin1 = 11;

int blackPin2 = 3;
int whitePin2 = 12;
int orangePin2 = 13;

int channelAPin1 = blackPin1;
int channelBPin1 = whitePin1;
int channelZPin1 = orangePin1;

int channelAPin2 = blackPin2;
int channelBPin2 = whitePin2;
int channelZPin2 = orangePin2;

static double PULLEY_DIAMETER = 0.012; // [m]
static double ENCODER_TICKS_PER_ROTATION = 1200;
static double CART_TICK_DISTANCE = PULLEY_DIAMETER * PI / ENCODER_TICKS_PER_ROTATION * (-1);
static double PENDILUM_TICK_ANGLE = 2 * PI / ENCODER_TICKS_PER_ROTATION;

bool isCalibrating = false;

static double TRACK_WIDTH = 1.5;
static double CART_WIDTH = 0.16;
static double EDGE_MIN_DISTANCE_LEFT =   isCalibrating? 0.22 : 0.2; 
static double EDGE_MIN_DISTANCE_RIGHT =  isCalibrating? 0.1 : 0.2;


// the minimum distance we want to be from the edge. If we're closer to the edge than this, then stop immidiatly. 

static double X_STOP_LEFT = -(TRACK_WIDTH/2 - CART_WIDTH/2 - EDGE_MIN_DISTANCE_LEFT);
static double X_STOP_RIGHT = TRACK_WIDTH/2 - CART_WIDTH/2 - EDGE_MIN_DISTANCE_RIGHT;
static double START_X = isCalibrating? 0.5 : 0;
static double END_T = 30000000;


static double BEGIN_SPEED = 0.15;

// how often we print to the serial monitor
static bool PRINT_CSV = true;
static double PRINT_FREQ = 100;
static double PRINT_T = 1/PRINT_FREQ;
static double CONTROL_FREQ = 100;
static double CONTROL_T = 1./CONTROL_FREQ;

static double CLAMPVAL = 0.25;
static double PWRMAXDELTA = 100*0.1;

static double STARTLQRANGLE = 10;
static double STOPLQRANGLE = 30;

static double MAXVOLTAGE = 20;

double dir_change_brk_t = 0.05;

bool enableReset = true;
double idleTime = 0.5;

double k_v = 0.12;
double f_static = 0.07;

double a_factor = 0.05;

double alf = 5, bet = -11;
double M=0.65, m=0.1562, g=9.81, l=0.56, d=0.015, width=0.046;
double k=l/2-d;
double I = 1.0/12.0 * m * (width*width + l*l);
double E0 = m*g*k;
double omega = sqrt((m*g*k)/(m*k*k + I));

// double Kphi = -64.1523 ;
// double Kphidot = -12.1865;
// double Kx = 31.6228;
// double Kxdot = 19.0424;

// double Kphi = -37.5406  ; // d==1
// double Kphidot = -7.0867 ;
// double Kx = 10.0000;
// double Kxdot = 7.7731;

// double Kphi = -37.2404  ; // d==2
// double Kphidot = -6.9504 ;
// double Kx = 10.0000;
// double Kxdot = 7.7337;

// double Kphi = -36.9711  ; // d==3
// double Kphidot = -6.8282 ;
// double Kx = 10.0000;
// double Kxdot = 7.6981;

// double Kphi = -36.7394  ; // d==4
// double Kphidot = -6.7228  ;
// double Kx = 10.0000;
// double Kxdot = 7.6673;

// double Kphi = -36.5540   ; // d==5
// double Kphidot = -6.6377 ;
// double Kx = 10.0000;
// double Kxdot = 7.6426;

// double Kphi = -36.4265   ; // d==6
// double Kphidot = -6.5775 ;
// double Kx = 10.0000;
// double Kxdot = 7.6256;

double Kphi = -44.8761   ; // L == l
double Kphidot = -10.5699 ;
double Kx = 10.0000;
double Kxdot = 8.6820;

// double Kphi = -41.7748   ; // L == l5/6
// double Kphidot = -9.0467   ;
// double Kx = 10.0000;
// double Kxdot = 8.3099;

// double Kphi = -39.2009   ; // L == l4/6
// double Kphidot = -7.8409   ;
// double Kx = 10.0000;
// double Kxdot = 7.9879;

// double Kphi = -37.5539   ; // L == l1/2
// double Kphidot = -7.0927;
// double Kx = 10.0000;
// double Kxdot = 7.7749;

bool isLQR = false;
double swingupt0 = 0;
double swingupOmega = omega;

double prev_t = 0;
double prev_pwr = 0;
double prev_x = START_X;
double prev_phi = 0;
double prev_phi_norm = 0;

double t_prev_update = 0;
double x_prev_update = START_X;
double phi_prev_update = 0;

enum state {LQR, swingup, returning, idle, beginRight, beginLeft, calibrating, calibrating_static, cosine_check, waitingForLQR};
state startState = isCalibrating ? calibrating : waitingForLQR;
state currState = startState;
double continue_t = 0;
double reset_t = 0;
long start_t = 0;

double exp2pwr = 0.1;

Encoder cartEncoder(channelAPin1, channelBPin1);
Encoder pendilumEncoder(channelAPin2, channelBPin2);

// PID
double pid_setpoint = 0;
double pid_input = 0;
double pid_output = 0;

double pid_k_p = 5; // d==1
// double pid_k_i = 0;
double pid_k_i = 0.003;
double pid_k_d = 0;
double pid_bias = 1;

// double pid_k_p = 6.5; // d==6
// // double pid_k_i = 0;
// double pid_k_i = 0.006;
// double pid_k_d = 0;
// double pid_bias = 1.5;

ArduPID swingUpPID;

// calibration
double calibrateBasePower = 0.1;
double calibratePowerDelta = 1./255.;
int currCalibrationIterations = 0;

bool swingupStarted = false;
bool expLQRStarted = false;

double expStartLQRAngle = 7 * DEG_TO_RAD;
// moving average

int movingAvgSteps = 3;

ArduinoQueue<double> phiVals(movingAvgSteps);
ArduinoQueue<double> tVals(movingAvgSteps);


void setup() { 
  // fill the queue with zeroes
  for (int i = 0;  i < movingAvgSteps; i++){
    phiVals.enqueue(0);
    tVals.enqueue(-0.1);
  }

  // motor    
  Serial.begin(250000);
  pinMode(motorPwmPin, OUTPUT);
  pinMode(motorControl1Pin, OUTPUT);
  pinMode(motorControl2Pin, OUTPUT);

  digitalWrite(motorControl1Pin, 0);
  digitalWrite(motorControl2Pin, 0);
  digitalWrite(motorPwmPin, 0);

  pinMode(channelZPin1, INPUT_PULLUP);
  pinMode(channelZPin2, INPUT_PULLUP);

  //delay(1000);
  start_t = millis();

  // initialize PID
  swingUpPID.begin(&pid_input, &pid_output, &pid_setpoint, pid_k_p, pid_k_i, pid_k_d);
  //swingUpPID.setOutputLimits(-0.3, 0.3);
  swingUpPID.setBias(pid_bias);
  swingUpPID.start();
}

void setPower(int power){
  power = constrain(power, -255, 255);
  // Serial.println(power);
  if(power >= 0){
      digitalWrite(motorControl1Pin, 1);
      digitalWrite(motorControl2Pin, 0);
      analogWrite(motorPwmPin, power);
  }else{
      digitalWrite(motorControl1Pin, 0);
      digitalWrite(motorControl2Pin, 1);
      analogWrite(motorPwmPin, -power);
  }
}

void setPowerF(double power){
  int powerInt = power * 255;
  setPower(powerInt);
}

void motorFloat(){
      digitalWrite(motorControl1Pin, 1);
      digitalWrite(motorControl2Pin, 1);
      analogWrite(motorPwmPin, 0);
}

void motorBreak(){
      digitalWrite(motorControl1Pin, 0);
      digitalWrite(motorControl2Pin, 0);
      analogWrite(motorPwmPin, 0);
}

double E(double angle, double angledot){
  return -m*g*k*cos(angle) + 0.5*(m*k*k + I)*angledot*angledot;
}

double GetU(double a, double v){
  double alpha, mu, mustatic, mukinetic, gamma;
  if (v >= 0){
    alpha = 1.56;
    mustatic = alpha * 1.84;
    mukinetic = 1.02; 
    gamma = 2.58;
  }
  else{
    alpha = 1.74; 
    mukinetic = 1.25;
    mustatic = alpha * 1.45;
    gamma = 2.59;
  }
  if (abs(v) < 0.05)
    mu = mustatic;
  else
    mu = mukinetic;
  return (a + gamma*v + mu*sign(v))/alpha/MAXVOLTAGE;
}

double LQRPower(double x, double phi, double phi_norm,  double phi_from_vert, double xdot, double phidot, double dt){
  double a;
  a = (Kphi*phi_from_vert + Kphidot*phidot + Kx*x + Kxdot*xdot)/(M);
  
  double v = xdot + a*dt;
  //double power = constrain( constrain(f_static*sign(v) + k_v * v + a*a_factor, -CLAMPVAL, CLAMPVAL), prev_pwr - PWRMAXDELTA, prev_pwr + PWRMAXDELTA);
  double power = constrain( constrain( GetU(a, v), -CLAMPVAL, CLAMPVAL), prev_pwr - PWRMAXDELTA, prev_pwr + PWRMAXDELTA);
  return power;
}

double swingupPower(double t, double x, double phi, double phi_norm, double phi_from_vert, double xdot, double phidot, double dt){
  double E_curr = E(phi, phidot);
  double E_diff = -(E0 - E_curr);

  double phi0 = acos(constrain(-E_curr/(m*g*k), -1, 1));
  if (phi0==0){
    swingUpPID.reset();
    return 0;
  }
  double phiratio = constrain(phi_norm / phi0, -1, 1);

  pid_input = E_diff;
  swingUpPID.compute();
  double swingUpAmplitude = pid_output;



  double a = -sign(phidot)*sqrt(1-(phiratio*phiratio)) * swingUpAmplitude;

  return a/12;
  
  double v = xdot + a*dt;
  // double power = constrain( constrain(f_static*sign(v) + k_v * v + a*a_factor, -CLAMPVAL, CLAMPVAL), prev_pwr - PWRMAXDELTA, prev_pwr + PWRMAXDELTA);
  double power = constrain( constrain( GetU(a, v), -CLAMPVAL, CLAMPVAL), prev_pwr - PWRMAXDELTA, prev_pwr + PWRMAXDELTA);
  return power;
  //return constrain(a, -0.2, 0.2);
}

void Reset(){
  prev_t = 0;
  prev_pwr = 0;
  //prev_phi = 0;
  //pendilumEncoder.write(0);

  t_prev_update = 0;
  phi_prev_update = 0;

  swingUpPID.reset();

  expLQRStarted = false;
  currState = startState;
  continue_t = 0;
  reset_t = 0;
  start_t = millis();
}

void loop() {
  double t = (millis() - start_t) * 0.001;
  double x = cartEncoder.read() * CART_TICK_DISTANCE + START_X;
  double phi = pendilumEncoder.read() * PENDILUM_TICK_ANGLE;
  double phi_norm = fmod(fmod(fmod(phi, 2*PI) + 2*PI, 2*PI) + PI, 2*PI) - PI;

  if (t - prev_t < CONTROL_T) return;

  double dt = t - prev_t;
  
  double phi_from_vert = fmod(fmod(phi, 2*PI) + 2*PI, 2*PI) - PI;
  double xdot = (x - prev_x) / dt;

  double phidot = (phi - phiVals.dequeue()) / (t - tVals.dequeue());
  phiVals.enqueue(phi);
  tVals.enqueue(t);




  bool doBreak = false, doReset = false;
  double power = 0;
  double acceleration;
  switch (currState){
  case beginRight:
      if(x > X_STOP_RIGHT || x < X_STOP_LEFT || t > END_T){
      currState = beginLeft;
      continue_t = t + dir_change_brk_t + 0.1;
      doBreak = true;
      power = 0;
      break;
    }
    power = BEGIN_SPEED;
    doBreak = false;
    break;

  case beginLeft:
    if(x <= 0.1){
      swingupStarted = false;
      currState = swingup;
      swingUpPID.compute();
      swingUpPID.reset();
      continue_t = t + dir_change_brk_t + 1;
      doBreak = true;
      power = 0;
      break;
    }
    power = -BEGIN_SPEED;
    doBreak = false;
    break;

  case LQR:
    if (abs(phi_from_vert) > STOPLQRANGLE*DEG_TO_RAD){
      doReset = true;
      currState = returning;
      break;
    }
    if(x > X_STOP_RIGHT || x < X_STOP_LEFT || t > END_T){
      currState = returning;
      continue_t = t + dir_change_brk_t + 3;
      doBreak = true;
      power = 0;
      break;
    }
    //power = -t/120 - 0.06;
    power = LQRPower(x, phi, phi_norm, phi_from_vert, xdot, phidot, dt);
    // power = -exp2pwr;
    // acceleration = - 4 * cos(t * 2 * PI * 1);
    // power = GetU(acceleration, xdot + acceleration * dt);

    doBreak = false;
    break;

  case swingup:
    if (abs(phi_from_vert) < STARTLQRANGLE*DEG_TO_RAD){
      currState = LQR;

      break;
    }
    if(x > X_STOP_RIGHT || x < X_STOP_LEFT || t > END_T){
      currState = returning;
      continue_t = t + dir_change_brk_t + 3;
      doBreak = true;
      power = 0;
      break;
    }
    if (!swingupStarted && continue_t < t){
      power = 0;
      doBreak = true;
      swingupStarted = sign(phi) != sign(prev_phi) && sign(phi) > 0;
      swingUpPID.reset();
      break;
    }
    power = swingupPower(t, x, phi, phi_norm, phi_from_vert, xdot, phidot, dt);
    // power = getU(cos((t - )*10)*, xdot);
    doBreak = false;
    break;

  case waitingForLQR:
    if (!expLQRStarted){
      expLQRStarted = abs(phi_from_vert) < expStartLQRAngle;
      power = 0;
      break;
    }
    else if (abs(phi_from_vert) > expStartLQRAngle){
      currState = LQR;
      break;
    }
    // if (abs(phi_from_vert) < STARTLQRANGLE*DEG_TO_RAD){
    //   currState = LQR;

    //   break;
    // }
    if(x > X_STOP_RIGHT || x < X_STOP_LEFT || t > END_T){
      currState = returning;
      continue_t = t + dir_change_brk_t + 3;
      doBreak = true;
      power = 0;
      break;
    }
    power = 0;
    doBreak = true;
    break;

  case returning:
    if (abs(x - START_X) < 0.01) {
      currState = idle;
      doBreak = true;
      power = 0;
      reset_t = t + idleTime;
      break;
    }
    power = 0.1*sign(START_X - x);
    break;

  case idle:
    if (enableReset && t >= reset_t){
      doReset = true;
      break;
    }
    doBreak = true;
    power = 0;
    break;

  case calibrating:
    if(x > X_STOP_RIGHT || x < X_STOP_LEFT || t > END_T){
      currCalibrationIterations++;
      currState = returning;
      continue_t = t + dir_change_brk_t + 0.1;
      doBreak = true;
      power = 0;
      break;
    }
    power = -(calibrateBasePower + calibratePowerDelta * currCalibrationIterations);
    break;
  case calibrating_static:
    if(x > 0.15 || x < -0.15 || t > END_T){
      currState = returning;
      continue_t = t + dir_change_brk_t + 0.1;
      doBreak = true;
      power = 0;
      break;
    }
    power = -(0.045 + 0.01 * t);
    break;

  case cosine_check:
    // power = GetU(cos(t*2*PI)*2.5, xdot);
    power = cos(t*2*PI)*0.15;
  }

  if(continue_t > t){
    power = 0;
  }

  //if ((prev_pwr > 0 && power <= 0) || (prev_pwr < 0 && power >= 0)) continue_t = max(t + dir_change_brk_t, continue_t);

  if(continue_t > t){
    doBreak = true;
    power = 0;
  }
  
  prev_pwr = power;
  prev_t = t;
  prev_x = x;
  prev_phi = phi;
  prev_phi_norm = phi_norm;
  
  if (doReset){
    power = 0;
    motorBreak();
    Reset();
    exp2pwr += 1.0/256.0;
    return;
  }
  else if(doBreak){
    power = 0;
    motorBreak();
  }  
  else{
    power = constrain(power, -0.3, 0.3);
    setPowerF(power);
  }

  // print data at a fixed frequency
  if(t - t_prev_update > PRINT_T &&  (currState==LQR || currState == swingup || currState == calibrating || currState == calibrating_static)){
    t_prev_update = t;
    if(PRINT_CSV){
      Serial.print(t, 8);
      Serial.print(",");
      Serial.print(x, 8);
      Serial.print(",");
      Serial.print(phi_norm, 8);
      Serial.print(",");
      Serial.print(xdot, 8);
      Serial.print(",");
      Serial.print(phidot, 8);
      Serial.print(",");
      Serial.print(power, 8);
      Serial.print(",");
      Serial.print(E0 - E(phi, phidot), 8);
      Serial.println();
    }else{
      // Serial.print("t:");
      // Serial.print(t_prev_update, 8);
      // Serial.print(", min_x:-1.5, max_x:1.5, x:");
      // Serial.print(x, 8);
      // Serial.print(", min_theta:-360, max_theta:360, theta:");
      // Serial.print(phi_from_vert * RAD_TO_DEG, 8);
      // Serial.print("                ");
      // Serial.print(pid_output);
      // Serial.println();
      swingUpPID.debug(&Serial, "val", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                              PRINT_OUTPUT   | // in the Serial plotter
                                              PRINT_SETPOINT |
                                              PRINT_BIAS     |
                                              PRINT_P        |
                                              PRINT_I        |
                                              PRINT_D);
    }
  }

}
