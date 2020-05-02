/****************************************************************************
   Shenzhen Yahboom Technology Co., Ltd.
   Product Name：Arduino smart balance car
   Product model：BST-ABC ver
****************************************************************************/

#include <MsTimer2.h>
#include <PinChangeInt.h>
// Speed PID control by speed dial counting
#include <BalanceCar.h>
#include <Filters.h>
#include <PIDControl.h>
// I2Cdev, MPU6050, and PID_v1 libraries need to be installed in libraries
// folder in the Arduino installation path.
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// TB6612FNG drive module control signal
#define IN1M 7
#define IN2M 6
#define IN3M 13
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8

#define PinA_left 2   // Interrupt 0
#define PinA_right 4  // Interrupt 1

MPU6050 mpu;  // Instantiate an MPU6050 object with the object is called mpu
BalanceCar balancecar;
Filters filters;
int16_t ax, ay, az, gx, gy, gz;
PIDController pid = {PIDGain{38, 0, 0.58}, PIDGain{28, 0, 0.29}, PIDGain{0, 0.1058, 3.5}};

//********************angle data*********************//
// float Q;
// float Angle_ax; //The tilt Angle calculated by acceleration
// float Angle_ay;
// Was K1
// float angle0 = 0.00; //Mechanical equilibrium Angle
// int slong;
//********************angle data*********************//

//***************Kalman_Filter*********************//
float Q_angle = 0.001, Q_gyro = 0.005;  // Used in angle test
float R_angle = 0.5, C_0 = 1;
constexpr float timeChange =
    0.005;  // Filter sampling time interval (unit:milliseconds)
constexpr float dt =
    timeChange * 0.001;  // The value of dt is the filter sampling time
constexpr float K1 =
    timeChange /
    (timeChange + dt);  // Calculated by tc/(tc+dt) where tc is how fast you
                        // want readings to respond and dt=1/sampling frequency
//***************Kalman_Filter*********************//

//////////////User Input States///////////////////
enum class CarStateChar{
  STOP = '0',
  RUN = '1',
  BACK = '2',
  TURN_LEFT = '3',
  TURN_RIGHT = '4'
};

/////////////Actual Car States///////////////////
enum class CarState {
  STOP = 0,
  RUN,
  BACK,
  TURN_LEFT,
  TURN_RIGHT,
  SPIN_LEFT,
  SPIN_RIGHT
};

CarState car_state = CarState::STOP;  //  1run 2back 3left 4right 0stop

int direction = 0;
bool turn_left_flag = false;
bool turn_right_flag = false;
bool spin_left_flag = false;
bool spin_right_flag = false;


int emergency_stop = 0;
int speeds = 0;

//Pulse interrupt counter registers
volatile long count_right_pulse = 0; 
volatile long count_left_pulse = 0;

//////////////////////Pulse calculation///////////////////////
void countpulse() {

  static int left_pulse = count_left_pulse;
  static int right_pulse = count_right_pulse;

  count_left_pulse = 0;
  count_right_pulse = 0;

  right_pulse = (balancecar.pwm2 < 0) ? -right_pulse : right_pulse;
  left_pulse = (balancecar.pwm1 < 0) ? -left_pulse : left_pulse;

  //Long living speed limiter
  emergency_stop += (right_pulse + left_pulse);

  // Short lived speed estimator
  speeds += (right_pulse + left_pulse);
}

double speed_filtered = 0;
double speed_output = 0;
void slow_inter() {
  cli();
  speed_filtered = band_limited_filter<double>(speed_filtered, speeds, pid.z.get_add_gain());
  speed_output = - pid.z.update_pid(speed_filtered + direction, speeds);
  speeds = 0;
  sei();
}

//////////////Steering and rotation parameters///////////////////////////////
float turn_error = 0;
float turn_output = 0;

void avg_inter() {
  turn_error = balancecar.turnspin(turn_left_flag, turn_right_flag, spin_left_flag, spin_right_flag, speeds);
  turn_output = pid.y.update_pid_bypass(turn_error, raw_to_gyro(gz));
}

//////////////////////////////////////////////////////////
//////////////////Interrupt timing 5ms////////////////////
/////////////////////////////////////////////////////////
void fast_inter() {
  sei();
  countpulse();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy,
                 &gz);  // IIC obtained MPU6050 six-axis data: ax ay az gx gy gz
  filters.angle_test(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle,
                         C_0, K1);  // Getting Angle  and kaman filtering
  balancecar.angleoutput = pid.x.update_pid_bypass(-filters.angle, -raw_to_gyro(gx)); // Signs may be weird for gx

  balancecar.posture++;
  balancecar.pwma(speed_output, turn_output, filters.angle, filters.angle6,
                  turn_left_flag, turn_right_flag, spin_left_flag,
                  spin_right_flag, direction, raw_to_accel(az), IN1M, IN2M,
                  IN3M, IN4M, PWMA, PWMB);
}



// ===  initial setting    ===
void setup() {
  // TB6612FNGN drive module control signal initialization
  pinMode(IN1M, OUTPUT);  // Controlling the direction of motor 1, 01 is
                          // positive, 10 is reverse
  pinMode(IN2M, OUTPUT);
  pinMode(IN3M, OUTPUT);  // Controlling the direction of motor 2, 01 is
                          // positive, 10 is reverse
  pinMode(IN4M, OUTPUT);
  pinMode(PWMA, OUTPUT);  // Left motor PWM
  pinMode(PWMB, OUTPUT);  // Right motor PWM
  pinMode(STBY, OUTPUT);  // TB6612FNG

  // TB6612FNG drive module control signals
  digitalWrite(IN1M, 0);  //
  digitalWrite(IN2M, 1);
  digitalWrite(IN3M, 1);
  digitalWrite(IN4M, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  pinMode(PinA_left, INPUT);
  pinMode(PinA_right, INPUT);

  // I2C
  Wire.begin();  // Adding the I2C bus sequence
  Serial.begin(
      9600);  // Opening the serial port and setting the baud rate to 115200
  delay(1500);
  mpu.initialize();  // Initialization of MPU6050
  delay(2);
  pid.z.set_i_limits(3550);
  pid.z.set_add_gain(0.7);
  balancecar.pwm1 = 0;
  balancecar.pwm2 = 0;
  MsTimer2::set(5, fast_inter);
  MsTimer2::set(20, avg_inter);
  MsTimer2::set(40, slow_inter);
  MsTimer2::start();
}

////////////////////////////////////////turn//////////////////////////////////

void ResetPID() {
  pid.x.reset_pid();
  pid.y.reset_pid();
  pid.z.reset_pid();
}
void ResetCarState() {
  turn_left_flag = false;
  turn_right_flag = false;
  direction = 0;
  spin_left_flag = false;
  spin_right_flag = false;
  turn_output = 0;
}
// ===       Main loop       ===
void loop() {
  //String returnstr = "$0,0,0,0,0,0,0,0,0,0,0,0cm,8.2V#";
  attachInterrupt(0, Code_left, CHANGE);
  attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);

  // Serial.println(kalmanfilter.angle);
  // Serial.print("\t");
  // Serial.print(bluetoothvalue);
  // Serial.print("\t");
  //      Serial.print( balancecar.angleoutput);
  //      Serial.print("\t");
  //      Serial.print(balancecar.pwm1);
  //      Serial.print("\t");
  //      Serial.println(balancecar.pwm2);
  //      Serial.print("\t");
  //      Serial.println(balancecar.stopr);
  //      Serial.print("\t");

  run_parser();

  switch (car_state) {
    case CarState::STOP:
      ResetCarState();
      break;
    case CarState::RUN:
      ResetCarState();
      direction = 250;
      break;
    case CarState::TURN_LEFT:
      turn_left_flag = true;
      break;
    case CarState::TURN_RIGHT:
      turn_right_flag = true;
      break;
    case CarState::BACK:
      ResetCarState();
      direction = -250;
      break;
    case CarState::SPIN_LEFT:
      spin_left_flag = true;
      break;
    case CarState::SPIN_RIGHT:
      spin_right_flag = true;
      break;
    default:
      ResetCarState();
      break;
  }

  SendAutoUp();
}

////////////////////////////////////////pwm///////////////////////////////////

//////////////////////////Pulse interrupt Calculation/////////////////////////

void Code_left() { count_left_pulse++; }

void Code_right() { count_right_pulse++; }


/////////Used for an APP thecompany has not part of our implementation////////
////////////////////////Parsing and wireless connectivity/////////////////////
String returntemp = "";               // It used to store return value
boolean g_autoup = false;
int g_uptimes = 5000;

void SendAutoUp() {
  g_uptimes--;
  if ((g_autoup == true) && (g_uptimes == 0)) {
    //Automatic Response
    String CSB, VT;
    char temp[10] = {0};
    float fgx;
    float fay;
    float leftspeed;
    float rightspeed;

    fgx = gx;
    fay = ay;
    leftspeed = balancecar.pwm1;
    rightspeed = balancecar.pwm2;

    double Gx = (double)((fgx - 128.1f) / 131.0f);  // angle conversion
    double Ay = ((double)fay / 16384.0f) * 9.8f;

    if (leftspeed > 255 || leftspeed < -255) return;
    if (rightspeed > 255 || rightspeed < -255) return;
    if ((Ay < -20) || (Ay > 20)) return;
    if ((Gx < -3000) || (Gx > 3000)) return;

    returntemp = "";

    memset(temp, 0x00, sizeof(temp));
    // sprintf(temp, "%3.1f", leftspeed);
    dtostrf(leftspeed, 3, 1, temp);  // == %3.2f
    String LV = temp;

    memset(temp, 0x00, sizeof(temp));
    // sprintf(temp, "%3.1f", rightspeed);
    dtostrf(rightspeed, 3, 1, temp);  // == %3.1f
    String RV = temp;

    memset(temp, 0x00, sizeof(temp));
    // sprintf(temp, "%2.2f", Ay);
    dtostrf(Ay, 2, 2, temp);  // == %2.2f
    String AC = temp;

    memset(temp, 0x00, sizeof(temp));
    // sprintf(temp, "%4.2f", Gx);
    dtostrf(Gx, 4, 2, temp);  // == %4.2f
    String GY = temp;

    CSB = "0.00";
    VT = "0.00";
    // AC =
    returntemp = "$LV" + LV + ",RV" + RV + ",AC" + AC + ",GY" + GY + ",CSB" +
                 CSB + ",VT" + VT + "#";
    Serial.print(returntemp);  // Returning the protocol data packet
  }

  if (g_uptimes == 0) g_uptimes = 5000;
}

// Parser Variables
int incomingByte = 0; //Serial port recieving byte
String inputString = "";  // It used to store received content
boolean newLineReceived = false;
boolean startBit = false;

// SerialEvent () is a new feature in IDE1.0 and later version.
int num = 0;
void serialEvent() {
  while (Serial.available()) {
    incomingByte = Serial.read();
    if (incomingByte == '$') {
      num = 0;
      startBit = true;
    }
    if (startBit == true) {
      num++;
      inputString += (char)incomingByte;
    }
    if (startBit == true && incomingByte == '#') {
      newLineReceived = true;
      startBit = false;
    }

    if (num >= 80) {
      num = 0;
      startBit = false;
      newLineReceived = false;
      inputString = "";
    }
  }
}


void run_parser(){  if (newLineReceived) {
    switch (inputString[1]) {
      case static_cast<char>(CarStateChar::RUN):
        car_state = CarState::RUN;
        break;
      case static_cast<char>(CarStateChar::BACK):
        car_state = CarState::BACK;
        break;
      case static_cast<char>(CarStateChar::TURN_LEFT):
        car_state = CarState::TURN_LEFT;
        break;
      case static_cast<char>(CarStateChar::TURN_RIGHT):
        car_state = CarState::TURN_RIGHT;
        break;
      case static_cast<char>(CarStateChar::STOP):
        car_state = CarState::STOP;
        break;
      default:
        car_state = CarState::STOP;
        break;
    }
    // This codes can be used to determine whether the protocol is lost during
    // the transmission process.
    /* if(inputString.length() < 21)
      {
        inputString = "";   // clear the string
        newLineReceived = false;
        //Serial.print(returnstr);
        goto a;
      }*/
    if (inputString[3] == '1' && inputString.length() == 21) {
      car_state = CarState::SPIN_LEFT;
      // Serial.print(returnstr);
    } else if (inputString[3] == '2' && inputString.length() == 21) {
      car_state = CarState::SPIN_RIGHT;
      // Serial.print(returnstr);
    }

    if (inputString[5] == '1')  // Inquiring PID
    {
      char charkp[7], charkd[7], charkpspeed[7], charkispeed[7];

      dtostrf(pid.x.get_kp(), 3, 2, charkp);             // == %3.2f
      dtostrf(pid.y.get_kd(), 3, 2, charkd);             // == %3.2f
      dtostrf(pid.z.get_kp(), 3, 2, charkpspeed);  // == %3.2f
      dtostrf(pid.z.get_ki(), 3, 2, charkispeed);  // == %3.2f

      String strkp = charkp;
      String strkd = charkd;
      String strkpspeed = charkpspeed;
      String strkispeed = charkispeed;

      returntemp = "$0,0,0,0,0,0,AP" + strkp + ",AD" + strkd + ",VP" +
                   strkpspeed + ",VI" + strkispeed + "#";

      Serial.print(returntemp);
    } else if (inputString[5] == '2') {
      ResetPID();
      Serial.print("$OK#");
    }

    if (inputString[7] == '1') {
      g_autoup = true;
      Serial.print("$OK#");
    } else if (inputString[7] == '2') {
      g_autoup = false;
      Serial.print("$OK#");
    }

    if (inputString[9] == '1') {
      int i = inputString.indexOf("AP");
      int ii = inputString.indexOf(",", i);
      if (ii > i) {
        String m_skp = inputString.substring(i + 2, ii);
        m_skp.replace(".", "");
        int m_kp = m_skp.toInt();
        pid.x.set_kp((double)((double)m_kp / 100.0f));
      }

      i = inputString.indexOf("AD");
      ii = inputString.indexOf(",", i);
      if (ii > i) {
        // ki = inputString.substring(i+2, ii);
        String m_skd = inputString.substring(i + 2, ii);
        m_skd.replace(".", "");
        int m_kd = m_skd.toInt();
        pid.x.set_kd((double)((double)m_kd / 100.0f));
      }
      Serial.print("$OK#");
    }

    if (inputString[11] == '1') {
      int i = inputString.indexOf("VP");
      int ii = inputString.indexOf(",", i);
      if (ii > i) {
        String m_svp = inputString.substring(i + 2, ii);
        m_svp.replace(".", "");
        int m_vp = m_svp.toInt();
        pid.z.set_kd((double)((double)m_vp / 100.0f));
      }

      i = inputString.indexOf("VI");
      ii = inputString.indexOf("#", i);
      if (ii > i) {
        String m_svi = inputString.substring(i + 2, ii);
        m_svi.replace(".", "");
        int m_vi = m_svi.toInt();
        pid.z.set_ki((double)((double)m_vi / 100.0f));
        Serial.print("$OK#");
      }
    }
    // restore default
    inputString = "";  // clear the string
    newLineReceived = false;
  }}

/*Backup*/
#if 0
char chartemp[7];
dtostrf(ax, 3, 2, chartemp);  // == %3.2f
String strax = chartemp;
strax = "\nax:" + strax;

memset(chartemp, 0x00, 7);
dtostrf(ay, 3, 2, chartemp);  // == %3.2f
String stray = chartemp;
stray = "\nay:" + stray;

memset(chartemp, 0x00, 7);
dtostrf(az, 3, 2, chartemp);  // == %3.2f
String straz = chartemp;
straz = "\naz:" + straz;

memset(chartemp, 0x00, 7);
dtostrf(gx, 3, 2, chartemp);  // == %3.2f
String strgx = chartemp;
strgx = "\ngx:" + strgx;

memset(chartemp, 0x00, 7);
dtostrf(gy, 3, 2, chartemp);  // == %3.2f
String strgy = chartemp;
strgy = "\ngy:" + strgy;

memset(chartemp, 0x00, 7);
dtostrf(gz, 3, 2, chartemp);  // ==%3.2f
String strgz = chartemp;
strgz = "\ngz:" + strgz;
Serial.print(strax + stray + straz + strgx + strgy + strgz); //Returning the protocol data packet
#endif
