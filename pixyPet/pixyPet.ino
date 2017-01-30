//==========================================================================
//
//  Pixy Pet Robot
//
//   Adafruit invests time and resources providing this open source code, 
//  please support Adafruit and open-source hardware by purchasing 
//  products from Adafruit!
//
// Written by: Bill Earl for Adafruit Industries
//
//==========================================================================
// begin license header
//
// All Pixy Pet source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
//
// end license header
//
//==========================================================================
//
// Portions of this code are derived from the Pixy CMUcam5 pantilt example code. 
//
//==========================================================================
#include <SPI.h>  
#include <Pixy.h>

#include <Servo.h>    //to define and control servos
#include <FlexiTimer2.h>        //to set a timer to manage all servos
#include <EEPROM.h>   //to save errors of all servos
#include <SPI.h>    //nRF24L01 module need 1/3
#include <nRF24L01.h>         //nRF24L01 module need 2/3
#include <RF24.h>   //nRF24L01 module need 3/3

#define X_CENTER    160L
#define Y_CENTER    100L
#define RCS_MIN_POS     0L
#define RCS_MAX_POS     1000L
#define RCS_CENTER_POS  ((RCS_MAX_POS-RCS_MIN_POS)/2)


/* Installation and Adjustment -----------------------------------------------*/

const float adjust_site[3] = { 100, 80, 42 };
const float real_site[4][3] = { { 126, 39, 36 }, { 103, 76, 22 },
                                { 88, 74, 41 }, { 130, 33, 3 }};
/* Servos --------------------------------------------------------------------*/
//define 12 servos for 4 legs
Servo servo[4][3];
//define servos' ports
const int servo_pin[4][3] = { 2, 3, 4, 5, 6, 7, 14, 15, 16, 17, 18, 19 };
/* Wireless communication ----------------------------------------------------*/
//dfine RF24 for nRF24l01
RF24 radio(9, 10);  
//define RF24 transmit pipe
//NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL;
/* Size of the robot ---------------------------------------------------------*/
const float length_a = 55;
const float length_b = 80;
const float length_c = 22.75;
const float length_side = 66;
const float z_absolute = -12;
/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_up = -10, z_boot = z_absolute;
const float x_default = 70, x_offset = 0;
const float y_start = 0, y_step = 50;
/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];      //real-time coordinates of the end of each leg 
volatile float site_expect[4][3];   //expected coordinates of the end of each leg
float temp_speed[4][3];             //each axis' speed, needs to be recalculated before each movement
float move_speed;                   //movement speed
float speed_multiple = 1;           //movement speed multiple
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter;          //+1/0.02s, for automatic rest  
const int wait_rest_time = 3 * 50;  //3s*50Hz, the time wait for automatic rest 
//functions' parameter
const float KEEP = 255;
//define PI for calculation
const float pi = 3.1415926;
/* Constants for turn --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b*cos(temp_alpha);
const float turn_y0 = temp_b*sin(temp_alpha) - turn_y1 - length_side;
 
//---------------------------------------
// Servo Loop Class
// A Proportional/Derivative feedback
// loop for pan/tilt servo tracking of
// blocks.
// (Based on Pixy CMUcam5 example code)
//---------------------------------------
class ServoLoop
{
public:
  ServoLoop(int32_t proportionalGain, int32_t derivativeGain);
 
  void update(int32_t error);
 
  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_proportionalGain;
  int32_t m_derivativeGain;
};
 
// ServoLoop Constructor
ServoLoop::ServoLoop(int32_t proportionalGain, int32_t derivativeGain)
{
  m_pos = RCS_CENTER_POS;
  m_proportionalGain = proportionalGain;
  m_derivativeGain = derivativeGain;
  m_prevError = 0x80000000L;
}
 
// ServoLoop Update 
// Calculates new output based on the measured
// error and the current state.
void ServoLoop::update(int32_t error)
{
  long int velocity;
  char buf[32];
  if (m_prevError!=0x80000000)
  { 
    velocity = (error*m_proportionalGain + (error - m_prevError)*m_derivativeGain)>>10;
 
    m_pos += velocity;
    if (m_pos>RCS_MAX_POS) 
    {
      m_pos = RCS_MAX_POS; 
    }
    else if (m_pos<RCS_MIN_POS) 
    {
      m_pos = RCS_MIN_POS;
    }
  }
  m_prevError = error;
}
// End Servo Loop Class
//---------------------------------------
 
Pixy pixy;  // Declare the camera object
 
ServoLoop panLoop(200, 200);  // Servo loop for pan
ServoLoop tiltLoop(150, 200); // Servo loop for tilt
 
//ZumoMotors motors;  // declare the motors on the zumo
 
//---------------------------------------
// Setup - runs once at startup
//---------------------------------------
void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
 
  pixy.init();

  Serial.println("Robot starts initialization");
  
  //initialize default parameter
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  //start servo service
  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();
  Serial.println("Servo service started");
  //initialize servos
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].attach(servo_pin[i][j]);
      delay(100);
    }
  }
  Serial.println("Servos initialized");
  Serial.println("Robot initialization Complete");

}
 
uint32_t lastBlockTime = 0;
 
//---------------------------------------
// Main loop - runs continuously after setup
//---------------------------------------
void loop()
{ 
  uint16_t blocks;
  blocks = pixy.getBlocks();
 
  // If we have blocks in sight, track and follow them
  if (blocks)
  {
    int trackedBlock = TrackBlock(blocks);
    
    FollowBlock(trackedBlock);
    
    lastBlockTime = millis();
  }  
  else if (millis() - lastBlockTime > 100)
  {
    ScanForBlocks();
  }
}
 
int oldX, oldY, oldSignature;
 
//---------------------------------------
// Track blocks via the Pixy pan/tilt mech
// (based in part on Pixy CMUcam5 pantilt example)
//---------------------------------------
int TrackBlock(int blockCount)
{
  int trackedBlock = 0;
  long maxSize = 0;
 
  Serial.print("blocks =");
  Serial.println(blockCount);
 
  for (int i = 0; i < blockCount; i++)
  {
    if ((oldSignature == 0) || (pixy.blocks[i].signature == oldSignature))
    {
      long newSize = pixy.blocks[i].height * pixy.blocks[i].width;
      if (newSize > maxSize)
      {
        trackedBlock = i;
        maxSize = newSize;
      }
    }
  }
 
  int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
  int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;
 
  panLoop.update(panError);
  tiltLoop.update(tiltError);
 
  pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
 
  oldX = pixy.blocks[trackedBlock].x;
  oldY = pixy.blocks[trackedBlock].y;
  oldSignature = pixy.blocks[trackedBlock].signature;
  return trackedBlock;
}
 
//---------------------------------------
// Follow blocks via the Zumo robot drive
//
// This code makes the robot base turn 
// and move to follow the pan/tilt tracking
// of the head.
//---------------------------------------
int32_t size = 400;
void FollowBlock(int trackedBlock)
{
  int32_t followError = RCS_CENTER_POS - panLoop.m_pos;  // How far off-center are we looking now?
 
  // Size is the area of the object.
  // We keep a running average of the last 8.
  size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height; 
  size -= size >> 3;
 
  // Forward speed decreases as we approach the object (size is larger)
  int forwardSpeed = constrain(400 - (size/256), -100, 400);  
 
  // Steering differential is proportional to the error times the forward speed
  int32_t differential = (followError + (followError * forwardSpeed))>>8;
 
  // Adjust the left and right speeds by the steering differential.
  int leftSpeed = constrain(forwardSpeed + differential, -400, 400);
  int rightSpeed = constrain(forwardSpeed - differential, -400, 400);

  int difference = 0;
  if (rightSpeed > leftSpeed)
  {
    difference = rightSpeed - leftSpeed;
  }
  else
  {
    difference = leftSpeed - rightSpeed;
  }

  if (difference < 300)
  {
    step_forward(1);
  }
  else if (leftSpeed < rightSpeed)
  {
  turn_left(1);
  }
  else
  {
  turn_right(1);
}
//  // And set the motor speeds
//  motors.setLeftSpeed(leftSpeed);
//  motors.setRightSpeed(rightSpeed);



}
 
//---------------------------------------
// Random search for blocks
//
// This code pans back and forth at random
// until a block is detected
//---------------------------------------
int scanIncrement = (RCS_MAX_POS - RCS_MIN_POS) / 150;
uint32_t lastMove = 0;
 
void ScanForBlocks()
{
  if (millis() - lastMove > 20)
  {
    lastMove = millis();
    panLoop.m_pos += scanIncrement;
    if ((panLoop.m_pos >= RCS_MAX_POS)||(panLoop.m_pos <= RCS_MIN_POS))
    {
      tiltLoop.m_pos = random(RCS_MAX_POS * 0.6, RCS_MAX_POS);
      scanIncrement = -scanIncrement;
      if (scanIncrement < 0)
      {
        turn_left(1);
      }
      else
      {
        turn_right(1);
      }
      delay(random(250, 500));
    }
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
  }
}



/*
 - adjustment function 
 - move each leg to adjustment site, so that you can measure the real sites.
 * ---------------------------------------------------------------------------*/
void adjust(void)
{
  //initializes eeprom's errors to 0
  //number -100 - +100 is map to 0 - +200 in eeprom 
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      EEPROM.write(i * 6 + j * 2, 100);
      EEPROM.write(i * 6 + j * 2 + 1, 100);
    }
  }

  //initializes the relevant variables to adjustment position
  for (int i = 0; i < 4; i++)
  {
    set_site(i, adjust_site[0], adjust_site[1], adjust_site[2] + z_absolute);
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  //start servo service
  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();
  //initialize servos
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].attach(servo_pin[i][j]);
      delay(100);
    }
  }
}


/*
 - sit
 - blocking function
 * ---------------------------------------------------------------------------*/
void sit(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}

/*
 - stand
 - blocking function
 * ---------------------------------------------------------------------------*/
void stand(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}

/*
 - is_stand
 * ---------------------------------------------------------------------------*/
bool is_stand(void)
{
  if (site_now[0][2] == z_default)
    return true;
  else
    return false;
}

/*
 - spot turn to left
 - blocking function
 - parameter step steps wanted to turn 
 * ---------------------------------------------------------------------------*/
void turn_left(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
 - spot turn to right
 - blocking function
 - parameter step steps wanted to turn
 * ---------------------------------------------------------------------------*/
void turn_right(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
 - go forward
 - blocking function
 - parameter step steps wanted to go
 * ---------------------------------------------------------------------------*/
void step_forward(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
 - go back
 - blocking function
 - parameter step steps wanted to go
 * ---------------------------------------------------------------------------*/
void step_back(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
 - microservos service /timer interrupt function/50Hz
 - when set site expected,this function move the end point to it in a straight line
 - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
 * ---------------------------------------------------------------------------*/
void servo_service(void)
{
  sei();
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++;
}

/*
 - set one of end points' expect site 
 - this founction will set temp_speed[4][3] at same time 
 - non - blocking function
 * ---------------------------------------------------------------------------*/
void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed*speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed*speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed*speed_multiple;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}

/*
 - wait one of end points move to expect site 
 - blocking function 
 * ---------------------------------------------------------------------------*/
void wait_reach(int leg)
{
  while (1)
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          break;
}

/*
 - wait one of end points move to one site 
 - blocking function 
 * ---------------------------------------------------------------------------*/
void wait_reach(int leg, float x, float y, float z)
{
  while (1)
    if (site_now[leg][0] == x)
      if (site_now[leg][1] == y)
        if (site_now[leg][2] == z)
          break;
}

/*
 - wait all of end points move to expect site 
 - blocking function 
 * ---------------------------------------------------------------------------*/
void wait_all_reach(void)
{
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}

/*
 - trans site from polar to cartesian 
 - mathematical model 1/2
 * ---------------------------------------------------------------------------*/
void polar_to_cartesian(volatile float alpha, volatile float beta, volatile float gamma, volatile float &x, volatile float &y, volatile float &z)
{
  //trans degree 180->pi
  alpha = alpha / 180 * pi;
  beta = beta / 180 * pi;
  gamma = gamma / 180 * pi;
  //calculate w-z position
  float v, w;
  v = length_a*cos(alpha) - length_b*cos(alpha + beta);
  z = length_a*sin(alpha) - length_b*sin(alpha + beta);
  w = v + length_c;
  //calculate x-y-z position
  x = w*cos(gamma);
  y = w*sin(gamma);
}

/*
 - trans site from cartesian to polar 
 - mathematical model 2/2
 * ---------------------------------------------------------------------------*/
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1)*(sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

/*
 - trans site from polar to microservos
 - mathematical model map to fact
 - the errors saved in eeprom will be add
 * ---------------------------------------------------------------------------*/
void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  float alpha_error = EEPROM.read(leg * 6 + 0) - 100 + ((float)EEPROM.read(leg * 6 + 1) - 100) / 100;
  float beta_error  = EEPROM.read(leg * 6 + 2) - 100 + ((float)EEPROM.read(leg * 6 + 3) - 100) / 100;
  float gamma_error = EEPROM.read(leg * 6 + 4) - 100 + ((float)EEPROM.read(leg * 6 + 5) - 100) / 100;

  alpha += alpha_error;
  beta += beta_error;
  gamma += gamma_error;

  if (leg == 0)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma = 90 - gamma;
  }
  else if (leg == 1)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma += 90;
  }
  else if (leg == 2)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma += 90;
  }
  else if (leg == 3)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma = 90 - gamma;
  }

  servo[leg][0].write(alpha);
  servo[leg][1].write(beta);
  servo[leg][2].write(gamma);
}
