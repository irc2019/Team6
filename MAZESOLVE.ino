/*
 * Simple3piMazeSolver - demo code for the Pololu 3pi Robot
 * 
 * This code will solve a line maze constructed with a black line on a
 * white background, as long as there are no loops.  It has two
 * phases: first, it learns the maze, with a "left hand on the wall"
 * strategy, and computes the most efficient path to the finish.
 * Second, it follows its most efficient solution.
 *
 * http://www.pololu.com/docs/0J21
 * http://www.pololu.com
 * http://forum.pololu.com
 *
 */

// The following libraries will be needed by this demo
#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanMotors.h>
#include <OrangutanAnalog.h>
#include <OrangutanLEDs.h>
#include <OrangutanLCD.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanBuzzer.h>
#include <avr/pgmspace.h>

#define CRUISE_SPEED 50
#define SLOW_SPEED CRUISE_SPEED * 0.8
#define MAX_SPEED 60
#define TURN_SPEED 80


Pololu3pi robot;
static int finished = 0;
unsigned int sensors[5]; // an array to hold sensor values

// Messages
const char welcome_line1[] PROGMEM = " Pololu";
const char welcome_line2[] PROGMEM = "3\xf7 Robot";
const char demo_name_line1[] PROGMEM = "Maze";
const char demo_name_line2[] PROGMEM = "solver";

// A couple of simple tunes, stored in program space.
const char welcome[] PROGMEM = ">g32>>c32";
const char go[] PROGMEM = "L16 cdegreg4";


// Data for generating the characters used in load_custom_characters
// and display_readings.  By reading levels[] starting at various
// offsets, we can generate all of the 7 extra characters needed for a
// bargraph.  This is also stored in program space.
const char levels[] PROGMEM = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

// This function loads custom characters into the LCD.  Up to 8
// characters can be loaded; we use them for 7 levels of a bar graph.
void load_custom_characters()
{
  OrangutanLCD::loadCustomCharacter(levels + 0, 0); // no offset, e.g. one bar
  OrangutanLCD::loadCustomCharacter(levels + 1, 1); // two bars
  OrangutanLCD::loadCustomCharacter(levels + 2, 2); // etc...
  OrangutanLCD::loadCustomCharacter(levels + 3, 3);
  OrangutanLCD::loadCustomCharacter(levels + 4, 4);
  OrangutanLCD::loadCustomCharacter(levels + 5, 5);
  OrangutanLCD::loadCustomCharacter(levels + 6, 6);
  OrangutanLCD::clear(); // the LCD must be cleared for the characters to take effect
}

// This function displays the sensor readings using a bar graph.
void display_readings(const unsigned int *calibrated_values)
{
  unsigned char i;

  for (i=0;i<5;i++) {
    // Initialize the array of characters that we will use for the
    // graph.  Using the space, an extra copy of the one-bar
    // character, and character 255 (a full black box), we get 10
    // characters in the array.
    const char display_characters[10] = { 
      ' ', 0, 0, 1, 2, 3, 4, 5, 6, 255     };

    // The variable c will have values from 0 to 9, since
    // calibrated values are in the range of 0 to 1000, and
    // 1000/101 is 9 with integer math.
    char c = display_characters[calibrated_values[i] / 101];

    // Display the bar graph character.
    OrangutanLCD::print(c);
  }
}

void calibrate() {
  unsigned int counter; // used as a simple timer
  for (counter=0; counter<80; counter++)
  {
    if (counter < 20 || counter >= 60)
      OrangutanMotors::setSpeeds(40, -40);
    else
      OrangutanMotors::setSpeeds(-40, 40);
    robot.calibrateLineSensors(IR_EMITTERS_ON);
    delay(20);
  }
}

void setup()
{
  robot.init(2000);

  load_custom_characters(); // load the custom characters

    // Play welcome music and display a message
  OrangutanLCD::printFromProgramSpace(welcome_line1);
  OrangutanLCD::gotoXY(0, 1);
  OrangutanLCD::printFromProgramSpace(welcome_line2);
  OrangutanBuzzer::playFromProgramSpace(welcome);
  delay(1000);

  OrangutanLCD::clear();
  OrangutanLCD::printFromProgramSpace(demo_name_line1);
  OrangutanLCD::gotoXY(0, 1);
  OrangutanLCD::printFromProgramSpace(demo_name_line2);
  delay(1000);

  // Display battery voltage and wait for button press
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    int bat = OrangutanAnalog::readBatteryMillivolts();

    OrangutanLCD::clear();
    OrangutanLCD::print(bat);
    OrangutanLCD::print("mV");
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::print("Press B");

    delay(100);
  }
  
  OrangutanPushbuttons::waitForRelease(BUTTON_B);
  delay(1000);

  calibrate();
  OrangutanMotors::setSpeeds(0, 0);

  // Display calibrated values as a bar graph.
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);
    OrangutanLCD::clear();
    OrangutanLCD::print(position);
    OrangutanLCD::gotoXY(0, 1);
    display_readings(sensors);

    delay(100);
  }
  OrangutanPushbuttons::waitForRelease(BUTTON_B);

  OrangutanLCD::clear();

  OrangutanLCD::print("Go!");		

  // Play music and wait for it to finish before we start driving.
  OrangutanBuzzer::playFromProgramSpace(go);
  while(OrangutanBuzzer::isPlaying());
}

void follow_segment()
{
  int last_proportional = 0;
  long integral=0;

  while(1)
  {

    // Get the position of the line.
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

    // The "proportional" term should be 0 when we are on the line.
    int proportional = ((int)position) - 2000;

    // Compute the derivative (change) and integral (sum) of the
    // position.
    int derivative = proportional - last_proportional;
    integral += proportional;

    // Remember the last position.
    last_proportional = proportional;

    // Compute the difference between the two motor power settings,
    // m1 - m2.  If this is a positive number the robot will turn
    // to the left.  If it is a negative number, the robot will
    // turn to the right, and the magnitude of the number determines
    // the sharpness of the turn.
    int power_difference = proportional/20 + integral/10000 + derivative*3/2;

    // Compute the actual motor settings.  We never set either motor
    // to a negative value.
    if (power_difference > MAX_SPEED)
      power_difference = MAX_SPEED;
    if (power_difference < -MAX_SPEED)
      power_difference = -MAX_SPEED;

    if (power_difference < 0)
      OrangutanMotors::setSpeeds(MAX_SPEED + power_difference, MAX_SPEED);
    else
      OrangutanMotors::setSpeeds(MAX_SPEED, MAX_SPEED - power_difference);

    // We use the inner three sensors (1, 2, and 3) for
    // determining whether there is a line straight ahead, and the
    // sensors 0 and 4 for detecting lines going to the left and
    // right.

    if (sensors[1] < 100 && sensors[2] < 100 && sensors[3] < 100)
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
      return;
    }
    else if (sensors[0] > 200 || sensors[4] > 200)
    {
      // Found an intersection.
      return;
    }
    else if (sensors[0] > 200 && sensors[1] > 200 && sensors[2] > 200 && sensors[3] > 200 && sensors[4] > 200) {
      finished = 1;
      return;
    }

  }
}

int lineInSight() {
  unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);
  return (sensors[1] + sensors[2] + sensors[3]) > 400;
}

// Code to perform various types of turns according to the parameter dir,
// which should be 'L' (left), 'R' (right), 'S' (straight), or 'B' (back).
// The delays here had to be calibrated for the 3pi's motors.
void turn(unsigned char dir)
{
  int i;
  switch(dir)
  {
  case 'L':
    // Turn left.
    OrangutanMotors::setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(150);
    for (i = 0; i < 2; ++i) {
      if (lineInSight()) break;
      delay(25);
    }
    break;
  case 'R':
    // Turn right.
    OrangutanMotors::setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(150);
    for (i = 0; i < 2; ++i) {
      if (lineInSight()) break;
      delay(25);
    }
    break;
  case 'B':
    // Turn around.
    OrangutanMotors::setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(300);
    for (i = 0; i < 4; ++i) {
      if (lineInSight()) break;
      delay(25  );
    }
    break;
  case 'S':
    // Don't do anything!
    break;
  }
}


// The path variable will store the path that the robot has taken.  It
// is stored as an array of characters, each of which represents the
// turn that should be made at one intersection in the sequence:
//  'L' for left
//  'R' for right
//  'S' for straight (going straight through an intersection)
//  'B' for back (U-turn)
//
// Whenever the robot makes a U-turn, the path can be simplified by
// removing the dead end.  The follow_next_turn() function checks for
// this case every time it makes a turn, and it simplifies the path
// appropriately.

typedef struct {
  char dir;
  unsigned long mils;
} pathChange;

pathChange path[100];
unsigned char path_length = 0; // the length of the path

char *pathToString() {
  char *str = (char*)malloc(100 * sizeof(char));
  int i;
  for (i = 0; i < path_length; ++i) {
    str[i] = path[i].dir;
  }
  str[path_length] = '\0';
  return str;
}

// Displays the current path on the LCD, using two rows if necessary.
void display_path()
{
  char *p = pathToString();
  
  OrangutanLCD::clear();
  OrangutanLCD::print(p);

  if (path_length > 8)
  {
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::print(p + 8);
  }
  free(p);
}

// This function decides which way to turn during the learning phase of
// maze solving.  It uses the variables found_left, found_straight, and
// found_right, which indicate whether there is an exit in each of the
// three directions, applying the "left hand on the wall" strategy.
unsigned char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right)
{
  // Make a decision about how to turn.  The following code
  // implements a left-hand-on-the-wall strategy, where we always
  // turn as far to the left as possible.
  if (found_left)
    return 'L';
  else if (found_straight)
    return 'S';
  else if (found_right)
    return 'R';
  else
    return 'B';
}

// Path simplification.  The strategy is that whenever we encounter a
// sequence xBx, we can simplify it by cutting out the dead end.  For
// example, LBL -> S, because a single S bypasses the dead end
// represented by LBL.
void simplify_path()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if (path_length < 3 || path[path_length-2].dir != 'B')
    return;

  int total_angle = 0;
  int i;
  for (i = 1; i <= 3; i++)
  {
    switch (path[path_length - i].dir)
    {
    case 'R':
      total_angle += 90;
      break;
    case 'L':
      total_angle += 270;
      break;
    case 'B':
      total_angle += 180;
      break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  total_angle = total_angle % 360;

  // Replace all of those turns with a single one.
  switch (total_angle)
  {
  case 0:
    path[path_length - 3].dir = 'S';
    break;
  case 90:
    path[path_length - 3].dir = 'R';
    break;
  case 180:
    path[path_length - 3].dir = 'B';
    break;
  case 270:
    path[path_length - 3].dir = 'L';
    break;
  }

  // The path is now two steps shorter.
  path_length -= 2;
}

// This function comprises the body of the maze-solving program.  It is called
// repeatedly by the Arduino framework.
void loop()
{
  while (!finished)
  {
    unsigned long startMils, endMils;
    startMils = millis();
    follow_segment();
    endMils = millis();

    // Drive straight a bit.  This helps us in case we entered the
    // intersection at an angle.
    // Note that we are slowing down - this prevents the robot
    // from tipping forward too much.
    OrangutanMotors::setSpeeds(CRUISE_SPEED, CRUISE_SPEED);
    delay(50);

    // These variables record whether the robot has seen a line to the
    // left, straight ahead, and right, whil examining the current
    // intersection.
    unsigned char found_left = 0;
    unsigned char found_straight = 0;
    unsigned char found_right = 0;

    // Now read the sensors and check the intersection type.
    unsigned int sensors[5];
    robot.readLine(sensors, IR_EMITTERS_ON);

    // Check for left and right exits.
    if (sensors[0] > 100)
      found_left = 1;
    if (sensors[4] > 100)
      found_right = 1;

    // Drive straight a bit more - this is enough to line up our
    // wheels with the intersection.
    OrangutanMotors::setSpeeds(SLOW_SPEED, SLOW_SPEED);
    delay(150);

    // Check for a straight exit.
    robot.readLine(sensors, IR_EMITTERS_ON);
    if (sensors[1] > 200 || sensors[2] > 200 || sensors[3] > 200)
      found_straight = 1;

    // Check for the ending spot.
    // If all three middle sensors are on dark black, we have
    // solved the maze.
    if (sensors[1] > 600 && sensors[2] > 600 && sensors[3] > 600)
      break;

    // Intersection identification is complete.
    // If the maze has been solved, we can follow the existing
    // path.  Otherwise, we need to learn the solution.
    unsigned char dir = select_turn(found_left, found_straight, found_right);

    // Make the turn indicated by the path.
    turn(dir);

    // Store the intersection in the path variable.
    path[path_length].dir = dir;
    path[path_length].mils = endMils - startMils;
    path_length++;

    // You should check to make sure that the path_length does not
    // exceed the bounds of the array.  We'll ignore that in this
    // example.

    // Simplify the learned path.
    simplify_path();

    // Display the path on the LCD.
    display_path();
  }

  // Solved the maze!

  // Now enter an infinite loop - we can re-run the maze as many
  // times as we want to.
  finished = 0;
  while (!finished)
  {
    // Beep to show that we solved the maze.
    OrangutanMotors::setSpeeds(0, 0);
    OrangutanBuzzer::play(">>a32");

    // Wait for the user to press a button, while displaying
    // the solution.
    while (!OrangutanPushbuttons::isPressed(BUTTON_B))
    {
      if (millis() % 2000 < 1000)
      {
        OrangutanLCD::clear();
        OrangutanLCD::print("Solved!");
        OrangutanLCD::gotoXY(0, 1);
        OrangutanLCD::print("Press B");
      }
      else
        display_path();
      delay(30);
    }
    while (OrangutanPushbuttons::isPressed(BUTTON_B));

    delay(1000);

    // Re-run the maze.  It's not necessary to identify the
    // intersections, so this loop is really simple.
    int i;
    for (i = 0; i < path_length; i++)
    {
      unsigned long startMils, endMils, delta;
      startMils = millis();
      follow_segment();
      endMils = millis();

      delta = endMils - startMils;

      while (delta < 0.94 * path[i].mils) {
        startMils = millis();
        follow_segment();
        endMils = millis();
        delta += endMils - startMils;
      }

      // Drive straight while slowing down, as before.
      OrangutanMotors::setSpeeds(CRUISE_SPEED, CRUISE_SPEED);
      delay(50);
      OrangutanMotors::setSpeeds(SLOW_SPEED, SLOW_SPEED);
      delay(200);

      // Make a turn according to the instruction stored in
      // path[i].
      turn(path[i].dir);
    }

    // Follow the last segment up to the finish.
    follow_segment();

    // Now we should be at the finish!  Restart the loop.
  }
}
