#include <hexapod.h>

using namespace std;

/**     _____
 * leg1 | ^ | leg2
 * leg3 | | | leg4
 * leg5 |___| leg6
 */
leg leg1, leg2, leg3, leg4, leg5, leg6;

// number of points in range to calculate
int steps;

/**1 = walk
 * 2 = walk to stand
 * 3 = stand
 * 4 = stand to walk
 */
int mode;

// walking width (distance from the center of body to the ground contact point), y-axis
float width, arduinoWidth;
// walking height (distance from the center of body to the ground contact point), z-axis, downwards is positive
float height, arduinoHeight;

// are legs 1, 4, 5 moving backwards
bool moveBackwards = false;

/**which legs won't be touching the ground
 * false = 1,4,5
 * true = 2,3,6
 */
bool flyingLegs = false;

bool continuous; // move continuously

bool allowPrint = true; // allow printing

int sequence[6] = {3, 1, 6, 4, 2, 5}; // sequence of which the legs move when setting back to standing position
int speed[6] = {1, 2, 3, 5, 50, 65};  // array of speed values
int currentSpeed = 3;                 // array address values / -1

float addedHeight = 20;
float addedPi = 0;
float addedRo = 0;

// if we have already sent data for first sync
bool syncData = false;

// stop everything
bool stop = true;

// bluetooth module pin
int bluetooth = 44;

void initServo()
{
    leg1.motor1.servo.attach(34);
    leg1.motor2.servo.attach(36);
    leg1.motor3.servo.attach(38);
    leg2.motor1.servo.attach(23);
    leg2.motor2.servo.attach(25);
    leg2.motor3.servo.attach(27);
    leg4.motor1.servo.attach(28);
    leg4.motor2.servo.attach(30);
    leg4.motor3.servo.attach(32);
    leg3.motor1.servo.attach(29);
    leg3.motor2.servo.attach(31);
    leg3.motor3.servo.attach(33);
    leg6.motor1.servo.attach(22);
    leg6.motor2.servo.attach(24);
    leg6.motor3.servo.attach(26);
    leg5.motor1.servo.attach(35);
    leg5.motor2.servo.attach(37);
    leg5.motor3.servo.attach(39);
}

// leg switch case
leg &legSwitch(int i)
{
    switch (i)
    {
    case 1:
        return leg1;
        break;
    case 2:
        return leg2;
        break;
    case 3:
        return leg3;
        break;
    case 4:
        return leg4;
        break;
    case 5:
        return leg5;
        break;
    case 6:
        return leg6;
        break;
    }
}

// get motor2 and motor3
// with height, width and motor1's angle
void leg_angle(int legNum)
{
    float leg_pos;
    leg *leg = &legSwitch(legNum);
    if (legNum == 1 || legNum == 2)
        leg_pos = -1;
    else if (legNum == 5 || legNum == 6)
        leg_pos = 1;
    else if (legNum == 3)
    {
        if (leg->motor1.angle >= 0)
            leg_pos = -1;
        else
            leg_pos = 1;
    }
    else if (legNum == 4)
    {
        if (leg->motor1.angle >= 0)
            leg_pos = 1;
        else
            leg_pos = -1;
    }

    leg->motor1.DegToRad();

    leg->l.x = abs(leg->D.x) - abs(leg->s1.x);
    leg->l.z = leg->D.z + leg->s1.z;

    leg->D.y = tan(leg->motor1.angle) * leg->l.x - leg->s1.y;

    leg->s2.x = leg->s1.x + pow(-1, legNum) * joint1 * cos(leg->motor1.angle);
    leg->s2.y = leg->s1.y + pow(-1, leg_pos) * joint1 * sin(leg->motor1.angle);
    leg->s2.z = leg->s1.z; // all negative

    leg->L.x = abs(leg->D.x) - abs(leg->s2.x);
    leg->L.y = leg->D.y + leg->s2.y; // depends on leg pos
    leg->L.z = leg->D.z + leg->s2.z; // D - s2 (s2 all legs negative)

    float P = atan(leg->L.z / (sqrt(leg->L.x * leg->L.x + leg->L.y * leg->L.y))) + addedPi * DEG_TO_RAD;
    float R = asin((leg->L.z - leg->l.z) / leg->l.length()) + addedRo * DEG_TO_RAD;
    leg->motor2.angle = PI / 2 + pow(-1, legNum + 1) * (acos((pow(joint2, 2) + pow(leg->L.length(), 2) - pow(joint3, 2)) / (2 * joint2 * leg->L.length())) - (P + R));
    if (legNum % 2 == 0) // for even legs
        leg->motor3.angle = pow(-1, legNum) * (acos((pow(joint2, 2) + pow(joint3, 2) - pow(leg->L.length(), 2)) / (2 * joint2 * joint3)));
    else // for odd legs
        leg->motor3.angle = pow(-1, legNum) * (PI - acos((pow(joint2, 2) + pow(joint3, 2) - pow(leg->L.length(), 2)) / (2 * joint2 * joint3)));
}

// increment current angle
void legRotation(int legNum, bool backwards)
{
    leg *local_leg = &legSwitch(legNum);
    int local_range = abs(local_leg->motor1.max + (-1) * local_leg->motor1.min); // maximal range of movement
    // add or substract the maximal range of movement divided by the amount of steps in a single back and forth cycle
    local_leg->motor1.angle += ((float)local_range / steps) * pow(-1, backwards);
    leg_angle(legNum); // calculate the angles of motor2 and motor3
}

// move a leg until it reaches its maximal or minimal allowed value
void move()
{
    // when finishing a cycle the flying legs boolen needs to be reversed so the robot keeps moving in the same direction
    // if it doesn't change he will go one gait forwards and then one gait backwards, and won't move anywhere
    leg *local_leg;
    bool max = false;
    for (int currentStep = 1; !max; currentStep++) // counts how many steps the robot is currently on
    {
        for (int leg = 1; leg <= 6; leg++) // for each leg
        {
            local_leg = &legSwitch(leg);
            // set the appropriate width and height
            // in one cycle legs 1, 4, 5 will be touching the ground
            // and on the other cycle they will be in the air (flyingLegs)
            if ((leg == 1 || leg == 4 || leg == 5) == !flyingLegs)
            {
                local_leg->D.x = width;
                local_leg->D.z = height;
            }
            else
            {
                // allows for a more gradual offset of legs
                float offset = pow(sin((float)currentStep / steps * PI), 2 / 9);
                local_leg->D.x = width;
                local_leg->D.z = height - addedHeight * offset;
            }
            if (leg == 3 || leg == 4)
                legRotation(leg, !moveBackwards);
            else
                legRotation(leg, moveBackwards);
            // after calculating the needed leg angles
            // change them to degrees so they can be compared to their minimal and maximal values
            local_leg->motorRadToDeg();
            // if we have reached the maximal or minimal point of motor1's movement range
            // switch which legs are in the air and change which legs are going backwards
            if (local_leg->motor1.angle > local_leg->motor1.max || local_leg->motor1.angle < local_leg->motor1.min)
                max = true;
            local_leg->motorDegToRad(); // change back to radians so they can be sent to their servos
        }
        servoWrite(); // write the calculated angle values to the servos
    }
    flyingLegs = !flyingLegs;       // change which legs are touching the ground
    moveBackwards = !moveBackwards; // change which legs are going backwards
}

// set stance of robot
void stance(char stance)
{
    switch (stance)
    {
    case 'z': // motor1 at its minimal value
        leg1.motor1.angle = leg1.motor1.min;
        leg2.motor1.angle = leg2.motor1.min;
        leg3.motor1.angle = leg3.motor1.min;
        leg4.motor1.angle = leg4.motor1.min;
        leg5.motor1.angle = leg5.motor1.min;
        leg6.motor1.angle = leg6.motor1.min;
        break;
    case 'n': // motor1 in the middle of its allowed values
        leg1.motor1.angle = leg1.motor1.min + (leg1.motor1.max + (-1) * leg1.motor1.min) / 2;
        leg2.motor1.angle = leg2.motor1.min + (leg2.motor1.max + (-1) * leg2.motor1.min) / 2;
        leg3.motor1.angle = leg3.motor1.min + (leg3.motor1.max + (-1) * leg3.motor1.min) / 2;
        leg4.motor1.angle = leg4.motor1.min + (leg4.motor1.max + (-1) * leg4.motor1.min) / 2;
        leg5.motor1.angle = leg5.motor1.min + (leg5.motor1.max + (-1) * leg5.motor1.min) / 2;
        leg6.motor1.angle = leg6.motor1.min + (leg6.motor1.max + (-1) * leg6.motor1.min) / 2;
        break;
    }
}

// change CATIA angles to robot angles
void angleFix(int legNum)
{
    leg *Leg = &legSwitch(legNum);

    Leg->motor3.angle = abs(Leg->motor3.angle);
    Leg->motor1.angle = 90 - Leg->motor1.angle;

    if (legNum == 3)
        Leg->motor2.angle -= 19;
    /*
else if (legNum == 4)
Leg->motor2.angle += 5;
else if (legNum == 5)
Leg->motor3.angle -= 5;
*/
}

// wait for given leg
void motorWait(int legNum)
{
    int motorNum = 1;
    while (motorNum <= 3)
    {
        bool waiting = false; // reset value
        motor currentMotor = legSwitch(legNum).motorSwitch(motorNum);
        int calcAngle = currentMotor.angle;         // calculated angle
        int servoAngle = currentMotor.servo.read(); // current angle on servo motor
        int difference = calcAngle - servoAngle;    // difference between the calculated angle and the servo motor
        // if motor is not in position wait
        // a motor is in postition if the difference is less than 1 degree
        if (difference >= 1)
            waiting = true;
        if (waiting)
        {
            serialPrint("Servo Wait..."); // print wait if the difference is more than 1 degree
        }
        else // when current motor is in position go to the next motor
        {
            motorNum++;
            waiting = false;
        }
    }
    delay(speed[currentSpeed]);
}

// wait for a given leg if given a legNum
// if not given anything wait for all legs
void servoWait(int legNum = 0)
{
    if (legNum != 0) // for a given legNum
        motorWait(legNum);
    else // for no given leg, ie. all legs
        for (int tempLegNum = 1; tempLegNum <= 6; tempLegNum++)
            motorWait(tempLegNum);
}

// write calculated values to all servos of a leg
void legWrite(leg *Leg, int legNum)
{
    Leg->motorRadToDeg(); // change calculated angle to degrees

    angleFix(legNum); // correct angles

    // write calculated angles to the servo motors
    Leg->motor1.servo.write(Leg->motor1.angle);
    Leg->motor2.servo.write(Leg->motor2.angle);
    Leg->motor3.servo.write(Leg->motor3.angle);

    // print to serial
    String buffer = "Leg: ";
    buffer.concat(legNum);
    buffer.concat(" motor1: ");
    buffer.concat(Leg->motor1.angle);
    buffer.concat(" motor2: ");
    buffer.concat(Leg->motor2.angle);
    buffer.concat(" motor3: ");
    buffer.concat(Leg->motor3.angle);
    serialPrint(buffer);

    // wait for the servo to reach the calculated angle
    servoWait(legNum);

    angleFix(legNum);
}

// write calculated values to all legs
void servoWrite()
{
    for (int legN = 1; legN <= 6; legN++)
        legWrite(&legSwitch(legN), legN);
    serialPrint("");
}

// read message from bluetooth
void readMessage()
{
    if (Serial1.available() > 0) // on message recieved via bluetooth
    {
        String buffer;
        String input = Serial1.readString();
        StringSplitter *recievedLine = new StringSplitter(input, '~', 5);

        for (int i = 0; i < recievedLine->getItemCount() && recievedLine->getItemAtIndex(i).length() > 0; i++) //  && text->getItemAtIndex(i).length() > 0
        {
            StringSplitter *line = new StringSplitter(recievedLine->getItemAtIndex(i), '&', 3);

            buffer = "Reading: ";
            buffer.concat(line->getItemAtIndex(0));
            serialPrint(buffer);
            if (line->getItemAtIndex(0) == "Move forwards")
            {
                flyingLegs = false;
                mode = 1;
            }
            else if (line->getItemAtIndex(0) == "Move backwards")
            {
                flyingLegs = true;
                mode = 1;
            }
            else if (line->getItemAtIndex(0) == "Continuous forwards")
            {
                flyingLegs = false;
                if (line->getItemAtIndex(1) == "true")
                    mode = 2;
                else if (line->getItemAtIndex(1) == "false")
                    mode = 3;
            }
            else if (line->getItemAtIndex(0) == "Continuous backwards")
            {
                flyingLegs = true;
                if (line->getItemAtIndex(1) == "true")
                    mode = 2;
                else if (line->getItemAtIndex(1) == "false")
                    mode = 3;
            }
            else if (line->getItemAtIndex(0) == "Update height and width")
            {
                buffer = arduinoHeight;
                buffer.concat(" -> ");
                arduinoHeight = line->getItemAtIndex(1).toFloat();
                buffer.concat(arduinoHeight);
                serialPrint(buffer);
                buffer = arduinoWidth;
                buffer.concat(" -> ");
                arduinoWidth = line->getItemAtIndex(2).toFloat();
                buffer.concat(arduinoWidth);
                serialPrint(buffer);
                mode = 4;
            }
            else if (line->getItemAtIndex(0) == "Speed change")
                currentSpeed = (int)line->getItemAtIndex(1).toFloat();
            else if (line->getItemAtIndex(0) == "Serial print")
            {
                if (line->getItemAtIndex(1) == "true")
                    allowPrint = true;
                else if (line->getItemAtIndex(1) == "false")
                    allowPrint = false;
            }
            else if (line->getItemAtIndex(0) == "Stop")
            {
                if (line->getItemAtIndex(1) == "true")
                    stop = true;
                else if (line->getItemAtIndex(1) == "false")
                    stop = false;
            }
            else if (line->getItemAtIndex(0) == "Stand")
                mode = 3;
        }
    }
}

// send message to bluetooth
void sendMessage()
{
    if (!syncData && digitalRead(bluetooth) == HIGH) // if we havent synced and have connected to a bluetooth device
    {
        String buffer;
        serialPrint("Sending data");
        buffer = "##width:";
        buffer.concat(width);
        buffer.concat("##height:");
        buffer.concat(height);
        buffer.concat("##stop:");
        buffer.concat(stop);
        buffer.concat("##print:");
        buffer.concat(allowPrint);
        buffer.concat("##speed:");
        buffer.concat(currentSpeed);
        buffer.concat("~");
        Serial1.print(buffer);
        syncData = true;
    }

    if (digitalRead(bluetooth) == LOW) // if bluetooth device is disconnected make it so we can sync again
        syncData = false;
}

// print to serial if its allowed
void serialPrint(String output)
{
    // if (allowPrint) // if we have allowed printing print given string
    Serial.println(output);
}

void legValues();

void setup()
{
    Serial.begin(115200);   // setup serial to baud rate 115200
    Serial1.begin(9600);    // setup bluetooth to baud rate 9600
    Serial1.setTimeout(20); // lower delay on bluetooth transmition

    legValues(); // set leg min, max, joint values
    initServo(); // attach servos to pins

    serialPrint("Startup"); // print startup to serial
    allowPrint = false;     // after printing startup disable printing until recieving data from bluetooth
    mode = 3;               // start in standing mode
    steps = 10;             // number of steps in a movement from min to max angle
    width = 90;             // width of gait
    height = 100;           // height of gait
}

void loop()
{
    // every cycle check if any new bluetooth data recieved
    // check if a bluetooth device has reconnected and sync data
    sendMessage();
    readMessage();

    if (stop)
        return;

    switch (mode)
    {
    case 1:
        // walk a bit
        for (int i = 0; i < 5; i++)
            move();
        mode = 3; // after finishing movement stand
        break;
    case 2:
        // continuous walk
        move();
        break;
    case 3:
        // stand
        stance('n'); // set motor1 angles to their center value
        for (int i = 0; i < 6; i++)
        {
            leg *local_leg = &legSwitch(sequence[i]);
            local_leg->D.x = width;                // set width of walking
            local_leg->D.z = height - addedHeight; // set height of walking to 20 above of desired point so legs dont drag on the ground
            leg_angle(sequence[i]);                // calculate motor2 and motor3 angles
            legWrite(local_leg, sequence[i]);      // write calculated values to leg servos
            delay(speed[currentSpeed]);            // delay to make movement smoother
            local_leg->D.z = height;               // set height of walking to desired value
            leg_angle(sequence[i]);                // calculate motor2 and motor3 angles
            legWrite(local_leg, sequence[i]);      // write calculated values to leg servos
            delay(speed[currentSpeed] * 9);        // delay between movement of different legs
        }
        serialPrint(""); // print new line
        mode = 0;        // wait until further input
        break;
    case 4:
        // update heigth and width
        width = arduinoWidth; // set width to arduino inputed width and move legs one by one
        for (int i = 0; i < 6; i++)
        {
            // same as mode 3
            leg *local_leg = &legSwitch(sequence[i]);
            local_leg->D.x = width;
            local_leg->D.z = height - addedHeight;
            leg_angle(sequence[i]);
            legWrite(local_leg, sequence[i]);
            delay(speed[currentSpeed]);
            local_leg->D.z = height;
            leg_angle(sequence[i]);
            legWrite(local_leg, sequence[i]);
            delay(speed[currentSpeed] * 9);
        }
        height = arduinoHeight; // set heigth to arduino inputed height and move legs all at the same time
        for (int i = 0; i < 6; i++)
        {
            leg *local_leg = &legSwitch(sequence[i]);
            local_leg->D.z = height; // set new height for each leg
            leg_angle(sequence[i]);  // calculate other motor angles for each leg
        }
        servoWrite();    // write to all legs at the same time
        serialPrint(""); // print new line
        mode = 0;        // wait until further input
        break;
    }
}

// set motor limits, joint values
void legValues()
{
    leg1.motor1.min = 10;
    leg1.motor1.max = 30;
    leg1.motor2.min = -60;
    leg1.motor2.max = 45;
    leg1.motor3.min = -160;
    leg1.motor3.max = 0;
    leg2.motor1.min = -30;
    leg2.motor1.max = -10;
    leg2.motor2.min = -45;
    leg2.motor2.max = 60;
    leg2.motor3.min = 0;
    leg2.motor3.max = 160;
    leg3.motor1.min = -10;
    leg3.motor1.max = 10;
    leg3.motor2.min = -60;
    leg3.motor2.max = 45;
    leg3.motor3.min = -160;
    leg3.motor3.max = 0;
    leg4.motor1.min = -10;
    leg4.motor1.max = 10;
    leg4.motor2.min = -45;
    leg4.motor2.max = 60;
    leg4.motor3.min = 0;
    leg4.motor3.max = 160;
    leg5.motor1.min = -30;
    leg5.motor1.max = -10;
    leg5.motor2.min = -60;
    leg5.motor2.max = 45;
    leg5.motor3.min = -160;
    leg5.motor3.max = 0;
    leg6.motor1.min = 10;
    leg6.motor1.max = 30;
    leg6.motor2.min = -45;
    leg6.motor2.max = 60;
    leg6.motor3.min = 0;
    leg6.motor3.max = 160;

    leg1.s1.y = leg2.s1.y = -57;
    leg3.s1.y = leg4.s1.y = 0;
    leg5.s1.y = leg6.s1.y = 57;

    leg1.s1.z = leg2.s1.z = leg3.s1.z = leg4.s1.z = leg5.s1.z = leg6.s1.z = -16.3;
    leg1.s1.x = leg3.s1.x = leg5.s1.x = -23.5;
    leg2.s1.x = leg4.s1.x = leg6.s1.x = 23.5;
}