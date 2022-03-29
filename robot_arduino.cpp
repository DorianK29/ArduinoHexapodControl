#include <hexapod.h>

using namespace std;

leg leg1, leg2, leg3, leg4, leg5, leg6;

// number of points in range to calculate
int steps;

/**
 * 1 = walk
 * 2 = walk to stand
 * 3 = stand
 * 4 = stand to walk
 */
int mode;

// širina hoda robota
float width, arduinoWidth;
// visina hoda robota
float height, arduinoHeight;

bool moveBackwards = false;

/**
 * false = 1,4,5
 * true = 2,3,6
 */
bool flyingLegs = false;

bool continuous;

// if we have already sent data for first sync
bool syncData = false;

// stop everything
bool stop = true;

int bluetooth = 44;

// ARDUINO:
// TEST: Try with min max values
void initServo()
{
    // leg1.motor1.servo.attach(34, leg1.motor1.min, leg1.motor1.max);
    // leg1.motor2.servo.attach(36, leg1.motor2.min, leg1.motor2.max);
    // leg1.motor3.servo.attach(38, leg1.motor3.min, leg1.motor3.max);
    // leg2.motor1.servo.attach(23, leg2.motor1.min, leg1.motor1.max);
    // leg2.motor2.servo.attach(25, leg2.motor2.min, leg1.motor2.max);
    // leg2.motor3.servo.attach(27, leg2.motor3.min, leg1.motor3.max);
    // leg3.motor1.servo.attach(28, leg3.motor1.min, leg3.motor1.max);
    // leg3.motor2.servo.attach(30, leg3.motor2.min, leg3.motor2.max);
    // leg3.motor3.servo.attach(32, leg3.motor3.min, leg3.motor3.max);
    // leg4.motor1.servo.attach(29, leg4.motor1.min, leg4.motor1.max);
    // leg4.motor2.servo.attach(31, leg4.motor2.min, leg4.motor2.max);
    // leg4.motor3.servo.attach(33, leg4.motor3.min, leg4.motor3.max);
    // leg5.motor1.servo.attach(22, leg5.motor1.min, leg5.motor1.max);
    // leg5.motor2.servo.attach(24, leg5.motor2.min, leg5.motor2.max);
    // leg5.motor3.servo.attach(26, leg5.motor3.min, leg5.motor3.max);
    // leg6.motor1.servo.attach(35, leg6.motor1.min, leg6.motor1.max);
    // leg6.motor2.servo.attach(37, leg6.motor2.min, leg6.motor2.max);
    // leg6.motor3.servo.attach(39, leg6.motor3.min, leg6.motor3.max);
    leg1.motor1.servo.attach(34);
    leg1.motor2.servo.attach(36);
    leg1.motor3.servo.attach(38);
    leg2.motor1.servo.attach(23);
    leg2.motor2.servo.attach(25);
    leg2.motor3.servo.attach(27);
    leg3.motor1.servo.attach(28);
    leg3.motor2.servo.attach(30);
    leg3.motor3.servo.attach(32);
    leg4.motor1.servo.attach(29);
    leg4.motor2.servo.attach(31);
    leg4.motor3.servo.attach(33);
    leg5.motor1.servo.attach(22);
    leg5.motor2.servo.attach(24);
    leg5.motor3.servo.attach(26);
    leg6.motor1.servo.attach(35);
    leg6.motor2.servo.attach(37);
    leg6.motor3.servo.attach(39);
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

// izracunaj motor2.angle i motor3.angle, ako imamo poziciju noge na x,z + motor1.angle
void leg_angle(int legNum)
{
    float leg_pos;
    leg *leg = &legSwitch(legNum);
    if (legNum == 1 || legNum == 2 || legNum == 3 || legNum == 4)
        leg_pos = -1;
    else if (legNum == 5 || legNum == 6)
        leg_pos = 1;

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

    float P = atan(leg->L.z / (sqrt(leg->L.x * leg->L.x + leg->L.y * leg->L.y)));
    float R = asin((leg->L.z - leg->l.z) / leg->l.length());

    leg->motor2.angle = PI / 2 + pow(-1, legNum) * (acos((pow(joint2, 2) + pow(leg->L.length(), 2) - pow(joint3, 2)) / (2 * joint2 * leg->L.length())) - (P + R));
    if (legNum % 2 == 0) // for even legs
        leg->motor3.angle = pow(-1, legNum) * (acos((pow(joint2, 2) + pow(joint3, 2) - pow(leg->L.length(), 2)) / (2 * joint2 * joint3)));
    else // for odd legs
        leg->motor3.angle = pow(-1, legNum) * (PI - acos((pow(joint2, 2) + pow(joint3, 2) - pow(leg->L.length(), 2)) / (2 * joint2 * joint3)));
}

// rotating the leg based on bool moveBackwards
void legRotation(int legNum, bool backwards)
{
    leg *local_leg = &legSwitch(legNum);
    int local_range = abs(local_leg->motor1.max + (-1) * local_leg->motor1.min);
    local_leg->motor1.angle += ((float)local_range / steps) * pow(-1, backwards);
    leg_angle(legNum);
}

void move()
{
    leg *local_leg;
    bool max = false;
    for (int i = 1; !max; i++)
    {
        for (int leg = 1; leg <= 6; leg++)
        {
            local_leg = &legSwitch(leg);
            if ((leg == 1 || leg == 4 || leg == 5) == !flyingLegs)
            {
                local_leg->D.x = width;
                local_leg->D.z = height;
            }
            else
            {
                float offset = pow(sin((float)i / steps * PI), 2 / 3);
                local_leg->D.x = width + 4 * offset;
                local_leg->D.z = height - 12 * offset;
            }
            // why this?
            if (leg == 3 || leg == 4)
                legRotation(leg, !moveBackwards);
            else
                legRotation(leg, moveBackwards);

            local_leg->motorRadToDeg();
            // TODO: min and max values changed big sad maybe no wrok?
            if (local_leg->motor1.angle > local_leg->motor1.max || local_leg->motor1.angle < local_leg->motor1.min)
                max = true;
            local_leg->motorDegToRad();
        }
        servoWrite();
    }
    flyingLegs = !flyingLegs;
    moveBackwards = !moveBackwards;
}

// ARDUINO:
// set stance of robot
void stance(char stance)
{
    switch (stance)
    {
    case 'z': // angles at zero
        leg1.motor1.angle = leg1.motor1.min;
        leg2.motor1.angle = leg2.motor1.min;
        leg3.motor1.angle = leg3.motor1.min;
        leg4.motor1.angle = leg4.motor1.min;
        leg5.motor1.angle = leg5.motor1.min;
        leg6.motor1.angle = leg6.motor1.min;
        break;
    case 'n': // normal stance
        leg1.motor1.angle = leg1.motor1.min + (leg1.motor1.max + (-1) * leg1.motor1.min) / 2;
        leg2.motor1.angle = leg2.motor1.min + (leg2.motor1.max + (-1) * leg2.motor1.min) / 2;
        leg3.motor1.angle = leg3.motor1.min + (leg3.motor1.max + (-1) * leg3.motor1.min) / 2;
        leg4.motor1.angle = leg4.motor1.min + (leg4.motor1.max + (-1) * leg4.motor1.min) / 2;
        leg5.motor1.angle = leg5.motor1.min + (leg5.motor1.max + (-1) * leg5.motor1.min) / 2;
        leg6.motor1.angle = leg6.motor1.min + (leg6.motor1.max + (-1) * leg6.motor1.min) / 2;
        break;
    }
}

// Leg angle fix CATIA to irl model
void angleFix(int legNum)
{
    leg *Leg = &legSwitch(legNum);

    Leg->motor3.angle = abs(Leg->motor3.angle);
    Leg->motor1.angle = 90 - Leg->motor1.angle;
    if (legNum == 4)
        Leg->motor2.angle -= 14;
}

// ARDUINO:
void servoWait(int legNum = 0)
{
    bool wait = true;
    bool waiting;
    int motorNum = 1;
    int tempLegNum = 1;
    do
    {
        if (legNum != 0) // for a given legNum
        {
            waiting = false;
            while (motorNum <= 3 && waiting == false)
            {
                int difference = legSwitch(legNum).motorSwitch(motorNum).angle - legSwitch(legNum).motorSwitch(motorNum).servo.read();
                if (difference >= 1)
                    waiting = true;
                if (!waiting)
                    motorNum++;
            }
        }
        else // for no given leg, ie. all legs
        {
            waiting = false;
            while (tempLegNum <= 6 && waiting == false)
            {
                while (motorNum <= 3 && waiting == false)
                {
                    int difference = legSwitch(tempLegNum).motorSwitch(motorNum).angle - legSwitch(tempLegNum).motorSwitch(motorNum).servo.read();
                    if (difference >= 1)
                        waiting = true;
                    if (!waiting)
                        motorNum++;
                }
                if (!waiting)
                {
                    tempLegNum++;
                    motorNum = 1;
                }
            }
        }
        wait = waiting;
        delay(50);
        if (wait)
            Serial.println("Servo Wait...");
    } while (wait);
}

void legWrite(leg *Leg, int legNum)
{
    Leg->motorRadToDeg();

    angleFix(legNum);

    Leg->motor1.servo.write(Leg->motor1.angle);
    Leg->motor2.servo.write(Leg->motor2.angle);
    Leg->motor3.servo.write(Leg->motor3.angle);

    // Print to Serial
    String buffer;
    buffer.concat("Leg: ");
    buffer.concat(legNum);
    buffer.concat(" motor1: ");
    buffer.concat(Leg->motor1.angle);
    buffer.concat(" motor2: ");
    buffer.concat(Leg->motor2.angle);
    buffer.concat(" motor3: ");
    buffer.concat(Leg->motor3.angle);
    Serial.println(buffer);

    servoWait(legNum);

    angleFix(legNum);

    // angleFix(legNum, true);
}

// ARDUINO:
void servoWrite()
{
    for (int legN = 1; legN <= 6; legN++)
        legWrite(&legSwitch(legN), legN);
    Serial.println();
}

void readMessage()
{
    if (Serial1.available() > 0) // on message recieved via bluetooth
    {
        String input = Serial1.readString();
        StringSplitter *recievedLine = new StringSplitter(input, '~', 5);

        for (int i = 0; i < recievedLine->getItemCount() && recievedLine->getItemAtIndex(i).length() > 0; i++) //  && text->getItemAtIndex(i).length() > 0
        {
            StringSplitter *line = new StringSplitter(recievedLine->getItemAtIndex(i), '&', 3);
            Serial.print("Reading: ");
            Serial.println(line->getItemAtIndex(0));
            if (line->getItemAtIndex(0) == "Move forwards")
            {
                flyingLegs = false;
                mode = 1;
                continuous = false;
            }
            else if (line->getItemAtIndex(0) == "Move backwards")
            {
                flyingLegs = true;
                mode = 1;
                continuous = false;
            }
            else if (line->getItemAtIndex(0) == "Continuous forwards")
            {
                flyingLegs = false;
                mode = 1;
                continuous = true;
            }
            else if (line->getItemAtIndex(0) == "Continuous backwards")
            {
                flyingLegs = true;
                mode = 1;
                continuous = true;
            }
            else if (line->getItemAtIndex(0) == "Update height and width")
            {
                Serial.print(arduinoHeight);
                Serial.print(" -> ");
                arduinoHeight = line->getItemAtIndex(1).toFloat();
                Serial.println(arduinoHeight);
                Serial.print(arduinoWidth);
                Serial.print(" -> ");
                arduinoWidth = line->getItemAtIndex(2).toFloat();
                Serial.println(arduinoWidth);
            }
            else if (line->getItemAtIndex(0) == "Stop")
            {
                if (line->getItemAtIndex(1) == "true")
                    stop = false;
                else if (line->getItemAtIndex(1) == "false")
                    stop = true;
                continuous = false;
            }
        }
    }
}

void sendMessage()
{
    if (!syncData && digitalRead(bluetooth) == HIGH) // if we havent synced and have connected to bluetooth
    {
        String buffer;
        Serial.println("Sending data");
        buffer = "##width:";
        buffer.concat(width);
        buffer.concat("##height:");
        buffer.concat(height);
        buffer.concat("##stop:");
        buffer.concat(stop);
        buffer.concat("~");
        Serial1.print(buffer);
        syncData = true;
    }

    if (digitalRead(bluetooth) == LOW) // if bluetooth device is disconnected reset sync data
        syncData = false;
}

void legValues();

// TODO: bluetooth variable change
void setup()
{
    Serial.begin(115200);
    Serial1.begin(9600);
    Serial1.setTimeout(20);

    legValues();
    initServo();

    Serial.println("Startup");
    mode = 3;
    steps = 10;
    width = 120; // APP:
    height = 80; // APP:
}

void loop()
{
    sendMessage();
    readMessage();

    if (stop)
        return;

    switch (mode)
    {
    case 1:
        // walk
        if (!continuous)
        {
            for (int i = 0; i < 5; i++)
                move();
            mode = 0;
        }
        else
            move();
        break;
    case 2:
        // walk to stand
        break;
    case 3:
        // stand
        stance('n');
        int sequence[6] = {3, 1, 6, 4, 2, 5};
        for (int i = 0; i < 6; i++)
        {
            // TODO: do motors one by one if they go too hard poggers
            leg *local_leg = &legSwitch(sequence[i]);
            local_leg->D.x = width;
            local_leg->D.z = height;
            leg_angle(sequence[i]);
            legWrite(local_leg, sequence[i]);
            delay(200); // looks better?
        }
        Serial.println();
        mode = 0;
        break;
    case 4:
        // stand to walk

        break;
    }
}

// set motor limits, joint values
void legValues()
{
    leg1.motor1.min = 10;
    leg1.motor1.max = 40;
    leg1.motor2.min = -60;
    leg1.motor2.max = 45;
    leg1.motor3.min = -160;
    leg1.motor3.max = 0;
    leg2.motor1.min = -40;
    leg2.motor1.max = -10;
    leg2.motor2.min = -45;
    leg2.motor2.max = 60;
    leg2.motor3.min = 0;
    leg2.motor3.max = 160;
    leg3.motor1.min = 0;
    leg3.motor1.max = 20;
    leg3.motor2.min = -60;
    leg3.motor2.max = 45;
    leg3.motor3.min = -160;
    leg3.motor3.max = 0;
    leg4.motor1.min = -20;
    leg4.motor1.max = 0;
    leg4.motor2.min = -45;
    leg4.motor2.max = 60;
    leg4.motor3.min = 0;
    leg4.motor3.max = 160;
    leg5.motor1.min = -30;
    leg5.motor1.max = 15;
    leg5.motor2.min = -60;
    leg5.motor2.max = 45;
    leg5.motor3.min = -160;
    leg5.motor3.max = 0;
    leg6.motor1.min = -15;
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

    // TEST:
    // TODO: will break program go to move if max :=)
    /*
    for (int i = 1; i <= 6; i++)
        for (int j = 1; j <= 3; j++)
        {
            if (j != 3)
            {
                legSwitch(i).motorSwitch(j).min += 90;
                legSwitch(i).motorSwitch(j).max += 90;
            }
            else
            {
                legSwitch(i).motorSwitch(j).min += 180;
                legSwitch(i).motorSwitch(j).max += 180;
            }

            // print values to change code above :=
            Serial.print("leg");
            Serial.print(i);
            Serial.print(".motor");
            Serial.print(j);
            Serial.print(".min = ");
            Serial.print(legSwitch(i).motorSwitch(j).min);
            Serial.print(";\n");

            Serial.print("leg");
            Serial.print(i);
            Serial.print(".motor");
            Serial.print(j);
            Serial.print(".max = ");
            Serial.print(legSwitch(i).motorSwitch(j).max);
            Serial.print(";\n");
        } */
}