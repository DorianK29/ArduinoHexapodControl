#include <hexapod.h>

using namespace std;

leg leg1, leg2, leg3, leg4, leg5, leg6;

// number of points in range to calculate
int steps, range;

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

// move forwards if false, backwards if true
bool backwards = false;

// if we have already sent data for first sync
bool syncData = false;

// stop everything
bool stop = true;

// TODO: ARDUINO:
int button = 1, led = 1;

int bluetooth = 44;

// ARDUINO: TODO: real pins
void initServo()
{
    // leg1.motor1.servo.attach(22);
    // leg1.motor2.servo.attach(24);
    // leg1.motor3.servo.attach(26);
    // leg2.motor1.servo.attach(23);
    // leg2.motor2.servo.attach(25);
    // leg2.motor3.servo.attach(27);
    // leg3.motor1.servo.attach(28);
    // leg3.motor2.servo.attach(30);
    // leg3.motor3.servo.attach(32);
    // leg4.motor1.servo.attach(29);
    // leg4.motor2.servo.attach(31);
    // leg4.motor3.servo.attach(33);
    // leg5.motor1.servo.attach(34);
    // leg5.motor2.servo.attach(36);
    // leg5.motor3.servo.attach(38);
    // leg6.motor1.servo.attach(35);
    // leg6.motor2.servo.attach(37);
    // leg6.motor3.servo.attach(39);
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

// TODO: check for collision
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
    leg->s2.z = leg->s1.z;

    leg->L.x = abs(leg->D.x) - abs(leg->s2.x);
    leg->L.y = leg->D.y + leg->s2.y;
    leg->L.z = leg->D.z + leg->s2.z;

    float P = atan(leg->L.z / (sqrt(leg->L.x * leg->L.x + leg->L.y * leg->L.y)));
    float R = asin((leg->L.z - leg->l.z) / leg->l.length());

    leg->motor2.angle = pow(-1, legNum) * (acos((pow(joint2, 2) + pow(leg->L.length(), 2) - pow(joint3, 2)) / (2 * joint2 * leg->L.length())) - (P + R));
    leg->motor3.angle = pow(-1, legNum) * (PI - acos((pow(joint2, 2) + pow(joint3, 2) - pow(leg->L.length(), 2)) / (2 * joint2 * joint3)));
}

// rotating the leg based on bool backwards
void legRotation(int j, bool backwards)
{
    leg *local_leg = &legSwitch(j);
    int local_range;
    if (range == -1)
        local_range = abs(local_leg->motor1.max) + abs(local_leg->motor1.min);
    else
        local_range = range;
    local_leg->motor1.angle += (local_range / steps) * pow(-1, backwards);
    leg_angle(j);
}

// robot moving based on bool backwards
void move()
{
    leg *local_leg;
    for (int j = 1; j <= 2;)
        for (int i = 1; i <= steps && j <= 2; i++)
        {
            bool max;
            for (int leg = 1; leg <= 6; leg++)
            {
                local_leg = &legSwitch(leg);
                if (leg == 1 || leg == 4 || leg == 5)
                {
                    local_leg->D.x = width;
                    local_leg->D.z = height;
                }
                else if (leg == 2 || leg == 3 || leg == 6)
                {
                    float offset = sin((float)i / steps * PI);
                    local_leg->D.x = width + 4 * offset;
                    local_leg->D.z = height - 12 * offset;
                }
                if (leg == 3 || leg == 4)
                    legRotation(leg, !backwards);
                else
                    legRotation(leg, backwards);
                local_leg->motorRadToDeg();

                if (local_leg->motor1.angle >= local_leg->motor1.max || local_leg->motor1.angle <= local_leg->motor1.min)
                    max = true;
                local_leg->motorDegToRad();
            }
            if (max)
            {
                max = false;
                j++;
                backwards = !backwards;
            }
            servoWrite();
        }
}

// ARDUINO:
// set stance of robot
void stance(char stance)
{
    switch (stance)
    {
    case 'z': // angles at zero
        leg1.motor1.angle = 0;
        leg2.motor1.angle = -40;
        leg3.motor1.angle = 20;
        leg4.motor1.angle = 0;
        leg5.motor1.angle = -30;
        leg6.motor1.angle = -15;
        break;
    case 'n': // normal stance
        leg1.motor1.angle = 20;
        leg2.motor1.angle = -20;
        leg3.motor1.angle = 10;
        leg4.motor1.angle = -10;
        leg5.motor1.angle = -8;
        leg6.motor1.angle = 7;
        break;
    }
}

// Leg angle fix CATIA to irl model
void angleFix(leg *Leg, bool minus)
{
    Leg->motor1.angle = 90 - Leg->motor1.angle;
    if (Leg == &leg2 || Leg == &leg4 || Leg == &leg6) // for legs 2 4 6 motors 2 and 3 are already positive so no need to change
        return;
    Leg->motor2.angle += 90 * pow(-1, minus);
    Leg->motor3.angle += 180 * pow(-1, minus);
    Leg->motor3.angle = 180 + Leg->motor3.angle * pow(-1, !minus);
}

void legWrite(leg *Leg, int legNum)
{
    Leg->motorRadToDeg();

    angleFix(Leg, false);

    Leg->motor1.servo.write(Leg->motor1.angle);
    Leg->motor2.servo.write(Leg->motor2.angle);
    Leg->motor3.servo.write(Leg->motor3.angle);

    // Print to Serial
    Serial.print("Leg: ");
    Serial.print(legNum);
    Serial.print(" motor1: ");
    Serial.print(Leg->motor1.angle);
    Serial.print(" motor2: ");
    Serial.print(Leg->motor2.angle);
    Serial.print(" motor3: ");
    Serial.println(Leg->motor3.angle);

    // servoWait(legNum);
}

// ARDUINO: TODO: check servo.read output, change accordingly
void servoWait(int legNum = 0)
{
    Serial.println("Servo Wait...");
    bool wait = true;
    bool waiting;
    int motorNum = 1;
    int tempLegNum = 1;
    do
    {
        // readMessage();
        if (legNum != 0) // for a given legNum
        {
            waiting = false;
            for (1; motorNum <= 3 && waiting == false;)
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
            for (1; tempLegNum <= 6 && waiting == false;)
            {
                for (1; motorNum <= 3 && waiting == false;)
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
        delay(250);
    } while (wait);
    if (legNum == 0)
        for (int legN = 1; legN <= 6; legN++)
            angleFix(&legSwitch(legN), true);
    else
        angleFix(&legSwitch(legNum), true);
}

// ARDUINO:
void servoWrite()
{
    for (int legN = 1; legN <= 6; legN++)
        legWrite(&legSwitch(legN), legN);

    servoWait();
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
                backwards = false;
            else if (line->getItemAtIndex(0) == "Move backwards")
                backwards = true;
            else if (line->getItemAtIndex(0) == "Update height and width")
            {
                Serial.println(line->getItemAtIndex(1).toFloat());
                Serial.println(line->getItemAtIndex(2).toFloat());
                arduinoHeight = line->getItemAtIndex(1).toFloat();
                arduinoWidth = line->getItemAtIndex(2).toFloat();
                Serial.println(arduinoHeight);
                Serial.println(arduinoWidth);
            }
            else if (line->getItemAtIndex(0) == "Stop")
                if (line->getItemAtIndex(1) == "true")
                    stop = false;
                else if (line->getItemAtIndex(1) == "false")
                    stop = true;
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
        syncData = true;
    }

    if (digitalRead(bluetooth) == LOW) // if bluetooth device is disconnected reset sync data
        syncData = false;
}

void legValues();
Servo tes;
// TODO: bluetooth variable change
// TODO: figure out range
void setup()
{
    Serial.begin(115200);
    Serial1.begin(9600);

    // pinMode(button, INPUT_PULLUP);
    // pinMode(led, OUTPUT);

    legValues();
    initServo();
    //  cout << "mode: ";
    //  cin >> mode;

    Serial.println("Startup");
    tes.attach(47);
    mode = 1;
    steps = 10; // TODO: figure out best num
    // max range = -1 = abs(motor1.max) + abs(motor1.min)
    range = -1;  // useless?
    width = 120; // APP:
    height = 80; // APP:

    // ARDUINO: turn led on while robot is turned on
    // TODO: maybe rgb for different states
    // digitalWrite(led, 1);
}

void loop()
{
    sendMessage();
    readMessage();

    if (stop)
        return;

    // TODO: EEPROM write read, + test

    switch (mode)
    {
    case -1:
        // shut down
        Serial.print("leg1");
        Serial.print("leg2");
        Serial.print("leg3");
        Serial.print("leg4");
        Serial.print("leg5");
        Serial.println("leg6");
        /*
        EEPROM.put(0, leg1);
        EEPROM.put(sizeof(leg), leg2);
        EEPROM.put(sizeof(leg) * 2, leg3);
        EEPROM.put(sizeof(leg) * 3, leg4);
        EEPROM.put(sizeof(leg) * 4, leg5);
        EEPROM.put(sizeof(leg) * 5, leg6); */
    case 0:
        // wake up
        Serial.print("leg1");
        Serial.print("leg2");
        Serial.print("leg3");
        Serial.print("leg4");
        Serial.print("leg5");
        Serial.println("leg6");
        /*EEPROM.get(0, leg1);
        EEPROM.get(sizeof(leg), leg2);
        EEPROM.get(sizeof(leg) * 2, leg3);
        EEPROM.get(sizeof(leg) * 3, leg4);
        EEPROM.get(sizeof(leg) * 4, leg5);
        EEPROM.get(sizeof(leg) * 5, leg6); */
        break;
    case 1:
        // walk
        stance('n');
        move();
        // mode++;
        break;
    case 2:
        // walk to stand
        break;
    case 3:
        // stand
        stance('n');
        int sequence[6] = {5, 4, 3, 2, 1, 6};
        for (int i = 0; i < 6; i++)
        {
            leg *local_leg = &legSwitch(sequence[i]);
            local_leg->D.x = width;
            local_leg->D.z = height;
            leg_angle(sequence[i]);
            legWrite(local_leg, sequence[i]);
            servoWait(sequence[i]);
        }
        break;
    case 4:
        // stand to walk
        delay(500);
        break;
    }
}

// set motor limits, joint values
void legValues()
{
    leg1.motor1.min = 0;
    leg1.motor1.max = 40;
    leg1.motor2.min = -60;
    leg1.motor2.max = 45;
    leg1.motor3.min = -160;
    leg1.motor3.max = 0;
    leg1.s1.y = -57;
    leg2.motor1.min = -40;
    leg2.motor1.max = 0;
    leg2.motor2.min = -45;
    leg2.motor2.max = 60;
    leg2.motor3.min = 0;
    leg2.motor3.max = 160;
    leg2.s1.y = -57;
    leg3.motor1.min = 0;
    leg3.motor1.max = 20;
    leg3.motor2.min = -60;
    leg3.motor2.max = 45;
    leg3.motor3.min = -160;
    leg3.motor3.max = 0;
    leg3.s1.y = 0;
    leg4.motor1.min = -20;
    leg4.motor1.max = 0;
    leg4.motor2.min = -45;
    leg4.motor2.max = 60;
    leg4.motor3.min = 0;
    leg4.motor3.max = 160;
    leg4.s1.y = 0;
    leg5.motor1.min = -30;
    leg5.motor1.max = 15;
    leg5.motor2.min = -60;
    leg5.motor2.max = 45;
    leg5.motor3.min = -160;
    leg5.motor3.max = 0;
    leg5.s1.y = 57;
    leg6.motor1.min = -15;
    leg6.motor1.max = 30;
    leg6.motor2.min = -45;
    leg6.motor2.max = 60;
    leg6.motor3.min = 0;
    leg6.motor3.max = 160;
    leg6.s1.y = 57;

    leg1.s1.z = leg2.s1.z = leg3.s1.z = leg4.s1.z = leg5.s1.z = leg6.s1.z = -16.3;
    leg1.s1.x = leg3.s1.x = leg5.s1.x = -23.5;
    leg2.s1.x = leg4.s1.x = leg6.s1.x = 23.5;
}