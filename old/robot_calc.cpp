#include <iostream>
#include <stdio.h>
#include <math.h>

#define joint1 33.5
#define joint2 57.5
#define joint3 113

#define PI 3.1415926535
#define Deg 180 / PI
#define Rad PI / 180

using namespace std;

// DEBUG:
struct test_struct
{
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;
    float g;
    float h;
} test;

struct motor
{
    float angle;
    int min, max;
    void DegToRad()
    {
        angle = angle * Rad;
    }
    void RadToDeg()
    {
        angle = angle * Deg;
    }
};

struct vector
{
    float x, y, z;
    float length()
    {
        float l = sqrt(x * x + y * y + z * z);
        return l;
    }
};

struct leg
{
    /**
     * angle = kut motora
     * min = najmanji kut koji motor moze imat
     * max = najveci kut koji motor moze imat
     */
    motor motor1, motor2, motor3;
    /**
     * D = od sredista tijela do tocke gdje noga dira pod
     * l = od tocke rotacije na m1 do tocke gdje noga dira pod
     * L = od tocke rotacija na m2 do tocke gdje noga dira pod
     * s1 = od sredista tijela robota do tocke rotacije na m1
     * s2 = od sredista tijela robota do tocke rotacije na m2
     * x, y, z = projekcije linija na odredenu os
     * length = duzina linije
     */
    vector D, l, L, s1, s2;
    void motorRadToDeg()
    {
        motor1.RadToDeg();
        motor2.RadToDeg();
        motor3.RadToDeg();
    }
    void motorDegToRad()
    {
        motor1.DegToRad();
        motor2.DegToRad();
        motor3.DegToRad();
    }
} leg1, leg2, leg3, leg4, leg5, leg6;

// number of points in range to calculate
int steps;
int range;

// širina hoda robota
float width;
// visina hoda robota
float height;

// void getM();
// void getHw();

motor motorSwitch(int num, leg leg);
leg &legSwitch(int i);
void leg_angle();
void move(bool backwards);
void legRotation(int j, bool backwards);
bool checkMotors(motor motor);
void printMotor();
void stance(char stance);

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

// TODO: bluetooth variable change
// TODO: figure out range
int main()
{
    int mode;
    legValues();

    // cout << "mode: ";
    // cin >> mode;

    // DEBUG:
    mode = 1;
    steps = 2;
    // max range = -1 = abs(motor1.max) + abs(motor1.min)
    range = -1;
    width = 150;
    height = 80;

    stance('s');

    switch (mode)
    {
    case 1:
        move(false);
        break;
    }

    return 0;

    // getHW();
}

// TODO: check for collision
// izracunaj motor2.angle i motor3.angle, ako imamo poziciju noge na x,z + motor1.angle
void leg_angle(int j)
{
    float leg_pos;
    leg *leg = &legSwitch(j);
    if (j == 1 || j == 2 || j == 3 || j == 4)
        leg_pos = -1;
    else if (j == 5 || j == 6)
        leg_pos = 1;

    leg->motor1.DegToRad();

    leg->l.x = abs(leg->D.x) - abs(leg->s1.x);
    leg->l.z = leg->D.z + leg->s1.z;

    leg->D.y = tan(leg->motor1.angle) * leg->l.x - leg->s1.y;

    leg->s2.x = leg->s1.x + pow(-1, j) * joint1 * cos(leg->motor1.angle);
    leg->s2.y = leg->s1.y + pow(-1, leg_pos) * joint1 * sin(leg->motor1.angle);
    leg->s2.z = leg->s1.z;

    leg->L.x = abs(leg->D.x) - abs(leg->s2.x);
    leg->L.y = leg->D.y + leg->s2.y;
    leg->L.z = leg->D.z + leg->s2.z;

    float P = atan(leg->L.z / (sqrt(leg->L.x * leg->L.x + leg->L.y * leg->L.y)));
    float R = asin((leg->L.z - leg->l.z) / leg->l.length());

    leg->motor2.angle = pow(-1, j) * (acos((pow(joint2, 2) + pow(leg->L.length(), 2) - pow(joint3, 2)) / (2 * joint2 * leg->L.length())) - (P + R));
    leg->motor3.angle = pow(-1, j) * (PI - acos((pow(joint2, 2) + pow(joint3, 2) - pow(leg->L.length(), 2)) / (2 * joint2 * joint3)));
}

// robot moving based on bool backwards
void move(bool backwards)
{
    leg *local_leg;
    for (int j = 1; j <= 2;)
        for (int i = 1; i <= steps && j <= 2; i++)
        {
            bool max;
            printf("\n");
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
            printMotor();
            if (max)
            {
                max = false;
                j++;
                backwards = !backwards;
            }
        }
    cout << endl;
    system("pause");
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

// motor switch case
motor motorSwitch(int num, leg leg)
{
    switch (num)
    {
    case 1:
        return leg.motor1;
        break;
    case 2:
        return leg.motor2;
        break;
    case 3:
        return leg.motor3;
        break;
    }
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

// throw error if motor angle is out of its working range
bool checkMotors(motor motor)
{
    return 0;
    for (int i = 1; i <= 3; i++)
        if (motor.angle < motor.min || motor.angle > motor.max)
        {
            printf("\n\nERROR: motor%i = %f\nLIMITS: min = %f max = %f\n\n", i, motor.angle, motor.min, motor.max);
            return 1;
        }
    return 0;
}

// print out motor values
void printMotor()
{
    leg1.motorRadToDeg();
    leg2.motorRadToDeg();
    leg3.motorRadToDeg();
    leg4.motorRadToDeg();
    leg5.motorRadToDeg();
    leg6.motorRadToDeg();

    for (int i = 1; i <= 6; i++)
    {
        for (int j = 1; j <= 3; j++)
            if (checkMotors(motorSwitch(j, legSwitch(i))))
                system("pause");
        printf("Leg %d: ", i);
        printf("motor1: %4.1f motor2: %5.3f motor3: %5.3f\n", legSwitch(i).motor1.angle, legSwitch(i).motor2.angle, legSwitch(i).motor3.angle);
    }
}

// set stance of robot
void stance(char stance)
{
    switch (stance)
    {
    case 'z':
        leg1.motor1.angle = 0;
        leg2.motor1.angle = -40;
        leg3.motor1.angle = 20;
        leg4.motor1.angle = 0;
        leg5.motor1.angle = -30;
        leg6.motor1.angle = -15;
        break;
    case 's':
        leg1.motor1.angle = 20;
        leg2.motor1.angle = -20;
        leg3.motor1.angle = 10;
        leg4.motor1.angle = -10;
        leg5.motor1.angle = -8;
        leg6.motor1.angle = 7;
        break;
    }
}

/*
//DEBUG: izracunaj motor2.angle i motor3.angle, ako imamo poziciju noge na x,y,z
void getM()
{
    D.x = 155.485;
    D.y = 127.178;
    D.z = 115.464;

    l.x = D.x + s1.x;
    l.y = D.y + s1.y;
    l.z = D.z + s1.z;

    motor1.angle = atan(l.y / l.x);

    s2.x = s1.x + pow(-1, leg) * joint1 * cos(motor1.angle);
    s2.y = s1.y + pow(-1, leg) * joint1 * sin(motor1.angle);
    s2.z = s1.z;

    L.x = D.x + s2.x;
    L.y = D.y + s2.y;
    L.z = D.z + s2.z;

    float P = atan(L.z / (sqrt(L.x * L.x + L.y * L.y)));
    float R = asin((L.z - l.z) / l.length());

    motor2.angle = pow(-1, leg) * (acos((pow(joint2, 2) + pow(L.length(), 2) - pow(joint3, 2)) / (2 * joint2 * L.length())) - (P + R));
    motor3.angle = pow(-1, leg) * (PI - acos((pow(joint2, 2) + pow(joint3, 2) - pow(L.length(), 2)) / (2 * joint2 * joint3)));

    printMotor();
}

// DEBUG: izracunaj D.x i D.z pomocu svih motor.angle
void getHW()
{
    do
    {
        cout << "motor1.angle: ";
        cin >> motor1.angle;
    } while (motor1.angle >= motor1.max || motor1.angle <= motor1.min);
    do
    {
        cout << "motor2.angle: ";
        cin >> motor2.angle;
    } while (motor2.angle >= motor2.max || motor2.angle <= motor2.min);
    do
    {
        cout << "motor3.angle: ";
        cin >> motor3.angle;
    } while (motor3.angle >= motor3.max || motor3.angle <= motor3.min);

    // iz kutova u radijane *PI/180
    motor1.angle = motor1.angle * PI / 180;
    motor2.angle = motor2.angle * PI / 180;
    motor3.angle = motor3.angle * PI / 180;
    // lengthY = duljina noge po osi y
    const float lengthY = 31.1;

    // lengthX = duljina noge po osi x
    float lengthX = joint1 + cos(motor2.angle) * joint2 + cos(abs(motor3.angle) - motor2.angle) * joint3;

    // formula za width bez potrebe racunanja nAngle, stara formula dolje u komentarima
    float width = lengthX * cos(abs(motor1.angle)) + lengthY * sin(abs(motor1.angle));

    // height = duljina noge po osi z
    float height = sin(abs(motor3.angle) - motor2.angle) * joint3 - sin(motor2.angle) * joint2;

    cout << "width = " << width << endl;
    cout << "height = " << height << endl;
}
*/