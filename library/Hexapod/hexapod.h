#include <Servo.h>
#include <SoftwareSerial.h>
#include <StringSplitter.h>
#include <EEPROM.h>

#define joint1 33.5
#define joint2 57.5
#define joint3 113

#define Deg 180 / PI
#define Rad PI / 180

struct motor
{
    // a servo variable for a motor
    Servo servo;
    // calculated angle
    float angle;
    // the minimum and maximum values a motor is allowed to go to
    int min, max;
    // convert a motor's angle from degrees to radians
    void DegToRad()
    {
        angle = angle * Rad;
    }
    // convert a motor's angle from radians to degrees
    void RadToDeg()
    {
        angle = angle * Deg;
    }
};

// a 3d vector
struct vector
{
    // displacements on a given axis
    float x, y, z;
    // returns the length of the vector
    float length()
    {
        float l = sqrt(x * x + y * y + z * z);
        return l;
    }
};

struct leg
{
    // first motor, connecting the body and the first joint of a leg
    motor motor1;
    // second motor, connecting the first joint of a leg and the second joint of a leg
    motor motor2;
    // third motor, connecting the second joint of a leg and the third joint of a leg
    motor motor3;
    // from the center of the body to the point of contact with the ground
    vector D;
    // from the center point of rotation of motor 1 to the point of contact with the ground
    vector l;
    // from the center point of rotation of motor 2 to the point of contact with the ground
    vector L;
    // from the center of the body to the center point of rotation of motor 1
    vector s1;
    // from the center of the body to the center point of rotation of motor 2
    vector s2;
    // convert all motor angles from radians to degrees
    void motorRadToDeg()
    {
        motor1.RadToDeg();
        motor2.RadToDeg();
        motor3.RadToDeg();
    }
    // convert all motor angles from degrees to radians
    void motorDegToRad()
    {
        motor1.DegToRad();
        motor2.DegToRad();
        motor3.DegToRad();
    }
    // return a motor variable for the given motor number
    // 1: motor1
    // 2: motor2
    // 3: motor3
    motor motorSwitch(int number)
    {
        switch (number)
        {
        case 1:
            return motor1;
            break;
        case 2:
            return motor2;
            break;
        case 3:
            return motor3;
            break;
        }
    }
};
