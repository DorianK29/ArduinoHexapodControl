#line 1 "c:\\Users\\Dorian\\Documents\\cad\\zavrsni rad\\calc\\old\\leg_angle.cpp"
#include <iostream>
#include <math.h>

using namespace std;

#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286

#define joint1 33.5
#define joint2 57.5
#define joint3 113

/**
 * LIMITS
 * front leg -40, 0
 * middle leg -20, 0
 * back leg -15, 30
*/
float motor1;

// LIMITS
// 60, -45
float motor2;

// LIMITS
// -160, 0
float motor3;

void getLx();
void getM2();
void getM3();
void getHW();

int main()
{
    /** 
     * motor = kut motora
     * joint = duljina od jedne tocke rotacije motora do druge
    */

    do
    {
        cout << "motor1: ";
        cin >> motor1;
    } while (motor1 >= 0 || motor1 <= -40);
    do
    {
        cout << "motor2: ";
        cin >> motor2;
    } while (motor2 >= 60 || motor2 <= -45);
    do
    {
        cout << "motor3: ";
        cin >> motor3;
    } while (motor3 >= 0 || motor3 <= -160);

    // iz kutova u radijane *PI/180
    motor1 = motor1 * PI / 180;
    motor2 = motor2 * PI / 180;
    motor3 = motor3 * PI / 180;

    getHW();
}

// DEBUG
void getLx()
{
    float lengthX, width, lengthY;

    motor1 = 24 * PI / 180;
    width = 162.84;
    lengthY = 31.1;

    lengthX = (width - lengthY * sin(motor1)) / cos(motor1);

    cout << "lengthX = " << lengthX;
}

// izracunaj motor3 i motor2, samo motor3 je moguce za sada ...
void getM2n3()
{
    float lengthY, width;
    motor1 = 24 * PI / 180;
    width = 162.84;
    motor2 = 45 * PI / 180;
    lengthY = 31.1;

    // ??????? - radi :)
    motor3 = (-acos((lengthY * sin(motor1)) / (joint3 * cos(motor1)) - width / (joint3 * cos(motor1)) + (joint2 * cos(abs(motor2))) / joint3 + joint1 / joint3)) + motor2 + PI;

    cout << "motor3 = " << motor3 << endl;
}

// izracunaj width i height pomocu svih kutova (motora)
void getHW()
{
    float test;
    // lengthY = duljina noge po osi y
    const float lengthY = 31.1;

    // lengthX = duljina noge po osi x
    float lengthX = joint1 + cos(motor2) * joint2 + cos(abs(motor3) - motor2) * joint3;

    // formula za width bez potrebe racunanja nAngle, stara formula dolje u komentarima
    float width = lengthX * cos(abs(motor1)) + lengthY * sin(abs(motor1));

    // height = duljina noge po osi z
    float height = sin(abs(motor3) - motor2) * joint3 - sin(motor2) * joint2;

    cout << "width = " << width << endl;
    cout << "height = " << height << endl;
}

/* STARE FORMULE ZA WIDTH
    // nAngle = kut koji noga ima zbog svoje konstukcije
    float nAngle = atan(lengthY / lengthX);

    // lengthX / cos(nAngle) = od tocke rotacije motora 1 do tocke gdje noga dodiruje pod
    // width = duljina noge po osi x kada smo uzeli u obzir rotaciju motora 1
    float width = lengthX / cos(nAngle) * cos(abs(motor1) - nAngle);
*/

/**
 * GOD LIKE FORMULA
 * joint1 + cos(motor2) * joint2 + cos(abs(motor3) - motor2) * joint3 = (width - lengthY * sin(abs(motor1))) / cos(abs(motor1))
 * x + cos(b) * y + cos(c - b) * z = (w - o * sin(a)) / cos(a)
 */