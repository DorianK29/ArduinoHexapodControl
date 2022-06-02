
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

// ARDUINO: TODO: real pins
void initServo()
{
    leg1.motor1.servo.attach(1);
    leg1.motor2.servo.attach(2);
    leg1.motor3.servo.attach(3);
    leg2.motor1.servo.attach(1);
    leg2.motor2.servo.attach(2);
    leg2.motor3.servo.attach(3);
    leg3.motor1.servo.attach(1);
    leg3.motor2.servo.attach(2);
    leg3.motor3.servo.attach(3);
    leg4.motor1.servo.attach(1);
    leg4.motor2.servo.attach(2);
    leg4.motor3.servo.attach(3);
    leg5.motor1.servo.attach(1);
    leg5.motor2.servo.attach(2);
    leg5.motor3.servo.attach(3);
    leg6.motor1.servo.attach(1);
    leg6.motor2.servo.attach(2);
    leg6.motor3.servo.attach(3);
}

// ARDUINO:
void servoWrite()
{
    leg1.motor1.servo.write(leg1.motor1.angle);
    leg1.motor2.servo.write(leg1.motor2.angle);
    leg1.motor3.servo.write(leg1.motor3.angle);
    leg2.motor1.servo.write(leg2.motor1.angle);
    leg2.motor2.servo.write(leg2.motor2.angle);
    leg2.motor3.servo.write(leg2.motor3.angle);
    leg3.motor1.servo.write(leg3.motor1.angle);
    leg3.motor2.servo.write(leg3.motor2.angle);
    leg3.motor3.servo.write(leg3.motor2.angle);
    leg4.motor1.servo.write(leg4.motor1.angle);
    leg4.motor2.servo.write(leg4.motor2.angle);
    leg4.motor3.servo.write(leg4.motor3.angle);
    leg5.motor1.servo.write(leg5.motor1.angle);
    leg5.motor2.servo.write(leg5.motor2.angle);
    leg5.motor3.servo.write(leg5.motor3.angle);
    leg6.motor1.servo.write(leg6.motor1.angle);
    leg6.motor2.servo.write(leg6.motor2.angle);
    leg6.motor3.servo.write(leg6.motor3.angle);

    servoWait();
}

// ARDUINO: TODO: check servo.read output, change accordingly
void servoWait()
{
    bool wait = true;
    do
    {
        if (leg1.motor1.angle == leg1.motor1.servo.read() && leg1.motor2.angle == leg1.motor2.servo.read() && leg1.motor3.angle == leg1.motor3.servo.read())
            if (leg2.motor1.angle == leg2.motor1.servo.read() && leg2.motor2.angle == leg2.motor2.servo.read() && leg2.motor3.angle == leg2.motor3.servo.read())
                if (leg3.motor1.angle == leg3.motor1.servo.read() && leg3.motor2.angle == leg3.motor2.servo.read() && leg3.motor3.angle == leg3.motor3.servo.read())
                    if (leg4.motor1.angle == leg4.motor1.servo.read() && leg4.motor2.angle == leg4.motor2.servo.read() && leg4.motor3.angle == leg4.motor3.servo.read())
                        if (leg5.motor1.angle == leg5.motor1.servo.read() && leg5.motor2.angle == leg5.motor2.servo.read() && leg5.motor3.angle == leg5.motor3.servo.read())
                            if (leg6.motor1.angle == leg6.motor1.servo.read() && leg6.motor2.angle == leg6.motor2.servo.read() && leg6.motor3.angle == leg6.motor3.servo.read())
                                wait = false;
        delay(100);
    } while (wait);
    moving = false;
}

void legWrite(int legNum)
{
    leg localLeg = legSwitch(legNum);

    localLeg.motor1.servo.write(localLeg.motor1.angle);
    localLeg.motor2.servo.write(localLeg.motor2.angle);
    localLeg.motor3.servo.write(localLeg.motor3.angle);

    // DEBUG:
    printf("Leg %d: ", legNum);
    printf("motor1: %4.1f motor2: %5.3f motor3: %5.3f\n", localLeg.motor1.angle, localLeg.motor2.angle, localLeg.motor3.angle);

    servoWait();
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

// robot moving based on bool backwards
void move(bool backwards)
{
    moving = true;
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
            // printMotor();
            if (max)
            {
                max = false;
                j++;
                backwards = !backwards;
            }
            // ARDUINO: right place?
            servoWrite();
            servoWait();
        }
    // printf("\n");
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