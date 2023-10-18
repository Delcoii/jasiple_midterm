#include <ros.h>
#include <std_msgs/Int32.h>

#define LOOP_MS                 16

#define PULSE_ERROR_MAX_US      2500
#define PULSE_ERROR_MIN_US      500
#define DETECTION_ERR           -999

#define STEERING_PULSE_PIN      18
#define ACCEL_PULSE_PIN         19

#define MODE_PULSE_PIN          21
#define MANUAL_MODE             1200    // for pretty plotting
#define AUTO_MODE               1500    // for pretty plotting     
#define BRAKE_MODE              1800    // for pretty plotting

#define CH4_PULSE_PIN           20

volatile long g_steering_edge_now_us = DETECTION_ERR;
volatile long g_steering_edge_before_us = DETECTION_ERR;
volatile long g_steering_us = DETECTION_ERR;

volatile long g_accel_edge_now_us = DETECTION_ERR;
volatile long g_accel_edge_before_us = DETECTION_ERR;
volatile long g_accel_us = DETECTION_ERR;

volatile long g_mode_edge_now_us = DETECTION_ERR;
volatile long g_mode_edge_before_us = DETECTION_ERR;
volatile long g_mode_us = DETECTION_ERR;


void SteeringPulseInt() {
    g_steering_edge_now_us = micros();

    g_steering_us = g_steering_edge_now_us - g_steering_edge_before_us;
    g_steering_edge_before_us = g_steering_edge_now_us;
}

void AccelPulseInt() {
    g_accel_edge_now_us = micros();

    g_accel_us = g_accel_edge_now_us - g_accel_edge_before_us;
    g_accel_edge_before_us = g_accel_edge_now_us;
}

void ModePulseInt() {
    g_mode_edge_now_us = micros();
    
    g_mode_us = g_mode_edge_now_us - g_mode_edge_before_us;
    g_mode_edge_before_us = g_mode_edge_now_us;
}



ros::NodeHandle nh;
std_msgs::Int32 steer_msg;
std_msgs::Int32 accel_msg;
std_msgs::Int32 mode_msg;
ros::Publisher steering_pub("/remocon_steer_us", &steer_msg);
ros::Publisher accel_pub("/remocon_accel_us", &accel_msg);
ros::Publisher mode_pub("/remocon_mode", &mode_msg);

void setup()
{   

    pinMode(STEERING_PULSE_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(STEERING_PULSE_PIN), SteeringPulseInt, CHANGE);
    pinMode(ACCEL_PULSE_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ACCEL_PULSE_PIN), AccelPulseInt, CHANGE);
    pinMode(MODE_PULSE_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MODE_PULSE_PIN), ModePulseInt, CHANGE);


    nh.initNode();
    nh.advertise(steering_pub);
    nh.advertise(accel_pub);
    nh.advertise(mode_pub);
}

void loop()
{   
    static int steering_val;
    static int accel_val;
    static int mode_val = BRAKE_MODE;

    cli();
    if ((g_steering_us > PULSE_ERROR_MIN_US) && (g_steering_us < PULSE_ERROR_MAX_US)) {
        steering_val = g_steering_us;
    }
    if ((g_accel_us > PULSE_ERROR_MIN_US) && (g_accel_us < PULSE_ERROR_MAX_US)) {
        accel_val = g_accel_us;
    }

    if ((g_mode_us > PULSE_ERROR_MIN_US) && (g_mode_us < PULSE_ERROR_MAX_US)) {
        if ((g_mode_us >= 800) && (g_mode_us <= 1200)) {
            mode_val = MANUAL_MODE;
        }
        else if ((g_mode_us > 1200) && g_mode_us <= 1700) {
            mode_val = AUTO_MODE;
        }
        else {
            mode_val = BRAKE_MODE;
        }
    }
    sei();

    steer_msg.data = steering_val;
    accel_msg.data = accel_val;
    mode_msg.data = mode_val;

    // steering_pub.publish(&steer_msg);
    // accel_pub.publish(&accel_msg);
    // mode_pub.publish(&mode_msg);
    // delay(16);
    static int before_ms = 0;
    int now_ms = millis();

    if ((now_ms - before_ms) >= LOOP_MS) {
        before_ms = now_ms;

        steering_pub.publish(&steer_msg);
        accel_pub.publish(&accel_msg);
        mode_pub.publish(&mode_msg);
    }

    nh.spinOnce();
}
