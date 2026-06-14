#include "pid.h"

_pid g_pid_speed1, g_pid_speed2;
_pid g_pid_location1, g_pid_location2;
_pid g_pid_turn_angle;
_pid g_pid_line;

void pid_param_init(void)
{
    g_pid_location1.target_val = 0.0f;
    g_pid_location1.actual_val = 0.0f;
    g_pid_location1.err = 0.0f;
    g_pid_location1.err_last = 0.0f;
    g_pid_location1.integral = 0.0f;
    g_pid_location1.Kp = 0.3f;
    g_pid_location1.Ki = 0.0f;
    g_pid_location1.Kd = 0.0f;

    g_pid_speed1.target_val = 0.0f;
    g_pid_speed1.actual_val = 0.0f;
    g_pid_speed1.err = 0.0f;
    g_pid_speed1.err_last = 0.0f;
    g_pid_speed1.integral = 0.0f;
    g_pid_speed1.Kp = 6.0f;
    g_pid_speed1.Ki = 8.5f;
    g_pid_speed1.Kd = 0.0f;

    g_pid_location2.target_val = 0.0f;
    g_pid_location2.actual_val = 0.0f;
    g_pid_location2.err = 0.0f;
    g_pid_location2.err_last = 0.0f;
    g_pid_location2.integral = 0.0f;
    g_pid_location2.Kp = 0.3f;
    g_pid_location2.Ki = 0.0f;
    g_pid_location2.Kd = 0.0f;

    g_pid_speed2.target_val = 0.0f;
    g_pid_speed2.actual_val = 0.0f;
    g_pid_speed2.err = 0.0f;
    g_pid_speed2.err_last = 0.0f;
    g_pid_speed2.integral = 0.0f;
    g_pid_speed2.Kp = 6.0f;
    g_pid_speed2.Ki = 8.5f;
    g_pid_speed2.Kd = 0.0f;

    g_pid_turn_angle.target_val = 0.0f;
    g_pid_turn_angle.actual_val = 0.0f;
    g_pid_turn_angle.err = 0.0f;
    g_pid_turn_angle.err_last = 0.0f;
    g_pid_turn_angle.integral = 0.0f;
    g_pid_turn_angle.Kp = 1.2f;
    g_pid_turn_angle.Ki = 0.0f;
    g_pid_turn_angle.Kd = 0.0f;

    g_pid_line.target_val = 0.0f;
    g_pid_line.actual_val = 0.0f;
    g_pid_line.err = 0.0f;
    g_pid_line.err_last = 0.0f;
    g_pid_line.integral = 0.0f;
    g_pid_line.Kp = 4.0f;
    g_pid_line.Ki = 0.0f;
    g_pid_line.Kd = 1.0f;
}

void set_pid_target(_pid *pid, float temp_val)
{
    pid->target_val = temp_val;
}

float get_pid_target(_pid *pid)
{
    return pid->target_val;
}

void set_p_i_d(_pid *pid, float p, float i, float d)
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
}

float location_pid_realize(_pid *pid, float actual_val)
{
    pid->err = pid->target_val - actual_val;
    pid->integral += pid->err;

    pid->actual_val = pid->Kp * pid->err
                    + pid->Ki * pid->integral
                    + pid->Kd * (pid->err - pid->err_last);

    pid->err_last = pid->err;
    return pid->actual_val;
}

float speed_pid_realize(_pid *pid, float actual_val)
{
    pid->err = pid->target_val - actual_val;

    if ((pid->err < 0.5f) && (pid->err > -0.5f))
    {
        pid->err = 0.0f;
    }

    pid->integral += pid->err;
    if (pid->integral >= 1000.0f)
    {
        pid->integral = 1000.0f;
    }
    else if (pid->integral < -1000.0f)
    {
        pid->integral = -1000.0f;
    }

    pid->actual_val = pid->Kp * pid->err
                    + pid->Ki * pid->integral
                    + pid->Kd * (pid->err - pid->err_last);

    pid->err_last = pid->err;
    return pid->actual_val;
}

float turn_angle_pid_realize(_pid *pid, float actual_val)
{
    pid->err = pid->target_val - actual_val;

    while (pid->err > 180.0f)
    {
        pid->err -= 360.0f;
    }
    while (pid->err < -180.0f)
    {
        pid->err += 360.0f;
    }

    if ((pid->err > -1.0f) && (pid->err < 1.0f))
    {
        pid->err = 0.0f;
        pid->integral = 0.0f;
    }

    pid->integral += pid->err;
    if (pid->integral > 300.0f)
    {
        pid->integral = 300.0f;
    }
    else if (pid->integral < -300.0f)
    {
        pid->integral = -300.0f;
    }

    pid->actual_val = pid->Kp * pid->err
                    + pid->Ki * pid->integral
                    + pid->Kd * (pid->err - pid->err_last);

    pid->err_last = pid->err;
    return pid->actual_val;
}

float line_pid_realize(_pid *pid, float actual_val)
{
    pid->err = pid->target_val - actual_val;

    if ((pid->err < 1.0f) && (pid->err > -1.0f))
    {
        pid->err = 1.0f;
    }

    pid->integral += pid->err;
    if (pid->integral >= 1000.0f)
    {
        pid->integral = 1000.0f;
    }
    else if (pid->integral < -1000.0f)
    {
        pid->integral = -1000.0f;
    }

    pid->actual_val = pid->Kp * pid->err
                    + pid->Ki * pid->integral
                    + pid->Kd * (pid->err - pid->err_last);

    pid->err_last = pid->err;
    return pid->actual_val;
}
