#include <ros/ros.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include "g29_force_feedback/ForceFeedback.h"

class G29ForceFeedback
{
private:
    ros::Subscriber sub_target;
    ros::Timer timer;
    float m_pub_rate;

    // variables from fouce feedback API
    int m_device_handle;
    int m_axis_code = ABS_X;
    int m_axis_min;
    int m_axis_max;
    struct ff_effect m_effect;

    // device config
    std::string m_device_name;
    double m_max_force;
    double m_min_force;

    // motion config 0:PID force, 1:constant force
    double m_Kp;
    double m_Ki;
    double m_Kd;
    double m_offset;

    // target and current state of the wheel
    bool m_pid_mode;
    double m_target_angle;
    double m_target_force;
    double m_current_angle;

public:
    G29ForceFeedback();

private:
    void targetCallback(const g29_force_feedback::ForceFeedback &in_target);
    void timerCallback(const ros::TimerEvent&);
    int testBit(int bit, unsigned char *array);
    void initFfDevice();
    void updateFfDevice();
};


G29ForceFeedback::G29ForceFeedback():
    m_device_name("/dev/input/event19"),
    m_Kp(0.1),
    m_Ki(0.0),
    m_Kd(0.0),
    m_offset(0.01),
    m_max_force(1.0),
    m_min_force(0.2),
    m_pub_rate(0.1),
    m_pid_mode(0)
{
    ros::NodeHandle n;
    sub_target = n.subscribe("/ff_target", 1, &G29ForceFeedback::targetCallback, this);

    n.getParam("device_name", m_device_name);
    n.getParam("Kp", m_Kp);
    n.getParam("Ki", m_Ki);
    n.getParam("Kd", m_Kd);
    n.getParam("offset", m_offset);
    n.getParam("max_force", m_max_force);
    n.getParam("min_force", m_min_force);
    n.getParam("pub_rate", m_pub_rate);

    initFfDevice();

    ros::Duration(1).sleep();
    timer = n.createTimer(ros::Duration(m_pub_rate), &G29ForceFeedback::timerCallback, this);
}


// update input event with timer callback
void G29ForceFeedback::timerCallback(const ros::TimerEvent&)
{
    updateFfDevice();
}


// update input event with writing information to the event file
void G29ForceFeedback::updateFfDevice()
{
    struct input_event event;
    static float diff_i = 0.0, diff = 0.0;
    double diff_d, force, buf;

    // if you wanna use I control, let's avoid integral value exploding
    // static int count = 0;
    // count ++;
    // if (force > 0.3 || count > 10)
    // {
        //     diff_i = 0.0;
        //     count = 0;
        // }

    // calcurate values for PID control
    buf = diff;
    diff = m_target_angle - m_current_angle;
    diff_i += diff;
    diff_d = diff - buf;

    if (m_pid_mode)
    {
        force = fabs(m_Kp * diff + m_Ki * diff_i + m_Kd * diff_d) * ((diff > 0.0) ? 1.0 : -1.0);

        // if wheel angle reached to the target
        if (fabs(diff) < m_offset)
        {
            force = 0.0;
        }
        else
        {
            // force less than 0.2 cannot turn the wheel
            force = (force > 0.0) ? std::max(force, m_min_force) : std::min(force, -m_min_force);
            // set max force for safety
            force = (force > 0.0) ? std::min(force, m_max_force) : std::max(force, -m_max_force);
        }
    }
    else
    {
        force = fabs(m_target_force) * ((diff > 0.0) ? 1.0 : -1.0);

        // if wheel angle reached to the target
        if (fabs(diff) < m_offset)
        {
            force = 0.0;
        }
    }

    // for safety
    force = (force > 0.0) ? std::min(force, m_max_force) : std::max(force, -m_max_force);

    // start effect
    m_effect.u.constant.level = (short)(force * 32767.0);
    m_effect.direction = 0xC000;
    m_effect.u.constant.envelope.attack_level = (short)(force * 32767.0);
    m_effect.u.constant.envelope.fade_level = (short)(force * 32767.0);

    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0)
    {
        std::cout << "failed to upload m_effect" << std::endl;
    }

    // get current state
    while (read(m_device_handle, &event, sizeof(event)) == sizeof(event))
    {
        if (event.type == EV_ABS && event.code == m_axis_code)
        {
            m_current_angle = (event.value - (m_axis_max + m_axis_min) * 0.5) * 2 / (m_axis_max - m_axis_min);
        }
    }
}


// get target information of wheel control from ros message
void G29ForceFeedback::targetCallback(const g29_force_feedback::ForceFeedback &in_target)
{
    m_pid_mode = in_target.pid_mode;
    m_target_angle = in_target.angle;
    m_target_force = in_target.force;
}


// initialize force feedback device
void G29ForceFeedback::initFfDevice()
{
    // setup device
    unsigned char key_bits[1+KEY_MAX/8/sizeof(unsigned char)];
    unsigned char abs_bits[1+ABS_MAX/8/sizeof(unsigned char)];
    unsigned char ff_bits[1+FF_MAX/8/sizeof(unsigned char)];

    struct input_event event;
    struct input_absinfo abs_info;

    m_device_handle = open(m_device_name.c_str(), O_RDWR|O_NONBLOCK);
    if (m_device_handle < 0)
    {
        std::cout << "ERROR: cannot open device : "<< m_device_name << std::endl;
        exit(1);
    }else{std::cout << "device opened" << std::endl;}

    // which axes has the device?
    memset(abs_bits, 0, sizeof(abs_bits));
    if (ioctl(m_device_handle, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0)
    {
        std::cout << "ERROR: cannot get abs bits" << std::endl;
        exit(1);
    }

    // get some information about force feedback
    memset(ff_bits, 0, sizeof(ff_bits));
    if (ioctl(m_device_handle, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0)
    {
        std::cout << "ERROR: cannot get ff bits" << std::endl;
        exit(1);
    }

    // get axis value range
    if (ioctl(m_device_handle, EVIOCGABS(m_axis_code), &abs_info) < 0)
    {
        std::cout << "ERROR: cannot get axis range" << std::endl;
        exit(1);
    }
    m_axis_max = abs_info.maximum;
    m_axis_min = abs_info.minimum;
    if (m_axis_min >= m_axis_max)
    {
        std::cout << "ERROR: axis range has bad value" << std::endl;
        exit(1);
    }

    // check force feedback is supported?
    if(!testBit(FF_CONSTANT, ff_bits))
    {
        std::cout << "ERROR: force feedback is not supported" << std::endl;
        exit(1);
    }else{std::cout << "force feedback supported" << std::endl;}

    // auto centering off
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = FF_AUTOCENTER;
    event.value = 0;
    if (write(m_device_handle, &event, sizeof(event)) != sizeof(event))
    {
        std::cout << "failed to disable auto centering" << std::endl;
        exit(1);
    }

    // initialize constant foce m_effect
    memset(&m_effect, 0, sizeof(m_effect));
    m_effect.type = FF_CONSTANT;
    m_effect.id = -1;
    m_effect.trigger.button = 0;
    m_effect.trigger.interval = 0;
    m_effect.replay.length = 0xffff;
    m_effect.replay.delay = 0;
    m_effect.u.constant.level = 0;
    m_effect.direction = 0xC000;
    m_effect.u.constant.envelope.attack_length = 0;
    m_effect.u.constant.envelope.attack_level = 0;
    m_effect.u.constant.envelope.fade_length = 0;
    m_effect.u.constant.envelope.fade_level = 0;

    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0)
    {
        std::cout << "failed to upload m_effect" << std::endl;
        exit(1);
    }

    // start m_effect
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = m_effect.id;
    event.value = 1;
    if (write(m_device_handle, &event, sizeof(event)) != sizeof(event))
    {
        std::cout << "failed to start event" << std::endl;
        exit(1);
    }
}


// util for initFfDevice()
int G29ForceFeedback::testBit(int bit, unsigned char *array)
{
    return ((array[bit / (sizeof(unsigned char) * 8)] >> (bit % (sizeof(unsigned char) * 8))) & 1);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "g29_force_feedback_node");
    G29ForceFeedback g29_ff;
    ros::spin();
    return(0);
}
