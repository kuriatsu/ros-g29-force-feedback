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

    // device info
    int m_device_handle;
    int m_axis_code = ABS_X;
    int m_axis_min;
    int m_axis_max;

    // rosparam
    std::string m_device_name;
    double m_update_rate;
    double m_tolerance = 0.01;

    // variables
    g29_force_feedback::ForceFeedback m_target;
    double m_last_force;
    struct ff_effect m_effect;
    bool m_passed = false;
    bool m_updated = false;
    double m_start_angle;
    double m_current_angle;

    // other
    double m_eps = 0.001;

public:
    G29ForceFeedback();
    ~G29ForceFeedback();

private:
    void targetCallback(const g29_force_feedback::ForceFeedback &in_target);
    void getState(const ros::TimerEvent&);
    int testBit(int bit, unsigned char *array);
    void initDevice();
    double calcRotateForce(const double &current_angle, const double &target_angle, const double &max_force, const double &rotate_speed);
    void uploadForce(const double &angle, const double &force);
};


G29ForceFeedback::G29ForceFeedback():
    m_device_name("/dev/input/event27"),
    m_update_rate(0.1)
{
    ros::NodeHandle n;
    sub_target = n.subscribe("/ff_target", 1, &G29ForceFeedback::targetCallback, this);

    n.getParam("device_name", m_device_name);
    n.getParam("update_rate", m_update_rate);

    initDevice();

    ros::Duration(1).sleep();
    timer = n.createTimer(ros::Duration(m_update_rate), &G29ForceFeedback::getState, this);
}

G29ForceFeedback::~G29ForceFeedback()
{
    m_effect.type = FF_CONSTANT;
    m_effect.id = -1;
    m_effect.u.constant.level = 0;
    m_effect.direction = 0;
    // upload m_effect
    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0)
    {
        std::cout << "failed to upload m_effect" << std::endl;
    }
}


// update input event with timer callback
void G29ForceFeedback::getState(const ros::TimerEvent&)
{
    struct input_event event;
    double buf = m_current_angle;

    // get current state
    while (read(m_device_handle, &event, sizeof(event)) == sizeof(event))
    {
        if (event.type == EV_ABS && event.code == m_axis_code)
        {
            m_current_angle = (event.value - (m_axis_max + m_axis_min) * 0.5) * 2 / (m_axis_max - m_axis_min);
        }
    }

    double rotate_speed = fabs((buf - m_current_angle) / m_update_rate);
    // std::cout << rotate_speed << std::endl;
    double force = calcRotateForce(m_current_angle, m_target.angle, fabs(m_target.force), rotate_speed);
    if (force == m_last_force) return;
    m_last_force = force;
    std::cout << force << std::endl;
    uploadForce(m_target.angle, force);

}


double G29ForceFeedback::calcRotateForce(const double &current_angle, const double &target_angle, const double &max_force, const double &rotate_speed)
{
    double diff = target_angle - current_angle;
    double initial_diff = m_target.angle - m_start_angle;

    std::cout << target_angle << "-" << current_angle << "=" << diff << std::endl;
    if (fabs(diff) < m_tolerance || fabs(max_force - 0.0) < m_eps)
        return 0.0;

    if ((diff < 0.0 && initial_diff < 0.0) || (diff >= 0.0 && initial_diff >= 0.0))
    {
        if (fabs(diff) < 0.1)
        {
            if (fabs(rotate_speed) > 0.2) // 0.2 = get closer speed
            {
                std::cout << "brake" << std::endl;
                return (diff >= 0.0) ? -0.3*rotate_speed : 0.3*rotate_speed;
            }
            std::cout << "get closer" << std::endl; // get close without brake
            return (diff >= 0.0) ? 0.2 : -0.2;
        }
        std::cout << "approach" << std::endl;
        return (diff >= 0.0) ? std::min(max_force, 1.0) : -std::min(max_force, 1.0);
    }
    else
    {
        if (fabs(diff) >= 0.1)
        {
            std::cout << "too over" << std::endl;
            m_start_angle = current_angle;
        }

        std::cout << "get closer" << std::endl;
        return (diff >= 0.0) ? 0.2 : -0.2;

        // return (diff >= 0.0) ? std::min(max_force, 1.0) : std::max(-max_force, -1.0);
    }
}


// update input event with writing information to the event file
void G29ForceFeedback::uploadForce(const double &angle, const double &force)
{
    // set effect
    m_effect.type = FF_CONSTANT;
    m_effect.u.constant.level = 0x7fff * force;
    m_effect.direction = 0x8000 * angle * m_axis_max; // just direction, + or - is important
    m_effect.u.constant.envelope.attack_level = 0;
    m_effect.u.constant.envelope.attack_length = 90;
    m_effect.u.constant.envelope.fade_level = 0;
    m_effect.u.constant.envelope.fade_length = 90;
    m_effect.trigger.button = 0;
    m_effect.trigger.interval = 0;
    m_effect.replay.length = 0xffff;
    m_effect.replay.delay = 0;
    // upload effect
    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0)
    {
        std::cout << "failed to upload effect" << std::endl;
    } else {
        std::cout << "uploaded" << std::endl;
    }
}


// get target information of wheel control from ros message
void G29ForceFeedback::targetCallback(const g29_force_feedback::ForceFeedback &in_target)
{
    if (fabs(m_target.force - in_target.force) < m_eps && fabs(m_target.angle - in_target.angle) < m_eps)
    {
        m_updated = false;
    } else
    {
        m_updated = true;
        m_passed = false;
    }
    m_target = in_target;
}


// initialize force feedback device
void G29ForceFeedback::initDevice()
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


// util for initDevice()
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
