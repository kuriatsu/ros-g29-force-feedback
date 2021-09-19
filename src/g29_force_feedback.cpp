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
    float m_update_rate;

    // variables
    g29_force_feedback::ForceFeedback m_target;
    float = m_force;

public:
    G29ForceFeedback();
    ~G29ForceFeedback();

private:
    void targetCallback(const g29_force_feedback::ForceFeedback &in_target);
    void getState(const ros::TimerEvent&);
    int testBit(int bit, unsigned char *array);
    void initDevice();
    void uploadForce(const float &current, const float &target, const float &force);
};


G29ForceFeedback::G29ForceFeedback():
    m_device_name("/dev/input/event19"),
    m_update_rate(0.1),
{
    ros::NodeHandle n;
    sub_target = n.subscribe("/ff_target", 1, &G29ForceFeedback::targetCallback, this);

    n.getParam("device_name", m_device_name);
    n.getParam("update_rate", m_update_rate);

    initDevice();

    ros::Duration(1).sleep();
    timer = n.createTimer(ros::Duration(m_pub_rate), &G29ForceFeedback::getState, this);
}

G29ForceFeedback::~G29ForceFeedback()
{
    m_force = force;
    struct ff_effect effect;
    memset(&effect, 0, sizeof(effect));
    effect.type = FF_CONSTANT;
    effect.id = -1;
    effect.u.constant.level = 0;
    effect.direction = 0;
    // upload effect
    if (ioctl(m_device_handle, EVIOCSFF, &effect) < 0)
    {
        std::cout << "failed to upload effect" << std::endl;
    }
}


// update input event with timer callback
void G29ForceFeedback::getState(const ros::TimerEvent&)
{
    struct input_event event;
    float current_angle;

    // get current state
    while (read(m_device_handle, &event, sizeof(event)) == sizeof(event))
    {
        if (event.type == EV_ABS && event.code == m_axis_code)
        {
            current_angle = (event.value - (m_axis_max + m_axis_min) * 0.5) * 2 / (m_axis_max - m_axis_min);
        }
    }

    if (m_is_target_updated)
    {
        uploadForce(current_angle, m_target.angle, m_target.force);
    }
}


// update input event with writing information to the event file
void G29ForceFeedback::uploadForce(const float &current, const float &target, const float &force)
{
    float diff = target - current;

    if (std::fabs(diff) < 0.1)
    {
        force = (diff >= 0.0) ? 0.3 : -0.3;
    } else if (diff > 0.0)
    {
        force = (target >= 0.0) ? force : 0.3;
    } else {
        force = (target >= 0.0) ? -0.3 : -force;
    }
   // for safety
    force = (force > 0.0) ? std::min(force, 1.0) : std::max(force, -1.0);

    if (force == m_force) return;

    m_force = force;
    // set effect
    struct ff_effect effect;
    memset(&effect, 0, sizeof(effect));
    effect.type = FF_CONSTANT;
    effect.id = -1;
    effect.u.constant.level = 0x7fff * force;
    effect.direction = 0x8000 * m_target_angle * m_axis_max; // just direction, + or - is important
    effect.u.constant.envelope.attack_level = 0;
    effect.u.constant.envelope.attack_length = 100;
    effect.u.constant.envelope.fade_level = 0;
    effect.u.constant.envelope.fade_length = 100;
    effect.trigger.button = 0;
    effect.trigger.interval = 0;
    effect.replay.length = 0xffff;
    effect.replay.delay = 0;
    // upload effect
    if (ioctl(m_device_handle, EVIOCSFF, &effect) < 0)
    {
        std::cout << "failed to upload effect" << std::endl;
    }
}


// get target information of wheel control from ros message
void G29ForceFeedback::targetCallback(const g29_force_feedback::ForceFeedback &in_target)
{
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
