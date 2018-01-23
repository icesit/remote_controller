#ifndef _REMOTECONTROLLER_H
#define _REMOTECONTROLLER_H
#include <stdio.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <linux/joystick.h>
#include "listop.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#define ONESECOND 50
using namespace std;

struct joystick_button{
int fd = 0;//存放手柄文件号
int up = 0;
int down = 0;
int left = 0;
int right = 0;
int but[10] = {0};
int back = 0;
int start = 0;
int home = 0;
int axisleft_ud = 0;
int axisleft_lr = 0;
int axisright_lr = 0;
int axisright_ud = 0;
};
#if 0
#define LOG_DBG(fmt, ...)  fprintf(stdout, fmt, ##__VA_ARGS__)
#else
#define LOG_DBG(fmt, ...)
#endif

#define LOG_ERR(fmt, ...)  fprintf(stderr, fmt, ##__VA_ARGS__)

typedef struct _joy_stick_ctx {
    struct list_head list;
    int i4_js_fd;
    unsigned int i4_op_block;
} JOYSTICK_CTX_T;

typedef struct _axes_t {
    int x;
    int y;
} AXES_T;

class RMcontroller
{
public:
    RMcontroller(ros::NodeHandle *_nh);
    ~RMcontroller();
    bool work();
    void resetAllKey();
private:
    ros::NodeHandle *nh;
    //store joystick data
    joystick_button myjoybutton;
    int js_fd, rc, i, print_init_stat;
    unsigned int buttons_state;
    js_event jse;
    AXES_T* tp_axes;
    char number_of_axes, number_of_btns, js_name_str[128];
    int cnt;

    ros::Publisher btn8_pub;
    ros::Publisher btn1_pub;
    ros::Publisher btn2_pub;
    ros::Publisher btn3_pub;
    ros::Publisher btn4_pub;
    ros::Publisher btn5_pub;
    ros::Publisher btn6_pub;
    ros::Publisher btn7_pub;
    ros::Publisher btnstart_pub;
    ros::Publisher btnback_pub;
    ros::Publisher btnhome_pub;
    ros::Publisher axes0lr_pub;
    ros::Publisher axes0ud_pub;
    ros::Publisher axes1lr_pub;
    ros::Publisher axes1ud_pub;

    std_msgs::Bool btnstart_b,btnback_b,btnhome_b,btn8_b,btn1_b,btn2_b,btn3_b,btn4_b,btn5_b,btn6_b,btn7_b;
    std_msgs::Float64 axes0lr_f,axes0ud_f,axes1lr_f,axes1ud_f;

    void initJoystick();
    void initPublisher();
    void pubAll();
    void display();

    int joystick_open(char* cp_js_dev_name, int i4_block);
    int joystick_close(int i4_fd);
    int joystick_read_one_event(int i4_fd, struct js_event* tp_jse);
    int joystick_read_ready(int i4_fd);
};

    void debug_list(void);
    void *joystick_loop(void *para);
#endif
