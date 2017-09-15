#include "Remotecontroller.h"


LIST_HEAD(_t_js_ctx_head);

RMcontroller::RMcontroller(ros::NodeHandle *_nh):
    nh(_nh), tp_axes(NULL)
{
    initJoystick();
    initPublisher();
}

RMcontroller::~RMcontroller()
{
    joystick_close(js_fd);
    if (tp_axes) {
        free(tp_axes);
    }
}

void RMcontroller::work()
{
    if (joystick_read_ready(js_fd)) {
        rc = joystick_read_one_event(js_fd, &jse);
        if (rc > 0) {
            if ((jse.type & JS_EVENT_INIT) == JS_EVENT_INIT) {
                if ((jse.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON) {
                    if (jse.value) {
                        buttons_state |= (1 << jse.number);
                    }
                    else {
                        buttons_state &= ~(1 << jse.number);
                    }
                }
                else if ((jse.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS) {
                    if (tp_axes) {
                        if ((jse.number & 1) == 0) {
                            tp_axes[jse.number / 2].x = jse.value;
                        }
                        else {
                            tp_axes[jse.number / 2].y = jse.value;
                        }
                    }

                }
            }
            else {
                //op_times++;
                if (print_init_stat == 0) {
                    for (i = 0; i < number_of_btns; i++) {
                        LOG_DBG("joystick init state: button %d is %s.\n", i, ((buttons_state & (1 << i)) == (1 << i)) ? "DOWN" : "UP");
                    }

                    if (tp_axes)
                        for (i = 0; i < number_of_axes; i++) {
                            LOG_DBG("joystick init state: axes %d is x=%d  y=%d.\n", i, tp_axes[i].x, tp_axes[i].y);
                        }
                    print_init_stat = 1;
                }

                if (jse.type  == JS_EVENT_BUTTON) {
                    if (jse.value) {
                        buttons_state |= (1 << jse.number);
                    }
                    else {
                        buttons_state &= ~(1 << jse.number);
                    }
                    //std::cout << "get number " << (int)jse.number <<std::endl;
                    switch(jse.number)
                    {
                        case 0:
                            myjoybutton.but[3]= ((buttons_state & (1 << jse.number)) >> jse.number);
                        break;
                        case 1:
                            myjoybutton.but[2]= ((buttons_state & (1 << jse.number)) >> jse.number);
                        break;
                        case 2:
                            myjoybutton.but[4]= ((buttons_state & (1 << jse.number)) >> jse.number);
                        break;
                        case 3:
                            myjoybutton.but[1]= ((buttons_state & (1 << jse.number)) >> jse.number);
                        break;
                        case 4:
                            myjoybutton.but[5]= ((buttons_state & (1 << jse.number)) >> jse.number);
                        break;
                        case 5:
                            myjoybutton.but[6]= ((buttons_state & (1 << jse.number)) >> jse.number);
                        break;
                        case 6:
                            myjoybutton.back= ((buttons_state & (1 << jse.number)) >> jse.number);
                        break;
                        case 7:
                            myjoybutton.start= ((buttons_state & (1 << jse.number)) >> jse.number);
                        break;
                        case 8:
                            myjoybutton.home= ((buttons_state & (1 << jse.number)) >> jse.number);
                        break;
                    }

                    LOG_DBG("joystick state: button %d is %s.\n", jse.number, ((buttons_state & (1 << jse.number)) == (1 << jse.number)) ? "DOWN" : "UP");
                }
                else if (jse.type == JS_EVENT_AXIS) {
                    if (tp_axes) {
                        if ((jse.number & 1) == 0) {
                            tp_axes[jse.number / 2].x = jse.value;
                        }
                        else {
                            tp_axes[jse.number / 2].y = jse.value;
                        }
                        //std::cout << "get axis " << (int)jse.number << std::endl;
                        switch(jse.number / 2)
                        {
                            case 0:
                                myjoybutton.axisleft_ud=tp_axes[jse.number / 2].y;
                                myjoybutton.axisleft_lr=tp_axes[jse.number / 2].x;
                            break;
                            case 1:
                                myjoybutton.axisright_lr=tp_axes[jse.number / 2].y;
                                myjoybutton.but[7]=((tp_axes[jse.number / 2].x>0)?1:0);
                            break;
                            case 2:
                                myjoybutton.axisright_ud=tp_axes[jse.number / 2].x;
                                myjoybutton.but[8]=((tp_axes[jse.number / 2].y>0)?1:0);
                            break;
                            case 3:
                                myjoybutton.up=((tp_axes[jse.number / 2].y<0)?1:0);
                                myjoybutton.down=((tp_axes[jse.number / 2].y>0)?1:0);
                                myjoybutton.left=((tp_axes[jse.number / 2].x<0)?1:0);
                                myjoybutton.right=((tp_axes[jse.number / 2].x>0)?1:0);
                            break;
                        }
                        LOG_DBG("joystick state: axes %d is x=%d  y=%d.\n", jse.number / 2, tp_axes[jse.number / 2].x, tp_axes[jse.number / 2].y);
                    }
                    else {
                        LOG_DBG("joystick state: axes %d is %s=%d.\n", jse.number / 2, ((jse.number & 1) == 0) ? "x" : "y", jse.value);
                    }
                }
            }
        }
    }

    pubAll();
    display();
}

int RMcontroller::joystick_open(char *cp_js_dev_name, int i4_block)
{
    int i4_open_flags = O_RDONLY;
    JOYSTICK_CTX_T*  pt_joystick_ctx = NULL;

    if (!cp_js_dev_name) {
        LOG_ERR("[%s] js device name is NULL\n", __func__);
        return -1;
    }

    pt_joystick_ctx = (JOYSTICK_CTX_T*)calloc(sizeof(JOYSTICK_CTX_T), 1);
    if (!pt_joystick_ctx) {
        LOG_ERR("[%s] no memory!!\n", __func__);
        return -1;
    }

    pt_joystick_ctx->i4_op_block = i4_block ? 1 : 0;

    if (pt_joystick_ctx->i4_op_block == 0) {
        i4_open_flags |= O_NONBLOCK;
    }

    pt_joystick_ctx->i4_js_fd = open(cp_js_dev_name, i4_open_flags);
    if (pt_joystick_ctx->i4_js_fd < 0) {
        LOG_ERR("[%s] open device %s error\n", __func__, cp_js_dev_name);
        free(pt_joystick_ctx);
        return -1;
    }

    list_add_tail(&pt_joystick_ctx->list, &_t_js_ctx_head);

    return pt_joystick_ctx->i4_js_fd;
}

int RMcontroller::joystick_close(int i4_fd)
{
    struct list_head* pt_entry;
    struct list_head* pt_next;
    JOYSTICK_CTX_T* pt_node;

    if (list_empty(&_t_js_ctx_head)) {
        LOG_ERR("[%s] device not opened\n", __func__);
        return -1;
    }

    list_for_each_safe(pt_entry, pt_next, &_t_js_ctx_head) {
        pt_node = list_entry(pt_entry, JOYSTICK_CTX_T, list);
        if (pt_node->i4_js_fd == i4_fd) {
            list_del_init(&pt_node->list);
            free(pt_node);
            return close(i4_fd);
        }
    }

    LOG_ERR("[%s] i4_fd=%d invalid\n", __func__, i4_fd);
    return -1;
}

int RMcontroller::joystick_read_one_event(int i4_fd, js_event *tp_jse)
{
    int i4_rd_bytes;

    /*do not check i4_fd again*/

    i4_rd_bytes = read(i4_fd, tp_jse, sizeof(struct js_event));

    if (i4_rd_bytes == -1) {
        if (errno == EAGAIN) { /*when no block, it is not error*/
            return 0;
        }
        else {
            return -1;
        }
    }

    return i4_rd_bytes;
}

int RMcontroller::joystick_read_ready(int i4_fd)
{
    int i4_block = 2;
    struct list_head* pt_entry;
    JOYSTICK_CTX_T* pt_node;

    if (list_empty(&_t_js_ctx_head)) {
        LOG_ERR("[%s] device not opened\n", __func__);
        return -1;
    }

    list_for_each(pt_entry, &_t_js_ctx_head) {
        pt_node = list_entry(pt_entry, JOYSTICK_CTX_T, list);
        if (pt_node->i4_js_fd == i4_fd) {
            i4_block = pt_node->i4_op_block;
            break;
        }
    }

    if (i4_block == 2) {
        LOG_ERR("[%s] i4_fd=%d invalid\n", __func__, i4_fd);
        return 0;
    }
    else if (i4_block == 1) {
        fd_set readfd;
        int i4_ret = 0;
        struct timeval timeout = {0, 0};
        FD_ZERO(&readfd);
        FD_SET(i4_fd, &readfd);

        i4_ret = select(i4_fd + 1, &readfd, NULL, NULL, &timeout);

        if (i4_ret > 0 && FD_ISSET(i4_fd, &readfd)) {
            return 1;
        }
        else {
            return 0;
        }

    }

    return 1; /*noblock read, aways ready*/
}

void RMcontroller::initPublisher()
{
    btn8_pub = nh->advertise<std_msgs::Bool>("/remotecontroller/btn8", 5);
    btn1_pub = nh->advertise<std_msgs::Bool>("/remotecontroller/btn1", 5);
    btn2_pub = nh->advertise<std_msgs::Bool>("/remotecontroller/btn2", 5);
    btn3_pub = nh->advertise<std_msgs::Bool>("/remotecontroller/btn3", 5);
    btn4_pub = nh->advertise<std_msgs::Bool>("/remotecontroller/btn4", 5);
    btn5_pub = nh->advertise<std_msgs::Bool>("/remotecontroller/btn5", 5);
    btn6_pub = nh->advertise<std_msgs::Bool>("/remotecontroller/btn6", 5);
    btn7_pub = nh->advertise<std_msgs::Bool>("/remotecontroller/btn7", 5);
    btnstart_pub = nh->advertise<std_msgs::Bool>("/remotecontroller/btnstart", 5);
    btnback_pub = nh->advertise<std_msgs::Bool>("/remotecontroller/btnback", 5);
    btnhome_pub = nh->advertise<std_msgs::Bool>("/remotecontroller/btnhome", 5);
    axes0lr_pub = nh->advertise<std_msgs::Float64>("/remotecontroller/axes0lr", 5);
    axes0ud_pub = nh->advertise<std_msgs::Float64>("/remotecontroller/axes0ud", 5);
    axes1lr_pub = nh->advertise<std_msgs::Float64>("/remotecontroller/axes1lr", 5);
    axes1ud_pub = nh->advertise<std_msgs::Float64>("/remotecontroller/axes1ud", 5);
}

void RMcontroller::initJoystick()
{
    js_fd = joystick_open("/dev/input/js0", 1);
    if (js_fd < 0) {
        LOG_ERR("open /dev/input/js0 failed.\n");
        exit(1);
    }

    rc = ioctl(js_fd, JSIOCGAXES, &number_of_axes);
    if (rc != -1) {
        LOG_DBG("number_of_axes:%d\n", number_of_axes);
        if (number_of_axes > 0) {
            tp_axes = (AXES_T*)calloc(sizeof(AXES_T), 1);
        }
    }

    rc = ioctl(js_fd, JSIOCGBUTTONS, &number_of_btns);
    if (rc != -1) {
        LOG_DBG("number_of_btns:%d\n", number_of_btns);
    }

    if (ioctl(js_fd, JSIOCGNAME(sizeof(js_name_str)), js_name_str) < 0) {
        LOG_DBG(js_name_str, "Unknown", sizeof(js_name_str));
    }

    LOG_DBG("joystick Name: %s\n", js_name_str);
}

void RMcontroller::pubAll()
{
    btnstart_b.data = myjoybutton.start;
    btnback_b.data = myjoybutton.back;
    btnhome_b.data = myjoybutton.home;
    btn8_b.data = myjoybutton.but[8];
    btn1_b.data = myjoybutton.but[1];
    btn2_b.data = myjoybutton.but[2];
    btn3_b.data = myjoybutton.but[3];
    btn4_b.data = myjoybutton.but[4];
    btn5_b.data = myjoybutton.but[5];
    btn6_b.data = myjoybutton.but[6];
    btn7_b.data = myjoybutton.but[7];
    axes0lr_f.data = - (double)(myjoybutton.axisleft_lr) / 32768;
    axes0ud_f.data = - (double)(myjoybutton.axisleft_ud) / 32768;
    axes1lr_f.data = - (double)(myjoybutton.axisright_lr) / 32768;
    axes1ud_f.data = - (double)(myjoybutton.axisright_ud) / 32768;
    btn8_pub.publish(btn8_b);
    btn1_pub.publish(btn1_b);
    btn2_pub.publish(btn2_b);
    btn3_pub.publish(btn3_b);
    btn4_pub.publish(btn4_b);
    btn5_pub.publish(btn5_b);
    btn6_pub.publish(btn6_b);
    btn7_pub.publish(btn7_b);
    btnstart_pub.publish(btnstart_b);
    btnback_pub.publish(btnback_b);
    btnhome_pub.publish(btnhome_b);
    axes0lr_pub.publish(axes0lr_f);
    axes0ud_pub.publish(axes0ud_f);
    axes1lr_pub.publish(axes1lr_f);
    axes1ud_pub.publish(axes1ud_f);
}

void RMcontroller::display()
{
    for(int i=1; i<9;++i)
    {
        if(myjoybutton.but[i])
        {
            //std::cout << "  " << i << "   buttom pressed!   \r";
            printf("  %d   buttom pressed!   \r", i);
            return;
        }
    }
    if(myjoybutton.start)
    {
        //std::cout << "start buttom pressed!   \r";
        printf("START buttom pressed!   \r");
        return;
    }
    if(myjoybutton.back)
    {
        //std::cout << " back buttom pressed!   \r";
        printf(" BACK buttom pressed!   \r");
        return;
    }
    if(myjoybutton.home)
    {
        printf(" HOME buttom pressed!   \r");
        return;
    }

    std::cout << "  no  buttom pressed!   \r";
//    if(myjoybutton.axisleft_lr)
//    {
//        cout << "left axis lr " << -myjoybutton.axisleft_lr << "/32768\r";
//    }
//    if(myjoybutton.axisleft_ud)
//    {
//        cout << "left axis ud " << -myjoybutton.axisleft_ud << "/32768\r";
//    }
}
