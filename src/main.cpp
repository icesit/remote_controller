
#include <iostream>
#include <fstream>
#include <iomanip>
#include <time.h>
#include <stdlib.h>
#include "Remotecontroller.h"
#include "ros/ros.h"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv,"RemoteController");
    ros::NodeHandle n;
    ros::Rate r(ONESECOND);

    RMcontroller rmc(&n);

    cout << "[JOYSTICK]joystick_thread start!" << endl;
    while (ros::ok()){
        rmc.work();
        ros::spinOnce();
        r.sleep();
    }

    ros::shutdown();
    return 0;
}
