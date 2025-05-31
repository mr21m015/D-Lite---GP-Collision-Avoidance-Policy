#include <ros/ros.h>
#include <ros/console.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "vector"
#include <stdlib.h> 
#include <string>
#include <random>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; ///< notwendig um das Goal zu publsihen

//

/** \brief Methode um das Goal zu senden
*   Hier werden die einzelnen Ziele nacheinander abgefahren. Es wurde eine Dauerschleife implementiert, dass sobald das letzte Goal angefahren wurde, er wieder das erste Goal anfahren soll usw.
*    Hierfür wurden folgende Quellen herangezogen:
*    http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
*    https://answers.ros.org/question/210987/sending-a-sequence-of-goals-to-move_base/
*    https://www.slideserve.com/xia/multi-robot-systems-with-ros-lesson-6
*    Es werden mehrere Goals angefahren, es lohnt sich daraus eine Methode zu machen --> Überblick besser & Codelänge kürzer
*/
void goToGoal(double x_goal, double y_goal, double w_goal, int goalNr, std::string robot)
{
    string move_base_str = "/" + robot + "/move_base";
    ROS_INFO("MovebaseClient: %s", move_base_str.c_str());
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac(move_base_str, true);
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x_goal;  //X-Wert übergeben
    goal.target_pose.pose.position.y = y_goal;  //Y-Wert übergeben
    goal.target_pose.pose.orientation.w = w_goal;   //Winkel übergeben

    // ROS_INFO("Sending goal");
    ac.sendGoal(goal);  //Goal senden
    // ROS_INFO("Fahre das Goal x=%f, y=%f, w=%f an",x_goal,y_goal,w_goal);    //Ausgabe dass Punkt i angefahren wird
    ac.waitForResult();

    // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //     ROS_INFO("Goal Nr. %d erreicht",goalNr);    //Punkt erreicht
    // else
    //     ROS_INFO("Anfahren des Goal Nr. %d fehlgeschlagen",goalNr); //Punkt nicht erreicht
}

//https://www.slideserve.com/xia/multi-robot-systems-with-ros-lesson-6
int main(int argc, char **argv){
    int randNumber;
    if(argc < 2){
        ROS_ERROR("You must specify robot ID");
        return -1;
    }
    string robotName = argv[1];
    int seedValue = stoi(argv[2]);
    // ROS_INFO("Seed Value of %s is %d",robotName.c_str(),seedValue);

    ros::init(argc, argv, "navigation_goals_" + robotName);
    vector<double> x,y,w;  //Vektor für Punktdaten
    
    //Goals aus goals.yaml einlesen
    ros::param::get("navigation_goals_" + robotName + "/x", x);  //X-Werte aus der Goals.yaml einlesen 
    ros::param::get("navigation_goals_" + robotName + "/y", y);  //Y-Werte aus der Goals.yaml einlesen
    ros::param::get("navigation_goals_" + robotName + "/w", w);  //Winkel aus der Goals.yaml einlesen 
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::console::notifyLoggerLevelsChanged();

    srand(seedValue);

    while (ros::ok)
    {   
        randNumber = rand() % x.size();
        // ROS_INFO("Roboter %s faehrt Punkt %d an",robotName.c_str(),randNumber);
        goToGoal(x[randNumber],y[randNumber],w[randNumber],randNumber, robotName); //Punkt i anfahren
        ros::spinOnce();
    }
  return 0;
}