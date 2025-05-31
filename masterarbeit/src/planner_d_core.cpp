#include "masterarbeit/planner_d_core.h"
#include "masterarbeit/CollisionAvoidance.h"
#include "masterarbeit/dstar.h"

#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <random>
#include <tf2/LinearMath/Quaternion.h>


PLUGINLIB_EXPORT_CLASS(masterarbeit::GlobalDPlanner, nav_core::BaseGlobalPlanner)

namespace masterarbeit {

void GlobalDPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

GlobalDPlanner::GlobalDPlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true),
        p_calc_(NULL), planner_dstar_(NULL), orientation_filter_(NULL),
        potential_array_(NULL), tfListener(tfBuffer){
}

GlobalDPlanner::GlobalDPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        GlobalDPlanner() {
    initialize(name, costmap, frame_id);
    
}

GlobalDPlanner::~GlobalDPlanner() {
    if (p_calc_)
        delete p_calc_;
    if (planner_dstar_)
        delete planner_dstar_;
    if (dsrv_)
        delete dsrv_;
}

void GlobalDPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}
  
void GlobalDPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        ros::NodeHandle nh("~"),g_nh;
        nh.param("robot", robot, std::string(""));
        costmap_ = costmap;
        frame_id_ = frame_id;

        intersect_sub_ = g_nh.subscribe("heatmap_processor/intersect", 1, &GlobalDPlanner::intersectCallback, this);

        odom_sub_ = g_nh.subscribe("odom", 1, &GlobalDPlanner::odomCallback, this);

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();
        double cr = costmap->getResolution();
        potential_array_ = new float[cx * cy];

        convert_offset_ = 0.0;

        p_calc_ = new PotentialDCalculator(cx, cy);
       
        planner_dstar_ = new DStarExpansion(p_calc_, cx, cy, costmap_->getCharMap());
        path_choice = -1;
        count = 0; 
        
        orientation_filter_ = new global_planner::OrientationFilter();

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        plan_failed_pub_ = private_nh.advertise<nav_msgs::Path>("alternative_plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);
        plan_basefootprint_pub_ = private_nh.advertise<nav_msgs::Path>("base_footprint_plan", 1);
        base_footprint_pub_ = g_nh.advertise<masterarbeit::CollisionAvoidance>("base_footprint_plan_data", 1);


        private_nh.param("allow_unknown", allow_unknown_, true);
        ROS_WARN("Parameter allow_unknown set to %s", allow_unknown_ ? "true" : "false");

        planner_dstar_->setHasUnknown(allow_unknown_);
        

        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);
        private_nh.param("outline_map", outline_map_, true);
        ROS_WARN("Parameter outline_map set to %s", outline_map_ ? "true" : "false");
        make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalDPlanner::makePlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>::CallbackType cb =
                [this](auto& config, auto level){ reconfigureCB(config, level); };
        dsrv_->setCallback(cb);
        gen_ = std::mt19937(rd_());
        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}

void GlobalDPlanner::intersectCallback(const masterarbeit::Intersect::ConstPtr& intersect_msg) {
    // Standardwerte zurücksetzen
    intersect_.header = intersect_msg->header;
    intersect_.center_x.clear();
    intersect_.center_y.clear();
    intersect_.radius.clear();
    intersect_.circles.clear();
    // transformed_circles_.clear();

    // Direkt übernehmen, keine Transformation mehr
    if (intersect_msg->intersect) 
    {
        intersect_.intersect = true;
        intersect_.time = intersect_msg->time;

        // Übernehme die Kreisinformationen 1:1
        intersect_.center_x = intersect_msg->center_x;
        intersect_.center_y = intersect_msg->center_y;
        intersect_.radius   = intersect_msg->radius;
        intersect_.header.frame_id = intersect_msg->header.frame_id;
        intersect_.circles = intersect_msg->circles;

    } 
    else 
    {
        intersect_.intersect = false;
    }
}

void GlobalDPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_vel = msg->twist.twist.linear.x;
    odom_pos_= msg->pose.pose.position;
}

void GlobalDPlanner::reconfigureCB(global_planner::GlobalPlannerConfig& config, uint32_t level)
{

    planner_dstar_->setLethalCost(config.lethal_cost);
    planner_dstar_->setNeutralCost(config.neutral_cost);
    planner_dstar_->setFactor(config.cost_factor);

    publish_potential_ = config.publish_potential;
    orientation_filter_->setMode(config.orientation_mode);
    orientation_filter_->setWindowSize(config.orientation_window_size);
}

void GlobalDPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool GlobalDPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

void GlobalDPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool GlobalDPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool GlobalDPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

bool GlobalDPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                        double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }
    
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    if (goal.header.frame_id != global_frame) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
        return false;
    }

    if (start.header.frame_id != global_frame) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    worldToMap(wx, wy, start_x, start_y);    

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    worldToMap(wx, wy, goal_x, goal_y);

    clearRobotCell(start, start_x_i, start_y_i);
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    
    p_calc_->setSize(nx, ny);

    planner_dstar_->setSize(nx, ny);

    if(outline_map_)
        outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);


    bool found_legal = planner_dstar_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                        nx * ny, potential_array_);
    
    if(publish_potential_) publishPotential(potential_array_);
    
    if (found_legal) {
        if (!getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, start, plan)) {
        }
    }else{
        ROS_ERROR_THROTTLE(5.0, "Failed to get a plan.");
    }

    
    return !plan.empty();
}

void GlobalDPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

void GlobalDPlanner::publishPlanAlternative(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_failed_pub_.publish(gui_path);
}

bool GlobalDPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                        const geometry_msgs::PoseStamped& goal, const geometry_msgs::PoseStamped& start,
                                        std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }
    ros::WallTime t_start = ros::WallTime::now();
    std::string global_frame = frame_id_;
    ros::Time plan_time;

    plan.clear();
    double primary_length, alternative_length;

    std::vector<geometry_msgs::PoseStamped> base_footprint_plan;
    std::vector<geometry_msgs::PoseStamped> base_footprint_alternative_plan;
    geometry_msgs::PoseStamped goal_base_footprint;

    std::vector<std::pair<float,float>> path;
    
   // Oben im Code: Nur einmal nachschauen
    tf2::Transform tf_map_to_base;
    geometry_msgs::TransformStamped transform_msg;
    bool tf_available = false;
    try 
    {
        transform_msg = tfBuffer.lookupTransform(robot + "_tf/base_footprint", "map", ros::Time(0), ros::Duration(0.1));
        tf2::fromMsg(transform_msg.transform, tf_map_to_base);
        tf_available = true;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_STREAM(robot << " Planner Core: Transform lookup failed: " << ex.what());
        return false;
    }

    
    if (planner_dstar_ && !planner_dstar_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path))
    {
        ROS_ERROR("Failed to get a path!");
        return false;
    } 
    else 
    {
        
        planner_dstar_->dStarPath = path;
        plan_time = ros::Time::now();
        for (const auto& point : path) 
        {
            double world_x, world_y;
            mapToWorld(point.first, point.second, world_x, world_y);

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = global_frame;
            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
        }

        orientation_filter_->processPath(start, plan);
        publishPlan(plan);
        std::vector<geometry_msgs::PoseStamped> alternativePlan;

       

        double mx_d, my_d;
        bool dist = false;
        
        if (intersect_.intersect && !intersect_.center_x.empty())
        {
            double wx = intersect_.center_x[0];
            double wy = intersect_.center_y[0];
            if (!worldToMap(wx, wy, mx_d, my_d))
            {
                ROS_WARN("Intersect-Zentrum liegt nicht im Costmap-Bereich!");
            }
            dist = (std::hypot(goal_x - mx_d, goal_y - my_d)  <= std::hypot(goal_x - start_x, goal_y - start_y) + 5);
        }
        
        if (intersect_.intersect && !intersect_.center_x.empty() && !path.empty() && intersect_.radius[0] > 3* costmap_->getResolution() && dist ) 
        {
            bool valid_intersect = true, main_direction_left = false;
            double r_cells;
            int mx, my;
            std::vector<std::pair<float,float>>  alternativePath;

            mx = int(std::round(mx_d));
            my = int(std::round(my_d));
            r_cells = std::ceil((intersect_.radius[0] +0.1) / costmap_->getResolution()); // +0.1 inflation_radius
        
            if (valid_intersect)
            {
                alternativePath = planner_dstar_->getAlternativePath(path, start_x, start_y, goal_x, goal_y, mx, my, r_cells, &main_direction_left);
                if(!alternativePath.empty())
                {
                    
                    for (const auto& point : alternativePath) 
                    {
                        double world_x, world_y;
                        mapToWorld(point.first, point.second, world_x, world_y);
                        geometry_msgs::PoseStamped pose;
                        pose.header.stamp = plan_time;
                        pose.header.frame_id = global_frame;
                        pose.pose.position.x = world_x;
                        pose.pose.position.y = world_y;
                        pose.pose.position.z = 0.0;
                        pose.pose.orientation.x = 0.0;
                        pose.pose.orientation.y = 0.0;
                        pose.pose.orientation.z = 0.0;
                        pose.pose.orientation.w = 1.0;
                        alternativePlan.push_back(pose);
                    }
                    primary_length = computePlanLength(plan);
                    alternative_length = computePlanLength(alternativePlan);
                    orientation_filter_->processPath(start, alternativePlan);
                    publishPlanAlternative(alternativePlan);
                
                    if(tf_available)
                    {
                        base_footprint_plan = plan;
                        base_footprint_alternative_plan = alternativePlan;
                        goal_base_footprint = goal;
                        if (intersect_.header.frame_id != robot + "_tf/base_footprint")
                        {
                            transformIntersectDataToBase(tf_map_to_base, base_footprint_plan, base_footprint_alternative_plan, goal_base_footprint);
                        }
                            
                        else
                        {
                            transformPlanAndGoal(tf_map_to_base,base_footprint_plan, base_footprint_alternative_plan, goal_base_footprint);
                        }
                    }
                    // ------Datensammlung-----
                    // if(path_choice == -1)
                    // {
                        
                    //     std::uniform_int_distribution<> dis(0, 1);
                    //     path_choice = dis(gen_);
                    //     if(path_choice == 0)
                    //     {
                    //         initial_side = {0,main_direction_left}; 
                    //     } 
                    //     else 
                    //     {
                    //         initial_side = {1,!main_direction_left};
                    //         std::swap(plan, alternativePlan);
                    //         std::swap(base_footprint_plan, base_footprint_alternative_plan);
                    //     }
                    //     count++;
                    //     // ROS_WARN_STREAM(robot << " Waehle Pfad: " << path_choice << " Ausweichzahl: " << count << " Hauptpfad war: " << (main_direction_left ? "LINKS" : "RECHTS"));
                    // }
                    // -----Phänotyp-------

                    bool decision_left;
                    geometry_msgs::PointStamped in, out;
                    in.point           = odom_pos_;

                    tf2::doTransform(in, out, transform_msg);

                    double robot_pos_x = out.point.x;
                    double robot_pos_y = out.point.y;
                    if (path_choice == -1)
                    {
                        decision_left = decide_action_from_phenotype(
                            main_direction_left,
                            intersect_.center_x[0], intersect_.center_y[0], intersect_.radius[0],
                            intersect_.circles[0].center.x, intersect_.circles[0].center.y,
                            intersect_.circles[0].velocity.x, intersect_.circles[0].velocity.y,
                            robot_pos_x, robot_pos_y
                        );
                        count++;
                        
                    } else
                    {
                        // Folgeaufrufe: Input ist die Entscheidung vom letzten Mal
                        bool prev_decision_left = initial_side.second;
                        decision_left = decide_action_from_phenotype(
                            prev_decision_left,
                            intersect_.center_x[0], intersect_.center_y[0], intersect_.radius[0],
                            intersect_.circles[0].center.x, intersect_.circles[0].center.y,
                            intersect_.circles[0].velocity.x, intersect_.circles[0].velocity.y,
                            robot_pos_x, robot_pos_y
                        );
                    }

                    // 3) Speichere die Entscheidung für den nächsten Zyklus
                    initial_side.second = decision_left;
                    // 4) Mappe Entscheidung auf path_choice (0=Hauptpfad, 1=Alternativpfad)
                    int new_choice = decision_left ? 0 : 1;

                    // 5) Wenn sich die Wahl geändert hat, swappe die beiden Pfade
                    if (new_choice != path_choice) {
                        path_choice = new_choice;
                        std::swap(plan,             alternativePlan);
                        std::swap(base_footprint_plan, base_footprint_alternative_plan);
                    }
                    ROS_WARN_STREAM(robot << " Waehle Pfad: " << path_choice << " Ausweichzahl: " << count << " Hauptpfad war: " << (main_direction_left ? "LINKS" : "RECHTS"));
                    ros::WallTime t_end = ros::WallTime::now();
                    ros::WallDuration d = t_end - t_start;
                    realtime_duration_ = d.toSec();  // double
                    publishBaseFootprintData(base_footprint_plan, base_footprint_alternative_plan, goal_base_footprint,
                        path_choice, intersect_, odom_vel, primary_length, alternative_length, initial_side.second);
                }
            }
        }
        else if(intersect_.intersect && !intersect_.center_x.empty() && !path.empty() && intersect_.radius[0] <= 3* costmap_->getResolution())
        {
            
        }
        else
        {
            path_choice = -1;
            initial_side = {};
            publishPlanAlternative(alternativePlan);
        }

    }
    return !plan.empty();
}

void GlobalDPlanner::publishPotential(float* potential)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }
    
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else {
            if (fabs(max) < DBL_EPSILON) {
                grid.data[i] = -1;
            } else {
                grid.data[i] = potential_array_[i] * publish_scale_ / max;
            }
        }
    }
    potential_pub_.publish(grid);
}

void GlobalDPlanner::publishBaseFootprintData(const std::vector<geometry_msgs::PoseStamped>& base_footprint_plan,
                                              const std::vector<geometry_msgs::PoseStamped>& base_footprint_alternative_plan,
                                              const geometry_msgs::PoseStamped& goal_base_footprint,
                                              int random_choice,
                                              const masterarbeit::Intersect& intersect,
                                              float odom_vel, float primary_length, float alternative_length, bool direction) 
                                              { 
    masterarbeit::CollisionAvoidance msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = robot + "_tf/base_footprint";
    msg.base_footprint_plan = base_footprint_plan;
    msg.base_footprint_alternative_plan = base_footprint_alternative_plan;
    msg.goal_base_footprint = goal_base_footprint;
    msg.path_choice = random_choice;
    msg.intersect = intersect;
    msg.main_path_length = primary_length;
    msg.alternative_path_length = alternative_length;
    msg.odom_vel = odom_vel;
    msg.direction = direction;
    msg.realtime_duration = realtime_duration_;

    base_footprint_pub_.publish(msg);
}

void GlobalDPlanner::transformIntersectDataToBase(
    const tf2::Transform& tf_map_to_base,
    std::vector<geometry_msgs::PoseStamped>& base_footprint_plan,
    std::vector<geometry_msgs::PoseStamped>& base_footprint_alternative_plan,
    geometry_msgs::PoseStamped& goal_base_footprint)
{
    if (intersect_.center_x.empty()) return;

    tf2::Vector3 first_map(intersect_.center_x[0], intersect_.center_y[0], 0.0);
    tf2::Vector3 first_base = tf_map_to_base * first_map;
    extra_yaw_ = -std::atan2(first_base.y(), first_base.x());

    tf2::Quaternion q_extra;
    q_extra.setRPY(0, 0, extra_yaw_);
    tf2::Transform tf_extra(q_extra);

    std::vector<float> tx, ty;
    tx.reserve(intersect_.center_x.size());
    ty.reserve(intersect_.center_y.size());
    for (size_t i = 0; i < intersect_.center_x.size(); ++i)
    {
        tf2::Vector3 p_orig(intersect_.center_x[i],
                             intersect_.center_y[i], 0.0);

        tf2::Vector3 p_base = tf_map_to_base * p_orig;
        tf2::Vector3 p_final = tf_extra * p_base;
        tx.push_back(p_final.x());
        ty.push_back(p_final.y());
    }
    intersect_.center_x = tx;
    intersect_.center_y = ty;
    intersect_.header.frame_id = robot + "_tf/base_footprint";

    std::vector<obstacle_detector::CircleObstacle> transformed;
    transformed.reserve(intersect_.circles.size());
    for (const auto& circle : intersect_.circles)
    {
        obstacle_detector::CircleObstacle nc;
        // Center
        tf2::Vector3 c_orig(circle.center.x, circle.center.y, 0.0);
        tf2::Vector3 c_base = tf_map_to_base * c_orig;
        tf2::Vector3 c_final = tf_extra * c_base;
        nc.center.x = c_final.x();
        nc.center.y = c_final.y();
        nc.center.z = 0.0;

        tf2::Vector3 v_base = tf_map_to_base.getBasis() * 
                              tf2::Vector3(circle.velocity.x, circle.velocity.y, 0.0);
        tf2::Vector3 v_final = tf_extra.getBasis() * v_base;
        nc.velocity.x = v_final.x();
        nc.velocity.y = v_final.y();
        nc.velocity.z = 0.0;

        nc.radius      = circle.radius;
        nc.true_radius = circle.true_radius;
        nc.particlefilter.header = intersect_.header;
        nc.particlefilter = circle.particlefilter;
        transformed.push_back(nc);
    }
    intersect_.circles = std::move(transformed);

    auto apply_two_step = [&](geometry_msgs::PoseStamped &p)
    {
        tf2::Vector3 orig(p.pose.position.x,
                          p.pose.position.y,
                          p.pose.position.z);
        tf2::Vector3 base = tf_map_to_base * orig;
        tf2::Vector3 fin  = tf_extra * base;
        p.pose.position.x = fin.x();
        p.pose.position.y = fin.y();
        p.pose.position.z = fin.z();
        p.header.frame_id = robot + "_tf/base_footprint";
    };

    for (auto &p : base_footprint_plan)
        apply_two_step(p);
    for (auto &p : base_footprint_alternative_plan)
        apply_two_step(p);
    apply_two_step(goal_base_footprint);
}

void GlobalDPlanner::transformPlanAndGoal(
    const tf2::Transform& tf_map_to_base,
    std::vector<geometry_msgs::PoseStamped>& base_footprint_plan,
    std::vector<geometry_msgs::PoseStamped>& base_footprint_alternative_plan,
    geometry_msgs::PoseStamped& goal_base_footprint)
{

    tf2::Quaternion q_extra;
    q_extra.setRPY(0, 0, extra_yaw_);
    tf2::Transform tf_extra(q_extra);

    auto apply_two_step = [&](geometry_msgs::PoseStamped &p)
    {
        tf2::Vector3 orig(p.pose.position.x,
                          p.pose.position.y,
                          p.pose.position.z);
        tf2::Vector3 base = tf_map_to_base * orig;
        tf2::Vector3 fin  = tf_extra * base;
        p.pose.position.x = fin.x();
        p.pose.position.y = fin.y();
        p.pose.position.z = fin.z();
        p.header.frame_id = robot + "_tf/base_footprint";
    };

    for (auto &p : base_footprint_plan)
        apply_two_step(p);
    for (auto &p : base_footprint_alternative_plan)
        apply_two_step(p);
    apply_two_step(goal_base_footprint);

}


double GlobalDPlanner::computePlanLength(const std::vector<geometry_msgs::PoseStamped>& plan) const
{
  double length = 0.0;
  for (size_t i = 1; i < plan.size(); ++i)
  {
    double dx = plan[i].pose.position.x   - plan[i-1].pose.position.x;
    double dy = plan[i].pose.position.y   - plan[i-1].pose.position.y;
    length += std::hypot(dx, dy);
  }
  return length;
}

double GlobalDPlanner::protected_division(double numerator, double denominator)
{
    if (std::abs(denominator) < 1e-9) 
    {
        return 0.0;
    }
    return numerator / denominator;
}

bool GlobalDPlanner::decide_action_from_phenotype( bool direction, double intersect_centers_x, double intersect_centers_y,
    double intersect_radius, double obstacle_x, double obstacle_y, double velocity_x, double velocity_y, double robot_pos_x, double robot_pos_y)
{
    try
    {
        // Bedingung 1
        if (direction && std::hypot(4.44, (std::hypot((intersect_radius * obstacle_y), std::hypot(velocity_y, intersect_radius)) + -3.00)) >= (((protected_division(intersect_centers_x, robot_pos_y) + robot_pos_x) + robot_pos_y) * (std::hypot(std::hypot(velocity_y, intersect_centers_x), std::hypot(intersect_centers_x, velocity_x)) - std::hypot(std::hypot(robot_pos_y, velocity_x), obstacle_y))) || (robot_pos_x - robot_pos_x) > velocity_x && intersect_centers_x >= std::hypot(intersect_radius, std::hypot(velocity_y, intersect_radius))) {
            return true;
        }
        // Bedingung 2 (else if ...)
        else if (direction && (protected_division((6.47 * (obstacle_x - std::hypot(obstacle_y, robot_pos_x))), velocity_y)) >= -1.15 || velocity_y >= obstacle_y && robot_pos_y >= (intersect_radius * velocity_y) || std::hypot(std::hypot(robot_pos_x, intersect_radius), intersect_centers_x) < std::hypot(obstacle_y, velocity_x) || ! (std::hypot((velocity_x + robot_pos_y), std::hypot(intersect_centers_y, (protected_division(robot_pos_x, intersect_centers_x))))) >= (obstacle_x * velocity_y)) {
            return true;
        }
        // Bedingung 3 (else if ...)
        else if ((intersect_centers_x + velocity_y) <= (protected_division(robot_pos_y, obstacle_y)) && direction) {
            return false;
        }
        // Bedingung 4 (else if ...)
        else if ((intersect_radius + std::hypot(obstacle_y, intersect_radius)) <= (protected_division(robot_pos_y, velocity_x)) && direction) {
            return false;
        }
        // Bedingung 5 (else if ...)
        else if (velocity_x >= obstacle_x && !direction) {
            return true;
        }
        // Bedingung 6 (else if ...)
        else if (((protected_division((intersect_centers_x * -0.93), -2.20)) + std::hypot(intersect_centers_y, intersect_radius)) <= (intersect_radius + velocity_x) && direction) {
            return false;
        }
        // Bedingung 7 (else if ...)
        else if ((intersect_centers_x + velocity_y) < (protected_division(robot_pos_y, (obstacle_x - (intersect_centers_x + intersect_centers_x)))) && direction) {
            return false;
        }
        // Bedingung 8 (else if ...)
        else if (std::hypot(velocity_x, intersect_centers_y) < std::hypot(intersect_centers_y, robot_pos_x) && direction) {
            return false;
        }
        // Bedingung 9 (else if ...)
        else if (robot_pos_x <= velocity_y && direction) {
            return false;
        }
        // Bedingung 10 (else if ...)
        else if ((((velocity_y * (robot_pos_x + intersect_centers_x)) >= velocity_y) && (obstacle_x < obstacle_y || robot_pos_x >= std::hypot(robot_pos_y, robot_pos_y) && intersect_centers_x > std::hypot(std::hypot(obstacle_y, intersect_radius), velocity_y) || velocity_x >= obstacle_y && (protected_division(intersect_radius, robot_pos_y)) >= (protected_division(velocity_y, robot_pos_x)) && (-3.77 * robot_pos_y) != -3.88)) && (std::hypot(velocity_x, velocity_y) >= intersect_radius || robot_pos_x > std::hypot(robot_pos_y, intersect_centers_y) && (protected_division(-7.12, velocity_y)) > (protected_division(((std::hypot(-6.84, (intersect_centers_x + velocity_y)) / std::hypot((robot_pos_x - velocity_x), -3.88)) + std::hypot(intersect_radius, robot_pos_y)), (protected_division(robot_pos_y, obstacle_y)))) || velocity_x > std::hypot(obstacle_y, robot_pos_y) && !(velocity_x > intersect_radius || !(velocity_y != obstacle_x)) && (protected_division((intersect_radius - intersect_centers_x), robot_pos_y)) > (protected_division(std::hypot(velocity_y, velocity_y), robot_pos_x))) || direction) {
            return true;
        }
        // Bedingung 11 (else if ...)
        else if (!direction && intersect_centers_x >= std::hypot(intersect_radius, std::hypot(intersect_centers_y, intersect_radius))) {
            return false;
        }
        // Bedingung 12 (else if ...)
        else if (!direction && velocity_y >= std::hypot(intersect_centers_y, obstacle_y) && !(robot_pos_y < intersect_centers_y) || velocity_x >= velocity_y && robot_pos_x > std::hypot(robot_pos_y, robot_pos_y)) {
            return false;
        }
        // Bedingung 13 (else if ...)
        else if (direction && (robot_pos_x > velocity_y) && (intersect_radius >= velocity_x) || (velocity_x * intersect_centers_x) >= (velocity_x - velocity_y) && intersect_centers_x > std::hypot(intersect_radius, (velocity_x + obstacle_y))) {
            return true;
        }
        // Bedingung 14 (else if ...)
        else if (!direction && (velocity_y * velocity_y) < velocity_x || (robot_pos_y >= obstacle_y) || std::hypot(robot_pos_y, intersect_centers_y) <= std::hypot(velocity_y, velocity_y) && (protected_division(-5.12, (protected_division(velocity_y, velocity_x)))) < (protected_division(std::hypot(obstacle_x, velocity_x), velocity_y)) && (-0.83 - std::hypot(intersect_radius, obstacle_x)) <= (protected_division(std::hypot(velocity_y, velocity_x), (obstacle_x - intersect_radius))) || std::hypot(robot_pos_y, intersect_centers_y) < (robot_pos_x * (intersect_radius + robot_pos_y)) && obstacle_x <= (protected_division(std::hypot(obstacle_y, velocity_x), velocity_y)) && obstacle_y >= -5.43) {
            return false;
        }
        // Bedingung 15 (else if ...)
        else if (!direction && intersect_centers_x >= std::hypot(intersect_radius, std::hypot(intersect_centers_y, intersect_centers_x))) {
            return false;
        }
        // Bedingung 16 (else if ...)
        else if (!direction && obstacle_y > intersect_centers_x && !(velocity_y >= obstacle_y) || robot_pos_y < intersect_centers_y && robot_pos_x > std::hypot(intersect_centers_y, robot_pos_y)) {
            return false;
        }
        // Bedingung 17 (else if ...)
        else if (direction && (!(velocity_x <= intersect_centers_y)) || (-3.32 <= -4.43 && std::hypot(robot_pos_x, (std::hypot(protected_division(std::hypot(obstacle_x, obstacle_y), protected_division(velocity_y, velocity_x)), -0.98) / protected_division(std::hypot(robot_pos_y, std::hypot(intersect_radius, intersect_radius)), protected_division(protected_division(robot_pos_y, robot_pos_y), intersect_radius)))) < (std::hypot(std::hypot(std::hypot(intersect_centers_x, (robot_pos_y + velocity_x)), protected_division(protected_division(intersect_radius, intersect_radius), intersect_radius)), std::hypot(4.44, robot_pos_x)) + 3.03))) {
            return false;
        }
        // Finale else Bedingung
        else {
            return true;
        }
    }
    catch(...)
    {
        return direction;
    }
}

} //end namespace global_planner