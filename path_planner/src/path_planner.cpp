#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <string.h>

#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "fcl/config.h"
#include "fcl/fcl.h"

using namespace fcl;
namespace ob = ompl::base;
namespace og = ompl::geometric;

ros::Publisher traj_pub;

class planner
{
private:
    ob::StateSpacePtr space;                        //解的状态空间
    ob::SpaceInformationPtr si;                     //状态空间的一个空间信息实例
    ob::ProblemDefinitionPtr pdef;                  //问题实例
    og::PathGeometric *pth;                         //得到的路径
    bool replan_flag = false;                       //重新规划标志
    std::shared_ptr<CollisionGeometryd> Quadcopter; //无人机碰撞模型
    std::shared_ptr<CollisionGeometryd> tree_obj;   //地图碰撞模型
    geometry_msgs::Point startp = geometry_msgs::Point();
    geometry_msgs::Point goalp = geometry_msgs::Point();
    double step_range; //搜索步长

    // 连续碰撞检测
    bool isContinuousCollisionFunction(geometry_msgs::PoseStamped pose, geometry_msgs::PoseStamped pose1)
    {
        // //检测
        // Vector3d translation(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        // Quaterniond rotation(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        // Transform3d tree_tf_goal = treeObj.getTransform();
        // Vector3d goal_translation(goalp.x, goalp.y, goalp.z);
        // Transform3d aircraft_tf_goal = Transform3d::Identity();
        // aircraft_tf_goal.translation() = goal_translation;

        // std::cout << "~~~~~~~~~~~~~~~~~~~~~~~" << result.is_collide << std::endl;

        // return (!result.is_collide);
        double current_x = pose.pose.position.x;
        double current_y = pose.pose.position.y;
        double current_z = pose.pose.position.z;

        double current_x1 = pose1.pose.position.x;
        double current_y1 = pose1.pose.position.y;
        double current_z1 = pose1.pose.position.z;

        //设计检测步长
        double distance = sqrt(pow((current_x1 - current_x), 2) + pow((current_y1 - current_y), 2) + pow((current_z1 - current_z), 2));
        int count = distance / (step_range);
        count++;

        double step_x = (current_x1 - current_x) / count;
        double step_y = (current_y1 - current_y) / count;
        double step_z = (current_z1 - current_z) / count;

        // 进行检测
        for (int i = 0; i <= count; i++)
        {
            if (!isVailedPoint(current_x, current_y, current_z))
            {
                return false;
            }
            current_x = current_x + step_x;
            current_y = current_y + step_y;
            current_z = current_z + step_z;
        }

        return true;
    }

    //碰撞检测
    bool isCollisionFunction(const ob::State *state)
    {
        const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        CollisionObjectd treeObj((tree_obj));
        CollisionObjectd aircraftObject(Quadcopter);

        // 检测当前位置是否会碰撞
        Vector3d translation(pos->values[0], pos->values[1], pos->values[2]);
        Quaterniond rotation(rot->w, rot->x, rot->y, rot->z);
        aircraftObject.setTransform(rotation, translation);
        CollisionRequestd requestType(1, false, 1, false);
        CollisionResultd collisionResult;
        collide(&aircraftObject, &treeObj, requestType, collisionResult);

        return (!collisionResult.isCollision());
    }

    bool isVailedPoint(double x, double y, double z)
    {
        CollisionObjectd treeObj((tree_obj));
        CollisionObjectd aircraftObject(Quadcopter);

        // 检测当前位置是否会碰撞
        Vector3d translation(x, y, z);
        Quaterniond rotation(0, 0, 0, 1);
        aircraftObject.setTransform(rotation, translation);
        CollisionRequestd requestType(1, false, 1, false);
        CollisionResultd collisionResult;
        collide(&aircraftObject, &treeObj, requestType, collisionResult);

        return (!collisionResult.isCollision());
    }

    //检测导航路径合法性
    bool isVailedWaypoints(const nav_msgs::Path::ConstPtr &msg)
    {
        int size = msg->poses.size();
        if (size < 2)
        {
            std::cout << "路径点数量小于2，请检查 " << std::endl;
            return false;
        }
        for (int i = 0; i < size - 1; ++i)
        {
            geometry_msgs::Point start_position = msg->poses[i].pose.position;
            geometry_msgs::Point goal_position = msg->poses[i + 1].pose.position;
            if (start_position.x == goal_position.x && start_position.y == goal_position.y && start_position.z == goal_position.z)
            {
                std::cout << "路径点" << i << "与" << i + 1 << "重合"
                          << "，请检查 " << std::endl;
                return false;
            }

            if (!isVailedPoint(start_position.x, start_position.y, start_position.z))
            {
                std::cout << "路径点" << i << "不在自由空间内，请检查重新选择" << std::endl;
                return false;
            }
        }

        geometry_msgs::Point end_position = msg->poses[size - 1].pose.position;
        if (!isVailedPoint(end_position.x, end_position.y, end_position.z))
        {
            std::cout << "路径点" << size - 1 << "不在自由空间内，请检查重新选择" << std::endl;
            return false;
        }
        else
            return true;
    }

    //优化路径
    ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr &si)
    {
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
        return obj;
    }

public:
    //构造方法
    planner(double uavl, double uavw, double uavh, double octo_resolution, double bound_xy, double bound_lowz, double bound_highz, double steprange)
    {
        Quadcopter = std::shared_ptr<CollisionGeometryd>(new fcl::Boxd(uavl, uavw, uavh));
        OcTreed *tree = new OcTreed(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(octo_resolution)));
        tree_obj = std::shared_ptr<CollisionGeometryd>(tree);

        //解的状态空间
        space = ob::StateSpacePtr(new ob::SE3StateSpace());

        // 开始状态
        ob::ScopedState<ob::SE3StateSpace> start(space);

        // 目标状态
        ob::ScopedState<ob::SE3StateSpace> goal(space);

        // 搜索的三维范围设置
        ob::RealVectorBounds bounds(3);

        bounds.setLow(0, -bound_xy);
        bounds.setHigh(0, bound_xy);
        bounds.setLow(1, -bound_xy);
        bounds.setHigh(1, bound_xy);
        bounds.setLow(2, bound_lowz);
        bounds.setHigh(2, bound_highz);

        space->as<ob::SE3StateSpace>()->setBounds(bounds);

        // 搜索步长
        step_range = steprange;

        //初始化起点和终点
        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));
        start->setXYZ(1, 0, 0);
        startp.x = 1;
        startp.y = 0;
        startp.z = 0;
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        goal->setXYZ(0, 0, 0);
        goalp.x = 0;
        goalp.y = 0;
        goalp.z = 0;
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

        //绑定碰撞检测
        si->setStateValidityChecker(std::bind(&planner::isCollisionFunction, this, std::placeholders::_1));

        //创建问题实例
        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

        //设置起点终点状态
        pdef->setStartAndGoalStates(start, goal);

        // set Optimizattion objective
        //pdef->setOptimizationObjective(planner::getPathLengthObjWithCostToGo(si));

        std::cout << "初始化完成" << std::endl;
    }

    //析构函数
    ~planner()
    {
    }

    //进行路径规划
    bool plan(nav_msgs::Path &msg)
    {
        // 创建规划器
        //og::InformedRRTstar *rrt = new og::InformedRRTstar(si);
        og::RRTConnect *rrt = new og::RRTConnect(si);
        rrt->setRange(step_range); //设置步长

        ob::PlannerPtr plan(rrt);

        // 设置问题
        plan->setProblemDefinition(pdef);

        // 完成规划器设置
        plan->setup();

        // 打印规划器设置信息
        si->printSettings(std::cout);

        std::cout << "problem setting\n";
        // 打印当前问题信息
        pdef->print(std::cout);

        // 进行求解尝试
        ob::PlannerStatus solved = plan->solve(7);

        if (solved.asString() == "Exact solution")
        {
            // 找到可用路径，将其打印出来
            std::cout << "找到路径:" << std::endl;
            ob::PathPtr path = pdef->getSolutionPath();
            pth = pdef->getSolutionPath()->as<og::PathGeometric>();
            //pth->printAsMatrix(std::cout);

            //生成路径消息
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";

            //for (std::size_t path_idx = 0; path_idx < pth->getStateCount(); path_idx++)
            std::size_t path_idx = 0;
            while (path_idx < pth->getStateCount())
            {
                const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

                // 生成路径的位置信息
                const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

                // 生成路径的方向信息
                const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

                geometry_msgs::PoseStamped pose;

                pose.pose.position.x = pos->values[0];
                pose.pose.position.y = pos->values[1];
                pose.pose.position.z = pos->values[2];

                pose.pose.orientation.x = rot->x;
                pose.pose.orientation.y = rot->y;
                pose.pose.orientation.z = rot->z;
                pose.pose.orientation.w = rot->w;

                msg.poses.push_back(pose);

                // if (isContinuousCollisionFunction(pose))
                // {
                //     path_idx = pth->getStateCount() - 1;
                //     const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

                //     // 生成路径的位置信息
                //     const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

                //     // 生成路径的方向信息
                //     const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

                //     geometry_msgs::PoseStamped pose_goal;

                //     pose_goal.pose.position.x = pos->values[0];
                //     pose_goal.pose.position.y = pos->values[1];
                //     pose_goal.pose.position.z = pos->values[2];

                //     pose_goal.pose.orientation.x = rot->x;
                //     pose_goal.pose.orientation.y = rot->y;
                //     pose_goal.pose.orientation.z = rot->z;
                //     pose_goal.pose.orientation.w = rot->w;

                //     msg.poses.push_back(pose_goal);
                //     break;
                // }
                for (std::size_t path_idy = pth->getStateCount() - 1; path_idy > path_idx; path_idy--)
                {
                    se3state = pth->getState(path_idy)->as<ob::SE3StateSpace::StateType>();

                    // 生成路径的位置信息
                    pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

                    // 生成路径的方向信息
                    rot = se3state->as<ob::SO3StateSpace::StateType>(1);

                    geometry_msgs::PoseStamped pose1;

                    pose1.pose.position.x = pos->values[0];
                    pose1.pose.position.y = pos->values[1];
                    pose1.pose.position.z = pos->values[2];

                    pose1.pose.orientation.x = rot->x;
                    pose1.pose.orientation.y = rot->y;
                    pose1.pose.orientation.z = rot->z;
                    pose1.pose.orientation.w = rot->w;
                    if (isContinuousCollisionFunction(pose, pose1))
                    {
                        msg.poses.push_back(pose1);
                        path_idx = path_idy + 1;
                        break;
                    }
                }
            }
            //traj_pub.publish(msg);

            //完成后重置问题实例状态
            pdef->clearSolutionPaths();
            replan_flag = false;

            return true;
        }
        else
        {
            pdef->clearSolutionPaths();
            //std::cout << "没有找到相应的路径" << std::endl;
            return false;
        }
    }

    //当地图发生变化调用
    void replan(void)
    {
        std::cout << "路径总点数:" << pth->getStateCount() << std::endl;
        if (pth->getStateCount() <= 2)
        {
            //plan();
        }
        else
        {
            for (std::size_t idx = 0; idx < pth->getStateCount(); idx++)
            {
                if (!replan_flag)
                    replan_flag = !isCollisionFunction(pth->getState(idx));
                else
                    break;
            }
            if (replan_flag)
            {
                //plan();
            }
            else
                // 每个点都没有碰撞
                std::cout << "不需要重新规划" << std::endl;
        }
    }

    void updateMap(std::shared_ptr<fcl::CollisionGeometryd> map)
    {
        tree_obj = map;
    }

    bool setStart(double x, double y, double z)
    {
        // if (x == goalp.x && y == goalp.y && z == goalp.z)
        // {
        //     std::cout << "起点与终点重合，请重新选择" << std::endl;
        //     return false;
        // }

        // if (isVailedPoint(x, y, z))
        // {
        //     ob::ScopedState<ob::SE3StateSpace> start(space);
        //     start->setXYZ(x, y, z);
        //     startp.x = x;
        //     startp.y = y;
        //     startp.z = z;
        //     start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        //     pdef->clearStartStates();
        //     pdef->addStartState(start);
        //     std::cout << "起点设置为: " << x << " " << y << " " << z << std::endl;
        //     return true;
        // }
        // else
        // {
        //     std::cout << "该点不在自由空间内，请重新选择" << std::endl;
        //     return false;
        // }

        ob::ScopedState<ob::SE3StateSpace> start(space);
        start->setXYZ(x, y, z);
        startp.x = x;
        startp.y = y;
        startp.z = z;
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        pdef->clearStartStates();
        pdef->addStartState(start);
    }

    bool setGoal(double x, double y, double z)
    {
        // if (x == startp.x && y == startp.y && z == startp.z)
        // {
        //     std::cout << "起点与终点重合，请重新选择" << std::endl;
        //     return false;
        // }

        // if (isVailedPoint(x, y, z))
        // {
        //     ob::ScopedState<ob::SE3StateSpace> goal(space);
        //     goal->setXYZ(x, y, z);
        //     goalp.x = x;
        //     goalp.y = y;
        //     goalp.z = z;
        //     goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        //     pdef->clearGoal();
        //     pdef->setGoalState(goal);
        //     std::cout << "终点设置为: " << x << " " << y << " " << z << std::endl;
        //     return true;
        // }
        // else
        // {
        //     std::cout << "该点不在自由空间内，请重新选择" << std::endl;
        //     return false;
        // }
        ob::ScopedState<ob::SE3StateSpace> goal(space);
        goal->setXYZ(x, y, z);
        goalp.x = x;
        goalp.y = y;
        goalp.z = z;
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        pdef->clearGoal();
        pdef->setGoalState(goal);
    }

    // 多点路径的规划
    void waypointsPlan(const nav_msgs::Path::ConstPtr &msg)
    {
        if (isVailedWaypoints(msg))
        {
            nav_msgs::Path nav_path_msg;
            for (int i = 0; i < msg->poses.size() - 1; ++i)
            {
                geometry_msgs::Point start_position = msg->poses[i].pose.position;
                geometry_msgs::Point goal_position = msg->poses[i + 1].pose.position;
                setStart(start_position.x, start_position.y, start_position.z);
                setGoal(goal_position.x, goal_position.y, goal_position.z);
                if (!plan(nav_path_msg))
                {
                    std::cout << "路径点" << i << "与" << i + 1 << "之间寻找路径失败"
                              << std::endl;
                    return;
                }

                // 去除重复终点
                if (i < msg->poses.size() - 2)
                {
                    nav_path_msg.poses.pop_back();
                }
            }
            traj_pub.publish(nav_path_msg);
        }
    }
};

void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg, planner *planner_ptr)
{
    octomap::OcTree *tree_oct = dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg));
    fcl::OcTreed *tree = new fcl::OcTreed(std::shared_ptr<const octomap::OcTree>(tree_oct));

    // 更新地图碰撞信息
    planner_ptr->updateMap(std::shared_ptr<fcl::CollisionGeometryd>(tree));
    //planner_ptr->plan();
}

// void startCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner *planner_ptr)
// {
//     if (planner_ptr->setStart(msg->point.x, msg->point.y, msg->point.z))
//     {
//     }
//     //planner_ptr->plan();
// }

// void goalCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner *planner_ptr)
// {
//     if (planner_ptr->setGoal(msg->point.x, msg->point.y, msg->point.z))
//     {
//     }
//     //planner_ptr->plan();
// }

// 接收标点功能发来的路径信息
void pathCb(const nav_msgs::Path::ConstPtr &msg, planner *planner_ptr)
{
    planner_ptr->waypointsPlan(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle n;

    // 定义参数
    double uavl;
    double uavw;
    double uavh;
    double octo_resolution;
    double bound_xy;
    double bound_lowz;
    double bound_highz;
    double step_range;
    // 获取参数
    n.param("path_planner_node/uavl", uavl, 0.8);
    n.param("path_planner_node/uavw", uavw, 0.8);
    n.param("path_planner_node/uavh", uavh, 0.3);
    n.param("path_planner_node/octo_resolution", octo_resolution, 5.0);
    n.param("path_planner_node/bound_xy", bound_xy, 8000.0);
    n.param("path_planner_node/bound_lowz", bound_lowz, 0.0);
    n.param("path_planner_node/bound_highz", bound_highz, 100.0);
    n.param("path_planner_node/step_range", step_range, 3.0);

    planner planner_object = planner(uavl, uavw, uavh, octo_resolution, bound_xy, bound_lowz, bound_highz, step_range);

    ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, boost::bind(&octomapCallback, _1, &planner_object));
    //ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PointStamped>("/nav/goal", 1, boost::bind(&goalCb, _1, &planner_object));
    //ros::Subscriber start_sub = n.subscribe<geometry_msgs::PointStamped>("/nav/start", 1, boost::bind(&startCb, _1, &planner_object));
    ros::Subscriber path_sub = n.subscribe<nav_msgs::Path>("/path/waypoints", 1, boost::bind(&pathCb, _1, &planner_object));

    traj_pub = n.advertise<nav_msgs::Path>("/nav/waypoints", 1);

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    ros::spin();

    return 0;
}