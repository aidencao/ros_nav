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
    void plan(void)
    {
        // 创建规划器
        og::InformedRRTstar *rrt = new og::InformedRRTstar(si);
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
        ob::PlannerStatus solved = plan->solve(1);

        if (solved.asString() == "Exact solution")
        {
            // 找到可用路径，将其打印出来
            std::cout << "找到路径:" << std::endl;
            ob::PathPtr path = pdef->getSolutionPath();
            pth = pdef->getSolutionPath()->as<og::PathGeometric>();
            pth->printAsMatrix(std::cout);

            //生成路径消息
            nav_msgs::Path msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "camera";

            for (std::size_t path_idx = 0; path_idx < pth->getStateCount(); path_idx++)
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
            }
            traj_pub.publish(msg);

            //完成后重置问题实例状态
            pdef->clearSolutionPaths();
            replan_flag = false;
        }
        else
        {
            pdef->clearSolutionPaths();
            std::cout << "没有找到相应的路径" << std::endl;
        }
    }

    //当地图发生变化调用
    void replan(void)
    {
        std::cout << "路径总点数:" << pth->getStateCount() << std::endl;
        if (pth->getStateCount() <= 2)
            plan();
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
                plan();
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
        if (x == goalp.x && y == goalp.y && z == goalp.z)
        {
            std::cout << "起点与终点重合，请重新选择" << std::endl;
            return false;
        }

        if (isVailedPoint(x, y, z))
        {
            ob::ScopedState<ob::SE3StateSpace> start(space);
            start->setXYZ(x, y, z);
            startp.x = x;
            startp.y = y;
            startp.z = z;
            start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
            pdef->clearStartStates();
            pdef->addStartState(start);
            std::cout << "起点设置为: " << x << " " << y << " " << z << std::endl;
            return true;
        }
        else
        {
            std::cout << "该点不在自由空间内，请重新选择" << std::endl;
            return false;
        }
    }

    bool setGoal(double x, double y, double z)
    {
        if (x == startp.x && y == startp.y && z == startp.z)
        {
            std::cout << "起点与终点重合，请重新选择" << std::endl;
            return false;
        }

        if (isVailedPoint(x, y, z))
        {
            ob::ScopedState<ob::SE3StateSpace> goal(space);
            goal->setXYZ(x, y, z);
            goalp.x = x;
            goalp.y = y;
            goalp.z = z;
            goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
            pdef->clearGoal();
            pdef->setGoalState(goal);
            std::cout << "终点设置为: " << x << " " << y << " " << z << std::endl;
            return true;
        }
        else
        {
            std::cout << "该点不在自由空间内，请重新选择" << std::endl;
            return false;
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

void startCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner *planner_ptr)
{
    if (planner_ptr->setStart(msg->point.x, msg->point.y, msg->point.z))
        planner_ptr->plan();
}

void goalCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner *planner_ptr)
{
    if (planner_ptr->setGoal(msg->point.x, msg->point.y, msg->point.z))
        planner_ptr->plan();
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
    n.param("path_planner/path", uavl, 0.8);
    n.param("path_planner/path", uavw, 0.8);
    n.param("path_planner/path", uavh, 0.3);
    n.param("path_planner/path", octo_resolution, 5.0);
    n.param("path_planner/path", bound_xy, 8000.0);
    n.param("path_planner/path", bound_lowz, 0.0);
    n.param("path_planner/path", bound_highz, 100.0);
    n.param("path_planner/path", step_range, 3.0);

    planner planner_object = planner(uavl, uavw, uavh, octo_resolution, bound_xy, bound_lowz, bound_highz, step_range);

    ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, boost::bind(&octomapCallback, _1, &planner_object));
    ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PointStamped>("/nav/goal", 1, boost::bind(&goalCb, _1, &planner_object));
    ros::Subscriber start_sub = n.subscribe<geometry_msgs::PointStamped>("/nav/start", 1, boost::bind(&startCb, _1, &planner_object));

    traj_pub = n.advertise<nav_msgs::Path>("waypoints", 1);

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    ros::spin();

    return 0;
}