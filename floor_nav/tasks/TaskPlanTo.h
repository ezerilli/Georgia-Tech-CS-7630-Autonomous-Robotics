#ifndef TASK_PlanTo_H
#define TASK_PlanTo_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskPlanToConfig.h"
#include <geometry_msgs/Pose2D.h>

using namespace task_manager_lib;

namespace floor_nav {
    class TaskPlanTo : public TaskInstance<TaskPlanToConfig,SimTasksEnv>
    {
        protected:
            ros::Publisher goalPub;
            ros::Subscriber trackSub;
            geometry_msgs::Pose2D error;
            ros::Time publish_time;
            void error_cb(geometry_msgs::Pose2DConstPtr msg);
        public:
            TaskPlanTo(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskPlanTo() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();
    };
    class TaskFactoryPlanTo : public TaskDefinition<TaskPlanToConfig, SimTasksEnv, TaskPlanTo>
    {

        public:
            TaskFactoryPlanTo(TaskEnvironmentPtr env) : 
                Parent("PlanTo","Reach a desired destination using the path planner",true,env) {}
            virtual ~TaskFactoryPlanTo() {};
    };
};

#endif // TASK_PlanTo_H
