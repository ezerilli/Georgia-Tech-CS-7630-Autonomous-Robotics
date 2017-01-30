#ifndef TASK_SET_HEADING_H
#define TASK_SET_HEADING_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskSetHeadingConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskSetHeading : public TaskInstance<TaskSetHeadingConfig,SimTasksEnv>
    {
        protected: 
            double initial_heading;
        public:
            TaskSetHeading(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetHeading() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactorySetHeading : public TaskDefinition<TaskSetHeadingConfig, SimTasksEnv, TaskSetHeading>
    {

        public:
            TaskFactorySetHeading(TaskEnvironmentPtr env) : 
                Parent("SetHeading","Reach a desired heading angle",true,env) {}
            virtual ~TaskFactorySetHeading() {};
    };
};

#endif // TASK_SET_HEADING_H
