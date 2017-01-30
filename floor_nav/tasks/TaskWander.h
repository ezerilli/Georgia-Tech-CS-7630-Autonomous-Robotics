#ifndef TASK_WANDER_H
#define TASK_WANDER_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskWanderConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskWander : public TaskInstance<TaskWanderConfig,SimTasksEnv>
    {
        public:
            TaskWander(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskWander() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryWander : public TaskDefinition<TaskWanderConfig, SimTasksEnv, TaskWander>
    {

        public:
            TaskFactoryWander(TaskEnvironmentPtr env) : 
                Parent("Wander","Wander aimlessly forever",true,env) {}
            virtual ~TaskFactoryWander() {};
    };
};

#endif // TASK_WANDER_H
