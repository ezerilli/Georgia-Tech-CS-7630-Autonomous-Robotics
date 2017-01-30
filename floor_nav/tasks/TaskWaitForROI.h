#ifndef TASK_WAIT_FOR_ROI_H
#define TASK_WAIT_FOR_ROI_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskWaitForROIConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskWaitForROI : public TaskInstance<TaskWaitForROIConfig,SimTasksEnv>
    {
        public:
            TaskWaitForROI(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskWaitForROI() {};

            virtual TaskIndicator iterate();

    };
    class TaskFactoryWaitForROI : public TaskDefinition<TaskWaitForROIConfig, SimTasksEnv, TaskWaitForROI>
    {
        public:
            TaskFactoryWaitForROI(TaskEnvironmentPtr env) : 
                Parent("WaitForROI","Do nothing until we reach a given destination",true,env) {}
            virtual ~TaskFactoryWaitForROI() {};
    };
};

#endif // TASK_WAIT_FOR_ROI_H
