#ifndef TASK_STARE_AT_FACE_H
#define TASK_STARE_AT_FACE_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskStareAtFaceConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskStareAtFace : public TaskInstance<TaskStareAtFaceConfig,SimTasksEnv>
    {
        protected: 
            double initial_heading;
        public:
            TaskStareAtFace(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskStareAtFace() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryStareAtFace : public TaskDefinition<TaskStareAtFaceConfig, SimTasksEnv, TaskStareAtFace>
    {

        public:
            TaskFactoryStareAtFace(TaskEnvironmentPtr env) : 
                Parent("StareAtFace","Reach a desired heading angle",true,env) {}
            virtual ~TaskFactoryStareAtFace() {};
    };
};

#endif // TASK_STARE_AT_FACE_H
