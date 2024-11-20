package org.firstinspires.ftc.teamcode.controllers;

import java.util.ArrayList;

abstract public class RobotTaskListImpl extends RobotTaskImpl {

    ArrayList<IRobotTask> tasks = new ArrayList<IRobotTask>();
    public void add(IRobotTask task) {
        tasks.add(task);
    }

    @Override
    public boolean hasStarted() {
        if(tasks.size()>0) {
            return tasks.get(0).hasStarted();
        } else {
            return true;
        }
    }

    @Override
    public boolean isRunning() {

        if(!hasStarted()) {
            return false;
        } else {
            return !isComplete();
        }
    }

    @Override
    public boolean isComplete() {
        boolean isComplete = true;
        for (IRobotTask task : tasks) {
            if(!task.isComplete()) {
                isComplete = false;
                break;
            }
        };
        return isComplete;
    }

    @Override
    public void stopTask() {
        for (IRobotTask task : tasks) {
            task.stopTask();
        };
    }
}
