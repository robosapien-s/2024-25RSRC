package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class TaskExecuter {

    ArrayList<IRobotTask> _taskArrayList = new ArrayList<IRobotTask>();

    public void add(IRobotTask task) {
        _taskArrayList.add(task);
    }

    public void executeTasks(Telemetry telemetry) {

        if(!_taskArrayList.isEmpty()) {

            boolean isStarted = _taskArrayList.get(0).hasStarted();
            boolean isRunning = _taskArrayList.get(0).isRunning();
            boolean isComplete = _taskArrayList.get(0).isComplete();

            _taskArrayList.get(0).execute(telemetry);


            if(isComplete){
                _taskArrayList.get(0).dispose();
                _taskArrayList.remove(0);
            }
        }
    }

    public boolean isComplete() {
        return _taskArrayList.isEmpty();
    }

}
