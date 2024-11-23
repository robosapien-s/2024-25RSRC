package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class RobotTaskSeries extends RobotTaskImpl{

    public RobotTaskSeries(){}
    ArrayList<IRobotTask> tasks = new ArrayList<IRobotTask>();

    public void add(IRobotTask task) {
        tasks.add(task);
    }

    @Override
    public void execute(Telemetry telemetry) {


        if(!tasks.isEmpty()) {

            boolean isStarted = tasks.get(0).hasStarted();
            boolean isRunning = tasks.get(0).isRunning();
            boolean isComplete = tasks.get(0).isComplete();

            tasks.get(0).execute(telemetry);


            if(isComplete){
                tasks.remove(0);
            }
        }

//        for (IRobotTask task : tasks) {
//            if(!task.isComplete()) {
//                task.execute(telemetry);
//                break;
//            }
//        };
    }

    @Override
    public boolean isBlocking() {
        return true;
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
}
