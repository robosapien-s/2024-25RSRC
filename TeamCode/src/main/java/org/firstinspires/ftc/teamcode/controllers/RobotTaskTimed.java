package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

abstract public class RobotTaskTimed extends  RobotTaskImpl{

    private RobotTaskTimed(){}

    RobotTaskTimed(long inDuration) {
        _duration = inDuration;
    }

    long _startTime = 0;
    long _duration = 0;
    @Override
    public void execute(Telemetry telemetry) {

        if(_startTime == 0) {
            _startTime = System.currentTimeMillis();//Start code here
        }
    }

    @Override
    public boolean isBlocking() {
        return blockingType == ETaskBlockingType.series;
    }

    @Override
    public boolean isRunning() {
        if(isComplete()) {
            return false;
        } else {
            return _startTime > 0;
        }
    }

    @Override
    public boolean hasStarted() {
        return _startTime > 0;
    }

    @Override
    public boolean isComplete() {
        if(_startTime == 0) {
            return false;
        } else {
            long currentTime = System.currentTimeMillis();
            long diff = currentTime - _startTime;
            return diff >= _duration;
        }
    }
}
