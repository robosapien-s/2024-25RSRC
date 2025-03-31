package org.firstinspires.ftc.teamcode.controllers;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SquidToPointTask extends  RobotTaskImpl {

    Follower _follower;
    Pose _pose;

    boolean _started = false;

    Pose _tolerance = new Pose(1, 1, 3);

    public SquidToPointTask(Follower follower, Pose pose) {
        _follower = follower;
        _pose = pose;
    }


    @Override
    public void execute(Telemetry telemetry) {
        if(!_started) {

        }



        _started = true;
    }


    @Override
    public boolean isBlocking() {
        return false;
    }

    @Override
    public boolean hasStarted() {
        return false;
    }

    @Override
    public boolean isRunning() {
        return false;
    }

    @Override
    public boolean isComplete() {

        return false;
    }
}
