package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.opencv.core.Mat;

public class DriveToPointTask implements IRobotTask {

    private long _startTime = 0;
    private boolean _isComplete = false;
    private final Robot _robot;
    private final Vector3D _targets;
    private final long _maxTime;

    private final long _tolerance;

    private final long _delay;

    public DriveToPointTask(Robot robot, Vector3D targets, long maxTime, long tolerance, long delay) {
        _robot = robot;
        _targets = targets;
        _maxTime = maxTime;
        _tolerance = tolerance;
        _delay = delay;
    }


    @Override
    public void execute(Telemetry telemetry) {

        if(_startTime == 0) {
            _startTime = System.currentTimeMillis();//Start code here
        }

        long delta = System.currentTimeMillis() - _startTime;

        if(delta>_delay){
            _robot.setAutoTarget(_targets.getX(), _targets.getY(), _targets.getZ());
        }

        Pose2d pose2d = _robot.getPose();

        double xDiff = Math.abs(pose2d.position.x - _targets.getX());
        double yDiff = Math.abs(pose2d.position.y - _targets.getY());
        double aDiff = Math.abs( Math.toDegrees(   pose2d.heading.toDouble()   ) - Math.toDegrees( _targets.getZ() ));

        //If robot was driving manually or is auto mode was turned off
        if((xDiff < _tolerance && yDiff < _tolerance && aDiff < _tolerance) || _robot.isAutoMode()) {
            _isComplete = true;
        }

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
        return _isComplete;
    }

    @Override
    public boolean isBlocking() {
        return false;
    }

    @Override
    public void stopTask() {
        _isComplete = true;
    }
}
