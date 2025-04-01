package org.firstinspires.ftc.teamcode.controllers;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.DriveToPointController;

public class SquidToPointTask extends  RobotTaskImpl {

    public interface SquidToPointListener {
        void onUpdate(double x, double y, double heading);

        Pose getCurrentValues();
    }
    Pose _targetPose;

    SquidToPointListener _listner;
    boolean _started = false;
    boolean _isComplete = false;

    Pose _tolerance = new Pose(1, 1, 3);

    DriveToPointController driveController = new DriveToPointController();

    public SquidToPointTask(Pose targetPose, SquidToPointListener listener) {
        _targetPose = targetPose;
        _listner = listener;
    }


    @Override
    public void execute(Telemetry telemetry) {
        if(!_started) {

        }

        Pose currentValues = _listner.getCurrentValues();

        if(Math.abs(_targetPose.getX() - currentValues.getX()) < _tolerance.getX() &&
                Math.abs(_targetPose.getY() - currentValues.getY()) < _tolerance.getY() &&
                Math.abs(_targetPose.getHeading() - currentValues.getHeading()) < _tolerance.getHeading()
        ) {
            _isComplete = true;
        } else {
            Vector3D powers = driveController.calculate(_targetPose.getX(), _targetPose.getY(), _targetPose.getHeading(), currentValues, telemetry);
            _listner.onUpdate(powers.getX(), powers.getY(), powers.getZ());
        }


        _started = true;
    }


    @Override
    public boolean isBlocking() {
        return false;
    }

    @Override
    public boolean hasStarted() {
        return _started;
    }

    @Override
    public boolean isRunning() {
        return hasStarted() && !isComplete();
    }

    @Override
    public boolean isComplete() {

        return _isComplete;
    }
}
