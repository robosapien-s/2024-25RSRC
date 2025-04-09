package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.robot.AngleDrive.lineToLinearHeading;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.DriveToPointController;

public class PedroPathingTask extends  RobotTaskImpl {

    public interface PedroPathingListener {
        Follower follower();

        void onUpdate();

//        Pose getCurrentValues();

//        void followPath(PathChain pathChain);
    }

    Pose _targetPose;
    Pose _startPose;

    PedroPathingListener _listener;
    boolean _started = false;
    boolean _isComplete = false;

    double ZPAM;

    PathChain pathChain = null;

    public static int _doContinueHack = 0;

    private boolean _didStartFollowing = false;

//    Pose _tolerance = new Pose(.3, .3, Math.toRadians(2));

//    DriveToPointController driveController = new DriveToPointController();

    public PedroPathingTask(Pose startPose, Pose targetPose, double ZPAM,  PedroPathingListener listener) {
        _startPose = startPose;
        _targetPose = targetPose;
        this.ZPAM = ZPAM;
        _listener = listener;
    }


    @Override
    public void execute(Telemetry telemetry) {
        if(!_started) {
            pathChain = lineToLinearHeading(_listener.follower(), _startPose, _targetPose, ZPAM);
            //_listener.follower().followPath(pathChain);
            _started=true;
        } else {

            if(_doContinueHack==1 && !_didStartFollowing) {
                _didStartFollowing = true;
                _listener.follower().followPath(pathChain);
            }

            if (!_listener.follower().isBusy()) {
                if(_doContinueHack==1) {
                    _isComplete = true;
                }
            }
        }

        _listener.onUpdate();

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
    public boolean isComplete() {return _isComplete;}

    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }
}
