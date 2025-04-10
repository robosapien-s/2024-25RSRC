package org.firstinspires.ftc.teamcode.controllers;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;

public class AutoCycleTask extends RobotTaskImpl{
    private boolean _isComplete = false;
    private boolean _started = false;

    private AutoCycleListener _listener;

    private PathChain _path = null;


    public interface AutoCycleListener {
        Follower follower();

        boolean breakFollowing();
    }

    public AutoCycleTask(AutoCycleListener listener, PathChain path) {
        this._listener = listener;
        this._path = path;
    }


    @Override
    public void execute(Telemetry telemetry) {
        if (!_started) {
            _started = true;
            _listener.follower().followPath(_path);
        } else {
            _listener.follower().update();
            if (_listener.breakFollowing()) {
                _listener.follower().breakFollowing();
                _isComplete = true;
            } else if (!_listener.follower().isBusy()) {
                _isComplete = true;
            }
        }
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
        return _started && !_isComplete;
    }

    @Override
    public boolean isComplete() {
        return _isComplete;
    }
}
