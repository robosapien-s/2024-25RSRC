package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ConditionalTask extends RobotTaskImpl{

    boolean _started = false;

    boolean _isComplete = false;

    public interface ConditionalTaskListener {
        boolean conditionMet();

        void executeTask();
    }

    ConditionalTaskListener _listener;

    public ConditionalTask(ConditionalTaskListener listener) {
        _listener = listener;
    }

    @Override
    public void execute(Telemetry telemetry) {
        if (!_started) {
            _started = true;
        } else if (_listener.conditionMet()) {
            _listener.executeTask();
            _isComplete = true;
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
