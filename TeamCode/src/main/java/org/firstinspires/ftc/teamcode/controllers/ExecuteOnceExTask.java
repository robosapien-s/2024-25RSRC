package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExecuteOnceExTask extends RobotTaskTimed {

    public interface ExecuteExListener {
        boolean execute();
    }

    private final ExecuteExListener _listener;

    String _name;
    boolean _executed = false;
    private ExecuteOnceExTask(){

        super(0);
        _listener = null;
    }
    public ExecuteOnceExTask(ExecuteExListener inListener, String name) {
        super(0);
        _listener = inListener;
        _name = name;
    }

    @Override
    public void execute(Telemetry telemetry) {

        super.execute(telemetry);

        if(!_executed) {
            if (_listener != null) {
                _executed = _listener.execute();
            }
        } else {
            _executed = true;
        }
    }

    @Override
    public boolean isComplete() {
       return _executed;
    }
}
