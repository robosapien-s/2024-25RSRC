package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExecuteOnceTask extends RobotTaskTimed {

    public interface ExecuteListener {
        void execute();
    }

    private final ExecuteListener _listener;

    String _name;
    boolean _executed = false;
    private ExecuteOnceTask(){

        super(0);
        _listener = null;
    }
    public ExecuteOnceTask(ExecuteListener inListener, String name) {
        super(0);
        _listener = inListener;
        _name = name;
    }

    @Override
    public void execute(Telemetry telemetry) {

        super.execute(telemetry);

        if(!_executed) {
            if (_listener != null) {
                _listener.execute();
            }
        }
        _executed = true;
    }

    @Override
    public boolean isComplete() {
       return _executed;
    }
}
