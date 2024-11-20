package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CallBackTask extends RobotTaskTimed {

    public interface CallBackListener {
        void setPosition(double value);
        double getPosition();
    }

    private final CallBackListener _listener;

    double _position;
    double _initialPosition;

    boolean _useSteps = false;

    String _name;
    private CallBackTask(){

        super(0);
        _listener = null;
    }
    public CallBackTask(CallBackListener inListener, double position, long inDuration, String name) {
        super(inDuration);
        _listener = inListener;
        _position = position;
        _duration = inDuration;
        _name = name;
        _useSteps = false;
    }

    public CallBackTask(CallBackListener inListener, double position, long inDuration, String name, boolean inSteps) {
        super(inDuration);
        _listener = inListener;
        _position = position;
        _duration = inDuration;
        _name = name;
        _useSteps = inSteps;
    }

    @Override
    public void execute(Telemetry telemetry) {

        boolean hasStarted = hasStarted();
        super.execute(telemetry);

        if(!_useSteps) {
            if(!hasStarted) {

                if(_listener != null) {
                    _listener.setPosition(_position);
                    telemetry.addData("Servo: " + _name, "Executed");
                }
            }
        } else {

            if(!hasStarted) {

                if(_listener != null) {
                    _initialPosition = _listener.getPosition();
                    telemetry.addData("Servo: " + _name, "Executed");
                }
            }

            double percentComplete = (double) (System.currentTimeMillis() - _startTime) / (double)_duration;
            if(percentComplete>1) {
                percentComplete = 1;
            }
            double diffPos = _position - _initialPosition;

            double newPos = _initialPosition + (diffPos*percentComplete);

           if(_listener != null) {
               _listener.setPosition(newPos);
               telemetry.addData("Servo Step: " + percentComplete + " - " + _name, "Executed: " + String.valueOf(newPos));
           }

        }
    }

    @Override
    public void stopTask() {
        super.stopTask();
    }
}
