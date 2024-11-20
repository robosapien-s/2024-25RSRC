package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoTask extends RobotTaskTimed {

    Servo _servo;
    double _position;
    double _initialPosition;

    boolean _useSteps = false;

    String _name;
    private ServoTask(){
        super(0);
    }
    public ServoTask(Servo servo, double position, long inDuration, String name) {
        super(inDuration);
        _servo = servo;
        _position = position;
        _duration = inDuration;
        _name = name;
        _useSteps = false;
    }

    public ServoTask(Servo servo, double position, long inDuration, String name, boolean inSteps) {
        super(inDuration);
        _servo = servo;
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
                _servo.setPosition(_position);
                telemetry.addData("Servo: " + _name, "Executed");
            }
        } else {

            if(!hasStarted) {
                _initialPosition = _servo.getPosition();
                telemetry.addData("Servo: " + _name, "Executed");
            }

            double percentComplete = (double) (System.currentTimeMillis() - _startTime) / (double)_duration;
            if(percentComplete>1) {
                percentComplete = 1;
            }
            double diffPos = _position - _initialPosition;

            double newPos = _initialPosition + (diffPos*percentComplete);
            _servo.setPosition(newPos);
            telemetry.addData("Servo Step: " + percentComplete + " - " +  _name, "Executed: " + String.valueOf(newPos));

        }
    }

    @Override
    public void stopTask() {
        super.stopTask();
    }
}
