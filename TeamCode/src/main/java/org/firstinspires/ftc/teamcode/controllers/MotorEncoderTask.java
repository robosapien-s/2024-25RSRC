package org.firstinspires.ftc.teamcode.controllers;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorEncoderTask extends RobotTaskTimed {

    DcMotorEx _motor;
    double _position;
    double _initialPosition;
    boolean _useSteps = false;

    String _name;
    private MotorEncoderTask(){
        super(0);
    }
    public MotorEncoderTask(DcMotorEx motor, double position, long inDuration, String name) {
        super(inDuration);
        _motor = motor;
        _position = position;
        _duration = inDuration;
        _name = name;
        _useSteps = false;
    }

    public MotorEncoderTask(DcMotorEx motor, double position, long inDuration, String name, boolean inSteps) {
        super(inDuration);
        _motor = motor;
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

                //_motor.setPosition(_position);
                _motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                _motor.setPower(.5);
                _motor.setTargetPosition((int) _position);
                _motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                telemetry.addData("DCMotorEx: " + _name, "Executed");

            }
        } else {

            if(!hasStarted) {
                _initialPosition = _motor.getCurrentPosition();
                telemetry.addData("Servo: " + _name, "Executed");
            }

            double percentComplete = (double) (System.currentTimeMillis() - _startTime) / (double)_duration;
            if(percentComplete>1) {
                percentComplete = 1;
            }
            double diffPos = _position - _initialPosition;

            double newPos = _initialPosition + (diffPos*percentComplete);

            _motor.setTargetPosition((int) _position);
            _motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            telemetry.addData("Servo Step: " + percentComplete + " - " +  _name, "Executed: " + String.valueOf(newPos));

        }
    }

    @Override
    public void stopTask() {
        super.stopTask();
    }
}
