package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class RobotTaskUntilTouchSensor extends RobotTaskListImpl {

    TouchSensor _touchSensor;

    boolean _isCompleteOverride = false;


    public RobotTaskUntilTouchSensor(TouchSensor sensor,  String name) {
        super();
        _touchSensor = sensor;
        _isCompleteOverride = false;
    }

    @Override
    public void execute(Telemetry telemetry) {

        if(_touchSensor.isPressed()) {
            _isCompleteOverride = true;
        }

        if(!_isCompleteOverride) {
            for (IRobotTask task : tasks) {
                if (!task.hasStarted()) {
                    task.execute(telemetry);
                }
                task.execute(telemetry);
            };
        }
    }

    @Override
    public boolean isBlocking() {
        return false;
    }

    @Override
    public boolean isComplete() {

        //If it has already been complete, its complete.  Otherwise wait until touch sensor
        return super.isComplete() || _isCompleteOverride;
    }

//    @Override
//    public void stopTask() {
//
//    }
}
