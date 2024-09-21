package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.MotorController;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.MotorController;

public class InitialState implements IRobot {

    private final JoystickWrapper joystick;
    private final MotorController motorController;

    // Constructor to pass in the joystick wrapper and motor manager
    public InitialState(JoystickWrapper joystick, MotorController motorController) {
        this.joystick = joystick;
        this.motorController = motorController;
    }

    @Override
    public void execute() {
        // stop all motors
        motorController.reqFlChange(0);
        motorController.reqBlChange(0);
        motorController.reqRfChange(0);
        motorController.reqBrChange(0);
        motorController.reqEmChange(0);

        if (joystick.gamepad1GetA()) {
            Robot.getInstance().switchState(State.EXTENDING);
        } else if(joystick.gamepad1GetB()){
            Robot.getInstance().switchState(State.DRIVE);
        }
    }
}
