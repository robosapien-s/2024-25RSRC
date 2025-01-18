package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class GoToAprilTag implements IRobot {

    private final JoystickWrapper joystick;

    public GoToAprilTag(JoystickWrapper joystick) {
        this.joystick = joystick;
    }

    @Override
    public void initialize(Robot robot, IRobot previousState) {
        // Initialize actions to move toward AprilTag
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {
        if (joystick.gamepad1GetA()) {
            robot.setAutoTarget(50,50,0);
        } else if (joystick.gamepad1GetB()) {
            robot.disableAutoMode();
        }
        telemetry.addData("State", "Moving to AprilTag");
        telemetry.update();
        // Add navigation logic toward the detected AprilTag
    }

    @Override
    public State getState() {
        return State.GO_TO_APRIL_TAG;
    }
}
