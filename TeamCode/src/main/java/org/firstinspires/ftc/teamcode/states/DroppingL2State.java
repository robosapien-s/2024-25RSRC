package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.CallBackTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.DriveTest;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class DroppingL2State extends BaseState {

    public DroppingL2State(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {

        RobotTaskSeries transferSeries = new RobotTaskSeries();
        transferSeries.add(createClawTask(robot, DriveTest.Params.CLAW_CLOSE, 1000, "ClawClose", true));
        transferSeries.add(createHorizontalSlideTask(robot, DriveTest.Params.HORIZONTAL_SLIDE_TRANSFER_POSITION, 1000, "HorizontalSlide", true));
        transferSeries.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_DROP_L2, 1000, "HorizontalSlide", true));
        transferSeries.add(createClawSlideTask(robot, DriveTest.Params.CLAW_SLIDER_BACK, 1000, "CLAW_SLIDER_BACK", true));
        transferSeries.add(createClawAngleTask(robot, DriveTest.Params.CLAW_ANGLE_BACK, 1000, "CLAW_ANGLE_BACK", true));
        transferSeries.add(createClawRotationTask(robot, DriveTest.Params.ROT_SERVO_DEFAULT, 1000, "ROT_SERVO_BACK", true));

        taskArrayList.add(transferSeries);
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {
        if(joystick.gamepad1GetB()) {
            robot.setClawPosition(DriveTest.Params.CLAW_OPEN);
        } else if(joystick.gamepad1GetA()) {
            robot.switchState(State.INITIAL);
        }

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.DROPPING_L2;
    }

}
