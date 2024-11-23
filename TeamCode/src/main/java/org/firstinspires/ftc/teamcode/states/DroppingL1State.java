package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.CallBackTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.opmodes.DriveTest;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class DroppingL1State extends BaseState {

    int substate = 0;

    public DroppingL1State(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {

        RobotTaskSeries transferSeries = new RobotTaskSeries();
        transferSeries.add(createClawTask(robot, DriveTest.Params.CLAW_CLOSE, 500, "ClawClose", false));
        transferSeries.add(createHorizontalSlideTask(robot, DriveTest.Params.HORIZONTAL_SLIDE_TRANSFER_POSITION, 500, "HorizontalSlide", false));
        transferSeries.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_DROP_L1, 1, "VerticalSlide", false));
        transferSeries.add(createClawSlideTask(robot, DriveTest.Params.CLAW_SLIDER_BACK, 1, "CLAW_SLIDER_BACK", false));
        transferSeries.add(createClawAngleTask(robot, DriveTest.Params.CLAW_ANGLE_BACK, 1, "CLAW_ANGLE_BACK", false));
        transferSeries.add(createClawRotationTask(robot, DriveTest.Params.ROT_SERVO_DEFAULT, 1, "ROT_SERVO_BACK", false));

        taskArrayList.add(transferSeries);

    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if(joystick.gamepad1GetB()) {

            if(substate == 0) {
                substate++;
                robot.setClawPosition(DriveTest.Params.CLAW_OPEN);
            } else {
                robot.switchState(State.INTAKING);
            }
        } else if(joystick.gamepad1GetA()) {
            robot.switchState(State.WALLPICKUP);
        }

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.DROPPING_L1;
    }

}
