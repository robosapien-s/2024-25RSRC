package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.CallBackTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskParallel;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.DriveTest;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class SpecimenHangState extends BaseState {

    boolean didLowerHeight = false;
    int subState = 0;
    public SpecimenHangState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {


        taskArrayList.add(createClawTask(robot, DriveTest.Params.CLAW_CLOSE, 500, "Claw", false));
        taskArrayList.add(createVerticalSlideTask(robot, robot.getVerticalSlidePosition() + 80, 500, "Vertical", false));
        taskArrayList.add(createClawSlideTask( robot, DriveTest.Params.CLAW_SLIDER_FORWARD, 500, "ClawSlide", false));

        RobotTaskParallel transferParallel = new RobotTaskParallel();

        transferParallel.add(createClawAngleTask( robot, DriveTest.Params.CLAW_ANGLE_FORWARD, 1000, "ClawAngle", true));
        transferParallel.add(createClawRotationTask( robot, DriveTest.Params.ROT_SERVO_DEFAULT, 1000, "ClawRotation", true));
        transferParallel.add(createHorizontalSlideTask(robot, 0, 1000, "Horizontal", true));
        transferParallel.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_HANG_PREP_POSITION, 1000, "Vertical", false));

        taskArrayList.add(transferParallel);
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if(joystick.gamepad1GetB()) {
            //robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
            robot.switchState(State.INTAKING);
        } else if(joystick.gamepad1GetA()) {
            if(subState == 0) {
                subState++;
                //robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
                robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
            }
            else if(subState == 1) {
                subState++;
                robot.setClawPosition(DriveTest.Params.CLAW_OPEN);
            } else {
                robot.switchState(State.WALLPICKUP);
            }
        } else if(joystick.gamepad1GetY()) {
            robot.switchState(State.DROPPING_L1);
        } else if(joystick.gamepad1GetDUp()) {
            robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_PREP_POSITION);
        } else if(joystick.gamepad1GetDDown()) {
            robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
        }

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.SPECIMEN_HANG;
    }
}
