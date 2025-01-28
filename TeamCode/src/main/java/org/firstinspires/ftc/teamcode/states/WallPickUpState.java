package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class WallPickUpState extends BaseState {

    public WallPickUpState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {

        RobotTaskSeries transferParallel = new RobotTaskSeries();
        if(prevState.getState() == State.INTAKINGCLAW) {
            transferParallel.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 0, "Claw", false));
            transferParallel.add(createHorizontalSlideTask(robot, 0, 300, "Horizontal", false));
//            transferParallel.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_DOWN_POSITION, 300, "Vertical", false));
            transferParallel.add(createClawAngleTask( robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_PREP_BACK, 200, "ClawAngle", false));
            transferParallel.add(createClawRotationTask(robot, RoboSapiensTeleOp.Params.ROT_SERVO_BACK,0,"ClawRotation",false));
            transferParallel.add(createClawSlideTask( robot, RoboSapiensTeleOp.Params.CLAW_SLIDER_BACK, 500, "ClawSlide", false));
            transferParallel.add(createClawAngleTask( robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_BACK, 600, "ClawAngle", false));
        transferParallel.add(createVerticalSlideTask(robot, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_DOWN_POSITION, 0, "Vertical", false));
        }
        else {
            transferParallel.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 0, "Claw", false));
            transferParallel.add(createHorizontalSlideTask(robot, 0, 300, "Horizontal", false));
            transferParallel.add(createVerticalSlideTask(robot, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_DOWN_POSITION, 300, "Vertical", false));
            transferParallel.add(createClawAngleTask( robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_PREP_BACK, 200, "ClawAngle", false));
            transferParallel.add(createClawRotationTask(robot, RoboSapiensTeleOp.Params.ROT_SERVO_BACK,0,"ClawRotation",false));
            transferParallel.add(createClawSlideTask( robot, RoboSapiensTeleOp.Params.CLAW_SLIDER_BACK, 1000, "ClawSlide", false));
            transferParallel.add(createClawAngleTask( robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_BACK, 0, "ClawAngle", false));
//        transferParallel.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_DOWN_POSITION, 0, "Vertical", false));
        }
        taskArrayList.add(transferParallel);
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if(joystick.gamepad1GetB()) {
            robot.switchState(State.SPECIMEN_HANG);
        } else if(joystick.gamepad1GetA())  {
            robot.switchState(State.INTAKINGCLAW);
        }

//        if(joystick.gamepad1GetLeftBumperRaw()) {
//            robot.increseClawSlideTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*-500));
//        } else {
//            robot.increseClawSlideTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*500));
//        }

        if(joystick.gamepad1GetRightBumperRaw()) {
            robot.increseVerticalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-100));
        } else {
            robot.increseVerticalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*100));
        }

        if (joystick.gamepad1GetDLeft()) {
            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT);
        } else if (joystick.gamepad1GetDRight()) {
            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT);
        } else if (joystick.gamepad1GetDDown()) {
            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER);
        }

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.WALLPICKUP;
    }
}
