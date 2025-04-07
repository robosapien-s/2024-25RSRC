package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class SpecimenHangState extends BaseState {

//    public static Pose2d _lastPose = null;
//    boolean didLowerHeight = false;
//    int subState = 0;
    public SpecimenHangState(JoystickWrapper joystick) {
        super(joystick);
    }


    @Override
    public void initialize(Robot robot, IRobot prevState) {

        robot.setSlideMinPosition(0);
        robot.setSlideMaxPosition(1000);

        if (prevState.getState() == State.INTAKINGCLAW) {
            taskArrayList.add(createSlideTask(robot, 0, 0, "Slide", false));
            taskArrayList.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_SPECIMEN, 0, "Intake Angle", false));
            taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_SPECIMEN, 0, "Rot and Angle", false));
            taskArrayList.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_SPECIMEN_POSITION, 300, "Rotation", false));
            taskArrayList.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_SPECIMEN_DROP_POSITION, 0, "Slide", false));



        } else {
            taskArrayList.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_CLOSE, 100, "Claw Close", false));
            taskArrayList.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_SPECIMEN_POSITION, 100, "Rotation", false));
            taskArrayList.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_SPECIMEN, 0, "Intake Angle", false));
            taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_SPECIMEN, 175, "Rot and Angle", false));
//            taskArrayList.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_SPECIMEN_POSITION, 300, "Rotation", false));
            taskArrayList.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_SPECIMEN_DROP_POSITION, 0, "Slide", false));
        }


    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {
        if (joystick.gamepad2GetA()) {
            robot.switchState(State.SERVO_TEST);
        }

        if(joystick.gamepad1GetA()) {
            robot.switchState(State.INTAKINGCLAW);
        } else if(joystick.gamepad1GetB()) {
            robot.switchState(State.WALLPICKUP);
        } else if(joystick.gamepad1GetY()) {
            robot.switchState(State.DROPPING_L1);
        }





        if(joystick.gamepad1GetRightBumperRaw()) {
            robot.increaseSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-100));
        } else {
            robot.increaseSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*100));
        }

        if(joystick.gamepad1GetLeftBumperRaw()) {
            robot.increaseSlideRotationTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*-20));
        } else {
            robot.increaseSlideRotationTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*20));
        }

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.SPECIMEN_HANG;
    }
}
