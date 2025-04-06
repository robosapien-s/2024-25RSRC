package org.firstinspires.ftc.teamcode.states;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class SpecimenHangFrontState extends BaseState {

    public static Pose2d _lastPose = null;
    boolean didLowerHeight = false;
    //    int subState = 0;
    public SpecimenHangFrontState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {

        robot.setSlideMaxPosition(RoboSapiensTeleOp.Params.SLIDE_DROP_L2);
        robot.setSlideMinPosition(110);

        taskArrayList.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_SPECIMEN_FRONT, 0,"", false));
        taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_SPECIMEN_FRONT, 0,"", false));
        taskArrayList.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_SPECIMEN_FRONT_DROP_POSITION,200, "", false));
        taskArrayList.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_SPECIMEN_FRONT_DROP_POSITION, 0, "", false));
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
            robot.increaseSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-20));
        } else {
            robot.increaseSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*20));
        }

        if (joystick.gamepad1GetLeftBumperRaw()) {
            robot.increaseSlideRotationTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*-20));
        } else {
            robot.increaseSlideRotationTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*20));
        }

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.SPECIMEN_HANG_FRONT;
    }
}
