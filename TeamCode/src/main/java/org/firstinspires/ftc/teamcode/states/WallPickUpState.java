package org.firstinspires.ftc.teamcode.states;

import com.acmerobotics.roadrunner.Pose2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.DriveToPointTask;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskParallel;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class WallPickUpState extends BaseState {


//    public static Pose2d _lastPose = null;

//    boolean angle_ready = false;
    boolean isDriftModeEnabled = false;
    public WallPickUpState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {



        //taskArrayList.add(createClawHorizontalAngleTask(robot, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER,1,"ClawHorizontalAngle",false));

        if(prevState.getState() == State.INTAKINGCLAW) {
            taskArrayList.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_WALL_POSITION, 0, "Slide", false));
            taskArrayList.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_WALL_PICKUP, 0, "Intake Angle", false));
            taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_WALL_PICKUP, 0, "Rot and Angle", false));
            taskArrayList.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_WALL_POSITION, 0, "Rotation", false));

        } else if (prevState.getState() == State.SPECIMEN_HANG) {
            taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_SPECIMEN_DOWN, 0, "Rot and Angle", false));
            taskArrayList.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_SPECIMEN_DOWN_POSITION, 100, "Slide", false));
            taskArrayList.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 100, "Claw Open", false));
            taskArrayList.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_WALL_POSITION, 0, "Slide", false));
            taskArrayList.add(createSlideRotationTask(robot, 0, 0, "Rotation", false));
            taskArrayList.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_WALL_PICKUP, 0, "Intake Angle", false));
            taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_WALL_PICKUP, 500, "Rot and Angle", false));
            taskArrayList.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_WALL_POSITION, 0, "Rotation", false));

        } else {
            taskArrayList.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_WALL_POSITION, 0, "Slide", false));
            taskArrayList.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_WALL_PICKUP, 0, "Intake Angle", false));
            taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_WALL_PICKUP, 0, "Rot and Angle", false));
            taskArrayList.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_WALL_POSITION, 0, "Rotation", false));
        }


    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if (joystick.gamepad2GetA()) {
            robot.switchState(State.SERVO_TEST);
        }

        if (joystick.gamepad1GetA()) {
            robot.switchState(State.INTAKINGCLAW);
        }

        if (joystick.gamepad1GetB()) {
            robot.switchState(State.SPECIMEN_HANG);
        }



        if(joystick.gamepad1GetRightBumperRaw()) {
            robot.increaseSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-20));
        } else {
            robot.increaseSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*20));
        }

        if(joystick.gamepad1GetLeftBumperRaw()) {
            robot.increaseSlideRotationTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*-20));
        } else {
            robot.increaseSlideRotationTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*20));
        }




//        if (joystick.gamepad1GetDLeft()) {
//            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT);
//        } else if (joystick.gamepad1GetDRight()) {
//            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT);
//        } else if (joystick.gamepad1GetDDown()) {
//            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER);
//        }


//        telemetry.addData("angle_ready", angle_ready);

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.WALLPICKUP;
    }
}
