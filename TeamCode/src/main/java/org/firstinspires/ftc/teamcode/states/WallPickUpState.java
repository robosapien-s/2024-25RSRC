package org.firstinspires.ftc.teamcode.states;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.localization.Pose;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.DriveToPointTask;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
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

        IRobotTask trajectoryTask = runTrajectory(robot);

        RobotTaskSeries trajectorySeries = new RobotTaskSeries();
        RobotTaskParallel parallelTask = new RobotTaskParallel();
        if (trajectoryTask != null) {
            trajectorySeries.add(createWaitTask(robot, 200, "wait"));
            trajectorySeries.add(trajectoryTask);

            parallelTask.add(trajectorySeries);
        }

        RobotTaskSeries stateTransition  = new RobotTaskSeries();

        if(prevState.getState() == State.INTAKINGCLAW) {
            stateTransition.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_WALL_POSITION, 0, "Slide", false));
            stateTransition.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_WALL_PICKUP, 0, "Intake Angle", false));
            stateTransition.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_WALL_PICKUP, 0, "Rot and Angle", false));
            stateTransition.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_WALL_POSITION, 0, "Rotation", false));

        } else if (prevState.getState() == State.SPECIMEN_HANG || prevState.getState() == State.AUTO_SPECIMEN_HANG) {
            stateTransition.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_SPECIMEN_DOWN, 0, "Rot and Angle", false));
            stateTransition.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_SPECIMEN_DOWN_POSITION, 150, "Slide", false));
            stateTransition.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 50, "Claw Open", false));
            stateTransition.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_WALL_POSITION, 0, "Slide", false));
//            stateTransition.add(createSlideRotationTask(robot, 0, 0, "Rotation", false));
            stateTransition.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_WALL_POSITION, 0, "Rotation", false));
            stateTransition.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_WALL_PICKUP, 0, "Intake Angle", false));
            stateTransition.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_WALL_PICKUP, 0, "Rot and Angle", false));
//            stateTransition.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_WALL_POSITION, 0, "Rotation", false));

        } else {
            stateTransition.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_WALL_POSITION, 0, "Slide", false));
            stateTransition.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_WALL_PICKUP, 0, "Intake Angle", false));
            stateTransition.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_WALL_PICKUP, 0, "Rot and Angle", false));
            stateTransition.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_WALL_POSITION, 0, "Rotation", false));
        }

        parallelTask.add(stateTransition);
        taskArrayList.add(parallelTask);
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

        if (joystick.gamepad1GetX()) {
            robot.getFollower().setPose(Robot.origin);
            robot.switchState(State.AUTO_SPECIMEN_HANG);
        }

        if (joystick.gamepad1GetDDown()) {
            robot.setTargetHeading(180);
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

    public IRobotTask runTrajectory(Robot robot) {return null;}

    @Override
    public State getState() {
        return State.WALLPICKUP;
    }
}
