package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskParallel;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
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

        IRobotTask trajectoryTask = runTrajectory(robot);

        RobotTaskParallel parallelTask = new RobotTaskParallel();

        RobotTaskSeries trajectorySeries = new RobotTaskSeries();

        if (trajectoryTask != null) {
            trajectorySeries.add(createWaitTask(robot, 50, "wait"));
            trajectorySeries.add(trajectoryTask);

            parallelTask.add(trajectorySeries);
        }

        RobotTaskSeries stateTransition  = new RobotTaskSeries();

        robot.setSlideMinPosition(0);
        robot.setSlideMaxPosition(1000);

        if (prevState.getState() == State.INTAKINGCLAW) {
            stateTransition.add(createSlideTask(robot, 0, 0, "Slide", false));
            stateTransition.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_SPECIMEN, 0, "Intake Angle", false));
            stateTransition.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_SPECIMEN, 0, "Rot and Angle", false));
            stateTransition.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_SPECIMEN_POSITION, 300, "Rotation", false));
            stateTransition.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_SPECIMEN_DROP_POSITION, 0, "Slide", false));



        } else {
            stateTransition.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_CLOSE, 100, "Claw Close", false));
            stateTransition.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_SPECIMEN_POSITION, 100, "Rotation", false));
            stateTransition.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_SPECIMEN, 0, "Intake Angle", false));
            stateTransition.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_SPECIMEN, 175, "Rot and Angle", false));
//            stateTransition.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_SPECIMEN_POSITION, 300, "Rotation", false));
            if (Robot.pathChains.size() == Robot.numCycles*2-1) {
                stateTransition.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_SPECIMEN_DROP_POSITION, 800, "Slide", false));
                stateTransition.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_SPECIMEN_DROP_POSITION-100, 0, "Slide", false));
            } else {
                stateTransition.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_SPECIMEN_DROP_POSITION, 0, "Slide", false));
            }
        }

        parallelTask.add(stateTransition);
        taskArrayList.add(parallelTask);


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

    public IRobotTask runTrajectory(Robot robot) {return null;};

    @Override
    public State getState() {
        return State.SPECIMEN_HANG;
    }
}
