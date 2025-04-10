package org.firstinspires.ftc.teamcode.states;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;


@Config
public class HangState extends BaseState {

    public static int HANG_SLIDE_ANGLE_ROTATION_PREP = 500;
    public static int HANG_SLIDE_EXTENSTION_PREP = 500;



    public static int HANG_SLIDE_ANGLE_ROTATION_L2 = 0;
    public static int HANG_SLIDE_EXTENSTION_L2 = 0;


    public HangState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void start(Robot robot, Telemetry telemetry) {
        //robot.setIntakeAnglePosition(RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY);
    }


    @Override
    public void initialize(Robot robot, IRobot prevState) {
        robot.setSlideTargetPosition(HANG_SLIDE_EXTENSTION_PREP);
        robot.setSlideRotationPosition(HANG_SLIDE_ANGLE_ROTATION_PREP);
        robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_SPECIMEN);
        //robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
        robot.setIntakeAnglePosition(RoboSapiensTeleOp.Params.INTAKE_ANGLE_SPECIMEN);

        robot.setSlideMinPosition(0);


    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

       if(joystick.gamepad1GetA()) {

           taskArrayList.add(createSlideTask(robot, HANG_SLIDE_EXTENSTION_L2, 0, "Slide", false));
           taskArrayList.add(createSlideRotationTask(robot, HANG_SLIDE_ANGLE_ROTATION_L2, 1000, "Rotation", false));

           taskArrayList.add(new ExecuteOnceTask(
                   new ExecuteOnceTask.ExecuteListener() {
                       @Override
                       public void execute() {
                           robot.setLeftHangServo(.3);
                           robot.setRightHangServo(.3);
                       }
                   }, "Engage Hooks"
           ));

           taskArrayList.add(createWaitTask(robot, 200, "Wait for hooks"));


           taskArrayList.add(new ExecuteOnceTask(
                   new ExecuteOnceTask.ExecuteListener() {
                       @Override
                       public void execute() {
                           robot.setLeftHangServo(0);
                           robot.setRightHangServo(0);
                       }
                   }, "Disengage Hooks"
           ));


           taskArrayList.add(createSlideTask(robot, HANG_SLIDE_EXTENSTION_L2-50, 30, "Slide", false));


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
    public void dispose(Robot robot) {
        robot.setSlideMinPosition(RoboSapiensTeleOp.Params.SLIDE_MIN_POSITION);
    }

    @Override
    public State getState() {
        return State.ROBOT_HANG;
    }
}
