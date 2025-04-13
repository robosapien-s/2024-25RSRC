package org.firstinspires.ftc.teamcode.states;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;


@Config
public class HangState extends BaseState {

    public static int HANG_SLIDE_ANGLE_ROTATION_PREP = 723;
    public static int HANG_SLIDE_EXTENSION_PREP = 1200;

    public static int HANG_SLIDE_EXTENSION_STAGE1 = 748;


    public static int HANG_SLIDE_ANGLE_ROTATION_L2 = 0;
    public static int HANG_SLIDE_EXTENSTION_L2 = 385;

    //1770 on rotation
    //1108 on slide
    //1710 on rotation
    //817 on slide

    double targetPos;


    public HangState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void start(Robot robot, Telemetry telemetry) {
        //robot.setIntakeAnglePosition(RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY);
    }


    @Override
    public void initialize(Robot robot, IRobot prevState) {
       robot.setDriveTrainEnabled(false);
        robot.setSlideTargetPosition(HANG_SLIDE_EXTENSION_PREP);
        robot.setSlideRotationPosition(HANG_SLIDE_ANGLE_ROTATION_PREP);
        robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_SPECIMEN);
        //robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
        robot.setIntakeAnglePosition(RoboSapiensTeleOp.Params.INTAKE_ANGLE_SPECIMEN);
        robot.setHangServo(RoboSapiensTeleOp.Params.HANG_SERVO_OPEN);

        robot.setSlideMinPosition(0);
        robot.setSlideMaxPosition(1500);

        robot.setSlideRotationMaxPosition(2000);

    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if(joystick.gamepad2GetY()) {
            robot.switchState(State.INTAKINGCLAW);
        } else if(joystick.gamepad2GetA()) {
            taskArrayList.add(createSlideTask(robot, HANG_SLIDE_EXTENSION_STAGE1, 200, "Slide", false));


            taskArrayList.add(createSlideRotationTask(robot, HANG_SLIDE_ANGLE_ROTATION_L2, 500, "Rotation", false));
            taskArrayList.add(createSlideTask(robot, HANG_SLIDE_EXTENSTION_L2, 1000, "Slide", false));

            taskArrayList.add(new ExecuteOnceTask(
                   new ExecuteOnceTask.ExecuteListener() {
                       @Override
                       public void execute() {
                           robot.setHangServo(RoboSapiensTeleOp.Params.HANG_SERVO_CLOSED);
                       }
                   }, "Engage Hooks"
           ));

           taskArrayList.add(createWaitTask(robot, 400, "Wait for hooks"));


           taskArrayList.add(new ExecuteOnceTask(
                   new ExecuteOnceTask.ExecuteListener() {
                       @Override
                       public void execute() {
                           robot.disableHangServo();
                       }
                   }, "Disengage Hooks"
           ));


//           taskArrayList.add(createSlideTask(robot, HANG_SLIDE_EXTENSTION_L2+50, 30, "Slide", false));


        }

        if(joystick.gamepad2GetRightBumperRaw()) {
            robot.increaseSlideTargetPosition((int) (joystick.gamepad2GetRightTrigger()*-100));
        } else {
            robot.increaseSlideTargetPosition((int) (joystick.gamepad2GetRightTrigger()*100));
        }

        if(joystick.gamepad2GetLeftBumperRaw()) {
            robot.increaseSlideRotationTargetPosition((int) (joystick.gamepad2GetLeftTrigger()*-20));
        } else {
            robot.increaseSlideRotationTargetPosition((int) (joystick.gamepad2GetLeftTrigger()*20));
        }

        if (joystick.gamepad2GetDUp()) {
            targetPos = RoboSapiensTeleOp.Params.HANG_SERVO_OPEN;
            robot.setHangServo(RoboSapiensTeleOp.Params.HANG_SERVO_OPEN);
        }

        if (joystick.gamepad2GetDDown()) {
            targetPos = RoboSapiensTeleOp.Params.HANG_SERVO_CLOSED;
            robot.setHangServo(RoboSapiensTeleOp.Params.HANG_SERVO_CLOSED);
        }

        if (joystick.gamepad2GetDLeft()) {
            targetPos-=0.05;
            robot.setHangServo(targetPos);
        }

        if (joystick.gamepad2GetDRight()) {
            targetPos+=0.05;
            robot.setHangServo(targetPos);
        }

        if (joystick.gamepad2GetLeftBumperDown()) {
            robot.disableHangServo();
        }

        if (joystick.gamepad2GetRightBumperDown()) {
            robot.enableHangServo();
        }

        robot.updateDriveTrainsRaw(telemetry,false, joystick.gamepad1GetLeftStickX(), joystick.gamepad1GetLeftStickY(), joystick.gamepad1GetRightStickX(), joystick.gamepad1GetRightStickY(), 1, 1);

        executeTasks(telemetry);

    }

    @Override
    public void dispose(Robot robot) {
        robot.setDriveTrainEnabled(true);
        robot.setSlideMinPosition(RoboSapiensTeleOp.Params.SLIDE_MIN_POSITION);
    }

    @Override
    public State getState() {
        return State.ROBOT_HANG;
    }
}
