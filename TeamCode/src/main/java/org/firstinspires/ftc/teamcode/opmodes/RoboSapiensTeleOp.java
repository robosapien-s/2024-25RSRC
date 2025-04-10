package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Config

@TeleOp(name="Robot Linear OpMode", group="test")
public class RoboSapiensTeleOp extends LinearOpMode {

    public static class Params {
        public static double CLAW_OPEN = .4439;//.9;
        public static double CLAW_CLOSE = .7;


        public static double INTAKE_ANGLE_OFFSET=0;
        public static double INTAKE_ANGLE_INIT = 0.24 + RoboSapiensTeleOp.Params.INTAKE_ANGLE_OFFSET;
//        public static double INTAKE_ANGLE_START = .38 +RoboSapiensTeleOp.Params.INTAKE_ANGLE_OFFSET;
        public static double INTAKE_ANGLE_BUCKET = 0.3144+RoboSapiensTeleOp.Params.INTAKE_ANGLE_OFFSET;
        public static double INTAKE_ANGLE_SPECIMEN_FRONT = 0.39 + Params.INTAKE_ANGLE_OFFSET;
        public static double INTAKE_ANGLE_READY =.4+RoboSapiensTeleOp.Params.INTAKE_ANGLE_OFFSET;
        public static double INTAKE_ANGLE_PICKUP = .55+RoboSapiensTeleOp.Params.INTAKE_ANGLE_OFFSET;
        public static double INTAKE_ANGLE_CAMERA = 0.45+RoboSapiensTeleOp.Params.INTAKE_ANGLE_OFFSET;
        public static double INTAKE_ANGLE_SPECIMEN = 0.4575 + Params.INTAKE_ANGLE_OFFSET;
        public static double INTAKE_ANGLE_WALL_PICKUP = .4575+Params.INTAKE_ANGLE_OFFSET;
//        public static double INTAKE_ANGLE_SWEEP = 0.565 + Params.INTAKE_ANGLE_OFFSET;


        public static double[] ROT_AND_ANGLE_SPECIMEN_FRONT = {0.70, .3};

        public static double[] ROT_AND_ANGLE_CAMERA = {0.03, 0.97};

        public static double[] ROT_AND_ANGLE_PREP = {0.0772, 0.915}; //{ 0.0978, .8894}; // R / L

//        public static double[] ROT_AND_ANGLE_PICKUP_VERTICAL_RIGHT = {0.02, 0.6333};

        public static double[] ROT_AND_ANGLE_PICKUP_VERTICAL = {.3072, 1};

        public static double[] ROT_AND_ANGLE_AUTO_PICKUP = {.2, 1};

        public static double[] ROT_AND_ANGLE_PICKUP_LEFT = {.1317 , 0.98};

        public static double[] ROT_AND_ANGLE_PICKUP_RIGHT = {0, .825};

        public static double[] ROT_AND_ANGLE_BASKET = {.925, 0.075};

        public static double[] ROT_AND_ANGLE_SPECIMEN = {.97, 0.03};

        public static double[] ROT_AND_ANGLE_SPECIMEN_DOWN = {1, 0};

        public static double[] ROT_AND_ANGLE_WALL_PICKUP = {0.1978, .2228};


        public static int SLIDE_SPECIMEN_FRONT_DROP_POSITION = 590;

        public static int SLIDE_SPECIMEN_DROP_POSITION = 670;

        public static int SLIDE_SPECIMEN_DOWN_POSITION = 220;

        public static int SLIDE_WALL_POSITION = 110;

        public static int SLIDE_DROP_L1 = 1000;

        public static int SLIDE_DROP_L2 = 1950;

        public static int SLIDE_MAX_POSITION = 1140;

        public static int SLIDE_MIN_POSITION = 70;


        public static int SLIDE_ROTATION_OFFSET = 0;

        public static int SLIDE_ROTATION_SPECIMEN_POSITION = 1330 + SLIDE_ROTATION_OFFSET;

        public static int SLIDE_ROTATION_WALL_POSITION = 218 + SLIDE_ROTATION_OFFSET;

        public static int SLIDE_ROTATION_WALL_MIDDLE_POSITION = 320 + SLIDE_ROTATION_OFFSET;

        public static int SLIDE_ROTATION_SPECIMEN_FRONT_DROP_POSITION = 590 + SLIDE_ROTATION_OFFSET;

        public static int SLIDE_ROTATION_DROP_POSITION = 1330 + SLIDE_ROTATION_OFFSET;

        public static int SLIDE_ROTATION_MIDDLE_POSITION= 1230 + SLIDE_ROTATION_OFFSET;

        public static int SLIDE_ROTATION_MIN_POSITION = SLIDE_ROTATION_OFFSET;

        public static int SLIDE_ROTATION_CAMERA_POSITION = 350 + SLIDE_ROTATION_OFFSET;

        public static int SLIDE_ROTATION_MAX_POSITION = 1330 + SLIDE_ROTATION_OFFSET;

        public static double SLIDE_PID_kp = 0.027;
        public static double SLIDE_PID_kd = 0.027;
        public static double SLIDE_PID_ki = 0.0001;

        public static int SLIDE_PID_target = 400;



        public static double CLAW_HORIZONTAL_ANGLE_CENTER = 0.54;
        public static double CLAW_HORIZONTAL_ANGLE_LEFT = 0.42;
        public static double CLAW_HORIZONTAL_ANGLE_RIGHT = 0.66;



        public static double ANGLE_DRIVE_KP = 0.01;
        public static double ANGLE_DRIVE_KD = .00004;
        public static double ANGLE_DRIVE_KI = 0.0;

        public static double ANGLE_DRIVE_TARGET_TEST = 0.0;

    }

    public static RoboSapiensTeleOp.Params PARAMS = new RoboSapiensTeleOp.Params();

    private Follower follower;

    @Override
    public void runOpMode() {

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(0,0,0));

        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry);

        //0
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"fL");
        //1
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"fR");
        //2
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"bL");
        //3
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"bR");


       // DcMotorEx horizontalSlide = hardwareMap.get(DcMotorEx.class, "horizontalSlide1");
       // DcMotorEx verticalSlide1 = hardwareMap.get(DcMotorEx.class, "verticalSlide1");
       // DcMotorEx verticalSlide2 = hardwareMap.get(DcMotorEx.class, "verticalSlide2");

//        while (!opModeIsActive() && !isStopRequested()) {
//            robot.execute(telemetry);
//        }
        waitForStart();
        robot.start();

        //11 and 6/16
        //8 and 5/16

        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
            robot.execute(telemetry);

//            telemetry.addData("FrontLeftMotor current (A): ", frontLeftMotor.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("FrontRightMotor current (A): ", frontRightMotor.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("BackLeftMotor current (A): ", backLeftMotor.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("BackRightMotor current (A): ", backRightMotor.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("horizontalSlide current (A): ", horizontalSlide.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("verticalSlide1 current (A): ", verticalSlide1.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("verticalSlide2 current (A): ", verticalSlide2.getCurrent(CurrentUnit.AMPS));
//
//            telemetry.update();

           // telemetry.addData("State", robot.getCurrentState().toString());
           // telemetry.addData("Vertical slide", )
           // telemetry.update();
        }
    }
}
