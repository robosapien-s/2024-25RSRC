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
        public static double CLAW_OPEN = .9;
        public static double CLAW_CLOSE = .4;

        public static double INTAKE_ANGLE_INIT = 0.575;
        public static double INTAKE_ANGLE_START = .46;
        public static double INTAKE_ANGLE_BUCKET = 0.44;
        public static double INTAKE_ANGLE_READY =.42;
        public static double INTAKE_ANGLE_PICKUP = .305;
        public static double INTAKE_ANGLE_CAMERA = 0.35;

        public static double[] ROT_AND_ANGLE_BACK = {1.0, 0.03};

        public static double[] ROT_AND_ANGLE_BUCKET = {1.0, 0.03};

        public static double[] ROT_AND_ANGLE_CAMERA = { 0.575, 0.885};

        public static double[] ROT_AND_ANGLE_PREP = { 0.606, .835}; //{ 0.5678, .875}; // R / L

        //public static double[] ROT_AND_ANGLE_PICKUP_HORIZONTAL = {0.513, 0.695};
        public static double[] ROT_AND_ANGLE_PICKUP_HORIZONTAL = { 0.59, 0.85};//{ 0.5678, .875}; // R / L


        public static double[] ROT_AND_ANGLE_PICKUP_VERTICAL = {0.505, 0.745};

        public static double[] ROT_AND_ANGLE_PICKUP_LEFT = {0.655 , 0.86};

        public static double[] ROT_AND_ANGLE_PICKUP_RIGHT = {0.56, 0.79};

        public static double[] ROT_AND_ANGLE_BASKET = {1, 0.4489};

        public static int SLIDE_ROTATION_INTAKE_INITIAL = 200;

        public static int SLIDE_POSITION = 140;

        public static int SLIDE_TRANSFER_POSITION = 0;

        public static int SLIDE_MIDDLE_POSITION = 500;

        public static int SLIDE_WALL_POSITION = 200;

        public static int SLIDE_DOWN_POSITION = 0;

        public static int SLIDE_HANG_PREP_POSITION = 1030;

        public static int SLIDE_HANG_DROP_POSITION = 600;

        public static int SLIDE_DROP_L1 = 1000;

        public static int SLIDE_DROP_L2 = 1900;

        public static int SLIDE_MAX_POSITION = 880;

        public static int SLIDE_MIN_POSITION = 70;

        public static int SLIDE_ROTATION_DROP_POSITION = 1400;

        public static int SLIDE_ROTATION_MIDDLE_POSITION= 1350;

        public static int SLIDE_ROTATION_MIN_POSITION = 0;

        public static int SLIDE_ROTATION_CAMERA_POSITION = 350;

        public static int SLIDE_ROTATION_MAX_POSITION = 1400;

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

        while (!opModeIsActive() && !isStopRequested()) {
            robot.execute(telemetry);
        }
        waitForStart();
        robot.start();

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
