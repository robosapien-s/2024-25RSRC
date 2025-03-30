package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.DriveToPointController;
import org.firstinspires.ftc.teamcode.robot.SquidController;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
@Config
@TeleOp
public class pidToPointTest extends LinearOpMode {

    public static double xTarget = 0;
    public static double yTarget = 0;
    public static double angleTarget = 0;
    JoystickWrapper joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);




    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;


//    public static double xKp = .11;
//    public static double xKd = 6.5;
//    public static double xKi = 0;

//    public static double yKp = .11;
//    public static double yKd = 6.5;
//    public static double yKi = 0;

//    public static double angleKp = 2.1;
//    public static double angleKd = 0.04;
//    public static double angleKi = 0;




    Follower localizer;

    @Override
    public void runOpMode() throws InterruptedException {

        localizer = new Follower(hardwareMap, FConstants.class, LConstants.class);

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bL");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "bR");


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DriveToPointController driveController = new DriveToPointController();

//        BasicPID xPid = new BasicPID(new PIDCoefficients(xKp,xKi,xKd));
//        BasicPID yPid = new BasicPID(new PIDCoefficients(yKp,yKi,yKd));
//        BasicPID anglePidInner = new BasicPID(new PIDCoefficients(angleKp,angleKi,angleKd));
//        AngleController anglePid = new AngleController(anglePidInner);
//
//        double xPower;
//        double yPower;
//        double anglePower;
//        double xRotated;
//        double yRotated;

//        SquidController xPid = new SquidController(xKp, xKd);

        waitForStart();


        while (!isStopRequested()) {
            localizer.update();

//
//            double angle = angleWrap(localizer.getPose().heading.toDouble());
//
//
//
//            xPower = xPid.calculate(xTarget, localizer.getPose().position.x);
//            yPower = -yPid.calculate(yTarget, localizer.getPose().position.y);
//            anglePower = -anglePid.calculate(Math.toRadians(angleTarget), angle);






//           xRotated = xPower*Math.cos(angle) - yPower*Math.sin(angle);
//           yRotated = xPower*Math.sin(angle) + yPower*Math.cos(angle);

//            frontLeftMotor.setPower(xRotated + yRotated + anglePower);
//            backLeftMotor.setPower(xRotated - yRotated + anglePower);
//            frontRightMotor.setPower(xRotated - yRotated - anglePower);
//            backRightMotor.setPower(xRotated + yRotated - anglePower);



            Vector3D powers = driveController.calculate(xTarget, yTarget, Math.toRadians(angleTarget), localizer.getPose(), telemetry);

            double xRotated = powers.getX();
            double yRotated = powers.getY();
            double anglePower = powers.getZ();


            frontLeftMotor.setPower(xRotated + yRotated + anglePower);
            backLeftMotor.setPower(xRotated - yRotated + anglePower);
            frontRightMotor.setPower(xRotated - yRotated - anglePower);
            backRightMotor.setPower(xRotated + yRotated - anglePower);


            telemetry.addData("xPos", localizer.getPose().getX());
            telemetry.addData("yPos", localizer.getPose().getY());
            telemetry.addData("angle", Math.toDegrees(localizer.getPose().getHeading()));

            telemetry.addData("xTarget", xTarget);
            telemetry.addData("yTarget", yTarget);
            telemetry.addData("angleTarget", angleTarget);

            telemetry.addData("xRotated Power",xRotated);
            telemetry.addData("yRotated Power",yRotated);
            telemetry.addData("anglePower",anglePower);

            telemetry.update();

        }
    }

    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }
}
