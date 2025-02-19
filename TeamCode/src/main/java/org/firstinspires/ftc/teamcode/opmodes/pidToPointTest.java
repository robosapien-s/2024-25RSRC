package org.firstinspires.ftc.teamcode.opmodes;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.fasterxml.jackson.databind.introspect.TypeResolutionContext;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
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


    public static double xKp = .1;
    public static double xKd = 0;
    public static double xKi = 0;

    public static double yKp = .1;
    public static double yKd = 0;
    public static double yKi = 0;

    public static double angleKp = .1;
    public static double angleKd = 0;
    public static double angleKi = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        ThreeDeadWheelLocalizer localizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick, new Pose2d(0,0,0));

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bL");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "bR");


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        BasicPID xPid = new BasicPID(new PIDCoefficients(xKp,xKi,xKd));
        BasicPID yPid = new BasicPID(new PIDCoefficients(yKp,yKi,yKd));
        BasicPID anglePidInner = new BasicPID(new PIDCoefficients(angleKp,angleKi,angleKd));
        AngleController anglePid = new AngleController(anglePidInner);

        double xPower;
        double yPower;
        double anglePower;
        double xRotated;
        double yRotated;

        waitForStart();


        while (!isStopRequested()) {
            localizer.update();


            double angle = angleWrap(localizer.getPose().heading.toDouble());

            xPower = xPid.calculate(xTarget, localizer.getPose().position.x);
            yPower = yPid.calculate(yTarget, localizer.getPose().position.y);
            anglePower = anglePid.calculate(angleTarget, angle);

            xRotated = xPower*Math.cos(angle) - yPower*Math.sin(angle);
            yRotated = xPower*Math.sin(angle) + yPower*Math.cos(angle);

//            frontLeftMotor.setPower(xRotated + yRotated + anglePower);
//            backLeftMotor.setPower(xRotated - yRotated + anglePower);
//            frontRightMotor.setPower(xRotated - yRotated - anglePower);
//            backRightMotor.setPower(xRotated + yRotated - anglePower);

            telemetry.addData("xPos", localizer.getPose().position.x);
            telemetry.addData("yPos", localizer.getPose().position.y);
            telemetry.addData("angle", Math.toDegrees(localizer.getPose().heading.toDouble()));

            telemetry.addData("xTarget", xTarget);
            telemetry.addData("yTarget", yTarget);
            telemetry.addData("angleTarget", Math.toDegrees(angleTarget));

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
