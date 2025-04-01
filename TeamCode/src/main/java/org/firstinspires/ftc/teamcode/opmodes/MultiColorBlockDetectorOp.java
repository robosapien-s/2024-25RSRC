package org.firstinspires.ftc.teamcode.opmodes;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.AngleDrive;
import org.firstinspires.ftc.teamcode.robot.MultiColorSampleDetector;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.openftc.easyopencv.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Multi-Color Block Detector", group = "Vision")
public class MultiColorBlockDetectorOp extends LinearOpMode {

    public static double xKp = 0.003;
    public static double xKd = 0.0;
    public static double xKi = 0.0;



    public static double yKp = 0.003;
    public static double yKd = 0.0;
    public static double yKi = 0.0;

    @Override
    public void runOpMode() {
        // Initialize webcam

        double startingHeading = Math.toRadians(90);
//        Localizer localizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick, new Pose2d(0,0,startingHeading));
        Follower localizer = new Follower(hardwareMap, FConstants.class, LConstants.class);
        AngleDrive drive = new AngleDrive(hardwareMap, false, localizer);
        JoystickWrapper joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);
        MultiColorSampleDetector sampleDetector = new MultiColorSampleDetector(hardwareMap, telemetry, MultiColorSampleDetector.ClosestSamplePipeline.SampleColorPriority.all);


        PIDCoefficientsEx pidXCoefficients = new PIDCoefficientsEx(
                xKp, xKi, xKd, 0, 0, 0
        );

        PIDEx xPid = new PIDEx(pidXCoefficients);


        PIDCoefficientsEx pidYCoefficients = new PIDCoefficientsEx(
                yKp, yKi, yKd, 0, 0, 0
        );

        PIDEx yPid = new PIDEx(pidYCoefficients);


        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();


        Point centerTarget = sampleDetector.getCenterOfScreen();
        RotatedRect prevRect = new RotatedRect();

        long lastAquireTime = 0;


        while (opModeIsActive()) {

            RotatedRect cloestRect = sampleDetector.getClosestSample();

            long timeFromLastAquire = System.currentTimeMillis() - lastAquireTime;


            if(cloestRect.size.width != 0 ||  timeFromLastAquire  < 200) {

                if(cloestRect.size.width == 0) {
                    cloestRect = prevRect;
                } else {
                    lastAquireTime = System.currentTimeMillis();
                    prevRect = cloestRect;
                }

                double xPower = 0;
                double yPower = 0;

                xPower = xPid.calculate(centerTarget.x, cloestRect.center.x);
                yPower = yPid.calculate(centerTarget.y, cloestRect.center.y);

                if (xPower * xPower + yPower * yPower > 1) {
                    double mag = Math.sqrt(xPower * xPower + yPower * yPower);
                    xPower = xPower / mag;
                    yPower = yPower / mag;
                }

                telemetry.addData("X Power", xPower);
                telemetry.addData("Y Power", yPower);
                telemetry.addData("Current Pos", cloestRect.center);
                telemetry.addData("Target", centerTarget);

                drive.updateRaw(telemetry, false, -xPower, -yPower, 0, 0, 1, 1);
            }

        }

        sampleDetector.stopStreaming();
    }


}
