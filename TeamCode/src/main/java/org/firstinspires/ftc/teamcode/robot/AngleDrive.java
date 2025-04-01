package org.firstinspires.ftc.teamcode.robot;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.interfaces.IDrive;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
public class AngleDrive implements IDrive {

    double normalizedHeadingError = 0;
    IMU imu;
    boolean cosineThing = false;
    HardwareMap hardwareMap;
    double pitch;
    double roll;
    double yaw;
    double PGain = .03;
    double targetHeading;
    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backRightMotor;
    boolean isAutoMode = false;
    double autoModeX = 0;
    double autoModeY = 0;
    double rotateAngleOffset = 0;


    Follower localizer = null;
    boolean isLerpEnabled;

    private final PIDEx pidController;

    private final PIDEx pidXController;
    private final PIDEx pidYController;

    public AngleDrive(HardwareMap hardwareMap, boolean isLerpEnabled, Follower localizer) {
        this(hardwareMap, isLerpEnabled);
        this.localizer = localizer;
    }
    public AngleDrive(HardwareMap hardwareMap, boolean isLerpEnabled) {
        if (Robot.resetEncoders) {
            InitializeResetImu(hardwareMap);
        } else {
            Initialize(hardwareMap);
        }
        this.isLerpEnabled = isLerpEnabled;

        PIDCoefficientsEx pidCoefficients = new PIDCoefficientsEx(
                RoboSapiensTeleOp.Params.ANGLE_DRIVE_KP,     // Proportional gain
                RoboSapiensTeleOp.Params.ANGLE_DRIVE_KI,    // Integral gain
                RoboSapiensTeleOp.Params.ANGLE_DRIVE_KD,    // Derivative gain
                -1.0,      // Minimum output limit
                1.0,       // Maximum output limit
                0.1       // Output ramp rate (optional)
                // Integral wind-up limit (optional)
        );

        pidController = new PIDEx(pidCoefficients);



        PIDCoefficientsEx pidCoefficientsX = new PIDCoefficientsEx(
                RoboSapiensTeleOp.Params.ANGLE_DRIVE_KP,     // Proportional gain
                RoboSapiensTeleOp.Params.ANGLE_DRIVE_KI,    // Integral gain
                RoboSapiensTeleOp.Params.ANGLE_DRIVE_KD,    // Derivative gain
                -1.0,      // Minimum output limit
                1.0,       // Maximum output limit
                0.1       // Output ramp rate (optional)
                // Integral wind-up limit (optional)
        );

        pidXController = new PIDEx(pidCoefficientsX);


        PIDCoefficientsEx pidCoefficientsY = new PIDCoefficientsEx(
                RoboSapiensTeleOp.Params.ANGLE_DRIVE_KP,     // Proportional gain
                RoboSapiensTeleOp.Params.ANGLE_DRIVE_KI,    // Integral gain
                RoboSapiensTeleOp.Params.ANGLE_DRIVE_KD,    // Derivative gain
                -1.0,      // Minimum output limit
                1.0,       // Maximum output limit
                0.1       // Output ramp rate (optional)
                // Integral wind-up limit (optional)
        );

        pidYController = new PIDEx(pidCoefficientsY);


    }

    public void Initialize(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bL");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "bR");

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    */

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }


    public void setPose(Pose pose) {
        if(localizer != null) {
            localizer.setPose(pose);
        }
    }

    public Pose getPose() {
        if(localizer != null) {
            localizer.update();
            return localizer.getPose();
        } else {
            return new Pose(0,0,0);
        }
    }

    public void InitializeResetImu(HardwareMap hardwareMap) {
        rotateAngleOffset = 0;
        Initialize(hardwareMap);
        imu.resetYaw();
    }

    @Override
    public Vector3D getRobotPosition() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentYaw = orientation.getYaw(AngleUnit.DEGREES);

        return new Vector3D(backLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition(), currentYaw);
    }

    @Override
    public void setTargetHeading(double heading) {
        targetHeading = heading;
    }

    @Override
    public void update(Telemetry telemetry, JoystickWrapper joystickWrapper, double speed, double rotSpeed) {
        updateRaw(telemetry, joystickWrapper.gamepad1GetLeftStick(), joystickWrapper.gamepad1GetExponentialLeftStickX(1), joystickWrapper.gamepad1GetExponentialLeftStickY(1), joystickWrapper.gamepad1GetRightStickX(), joystickWrapper.gamepad1GetRightStickY(), speed, rotSpeed);
    }

    @Override
    public void updateRaw(Telemetry telemetry, boolean isLeftStickPressed, double leftStickX, double leftStickY, double rightStickX, double rightStickY, double speed, double rotSpeed) {

        if(localizer != null) {
            localizer.update();
        }


        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        pitch = orientation.getPitch(AngleUnit.DEGREES);
        roll = orientation.getRoll(AngleUnit.DEGREES);
        yaw = orientation.getYaw(AngleUnit.DEGREES);

//        telemetry.addData("yaw", yaw);

        if (length(rightStickX, rightStickY) > 0.5) {
            targetHeading = normalize(Math.toDegrees(Math.atan2(-rightStickY, rightStickX)) - 90);
        }

        if(length(rightStickX, rightStickY) > 0) {
            isAutoMode = false;
        }

        if(isAutoMode) {
            leftStickX =  Range.clip( pidXController.calculate( autoModeX, frontRightMotor.getCurrentPosition()), -1, 1 );
            leftStickY=  Range.clip( pidXController.calculate( autoModeY, frontLeftMotor.getCurrentPosition()), -1, 1 );
        }

        double smoothingFactor = 0.1;
        double smoothedYaw = this.isLerpEnabled ? lerp(yaw, targetHeading, smoothingFactor) : targetHeading;

        double headingError = normalize((smoothedYaw - yaw) + rotateAngleOffset);

        Translation2d translation2d = RotateAngle(leftStickX * Math.abs(leftStickX), leftStickY * Math.abs(leftStickY), yaw);

        double newRx = pidController.calculate( yaw+headingError, yaw);

        if (cosineThing) {
            MoveMecanum(-translation2d.getX() * speed,  (translation2d.getY()*speed) * Math.cos(Math.toRadians(headingError)), speed * Range.clip(headingError * PGain, -1, 1));
        } else {
           // MoveMecanum(-translation2d.getX(), translation2d.getY(), Range.clip(headingError * PGain, -1, 1));
            MoveMecanum(-translation2d.getX() * speed, translation2d.getY() * speed, Range.clip(newRx, -1, 1) * speed);
        }

//        telemetry.update();
    }

    public double lerp(double current, double target, double t) {
        return current + (target - current) * t;
    }



    double length(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    void MoveMecanumPidToPoint(double xRotated, double yRotated, double anglePower) {
        frontLeftMotor.setPower(xRotated + yRotated + anglePower);
        backLeftMotor.setPower(xRotated - yRotated + anglePower);
        frontRightMotor.setPower(xRotated - yRotated - anglePower);
        backRightMotor.setPower(xRotated + yRotated - anglePower);
    }
    void MoveMecanum(double x, double y, double rx) {
        frontLeftMotor.setPower(y + x + rx);
        backLeftMotor.setPower(y - x + rx);
        frontRightMotor.setPower(y - x - rx);
        backRightMotor.setPower(y + x - rx);
    }

    Translation2d RotateAngle(double x, double y, double angle) {
        double rAngle = Math.toRadians(angle + rotateAngleOffset);

        double rotatedX = x * Math.cos(rAngle) - y * Math.sin(rAngle);
        double rotatedY = x * Math.sin(rAngle) + y * Math.cos(rAngle);
        return new Translation2d(rotatedX, rotatedY);
    }

    @Override
    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    @Override
    public void setAutoMode(double inX, double inY) {

       /*
        isAutoMode = true;
        autoModeX = inX;
        autoModeY = inY;
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
*/


    }

    public void disableAutoMode() {

        /*
        isAutoMode = false;
        autoModeX = 0;
        autoModeY = 0;

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

         */
    }

    public double getNormalizedHeadingError() {
        return normalizedHeadingError;
    }

    public double normalize(double angle) {
        if (angle > 180) {
            return angle - 360;
        } else if (angle < -180) {
            return angle + 360;
        } else {
            return angle;
        }
    }
}
