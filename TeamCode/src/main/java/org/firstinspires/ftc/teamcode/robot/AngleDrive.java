package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.interfaces.IDrive;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@Disabled
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

    double rotateAngleOffset = 180;

    public AngleDrive(HardwareMap hardwareMap) {
        InitializeResetImu(hardwareMap);
    }

    public void Initialize(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bL");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "bR");

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void InitializeResetImu(HardwareMap hardwareMap) {
        rotateAngleOffset = 0;
        Initialize(hardwareMap);
        imu.resetYaw();
    }

    @Override
    public void update(Telemetry telemetry, JoystickWrapper joystickWrapper, double speed, double rotSpeed) {
        updateRaw(telemetry, joystickWrapper.gamepad1GetLeftStick(), joystickWrapper.gamepad1GetExponentialLeftStickX(1), joystickWrapper.gamepad1GetExponentialLeftStickY(1), joystickWrapper.gamepad1GetRightStickX(), joystickWrapper.gamepad1GetRightStickY(), speed, rotSpeed);
    }

    @Override
    public void updateRaw(Telemetry telemetry, boolean isLeftStickPressed, double leftStickX, double leftStickY, double rightStickX, double rightStickY, double speed, double rotSpeed) {

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        pitch = orientation.getPitch(AngleUnit.DEGREES);
        roll = orientation.getRoll(AngleUnit.DEGREES);
        yaw = orientation.getYaw(AngleUnit.DEGREES);

        telemetry.addData("yaw", yaw);

        if (length(rightStickX, rightStickY) > 0.5) {
            targetHeading = normalize(Math.toDegrees(Math.atan2(-rightStickY, rightStickX)) - 90);
        }

        double smoothingFactor = 0.1;
        double smoothedYaw = lerp(yaw, targetHeading, smoothingFactor);

        double headingError = normalize((smoothedYaw - yaw) + rotateAngleOffset);

        Translation2d translation2d = RotateAngle(leftStickX * Math.abs(leftStickX), leftStickY * Math.abs(leftStickY), yaw);

        if (cosineThing) {
            MoveMecanum(-translation2d.getX(), translation2d.getY() * Math.cos(Math.toRadians(headingError)), Range.clip(headingError * PGain, -1, 1));
        } else {
            MoveMecanum(-translation2d.getX(), translation2d.getY(), Range.clip(headingError * PGain, -1, 1));
        }

        telemetry.update();
    }

    // Linear interpolation method
    public double lerp(double current, double target, double t) {
        return current + (target - current) * t;
    }

    double length(double x, double y) {
        return Math.sqrt(x * x + y * y);
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

    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setAutoMode(double inX, double inY) {
        isAutoMode = true;
        autoModeX = inX;
        autoModeY = inY;
    }

    public void disableAutoMode() {
        isAutoMode = false;
        autoModeX = 0;
        autoModeY = 0;
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
