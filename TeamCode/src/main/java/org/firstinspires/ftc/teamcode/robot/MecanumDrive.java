package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IDrive;
import org.firstinspires.ftc.teamcode.wrappers.FCDrivingWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.RevIMUv3;

public class MecanumDrive implements IDrive {


    final DcMotor frontLeftMotor;
    final DcMotor backLeftMotor;
    final DcMotor frontRightMotor;
    final DcMotor backRightMotor;


    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("fL");
        backLeftMotor = hardwareMap.dcMotor.get("bL");
        frontRightMotor = hardwareMap.dcMotor.get("fR");
        backRightMotor = hardwareMap.dcMotor.get("bR");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
//    @Override
//    public void init() {
//
//    }

    @Override
    public void update(Telemetry telemetry, JoystickWrapper joystickWrapper, double speed, double rotSpeed) {
        double y = -joystickWrapper.gamepad1GetLeftStickY(); // Remember, Y stick value is reversed
        double x = joystickWrapper.gamepad1GetLeftStickX() * 1.1; // Counteract imperfect strafing
        double rx = joystickWrapper.gamepad1GetRightStickX();

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}
