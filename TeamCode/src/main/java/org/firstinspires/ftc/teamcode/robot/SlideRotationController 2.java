package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class SlideRotationController extends RobotPidMechanism {

    private final DcMotorEx motorSlide1;

    public static double kP = 0.015;
    public static double kI = 0.000;
    public static double kD = 0.004;
    public static int targetPosition = 0;

    // Constructor
    public SlideRotationController(HardwareMap hardwareMap, String mainSlide, int inMaxPosition, int inMinPosition, boolean isBraking) {
        super(
                kP,          // Proportional gain
                kI,          // Integral gain
                kD,          // Derivative gain
                -1.0,        // Minimum output limit
                1.0,         // Maximum output limit
                0.1,         // Output ramp rate (optional)
                inMaxPosition,
                inMinPosition
        );

        motorSlide1 = hardwareMap.get(DcMotorEx.class, mainSlide);
        motorSlide1.setDirection(DcMotorSimple.Direction.REVERSE);
        if (Robot.resetEncoders) motorSlide1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorSlide1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        if(isBraking) {
            motorSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public int getCurrentPosition() {
        return motorSlide1.getCurrentPosition();
    }

    @Override
    public void onSetPower(double power) {
        motorSlide1.setPower(-power);
    }


    public double calculateOutput(int targetPosition, int currentPosition) {
        double error = targetPosition - currentPosition;

        integral += error;
        double derivative = error - previousError;

        double output = (kP * error) + (kI * integral) + (kD * derivative);

        output = Math.max(-1.0, Math.min(1.0, output));

        previousError = error;

        return output;
    }

    @Override
    public String getName() {
        return "Rotational Controller";
    }

    public void resetPID() {
        integral = 0;
        previousError = 0;
    }

    private double integral = 0;
    private double previousError = 0;}