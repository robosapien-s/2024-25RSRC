package org.firstinspires.ftc.teamcode.robot;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class HorizontalSlideController extends RobotPidMechanism {

    private final DcMotorEx motorSlide1;

    public static double kP = 0.015;
    public static double kI = 0.000;
    public static double kD = 0.002;
    public static int targetPosition = 0;

    // Constructor
    public HorizontalSlideController(HardwareMap hardwareMap, String mainSlide, int inMaxPosition, int inMinPosition) {
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
        motorSlide1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorSlide1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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
        return "Horizontal Slider";
    }

    public void resetPID() {
        integral = 0;
        previousError = 0;
    }

    private double integral = 0;
    private double previousError = 0;}