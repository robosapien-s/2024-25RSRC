package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class SlideController extends RobotPidMechanism {

    private final DcMotorEx motorSlide1;
    private final DcMotorEx motorSlide2;

    private DcMotorEx motorSlide3 = null;

    // Dashboard-configurable PID coefficients and target position
    public static double kP = 0.01;
    public static double kI = 0.0000;
    public static double kD = 0.0001;
    public static int targetPosition = 0;

    private boolean isLeftMotorEncoded = true;

    // Constructor
    public SlideController(HardwareMap hardwareMap, String mainSlide, String secondSlide, String thridSlide, boolean inIsLeftMotorEncoded, int inMaxPosition, int inMinPosition, boolean isBreaking) {
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
        motorSlide2 = hardwareMap.get(DcMotorEx.class, secondSlide);

        if(thridSlide != null &&  !thridSlide.isEmpty()) {
            motorSlide3 = hardwareMap.get(DcMotorEx.class, thridSlide);
        }

        if (isBreaking) {
            motorSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if(motorSlide3 != null) {
                motorSlide3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        isLeftMotorEncoded = inIsLeftMotorEncoded;

        // Reverse the direction of the left motor if it's encoded
        if (isLeftMotorEncoded) {
            motorSlide1.setDirection(DcMotorSimple.Direction.REVERSE);
            motorSlide2.setDirection(DcMotorSimple.Direction.REVERSE);

            if(motorSlide3 != null) {
                motorSlide3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        // Reset and configure encoders
        if (Robot.resetEncoders) {
            motorSlide1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorSlide2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        }

        motorSlide1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorSlide2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        if(motorSlide3 != null) {
            if (Robot.resetEncoders) motorSlide3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorSlide3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // Get the current position from the main motor's encoder
    public int getCurrentPosition() {
        return motorSlide1.getCurrentPosition();
    }


    // Set power to both motors, adjusting direction for synchronization
    @Override
    public void onSetPower(double power) {

        if (isLeftMotorEncoded) {
          motorSlide1.setPower(-power);
          motorSlide2.setPower(-power);

          if(motorSlide3 != null) {
              motorSlide3.setPower(-power);
          }
        } else {
           //motorSlide1.setPower(-power);
           //motorSlide2.setPower(power);
        }

        currentPower = power;
    }

    double currentPower = 0;
    public double getCurrentPower() {
        return currentPower;
    }

    // PID reset method to clear integral and previous error
    public void resetPID() {
        integral = 0;
        previousError = 0;
    }

    // Internal fields for PID calculations
    private double integral = 0;
    private double previousError = 0;

    /**
     * Calculate PID output based on the target and current positions.
     *
     * @param targetPosition The desired target position.
     * @param currentPosition The current position of the motor.
     * @return The power to be set to the motor.
     */
    public double calculateOutput(int targetPosition, int currentPosition) {
        double error = targetPosition - currentPosition;

        integral += error;
        double derivative = error - previousError;

        double output = (kP * error) + (kI * integral) + (kD * derivative);

        output = Math.max(-1.0, Math.min(1.0, output)); // Limit output to [-1.0, 1.0]

        previousError = error;

        return output;
    }

    @Override
    public String getName() {
        return "Slider";
    }
}
