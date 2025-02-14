package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class DualServoSlideController extends RobotPidMechanism{

    private final CRServo servo1;
    private final CRServo servo2;
    private final DcMotorEx servoEncoder;

    public static double kP = 0.00044;
    public static double kI = 0.000;
    public static double kD = 0.0046;
    public static int targetPosition = 0;

    public DualServoSlideController(HardwareMap hardwareMap, String servo1Name, String servo2name, String encoderName, int inMaxPosition, int inMinPosition) {

        super(  kP,     // Proportional gain
                kI,    // Integral gain
                kD,    // Derivative gain
                -1.0,      // Minimum output limit
                1.0,       // Maximum output limit
                0.1,  // Output ramp rate (optional)
                // Integral wind-up limit (optional),
                inMaxPosition,
                inMinPosition
        );


        servo1 = hardwareMap.get(CRServo.class, servo1Name);
        servo2 = hardwareMap.get(CRServo.class, servo2name);
        servoEncoder = hardwareMap.get(DcMotorEx.class, encoderName);
        if (Robot.resetEncoders) servoEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        servoEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public int getCurrentPosition() {
        return servoEncoder.getCurrentPosition();
    }


    public void onSetPower(double power) {
        double cappedPower = testCapPower(power, 1);
        servo1.setPower(cappedPower);
        servo2.setPower(cappedPower);
    }

    @Override
    public String getName() {
        return "Dual Claw Slider";
    }
}
