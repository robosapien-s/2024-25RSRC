package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClawSlideController extends RobotPidMechanism {

    private final CRServo servo;
    private final DcMotorEx servoEncoder;

    public ClawSlideController(HardwareMap hardwareMap, String servoName, String encoderName) {

        super(  0.0009,     // Proportional gain
                0.0,    // Integral gain
                .00007,    // Derivative gain
                -1.0,      // Minimum output limit
                1.0,       // Maximum output limit
                0.1       //
        );


        servo = hardwareMap.get(CRServo.class, servoName);
        servoEncoder = hardwareMap.get(DcMotorEx.class, encoderName);
    }

    @Override
    public int getCurrentPosition() {
        return servoEncoder.getCurrentPosition();
    }

    @Override
    public void onSetPower(double power) {
        double cappedPower = testCapPower(power, 1);
        servo.setPower(cappedPower);
    }

    @Override
    public String getName() {
        return "Claw Slider";
    }
}
