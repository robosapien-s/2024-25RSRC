package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DualServoSlideController extends RobotPidMechanism{

    private final CRServo servo1;
    private final CRServo servo2;
    private final DcMotorEx servoEncoder;

    public DualServoSlideController(HardwareMap hardwareMap, String servo1Name, String servo2name, String encoderName, int inMaxPosition, int inMinPosition) {

        super(  0.00044,     // Proportional gain
                0.0,    // Integral gain
                0.004,    // Derivative gain
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
