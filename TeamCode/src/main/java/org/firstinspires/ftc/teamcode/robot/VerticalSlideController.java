package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VerticalSlideController extends RobotPidMechanism {

    private final DcMotorEx motorSlide1;
    private final DcMotorEx motorSlide2;

    private boolean isLeftMotorEncoded = true;

    public VerticalSlideController(HardwareMap hardwareMap, String mainSlide, String secondSlide, boolean inIsLeftMotorEncoded) {

        super(
                0.05,     // Proportional gain
                0.0001,    // Integral gain
                0.0003,    // Derivative gain
                -1.0,      // Minimum output limit
                1.0,       // Maximum output limit
                0.1       // Output ramp rate (optional)
                // Integral wind-up limit (optional)
        );

        motorSlide1 = hardwareMap.get(DcMotorEx.class, mainSlide);
        motorSlide2 = hardwareMap.get(DcMotorEx.class, secondSlide);
        isLeftMotorEncoded = inIsLeftMotorEncoded;
        if(isLeftMotorEncoded) {
            motorSlide1.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        motorSlide1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorSlide2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorSlide1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorSlide2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getCurrentPosition() {
        return motorSlide1.getCurrentPosition();
    }

    @Override
    public void onSetPower(double power) {
        if(isLeftMotorEncoded) {
            motorSlide1.setPower(-power);
            motorSlide2.setPower(-power);
        } else {
            motorSlide1.setPower(-power);
            motorSlide2.setPower(power);
        }
    }
}
