package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlideController extends RobotPidMechanism {

    private final DcMotorEx motorSlide1;
    private final DcMotorEx motorSlide2;

    public SlideController(HardwareMap hardwareMap) {
        motorSlide1 = hardwareMap.get(DcMotorEx.class, "motorSlide1");
        motorSlide2 = hardwareMap.get(DcMotorEx.class, "motorSlide2");

        //motorSlide1.setDirection(DcMotorSimple.Direction.REVERSE);

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
        motorSlide1.setPower(-power);
    }
}
