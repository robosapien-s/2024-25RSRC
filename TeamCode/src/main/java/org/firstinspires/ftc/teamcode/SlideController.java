package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;

public class SlideController {

    private final DcMotor motorSlide1;
    private final DcMotor motorSlide2;
    private final PIDEx pidController;

    private int targetPosition;

    public SlideController(HardwareMap hardwareMap) {
        motorSlide1 = hardwareMap.get(DcMotor.class, "motorSlide1");
        motorSlide2 = hardwareMap.get(DcMotor.class, "motorSlide2");

        motorSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorSlide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDCoefficientsEx pidCoefficients = new PIDCoefficientsEx(
                0.005,     // Proportional gain
                0.0001,    // Integral gain
                0.0003,    // Derivative gain
                -1.0,      // Minimum output limit
                1.0,       // Maximum output limit
                0.1       // Output ramp rate (optional)
                // Integral wind-up limit (optional)
        );

        pidController = new PIDEx(pidCoefficients);

        targetPosition = 0;
    }

    public void update() {
        int currentPosition = motorSlide1.getCurrentPosition();

        double power = pidController.calculate(currentPosition, targetPosition);

        motorSlide1.setPower(power);
        motorSlide2.setPower(power);
    }

    public void setTargetPosition(int target) {
        targetPosition = target;
    }

    public int getCurrentPosition() {
        return motorSlide1.getCurrentPosition();
    }
}
