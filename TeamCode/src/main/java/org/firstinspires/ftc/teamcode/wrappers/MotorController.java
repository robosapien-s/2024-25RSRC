package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorController {

    private final DcMotorEx leftFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightBack;
    private final DcMotorEx rightFront;
    private final DcMotorEx extendingMotor;

    public MotorController(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "fL");
        leftBack = hardwareMap.get(DcMotorEx.class, "bL");
        rightBack = hardwareMap.get(DcMotorEx.class, "bR");
        rightFront = hardwareMap.get(DcMotorEx.class, "fR");
        extendingMotor = hardwareMap.get(DcMotorEx.class, "eM");

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extendingMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public boolean reqFlChange(double power) {
        if (canChangeMotor()) {
            leftFront.setPower(power);
            return true;
        }
        return false;
    }

    public boolean reqBlChange(double power) {
        if (canChangeMotor()) {
            leftBack.setPower(power);
            return true;
        }
        return false;
    }

    public boolean reqBrChange(double power) {
        if (canChangeMotor()) {
            rightBack.setPower(power);
            return true;
        }
        return false;
    }

    public boolean reqRfChange(double power) {
        if (canChangeMotor()) {
            rightFront.setPower(power);
            return true;
        }
        return false;
    }

    public boolean reqEmChange(double power) {
        if (canChangeMotor()) {
            extendingMotor.setPower(power);
            return true;
        }
        return false;
    }

    private boolean canChangeMotor() {
        return true;
    }
}
