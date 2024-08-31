package org.firstinspires.ftc.teamcode.wrappers;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DiffArmWrapper {

    enum Mode {
        EXTENDO,
        ARM,
        DUAL
    }

    public Mode currMode = Mode.EXTENDO;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public DcMotorEx diff1;
    public DcMotorEx diff2;
    public DiffArmWrapper(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        diff1 = hardwareMap.get(DcMotorEx.class, "diff1");
        diff2 = hardwareMap.get(DcMotorEx.class, "diff2");
    }

    public void changeCurrMode() {
        if (currMode == Mode.EXTENDO) {
            currMode = Mode.ARM;
        } else if (currMode == Mode.ARM) {
            currMode = Mode.DUAL;
        } else {
            currMode = Mode.EXTENDO;
        }
    }

    public Mode getCurrMode() {
        return currMode;
    }

    public void move(double leftStick, double rightStick) {
        if (currMode == Mode.EXTENDO) {
            diff1.setPower(leftStick);
            diff2.setPower(leftStick);
        } else if (currMode == Mode.ARM) {
            diff1.setPower(leftStick);
            diff2.setPower(-leftStick);
        } else {
            if (Math.abs(leftStick+rightStick) <= 1) {
                diff1.setPower(leftStick+rightStick);
                diff2.setPower(leftStick-rightStick);
            } else {
                diff1.setPower(1);
                diff2.setPower((leftStick-rightStick)/(leftStick+rightStick));
            }
        telemetry.addData("Mode", currMode);
        telemetry.update();
        }
    }
}
