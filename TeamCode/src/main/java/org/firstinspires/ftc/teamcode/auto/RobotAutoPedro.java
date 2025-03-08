package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class RobotAutoPedro {

    private Robot robot;
    private Telemetry telemetry;

    private double lastTime = 0;
    private int lastDuration = 0;
    private boolean waitRunning = true;

    IMU imu;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    public RobotAutoPedro(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, true);
        Robot.resetEncoders = false;

        robot.setYawOverride(new Robot.YawOverrride() {
            @Override
            public double getYaw() {
                return getAutoYaw();
            }
        });

        this.telemetry = telemetry;

        imu = hardwareMap.get(IMU .class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    public double getAutoYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void openTopClaw() {
        robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
    }

    public void closeTopClaw() {
        robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_CLOSE);
    }

    public void execute() {
        robot.executeAuto(telemetry);
    }

    public void setState(IRobot.State state) {
        robot.switchState(state);
    }

    public void setIntakeClawAnglePos(double pos) {
        robot.setIntakeAngleServo(pos);
    }

    public void startWait(int durationMillis) {
        lastTime = System.currentTimeMillis();
        waitRunning = true;
        lastDuration = durationMillis;
    }

    public boolean checkWait() {
        if (!waitRunning) {
            return true;
        }

        if (System.currentTimeMillis()-lastTime>lastDuration) {
            waitRunning = false;
            return true;
        }

        return false;
    }

    public void setHorizontalSlidePos(int pos) {
        robot.setHorizontalSlideTargetPosition(pos);
    }

    public void setIntakeRotServo(double pos) {
        robot.setIntakeRotationServo(pos);
    }


}
