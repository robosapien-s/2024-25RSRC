package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class RobotAuto {

    private Robot robot;
    private Telemetry telemetry;

    private double lastTime = 0;
    private int lastDuration = 0;
    private boolean waitRunning = true;

    IMU imu;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    public RobotAuto(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, Follower follower, double rotateAngleOffset) {
        robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, true, follower);
        Robot.resetEncoders = false;
        Robot.rotateAngleOffset = rotateAngleOffset;

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

    public void openClaw() {
        robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
    }

    public void closeClaw() {
        robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_CLOSE);
    }

    public void execute() {
        robot.executeAuto(telemetry);
    }

    public void setState(IRobot.State state) {
        robot.switchState(state);
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

    public void setSlideRotationPos(int pos) {
        robot.setSlideRotationTargetPosition(pos);
    }

    public void setSlidePos(int pos) {
        robot.setSlideTargetPosition(pos);
    }


    public void setRotAndAnglePos(double[] pos) {
        robot.setRotAndAnglePosition(pos);
    }

    public void setIntakeAnglePos (double pos) {
        robot.setIntakeAnglePosition(pos);
    }

    public IRobot.State getState() {
        return robot.getCurrentState();
    }




}
