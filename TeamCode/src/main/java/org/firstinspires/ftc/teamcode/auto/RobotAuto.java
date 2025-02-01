package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
    private long lastTime = 0;
    final private Robot robot;
    final private Telemetry telemetry;
    IMU imu;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


    public RobotAuto(HardwareMap inHardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry inTelemetry) {
        robot = new Robot(inHardwareMap, gamepad1, gamepad2, inTelemetry, true);
        robot.setYawOverride(new Robot.YawOverrride() {
            @Override
            public double getYaw() {
                return getAutoYaw();
            }
        });
        telemetry = inTelemetry;

        imu = inHardwareMap.get(IMU .class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
//        robot.initPid();
    }

    public double getAutoYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }


    public Action robotExecute() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetry.addData("Delta time", System.currentTimeMillis()-lastTime);
                lastTime = System.currentTimeMillis();
                robot.executeAuto(telemetry);
                return true;
            }
        };
    }

    public Action setState(IRobot.State state) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.switchState(state);
                return false;
            };
        };
    }

    public Action specimenHangSubstate1() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.setVerticalSlideTargetPosition(RoboSapiensTeleOp.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
                robot.setClawAnglePosition(RoboSapiensTeleOp.Params.CLAW_ANGLE_FORWARD);
                robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER);
                return false;
            }
        };
    }

    public Action openTopClawAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
                return false;
            }
        };
    }

    public Action setHorizontalSlidePosition(int pos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.setHorizontalSlideTargetPosition(pos);
                return false;
            }
        };
    }

    public Action setClawHorizontalAnglePosition(double pos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.setClawHorizontalAnglePosition(pos);
                return false;
            }
        };
    }

    public Action setIntakeRotationServo(double pos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.setIntakeRotationServo(pos);
                return false;
            }
        };
    }


    public void closeTopClaw() {
        robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_CLOSE);
    }

    public void openTopClaw() {
        robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
    }

    private long waitTime = 0;
    private long lastWaitTime = 0;
    private boolean isWaitInit = false;
    public Action waitAction(int durationInMillis) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isWaitInit) {
                    isWaitInit = true;
                    lastWaitTime = System.currentTimeMillis();
                }

                if (isWaitInit) {
                    waitTime = System.currentTimeMillis()-lastWaitTime;
                    if (waitTime > durationInMillis)
                        isWaitInit = false;
                }
                return isWaitInit;
            }
        };
    }

//    public Action setHorizontalSlidePos(int pos) {
//
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                robot.setHorizontalSlideTargetPosition(0);
//                return false;
//            }
//        };
//
//    }

}
