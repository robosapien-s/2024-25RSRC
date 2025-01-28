package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.states.BaseState.createIntakeClawTask;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.DriveTest;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.states.BaseState;

public class RobotAuto {
    private long lastTime = 0;
    final private Robot robot;
    final private Telemetry telemetry;
    public RobotAuto(HardwareMap inHardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry inTelemetry) {
        robot = new Robot(inHardwareMap, gamepad1, gamepad2, inTelemetry, true);
        telemetry = inTelemetry;
//        robot.initPid();
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
                robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
                robot.setClawAnglePosition(DriveTest.Params.CLAW_ANGLE_FORWARD);
                robot.setClawHorizontalAnglePosition(DriveTest.Params.CLAW_HORIZONTAL_ANGLE_CENTER);
                return false;
            }
        };
    }

    public Action openTopClawAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.setClawPosition(DriveTest.Params.CLAW_OPEN);
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
        robot.setClawPosition(DriveTest.Params.CLAW_CLOSE);
    }

    public void openTopClaw() {
        robot.setClawPosition(DriveTest.Params.CLAW_OPEN);
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
