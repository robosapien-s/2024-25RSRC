package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.HorizontalSlideController;
import org.firstinspires.ftc.teamcode.robot.VerticalSlideController;

@Autonomous
public class AutoTest extends LinearOpMode {
    public class HorizontalSlide {
        private final HorizontalSlideController horizontalSlideController;
        private int targetPos = 0;

        public HorizontalSlide(HardwareMap hardwareMap) {
            horizontalSlideController = new HorizontalSlideController(hardwareMap, "horizontalSlide1", RoboSapiensTeleOp.Params.HORIZONTAL_SLIDE_MAX_POSITION, 0, true);
        }

        public class HorizontalSlideAction implements Action {
            private boolean initialized = false;

            public HorizontalSlideAction(int inTargetPos) {
                targetPos = inTargetPos;
            }

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    initialized = true;
                    horizontalSlideController.setTargetPosition(targetPos);
                }

                // checks lift's current position
                double pos = horizontalSlideController.getCurrentPosition();
                packet.put("horizontalSlideController", pos);

                horizontalSlideController.update(null);
                if (pos < targetPos) {
                    // true causes the action to rerun
                    return true;
                } else {
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public Action horizontalSlideAction(int inTargetPos) {
            return new HorizontalSlideAction(inTargetPos);
        }
    }

    public class VerticalSlide {
        private final VerticalSlideController verticalSlideController;
        private int targetPos = 0;

        public VerticalSlide(HardwareMap hardwareMap) {
            verticalSlideController = new VerticalSlideController(hardwareMap, "verticalSlide1", "verticalSlide2", "clawSliderEncoder", true, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_MAX_POSITION, 0, true);
        }

        public class VerticalSlideAction implements Action {
            private boolean initialized = false;

            public VerticalSlideAction(int inTargetPos) {
                targetPos = inTargetPos;
            }

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    initialized = true;
                    verticalSlideController.setTargetPosition(targetPos);
                }

                // checks lift's current position
                double pos = verticalSlideController.getCurrentPosition();
                packet.put("verticalSlideController", pos);

                verticalSlideController.update(null);
                if (pos < targetPos) {
                    // true causes the action to rerun
                    return true;
                } else {
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public Action verticalSlideAction(int inTargetPos) {
            return new VerticalSlideAction(inTargetPos);
        }
    }

    @Override
    public void runOpMode() {
        HorizontalSlide horizontalSlide = new HorizontalSlide(hardwareMap);
        VerticalSlide verticalSlide = new VerticalSlide(hardwareMap);
        Pose2d initialPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,initialPose);

        TrajectoryActionBuilder moveForward10 = drive.actionBuilder(initialPose)
                .lineToX(10)
                .waitSeconds(10);

        Action trajectory = moveForward10.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                    horizontalSlide.horizontalSlideAction(300),
                    verticalSlide.verticalSlideAction(300),
                    trajectory
                )
        );

    }

}
