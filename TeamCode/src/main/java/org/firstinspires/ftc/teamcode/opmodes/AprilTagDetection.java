package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "AprilTag 3D Detection", group = "Sensor")
public class AprilTagDetection extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Limelight camera
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize Limelight.");
            telemetry.update();
            return; // Exit if initialization fails
        }

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        // Start polling for data
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        // Poll continuously for AprilTag detections
        while (opModeIsActive()) {
            processAprilTags();
            telemetry.update();
        }

        // Stop polling for data
        limelight.stop();
    }

    /**
     * Processes AprilTag detection results and updates telemetry with 3D pose data.
     */
    private void processAprilTags() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();

            // Extract fiducial results (AprilTags)
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (!fiducialResults.isEmpty()) {
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    int tagId = fr.getFiducialId();
                    Pose3D robotPos = fr.getRobotPoseFieldSpace();
                    if (tagId != 0) {

                        telemetry.addData("AprilTag ID", tagId);
                        telemetry.addData("Bot Pose", robotPos.getPosition());
                        telemetry.addData("Capture Latency", "%.2f ms", captureLatency);
                        telemetry.addData("Targeting Latency", "%.2f ms", targetingLatency);
                    } else {
                        telemetry.addData("Tag ID", tagId);
                        telemetry.addData("Pose3D", "No pose data available.");
                    }
                }
            } else {
                telemetry.addData("AprilTag", "No AprilTag detected.");
            }
        } else {
            telemetry.addData("Limelight", "No valid result.");
        }
    }


}
