package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@TeleOp(name = "AprilTag Detection1", group = "Sensor")
public class AprilTagDetection extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize Limelight.");
            telemetry.update();
            return;
        }

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(1);

        limelight.start();
        waitForStart();

        while (opModeIsActive()) {
            processAprilTags();
            telemetry.update();
        }

        limelight.stop();
    }

    private void processAprilTags() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (!fiducialResults.isEmpty()) {
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    int tagId = fr.getFiducialId();
                    double xDegrees = fr.getTargetXDegrees();
                    double yDegrees = fr.getTargetYDegrees();

                    telemetry.addData("AprilTag ID", tagId);
                    telemetry.addData("Bot Pose", botpose.toString());
                    telemetry.addData("Relative Position", "X: %.2f, Y: %.2f", xDegrees, yDegrees);
                    telemetry.addData("Capture Latency", "%.2f ms", captureLatency);
                    telemetry.addData("Targeting Latency", "%.2f ms", targetingLatency);

                    logAprilTagValues(tagId, botpose, xDegrees, yDegrees);
                }
            } else {
                telemetry.addData("AprilTag", "No AprilTag detected.");
            }
        } else {
            telemetry.addData("Limelight", "No valid result.");
        }
    }
    private void logAprilTagValues(int tagId, Pose3D botpose, double xDegrees, double yDegrees) {
        telemetry.addData("Log - Tag ID", tagId);
        telemetry.addData("Log - Pose", botpose.toString());
        telemetry.addData("Log - Position", "X: %.2f, Y: %.2f", xDegrees, yDegrees);
    }
}
