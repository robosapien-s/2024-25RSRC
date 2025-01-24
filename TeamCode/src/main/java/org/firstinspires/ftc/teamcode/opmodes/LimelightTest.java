package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")
public class LimelightTest extends LinearOpMode {

    private Limelight3A limelight;

    public static double calculateRectangleAngle(List<List<Double>> points) {
        // Step 1: Compute all distances and group adjacent points
        List<double[]> sides = new ArrayList<>();
        for (int i = 0; i < points.size(); i++) {
            for (int j = i + 1; j < points.size(); j++) {
                double distance = Math.sqrt(Math.pow(points.get(i).get(0) - points.get(j).get(0), 2) +
                        Math.pow(points.get(i).get(1) - points.get(j).get(1), 2));
                sides.add(new double[] {i, j, distance}); // Store index pairs and their distance
            }
        }

        // Step 2: Sort by distance to identify sides and diagonals
        sides.sort((a, b) -> Double.compare(a[2], b[2]));

        // The two shortest distances are the sides; find the longer one
        double[] longerSide = sides.get(1); // Second shortest distance is the longer side

        // Step 3: Use the longer side to calculate the angle relative to the x-axis
        List<Double> p1 = points.get((int) longerSide[0]);
        List<Double> p2 = points.get((int) longerSide[1]);

        double deltaX = p2.get(0) - p1.get(0);
        double deltaY = p2.get(1) - p1.get(1);

        return Math.atan2(deltaY, deltaX); // Angle in radians
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        double lastResult = 0;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(2);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {

                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

                if(!colorResults.isEmpty()) {

                    LLResultTypes.ColorResult colorResult = colorResults.get(0);

                    List<List<Double>> corners =  colorResult.getTargetCorners();

                    if(corners.size() == 4) {
                        lastResult = calculateRectangleAngle(corners);



                }

                /*
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());

                    telemetry.addData("Botpose", botpose.toString());

                    // Access barcode results
                    List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                    for (LLResultTypes.BarcodeResult br : barcodeResults) {
                        telemetry.addData("Barcode", "Data: %s", br.getData());
                    }

                    // Access classifier results
                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                    }

                    // Access detector results
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }

                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    }
                }

                 */
            } else {
                telemetry.addData("Limelight", "No data available");
            }

                telemetry.addData("angle", "The angle of the rectangle relative to the x-axis (longer side) is: "
                        + Math.toDegrees(lastResult) + " degrees");
            }

            telemetry.update();
        }
        limelight.stop();
    }
}
