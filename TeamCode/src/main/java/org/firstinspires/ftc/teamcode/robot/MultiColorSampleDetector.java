package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class MultiColorSampleDetector {
    private final OpenCvCamera webcam;
    private final Telemetry telemetry;
    public final ColorPipeline pipeline;
    public static String lastDetectedColor = "None";
    private HardwareMap hardwareMap;

    public MultiColorSampleDetector(HardwareMap hardwareMap, Telemetry inTelemetry) {
        this.hardwareMap = hardwareMap;
        telemetry = inTelemetry;
        // Initialize webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        // Set pipeline
        pipeline = new ColorPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera failed to open.");
                telemetry.update();
            }
        });
    }

    public void stopStreaming() {
        webcam.stopStreaming();
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public String getLastDetectedColor() {
        return lastDetectedColor;
    }

    public Rect detectClosestSample() {
        if (pipeline != null) {
            return pipeline.getClosestSample();
        }
        return null;
    }

    public void getCounts() {
        telemetry.addData("Blue Blocks:", pipeline.getBlueCount());
        telemetry.addData("Red Blocks:", pipeline.getRedCount());
        telemetry.addData("Yellow Blocks:", pipeline.getYellowCount());
        telemetry.update();
    }

    public static class ColorPipeline extends OpenCvPipeline {
        private int blueCount = 0, redCount = 0, yellowCount = 0;
        private Rect closestSample = null;
        private List<MatOfPoint> blueContours = new ArrayList<>();
        private List<MatOfPoint> redContours = new ArrayList<>();
        private List<MatOfPoint> yellowContours = new ArrayList<>();

        public int getBlueCount() {
            return blueCount;
        }

        public int getRedCount() {
            return redCount;
        }

        public int getYellowCount() {
            return yellowCount;
        }

        public Rect getClosestSample() {
            return closestSample;
        }

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat maskBlue = new Mat(), maskRed = new Mat(), maskYellow = new Mat();
            Mat output = input.clone();

            // Clear previous contours
            blueContours.clear();
            redContours.clear();
            yellowContours.clear();

            // Convert RGB to HSV color space
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define color ranges in HSV
            Scalar lowerBlue = new Scalar(100, 150, 50), upperBlue = new Scalar(140, 255, 255);
            Scalar lowerRed1 = new Scalar(0, 150, 50), upperRed1 = new Scalar(10, 255, 255);
            Scalar lowerRed2 = new Scalar(170, 150, 50), upperRed2 = new Scalar(180, 255, 255);
            Scalar lowerYellow = new Scalar(20, 150, 100), upperYellow = new Scalar(30, 255, 255);

            // Create color masks
            Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);
            Core.inRange(hsv, lowerRed1, upperRed1, maskRed);
            Mat maskRed2 = new Mat();
            Core.inRange(hsv, lowerRed2, upperRed2, maskRed2);
            Core.bitwise_or(maskRed, maskRed2, maskRed);
            Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

            // Find and label contours for each color
            blueCount = detectAndLabel(maskBlue, output, "Blue", new Scalar(0, 255, 0), blueContours);
            redCount = detectAndLabel(maskRed, output, "Red", new Scalar(255, 0, 0), redContours);
            yellowCount = detectAndLabel(maskYellow, output, "Yellow", new Scalar(0, 0, 255), yellowContours);

            // Find sample closest to center
            findClosestSample(output);

            // Release memory
            hsv.release();
            maskBlue.release();
            maskRed.release();
            maskRed2.release();
            maskYellow.release();

            return output;
        }

        private void findClosestSample(Mat output) {
            // Define center of frame
            double centerX = output.cols() / 2.0;
            double centerY = output.rows() / 2.0;

            double closestDistance = Double.MAX_VALUE;
            Rect closestRect = null;
            String closestColor = "None";

            // Process blue contours
            Rect blueRect = findClosestInContours(blueContours, centerX, centerY);
            if (blueRect != null) {
                double blueDistance = getDistanceToCenter(blueRect, centerX, centerY);
                if (blueDistance < closestDistance) {
                    closestDistance = blueDistance;
                    closestRect = blueRect;
                    closestColor = "Blue";
                }
            }

            // Process red contours
            Rect redRect = findClosestInContours(redContours, centerX, centerY);
            if (redRect != null) {
                double redDistance = getDistanceToCenter(redRect, centerX, centerY);
                if (redDistance < closestDistance) {
                    closestDistance = redDistance;
                    closestRect = redRect;
                    closestColor = "Red";
                }
            }

            // Process yellow contours
            Rect yellowRect = findClosestInContours(yellowContours, centerX, centerY);
            if (yellowRect != null) {
                double yellowDistance = getDistanceToCenter(yellowRect, centerX, centerY);
                if (yellowDistance < closestDistance) {
                    closestDistance = yellowDistance;
                    closestRect = yellowRect;
                    closestColor = "Yellow";
                }
            }

            // Update closest sample and color
            this.closestSample = closestRect;
            lastDetectedColor = closestColor;

            // Draw indicator for closest sample
            if (closestRect != null) {
                Scalar highlightColor = new Scalar(255, 255, 255); // White
                Imgproc.rectangle(output, closestRect, highlightColor, 5);
                Imgproc.putText(output, "TARGET",
                        new Point(closestRect.x, closestRect.y - 20),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, highlightColor, 2);

                // Draw line from center to target
                Imgproc.line(output,
                        new Point(centerX, centerY),
                        new Point(closestRect.x + closestRect.width/2, closestRect.y + closestRect.height/2),
                        highlightColor, 2);
            }
        }

        private Rect findClosestInContours(List<MatOfPoint> contours, double centerX, double centerY) {
            if (contours.isEmpty()) return null;

            Rect closestRect = null;
            double minDistance = Double.MAX_VALUE;

            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                double distance = getDistanceToCenter(rect, centerX, centerY);

                if (distance < minDistance && rect.area() > 500) {
                    minDistance = distance;
                    closestRect = rect;
                }
            }

            return closestRect;
        }

        private double getDistanceToCenter(Rect rect, double centerX, double centerY) {
            double rectCenterX = rect.x + rect.width / 2.0;
            double rectCenterY = rect.y + rect.height / 2.0;

            return Math.sqrt(
                    Math.pow(rectCenterX - centerX, 2) +
                            Math.pow(rectCenterY - centerY, 2)
            );
        }

        private int detectAndLabel(Mat mask, Mat output, String label, Scalar color, List<MatOfPoint> contours) {
            // Clear previous contours for this color
            contours.clear();

            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            int count = 0;
            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                if (rect.area() > 500) {
                    double midX = rect.x + rect.width / 2.0;
                    double midY = rect.y + rect.height / 2.0;
                    Imgproc.rectangle(output, rect, color, 3);
                    Imgproc.putText(output, label, new Point(rect.x, rect.y - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, color, 2);

                    // Add distance from center
                    double centerX = output.cols() / 2.0;
                    double centerY = output.rows() / 2.0;
                    double distance = Math.sqrt(Math.pow(midX - centerX, 2) + Math.pow(midY - centerY, 2));
                    Imgproc.putText(output, String.format("D: %.0f", distance),
                            new Point(rect.x, rect.y + rect.height + 15),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

                    count++;
                }
            }
            hierarchy.release();
            return count;
        }
    }
}