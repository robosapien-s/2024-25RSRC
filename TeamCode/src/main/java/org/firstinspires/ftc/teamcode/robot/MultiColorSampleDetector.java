package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class MultiColorSampleDetector {

    private final OpenCvCamera webcam;
    private final Telemetry telemetry;
    private AtomicBoolean cameraInitialized = new AtomicBoolean(false);
    private AtomicBoolean pipelineReady = new AtomicBoolean(false);

    ClosestSamplePipeline pipeline;

    public MultiColorSampleDetector(HardwareMap hardwareMap, Telemetry inTelemetry, ClosestSamplePipeline.SampleColorPriority colorPriority) {
        telemetry = inTelemetry;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        pipeline = new ClosestSamplePipeline(colorPriority);

        webcam.setPipeline(pipeline);

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);


        initializeCameraAsync();
    }


    private void initializeCameraAsync() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                CameraStreamServer.getInstance().setSource(webcam);
                cameraInitialized.set(true);

                new Thread(() -> {
                    try {
                        Thread.sleep(500);
                        pipelineReady.set(true);
                    } catch (InterruptedException e) {
                    }
                }).start();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera failed to open: Error " + errorCode);
                telemetry.update();
                cameraInitialized.set(false);
            }
        });
    }


    public RotatedRect getClosestSample() {
        if (!cameraInitialized.get()) {
            tryRecoverCamera();
            return new RotatedRect();
        }

        if (!pipelineReady.get()) {
            telemetry.addData("Camera Status", "Initializing, please wait...");
            telemetry.update();
            return new RotatedRect();
        }

        return pipeline.getClosestSample();
    }

    public Point getCenterOfScreen() {
        return new Point(160, 120);
    }

    private void tryRecoverCamera() {
        if (!cameraInitialized.get()) {
            telemetry.addData("Camera Status", "Trying to recover...");
            telemetry.update();
            initializeCameraAsync();
        }
    }

    public void stopStreaming() {
        if (cameraInitialized.get()) {
            try {
                webcam.stopStreaming();
                webcam.closeCameraDevice();
            } catch (Exception e) {
                telemetry.addData("Camera Error", "Error closing camera: " + e.getMessage());
                telemetry.update();
            }
        }
        pipeline.dispose();
    }

    public static class ClosestSamplePipeline extends OpenCvPipeline {
        private final Mat hsv = new Mat();
        private final Mat maskBlue = new Mat();
        private final Mat maskRed = new Mat();
        private final Mat maskRed2 = new Mat();
        private final Mat maskYellow = new Mat();

        private int frameCounter = 0;
        private static final int FRAMES_TO_SKIP = 1;
        private long lastProcessingTime = 0;
        private boolean simplifiedMode = false;

        private RotatedRect closestSample = new RotatedRect();
        private final Object resultLock = new Object();

        public enum SampleColorPriority {
            all,
            blue,
            red,
            yellow,
            yellow_or_red,
            yellow_then_red,
            yellow_or_blue,
            yellow_then_blue;
        }

        SampleColorPriority samplePriority = SampleColorPriority.yellow;

        private static final Scalar LOWER_BLUE = new Scalar(100, 150, 50);
        private static final Scalar UPPER_BLUE = new Scalar(140, 255, 255);
        private static final Scalar LOWER_RED1 = new Scalar(0, 150, 50);
        private static final Scalar UPPER_RED1 = new Scalar(10, 255, 255);
        private static final Scalar LOWER_RED2 = new Scalar(170, 150, 50);
        private static final Scalar UPPER_RED2 = new Scalar(180, 255, 255);
        private static final Scalar LOWER_YELLOW = new Scalar(20, 150, 100);
        private static final Scalar UPPER_YELLOW = new Scalar(30, 255, 255);

        public ClosestSamplePipeline(SampleColorPriority inSamplePriority) {
            samplePriority = inSamplePriority;
        }

        public RotatedRect getClosestSample() {
            synchronized (resultLock) {
                return closestSample;
            }
        }


        public void dispose() {
            hsv.release();
            maskBlue.release();
            maskRed.release();
            maskRed2.release();
            maskYellow.release();
        }

        @Override
        public Mat processFrame(Mat input) {
            long startTime = System.currentTimeMillis();

            frameCounter++;
            if (frameCounter % (FRAMES_TO_SKIP + 1) != 0) {
                return input;
            }

            if (lastProcessingTime > 50) {
                simplifiedMode = true;
            } else if (lastProcessingTime < 30) {
                simplifiedMode = false;
            }

            Mat output = input.clone();

            Rect roi = new Rect(input.width()/4, input.height()/4, input.width()/2, input.height()/2);
            Mat roiMat = new Mat(input, roi);

            Imgproc.cvtColor(simplifiedMode ? roiMat : input, hsv, Imgproc.COLOR_RGB2HSV);

            if(shouldProcessColor(SampleColorPriority.yellow)) {
                Core.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW, maskYellow);
            }

            if(shouldProcessColor(SampleColorPriority.red)) {
                Core.inRange(hsv, LOWER_RED1, UPPER_RED1, maskRed);
                Core.inRange(hsv, LOWER_RED2, UPPER_RED2, maskRed2);
                Core.bitwise_or(maskRed, maskRed2, maskRed);
            }

            if(shouldProcessColor(SampleColorPriority.blue)) {
                Core.inRange(hsv, LOWER_BLUE, UPPER_BLUE, maskBlue);
            }

            RotatedRect newClosestSample = new RotatedRect();

            if(shouldProcessColor(SampleColorPriority.yellow)) {
                newClosestSample = detectAndLabel(maskYellow, output, "Yellow", new Scalar(0, 0, 255), simplifiedMode ? roi : null);
            }

            if(shouldProcessColor(SampleColorPriority.red) &&
                    (samplePriority == SampleColorPriority.all ||
                            samplePriority == SampleColorPriority.yellow_or_red ||
                            (samplePriority == SampleColorPriority.yellow_then_red && newClosestSample.boundingRect().empty()))) {

                RotatedRect redClosestRect = detectAndLabel(maskRed, output, "red", new Scalar(255, 0, 0), simplifiedMode ? roi : null);

                if(calculateDistanceToCenter(redClosestRect) < calculateDistanceToCenter(newClosestSample) ||
                        newClosestSample.boundingRect().empty()) {
                    newClosestSample = redClosestRect;
                }
            }

            if(shouldProcessColor(SampleColorPriority.blue) &&
                    (samplePriority == SampleColorPriority.all ||
                            samplePriority == SampleColorPriority.yellow_or_blue ||
                            (samplePriority == SampleColorPriority.yellow_then_blue && newClosestSample.boundingRect().empty()))) {

                RotatedRect blueClosestRect = detectAndLabel(maskBlue, output, "blue", new Scalar(0, 255, 0), simplifiedMode ? roi : null);

                if(calculateDistanceToCenter(blueClosestRect) < calculateDistanceToCenter(newClosestSample) ||
                        newClosestSample.boundingRect().empty()) {
                    newClosestSample = blueClosestRect;
                }
            }

            if(!newClosestSample.boundingRect().empty()) {
                Imgproc.rectangle(output, newClosestSample.boundingRect(), new Scalar(255, 255, 255), 3);
                String angleText = String.format("angle: %.1f", newClosestSample.angle);
                Imgproc.putText(output, angleText,
                        new Point(newClosestSample.boundingRect().x, newClosestSample.boundingRect().y - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
            }

            synchronized (resultLock) {
                closestSample = newClosestSample;
            }

            lastProcessingTime = System.currentTimeMillis() - startTime;

            if (simplifiedMode) {
                roiMat.release();
            }

            return output;
        }

        private boolean shouldProcessColor(SampleColorPriority color) {
            if (samplePriority == SampleColorPriority.all) {
                return true;
            }

            switch (color) {
                case yellow:
                    return samplePriority == SampleColorPriority.yellow ||
                            samplePriority == SampleColorPriority.yellow_or_red ||
                            samplePriority == SampleColorPriority.yellow_or_blue ||
                            samplePriority == SampleColorPriority.yellow_then_red ||
                            samplePriority == SampleColorPriority.yellow_then_blue;
                case red:
                    return samplePriority == SampleColorPriority.red ||
                            samplePriority == SampleColorPriority.yellow_or_red ||
                            samplePriority == SampleColorPriority.yellow_then_red;
                case blue:
                    return samplePriority == SampleColorPriority.blue ||
                            samplePriority == SampleColorPriority.yellow_or_blue ||
                            samplePriority == SampleColorPriority.yellow_then_blue;
                default:
                    return false;
            }
        }

        public double calculateDistance(Point p1, Point p2) {
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            return Math.sqrt(dx * dx + dy * dy);
        }

        public double calculateDistanceToCenter(RotatedRect inRect) {
            if (inRect.boundingRect().empty()) {
                return Double.MAX_VALUE;
            }
            Point centerOfScreen = new Point(160, 120);
            return calculateDistance(centerOfScreen, inRect.center);
        }


        private RotatedRect detectAndLabel(Mat mask, Mat output, String label, Scalar color, Rect roi) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_TC89_KCOS);

            RotatedRect closestRect = new RotatedRect();
            double closestDistance = Double.MAX_VALUE;

            if (contours.isEmpty()) {
                hierarchy.release();
                return closestRect;
            }

            Point centerOfScreen = new Point(160, 120);
            final int MIN_CONTOUR_AREA = 100;
            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) < MIN_CONTOUR_AREA) {
                    continue;
                }

                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                RotatedRect rect = Imgproc.minAreaRect(contour2f);
                contour2f.release();

                if (roi != null) {
                    rect.center.x += roi.x;
                    rect.center.y += roi.y;
                }

                if (rect.boundingRect().area() <= 10) {
                    continue;
                }

                double distance = calculateDistance(centerOfScreen, rect.center);

                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestRect = rect;
                }

                Imgproc.rectangle(output, rect.boundingRect(), color, 2);
                Imgproc.putText(output,
                        label + " - " + rect.boundingRect().area(),
                        new Point(rect.boundingRect().x, rect.boundingRect().y - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            }

            hierarchy.release();
            return closestRect;
        }
    }
}