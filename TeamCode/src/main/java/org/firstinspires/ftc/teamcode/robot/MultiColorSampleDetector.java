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

public class MultiColorSampleDetector {

    final private OpenCvCamera webcam;
    final private Telemetry telemetry;

    ClosestSamplePipeline pipeline;

    public MultiColorSampleDetector(HardwareMap hardwareMap, Telemetry inTelemetry, ClosestSamplePipeline.SampleColorPriority colorPriority) {
        telemetry = inTelemetry;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        // Set pipeline
        //pipeline = new ColorPipeline();
        pipeline = new ClosestSamplePipeline(colorPriority);

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                CameraStreamServer.getInstance().setSource(webcam);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera failed to open.");
                telemetry.update();
            }
        });
    }

    public void getCounts() {
//        telemetry.addData("Blue Blocks:", pipeline.getBlueCount());
//        telemetry.addData("Red Blocks:", pipeline.getRedCount());
//        telemetry.addData("Yellow Blocks:", pipeline.getYellowCount());
//        telemetry.update();
    }

    public RotatedRect getClosestSample() {
        telemetry.addData("Closest Sample:", pipeline.getClosestSample());
        telemetry.update();
        return pipeline.getClosestSample();
    }

    public Point getCenterOfScreen() {
        return new Point(320, 240);
    }

    public void stopStreaming() {
        webcam.stopStreaming();
    }


    public static class ClosestSamplePipeline extends OpenCvPipeline {

        RotatedRect closestSample = new RotatedRect();
        RotatedRect closestRotatedSample = new RotatedRect();


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

        public ClosestSamplePipeline(SampleColorPriority inSamplePriority) {
            samplePriority = inSamplePriority;
        }

        public RotatedRect getClosestSample() {
            return closestSample;
        }

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat maskBlue = new Mat(), maskRed = new Mat(), maskYellow = new Mat();
            Mat output = input.clone();

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lowerBlue = new Scalar(100, 150, 50), upperBlue = new Scalar(140, 255, 255);
            Scalar lowerRed1 = new Scalar(0, 150, 50), upperRed1 = new Scalar(10, 255, 255);
            Scalar lowerRed2 = new Scalar(170, 150, 50), upperRed2 = new Scalar(180, 255, 255);
            Scalar lowerYellow = new Scalar(20, 150, 100), upperYellow = new Scalar(30, 255, 255);

            Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);
            Core.inRange(hsv, lowerRed1, upperRed1, maskRed);
            Mat maskRed2 = new Mat();
            Core.inRange(hsv, lowerRed2, upperRed2, maskRed2);
            Core.bitwise_or(maskRed, maskRed2, maskRed);
            Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

            closestSample = new RotatedRect();

            if(samplePriority == SampleColorPriority.all ||
                    samplePriority == SampleColorPriority.yellow ||
                    samplePriority == SampleColorPriority.yellow_or_red ||
                    samplePriority == SampleColorPriority.yellow_or_blue ||
                    samplePriority == SampleColorPriority.yellow_then_red ||
                    samplePriority == SampleColorPriority.yellow_then_blue ) {


                closestSample = detectAndLabel(maskYellow, output, "Yellow", new Scalar(0, 0, 255));


            }

            if(samplePriority == SampleColorPriority.all ||
                    samplePriority == SampleColorPriority.yellow_or_red ||
                    (samplePriority == SampleColorPriority.yellow_then_red && closestSample.boundingRect().area() == 0 )
            ) {

                RotatedRect redClosestRect = detectAndLabel(maskRed, output, "red", new Scalar(255, 0, 0));

                if( calculateDistanceToCenter(redClosestRect) < calculateDistanceToCenter(closestSample)) {
                    closestSample = redClosestRect;
                }
            }


            if(samplePriority == SampleColorPriority.all ||
                    samplePriority == SampleColorPriority.yellow_or_blue ||
                    (samplePriority == SampleColorPriority.yellow_then_blue && closestSample.boundingRect().area() == 0 )
            ) {

                RotatedRect blueClosestRect = detectAndLabel(maskBlue, output, "blue", new Scalar(0, 255, 0));

                if( calculateDistanceToCenter(blueClosestRect) < calculateDistanceToCenter(closestSample)) {
                    closestSample = blueClosestRect;
                }
            }

            Imgproc.rectangle(output, closestSample.boundingRect(),  new Scalar(255, 255, 255), 3);
            Imgproc.putText(output, "center: " + String.valueOf(closestSample.angle), new Point(closestSample.boundingRect().x, closestSample.boundingRect().y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);


            // Release memory
            hsv.release();
            maskBlue.release();
            maskRed.release();
            maskRed2.release();
            maskYellow.release();

            return output;
        }

        public double calculateDistance(Point p1, Point p2) {
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            return Math.sqrt(dx * dx + dy * dy);
        }

        public double calculateDistanceToCenter(RotatedRect inRect) {
            Point centerOfScreen = new Point(320, 240);
            return  calculateDistance(centerOfScreen, inRect.center);
        }

        private RotatedRect detectAndLabel(Mat mask, Mat output, String label, Scalar color) { List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            RotatedRect clostestRect = new RotatedRect();

            for (MatOfPoint contour : contours) {
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

                if (rect.boundingRect().area() > 15000) {

                    if( calculateDistanceToCenter(rect) < calculateDistanceToCenter(clostestRect)) {
                        clostestRect = rect;
                    }

                    Imgproc.rectangle(output, rect.boundingRect(), color, 3);
                    Imgproc.putText(output, label + " - " + String.valueOf(rect.boundingRect().area()), new Point(rect.boundingRect().x, rect.boundingRect().y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, color, 2);

                }
            }
            hierarchy.release();
            return clostestRect;
        }
    }

    static class ColorPipeline extends OpenCvPipeline {
        private int blueCount = 0, redCount = 0, yellowCount = 0;

        public int getBlueCount() { return blueCount; }
        public int getRedCount() { return redCount; }
        public int getYellowCount() { return yellowCount; }

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat maskBlue = new Mat(), maskRed = new Mat(), maskYellow = new Mat();
            Mat output = input.clone();

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lowerBlue = new Scalar(100, 150, 50), upperBlue = new Scalar(140, 255, 255);
            Scalar lowerRed1 = new Scalar(0, 150, 50), upperRed1 = new Scalar(10, 255, 255);
            Scalar lowerRed2 = new Scalar(170, 150, 50), upperRed2 = new Scalar(180, 255, 255);
            Scalar lowerYellow = new Scalar(20, 150, 100), upperYellow = new Scalar(30, 255, 255);

            Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);
            Core.inRange(hsv, lowerRed1, upperRed1, maskRed);
            Mat maskRed2 = new Mat();
            Core.inRange(hsv, lowerRed2, upperRed2, maskRed2);
            Core.bitwise_or(maskRed, maskRed2, maskRed);
            Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

            blueCount = detectAndLabel(maskBlue, output, "Blue", new Scalar(0, 255, 0));
            redCount = detectAndLabel(maskRed, output, "Red", new Scalar(255, 0, 0));
            yellowCount = detectAndLabel(maskYellow, output, "Yellow", new Scalar(0, 0, 255));

            // Release memory
            hsv.release();
            maskBlue.release();
            maskRed.release();
            maskRed2.release();
            maskYellow.release();

            return output;
        }

        private int detectAndLabel(Mat mask, Mat output, String label, Scalar color) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            int count = 0;
            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                if (rect.area() > 500) {
                    Imgproc.rectangle(output, rect, color, 3);
                    Imgproc.putText(output, label, new Point(rect.x, rect.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
                    count++;
                }
            }
            hierarchy.release();
            return count;
        }
    }
}
