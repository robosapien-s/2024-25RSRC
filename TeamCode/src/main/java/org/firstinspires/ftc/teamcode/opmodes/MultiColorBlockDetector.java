package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Multi-Color Block Detector", group = "Vision")
public class MultiColorBlockDetector extends LinearOpMode {
    private OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        // Initialize webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        // Set pipeline
        ColorPipeline pipeline = new ColorPipeline();
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

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Blue Blocks:", pipeline.getBlueCount());
            telemetry.addData("Red Blocks:", pipeline.getRedCount());
            telemetry.addData("Yellow Blocks:", pipeline.getYellowCount());
            telemetry.update();
        }

        webcam.stopStreaming();
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
