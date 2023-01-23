package frc.robot.Vision;

import java.util.HashSet;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;

public class AprilTagReader {
    private AprilTagDetection[] results = new AprilTagDetection[] {};

    public AprilTagReader(int cameraWidth, int cameraHeight) {
        var th = new Thread(() -> {
            var camera = CameraServer.startAutomaticCapture();

            camera.setResolution(cameraWidth, cameraHeight);

            var cvSink = CameraServer.getVideo();
            var outputStream = CameraServer.putVideo("RoboRioApriltags", cameraWidth, cameraHeight);

            var mat = new Mat();
            var grayMat = new Mat();

            var pt0 = new Point();
            var pt1 = new Point();
            var pt2 = new Point();
            var pt3 = new Point();
            var center = new Point();
            var red = new Scalar(0, 0, 255);
            var green = new Scalar(0, 255, 0);

            var aprilTagDetector = new AprilTagDetector();

            var config = aprilTagDetector.getConfig();
            config.quadSigma = 0.8f;
            aprilTagDetector.setConfig(config);

            var quadThreshParams = aprilTagDetector.getQuadThresholdParameters();
            quadThreshParams.minClusterPixels = 250;
            quadThreshParams.criticalAngle *= 5; // default is 10
            quadThreshParams.maxLineFitMSE *= 1.5;
            aprilTagDetector.setQuadThresholdParameters(quadThreshParams);

            aprilTagDetector.addFamily("tag16h5");

            var timer = new Timer();
            timer.start();

            while (!Thread.interrupted()) {
                if (cvSink.grabFrame(mat) == 0) {
                    outputStream.notifyError(cvSink.getError());
                    continue;
                }

                Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

                results = aprilTagDetector.detect(grayMat);

                var set = new HashSet<Integer>();

                for (var result : results) {
                    pt0.x = result.getCornerX(0);
                    pt1.x = result.getCornerX(1);
                    pt2.x = result.getCornerX(2);
                    pt3.x = result.getCornerX(3);

                    pt0.y = result.getCornerY(0);
                    pt1.y = result.getCornerY(1);
                    pt2.y = result.getCornerY(2);
                    pt3.y = result.getCornerY(3);

                    center.x = result.getCenterX();
                    center.y = result.getCenterY();

                    set.add(result.getId());

                    Imgproc.line(mat, pt0, pt1, red, 5);
                    Imgproc.line(mat, pt1, pt2, red, 5);
                    Imgproc.line(mat, pt2, pt3, red, 5);
                    Imgproc.line(mat, pt3, pt0, red, 5);

                    Imgproc.circle(mat, center, 4, green);
                    Imgproc.putText(mat, String.valueOf(result.getId()), pt2, Imgproc.FONT_HERSHEY_SIMPLEX, 2, green, 7);
                }

                outputStream.putFrame(mat);

            }
            aprilTagDetector.close();
        });
        th.setDaemon(true);
        th.start();
    }
}
