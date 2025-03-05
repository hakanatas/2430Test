package org.firstinspires.ftc.teamcode.opmode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class limelight extends OpenCvPipeline {

    private Scalar lowerBound;
    private Scalar upperBound;
    private Point blockCenter = new Point(-1, -1);
    private double blockAngle = 0;
    private double blockY = 0;
    private List<Rect> detectedBlocks = new ArrayList<>();

    public limelight(String color) {
        switch (color.toLowerCase()) {
            case "red":
                lowerBound = new Scalar(0, 120, 70);
                upperBound = new Scalar(10, 255, 255);
                break;
            case "blue":
                lowerBound = new Scalar(100, 150, 50);
                upperBound = new Scalar(140, 255, 255);
                break;
            case "yellow":
                lowerBound = new Scalar(20, 150, 100);
                upperBound = new Scalar(30, 255, 255);
                break;
            default:
                lowerBound = new Scalar(0, 0, 0);
                upperBound = new Scalar(0, 0, 0);
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat mask = new Mat();
        Core.inRange(hsv, lowerBound, upperBound, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        detectedBlocks.clear();
        double maxArea = 0;
        Rect largestBlock = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > 500) { // Filter out small noise
                Rect rect = Imgproc.boundingRect(contour);
                detectedBlocks.add(rect);
                if (area > maxArea) {
                    maxArea = area;
                    largestBlock = rect;
                }
            }
        }

        for (Rect rect : detectedBlocks) {
            Imgproc.rectangle(input, rect, new Scalar(255, 0, 0), 2); // Draw bounding box around all detected blocks
        }

        if (largestBlock != null) {
            blockCenter = new Point(largestBlock.x + largestBlock.width / 2.0, largestBlock.y + largestBlock.height / 2.0);
            blockY = 1000.0 / largestBlock.width; // Approximate forward/backward distance based on block size
            blockAngle = Math.toDegrees(Math.atan2(blockCenter.x - (input.width() / 2), blockY));
        } else {
            blockCenter = new Point(-1, -1);
            blockY = 0;
        }

        hsv.release();
        mask.release();
        hierarchy.release();

        return input;
    }

    public Point getBlockCenter() { return blockCenter; }
    public double getBlockAngle() { return blockAngle; }
    public double getBlockY() { return blockY; }
}
