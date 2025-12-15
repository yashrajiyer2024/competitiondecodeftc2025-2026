package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class BallColumnPipeline extends OpenCvPipeline {

    public int columnCenterX = -1;   // horizontal position of column center
    public boolean foundColumn = false;
    public int error = 0;            // output: columnCenterX - imageCenterX

    // HSV ranges — tune these for your lighting
    private static final Scalar PURPLE_LOW  = new Scalar(125, 70, 70);
    private static final Scalar PURPLE_HIGH = new Scalar(155, 255, 255);

    private static final Scalar GREEN_LOW   = new Scalar(40, 70, 70);
    private static final Scalar GREEN_HIGH  = new Scalar(85, 255, 255);

    private final int IMG_WIDTH = 640;    // must match your streaming resolution

    @Override
    public Mat processFrame(Mat input) {

        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat purpleMask = new Mat();
        Mat greenMask  = new Mat();
        Mat combined   = new Mat();

        Core.inRange(hsv, PURPLE_LOW, PURPLE_HIGH, purpleMask);
        Core.inRange(hsv, GREEN_LOW, GREEN_HIGH, greenMask);

        Core.bitwise_or(purpleMask, greenMask, combined);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(combined, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Rect> balls = new ArrayList<>();

        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area < 150) continue;   // ignore noise

            Rect r = Imgproc.boundingRect(c);
            balls.add(r);

            // Debug draw
            Imgproc.rectangle(input, r, new Scalar(0, 255, 0), 2);
        }

        if (balls.size() < 2) {
            foundColumn = false;
            columnCenterX = -1;
            error = 0;
            return input;
        }

        // Sort by x center
        Collections.sort(balls, (a, b) ->
                (a.x + a.width / 2) - (b.x + b.width / 2));

        // Identify the vertical column by grouping similar X positions
        List<Rect> column = new ArrayList<>();
        for (Rect r : balls) {
            if (column.isEmpty()) {
                column.add(r);
            } else {
                int cx = r.x + r.width/2;
                int baseCx = column.get(0).x + column.get(0).width/2;

                if (Math.abs(cx - baseCx) < 30) {  // 30 px tolerance
                    column.add(r);
                }
            }
        }

        if (column.size() >= 2) {
            foundColumn = true;

            int sum = 0;
            for (Rect r : column) {
                int cx = r.x + r.width/2;
                sum += cx;

                Imgproc.circle(input, new Point(cx, r.y + r.height/2), 5, new Scalar(255, 0, 0), -1);
            }

            columnCenterX = sum / column.size();

            // Draw column centerline
            Imgproc.line(input,
                    new Point(columnCenterX, 0),
                    new Point(columnCenterX, input.height()),
                    new Scalar(0, 0, 255), 2);

            // Compute error = column center - image center
            int imageCenter = IMG_WIDTH / 2; // 640 → 320
            error = columnCenterX - imageCenter;
        }
        else {
            foundColumn = false;
            columnCenterX = -1;
            error = 0;
        }

        return input;
    }
}