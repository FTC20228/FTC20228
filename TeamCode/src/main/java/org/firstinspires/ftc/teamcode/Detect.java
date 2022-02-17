package org.firstinspires.ftc.teamcode;

import android.content.pm.PackageInfo;

import org.opencv.core.Core;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Detect extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        MID
    }
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(30, 30 ),
            new Point(50, 50 ));
    static final Rect MID_ROI = new Rect(
            new Point( 10, 30),
            new Point( 30, 50));
    static final Rect RIGHT_ROI = new Rect(
            new Point( 50, 30),
            new Point( 70, 50));
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public Detect(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(19, 100, 100 );
        Scalar highHSV = new Scalar(25, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat mid = mat.submat(MID_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double midValue = Core.sumElems(mid).val[0] / MID_ROI.area() / 255;


        left.release();
        right.release();
        mid.release();
        mat.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[2] / 255);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[2] / 255);
        telemetry.addData("Mid raw value", (int) Core.sumElems(mid).val[2] / 255);


        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData( "Mid percentage", Math.round(midValue * 100) + "%");

        boolean cupLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean cupRight = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean cupMid = midValue > PERCENT_COLOR_THRESHOLD;

        if(cupLeft) {
        location = Location.LEFT;
        telemetry.addData("Cup Location", "left");

        }
        else if(cupRight) {
        location = Location.RIGHT;
        telemetry.addData("Cup Location", "right");

        }
        else{
        location = Location.MID;
        telemetry.addData("Cup Location", "mid");

        }

        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);

        Scalar colorCup = new Scalar(255, 0, 0);
        Scalar colorNup = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorNup:colorCup);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorNup:colorCup);
        Imgproc.rectangle(mat, MID_ROI, location == Location.MID? colorNup:colorCup);

        return mat;


    }
}
