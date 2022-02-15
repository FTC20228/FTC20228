package org.firstinspires.ftc.teamcode;

import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class Detect extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Math();
    public Detect(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat inpput) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

    }

}
