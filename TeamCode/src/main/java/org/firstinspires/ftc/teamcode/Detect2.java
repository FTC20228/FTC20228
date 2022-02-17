package org.firstinspires.ftc.teamcode;

import android.content.pm.PackageInfo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvWebcam;

public class Detect2 extends OpenCvPipeline {
    private OpenCvWebcam cam;
    private Telemetry telemetry;
    private Mat mat = new Mat();
    private Mat leftMat;
    private Mat rightMat;
    private Target target;
    private static final Rect LEFT_ROI = new Rect(new Point(30, 30), new Point(50, 50));
    private static final Rect RIGHT_ROI = new Rect(new Point(50, 30), new Point(70, 50));

    public Detect2(HardwareMap hwMap, Telemetry t) {
        telemetry = t;
        int camMonViewId = hwMap.appContext.getResources().getIdentifier(
                "camera monitor view id",
                "id",
                hwMap.appContext.getPackageName()
        );
        cam = OpenCvCameraFactory.getInstance().createWebcam(
                hwMap.get(WebcamName.class, "Webcam 1"),
                camMonViewId
        );
        cam.setPipeline(this);
        cam.openCameraDeviceAsync(
                ()->cam.startStreaming(320, 420, OpenCvCameraRotation.UPRIGHT)
        );
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(19, 100, 100);
        Scalar highHSV = new Scalar(25, 255, 255);
        Core.inRange(mat, lowHSV, highHSV, mat);

        leftMat = mat.submat(LEFT_ROI);
        rightMat = mat.submat(RIGHT_ROI);

        double leftValue = Math.round(Core.mean(leftMat).val[2] / 255);
        double rightValue = Math.round(Core.mean(rightMat).val[2] / 255);

        leftMat.release();
        rightMat.release();

        mat.release();


        final double THRESHOLD = 10;

        if (leftValue > THRESHOLD) {
            target = Target.A;
        } else if (rightValue > THRESHOLD) {
            target = Target.B;
        } else {
            target = Target.C;
        }


        return null;


    }

    public Target getTarget() {
        return target;
    }

    public void stop() {
        cam.closeCameraDeviceAsync(() -> {
        });
    }
}

