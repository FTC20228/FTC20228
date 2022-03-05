package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="LeftAutoBlue", group="Autonomous")

public class LeftAutoBlue extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    static final double COUNTS_PER_MOTOR_REV = 537.7; // eg: TETRIX Motor Encoder
    static final double ARM_COUNTS_PER_DEGREE = (1440 / 360 * 0.5);
    static final double DRIVE_GEAR_REDUCTION = 1.0; // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DS = 0.6;
    static final double TURN_SPEED = 0.6;


    private ElapsedTime runtime = new ElapsedTime();
    //Define Gyro

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    private BNO055IMU imu;
    double firstAngles = 0;


    // Define Webcam
    OpenCvCamera webcam;

    // Create Pipeline
    static OpenCV_Pipeline pipeline;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders"); //
        telemetry.update();

        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


// Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.frontLeftDrive.getCurrentPosition(),
                robot.backRightDrive.getCurrentPosition());
        telemetry.update();

        // Set up webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set up pipeline
        pipeline = new OpenCV_Pipeline();
        webcam.setPipeline(pipeline);

        // Start camera streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        telemetry.addLine("Waiting for start");



        /* Telemetry readings for the HSV values in each region
            telemetry.addData("Region 1", "%7d, %7d, %7d", pipeline.HSV_Value_1[0], pipeline.HSV_Value_1[1], pipeline.HSV_Value_1[2]);
            telemetry.addData("Region 2", "%7d, %7d, %7d", pipeline.HSV_Value_2[0], pipeline.HSV_Value_2[1], pipeline.HSV_Value_2[2]);
            telemetry.addData("Region 3", "%7d, %7d, %7d", pipeline.HSV_Value_3[0], pipeline.HSV_Value_3[1], pipeline.HSV_Value_3[2]);

            //telemetry.update();

            if ((pipeline.HSV_Value_1[0] < pipeline.HSV_Value_2[0]) && (pipeline.HSV_Value_1[0] < pipeline.HSV_Value_3[0])){
                telemetry.addLine("Left");
            }
            if ((pipeline.HSV_Value_2[0] < pipeline.HSV_Value_1[0]) && (pipeline.HSV_Value_2[0] < pipeline.HSV_Value_3[0])){
                telemetry.addLine("Mid");
            }
            if ((pipeline.HSV_Value_3[0] < pipeline.HSV_Value_1[0]) && (pipeline.HSV_Value_3[0] < pipeline.HSV_Value_2[0])){
                telemetry.addLine("Right");
            }*/

        telemetry.update();

        waitForStart();

        //while (opModeIsActive()){

        //Telemetry readings for the HSV values in each region
        telemetry.addData("Region 1", "%7d, %7d, %7d", pipeline.HSV_Value_1[0], pipeline.HSV_Value_1[1], pipeline.HSV_Value_1[2]);
        telemetry.addData("Region 2", "%7d, %7d, %7d", pipeline.HSV_Value_2[0], pipeline.HSV_Value_2[1], pipeline.HSV_Value_2[2]);
        telemetry.addData("Region 3", "%7d, %7d, %7d", pipeline.HSV_Value_3[0], pipeline.HSV_Value_3[1], pipeline.HSV_Value_3[2]);

        //telemetry.update();

        if ((pipeline.HSV_Value_1[0] < pipeline.HSV_Value_2[0]) && (pipeline.HSV_Value_1[0] < pipeline.HSV_Value_3[0])) {
            telemetry.addLine("Left");

            encoderDrive(DS, 2, -2, -2, 2);
            sleep(100);

            encoderDrive(DS, -30, 30, -30, 30);
            sleep(100);

            encoderDrive(DS, 16, -16, -16, 16);
            sleep(100);

            encoderArm(1, -480);
            sleep(100);

            robot.spinnerBlock.setPower(-1);
            robot.armMotor.setPower(-0.08);
            sleep(750);

            robot.spinnerBlock.setPower(0);
            robot.armMotor.setPower(0);
            sleep(100);

            encoderDrive(1, 20.75, 20.75, 20.75, 20.75);
            sleep(2000);

            encoderDrive(1, 27, -27, 27, -27);
            sleep(100);

            encoderDrive(1, 53.75, -53.75, -53.75, 53.75);
            sleep(100);

            encoderDrive(1, -25, 25, -25, 25);
            sleep(100);

        } else if ((pipeline.HSV_Value_2[0] < pipeline.HSV_Value_1[0]) && (pipeline.HSV_Value_2[0] < pipeline.HSV_Value_3[0])) {
            telemetry.addLine("Mid");

            encoderDrive(DS, 2, -2, -2, 2);
            sleep(100);

            encoderDrive(DS, -30, 30, -30, 30);
            sleep(100);

            encoderDrive(DS, 16, -16, -16, 16);
            sleep(100);

            encoderArm(1, -770);
            sleep(100);

            robot.spinnerBlock.setPower(-1);
            robot.armMotor.setPower(-0.08);
            sleep(750);

            robot.spinnerBlock.setPower(0);
            robot.armMotor.setPower(0);
            sleep(100);

            encoderDrive(1, 20.75, 20.75, 20.75, 20.75);
            sleep(2000);

            encoderDrive(1, 27, -27, 27, -27);
            sleep(100);

            encoderDrive(1, 53.75, -53.75, -53.75, 53.75);
            sleep(100);

            encoderDrive(1, -25, 25, -25, 25);
            sleep(100);

        } else if ((pipeline.HSV_Value_3[0] < pipeline.HSV_Value_1[0]) && (pipeline.HSV_Value_3[0] < pipeline.HSV_Value_2[0])) {
            telemetry.addLine("Right");

            encoderDrive(DS, 2, -2, -2, 2);
            sleep(100);

            encoderDrive(DS, -30, 30, -30, 30);
            sleep(100);

            encoderDrive(DS, 16.5, -16.5, -16.5, 16.5);
            sleep(100);

            encoderArm(1, -1120);
            sleep(100);

            robot.spinnerBlock.setPower(-1);
            robot.armMotor.setPower(-0.08);
            sleep(750);

            robot.spinnerBlock.setPower(0);
            robot.armMotor.setPower(0);
            sleep(100);

            encoderDrive(1, 20.75, 20.75, 20.75, 20.75);
            sleep(2000);

            encoderDrive(1, 27, -27, 27, -27);
            sleep(100);

            encoderDrive(1, 53.75, -53.75, -53.75, 53.75);
            sleep(100);

            encoderDrive(1, -25, 25, -25, 25);
            sleep(100);
        }

        telemetry.update();

    }

    public static class OpenCV_Pipeline extends OpenCvPipeline {

        /**
         * Most important section of the code: Colors
         **/
        static final Scalar GOLD = new Scalar(255, 215, 0);
        static final Scalar CRIMSON = new Scalar(220, 20, 60);
        static final Scalar RED = new Scalar(0, 255, 255);
        static final Scalar PARAKEET = new Scalar(3, 192, 74);
        static final Scalar CYAN = new Scalar(0, 139, 139);

        // Create a Mat object that will hold the color data
        Mat HSV = new Mat();

        // Make a Constructor
        public OpenCV_Pipeline() {

        }

        // Define the dimensions and location of each region
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(20, 115);
        static final int REGION1_WIDTH = 33;
        static final int REGION1_HEIGHT = 33;
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(130, 115);
        static final int REGION2_WIDTH = 33;
        static final int REGION2_HEIGHT = 33;
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(250, 115);
        static final int REGION3_WIDTH = 33;
        static final int REGION3_HEIGHT = 33;

        // Create the points that will be used to make the rectangles for the region
        Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);

        Point region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);

        Point region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);

        // Creates a field of type "Mat"
        Mat region1, region2, region3;

        // Creating an array for each region which have an element for each channel of interest
        public int[] HSV_Value_1 = new int[3];
        public int[] HSV_Value_2 = new int[3];
        public int[] HSV_Value_3 = new int[3];

        @Override
        public Mat processFrame(Mat input) {

            // Converts the RGB colors from the video to HSV, which is more useful for image analysis
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV_FULL);

            // Creates the regions and finds the HSV values for each of the regions
            region1 = HSV.submat(new Rect(region1_pointA, region1_pointB));
            region2 = HSV.submat(new Rect(region2_pointA, region2_pointB));
            region3 = HSV.submat(new Rect(region3_pointA, region3_pointB));

            // Loops through each channel of interest
            for (int i = 0; i < 3; i++) {
                // Finds the average HSV value for each channel of interest (The "i" representing the channel of interest)
                HSV_Value_1[i] = (int) Core.mean(region1).val[i];
                HSV_Value_2[i] = (int) Core.mean(region2).val[i];
                HSV_Value_3[i] = (int) Core.mean(region3).val[i];
            }

            // Draws rectangles representing the regions in the camera stream
            Imgproc.rectangle(HSV, region1_pointA, region1_pointB, GOLD, 1);
            Imgproc.rectangle(HSV, region2_pointA, region2_pointB, CRIMSON, 1);
            Imgproc.rectangle(HSV, region3_pointA, region3_pointB, GOLD, 1);

            return HSV;

        }


    }

    /*
    public void resetAngle(){
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle(){

        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
        if(deltaAngle > 180){
            deltaAngle -= 360;
        }else if (deltaAngle <= -180){
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;

    }

    public void turn(double degrees){

        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2){
            double motorPower = (error < 0 ? -0.8 : 0.8);
            robot.frontLeftDrive.setPower(motorPower);
            robot.backLeftDrive.setPower(motorPower);
            robot.frontRightDrive.setPower(motorPower);
            robot.backRightDrive.setPower(motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();

        }

        robot.frontLeftDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.backRightDrive.setPower(0);
        robot.spinnerDuck.setPower(0);
        robot.armMotor.setPower(0);
        robot.spinnerBlock.setPower(0);

    }

    public void turnTo(double degrees){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double error = degrees - orientation.firstAngle;

        if(error > 180){
            error -= 360;
        }else if (error <= -180){
            error += 360;
        }

        turn(error);

     }*/
    public void encoderDrive(double speed,
                             double frontLeftInches, double backLeftInches, double frontRightInches, double backRightInches) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

// Ensure that the opmode is still active
        if (opModeIsActive()) {

// Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRightDrive.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = robot.backLeftDrive.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.backRightDrive.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);
            robot.frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            robot.frontRightDrive.setTargetPosition(newFrontRightTarget);
            robot.backLeftDrive.setTargetPosition(newBackLeftTarget);
            robot.backRightDrive.setTargetPosition(newBackRightTarget);

// Turn On RUN_TO_POSITION
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

// reset the timeout time and start motion.
            runtime.reset();
            robot.frontRightDrive.setPower(Math.abs(speed));
            robot.frontLeftDrive.setPower(Math.abs(speed));
            robot.backRightDrive.setPower(Math.abs(speed));
            robot.backLeftDrive.setPower(Math.abs(speed));

// keep looping while we are still active, and there is time left, and both motors are running.
// Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
// its target position, the motion will stop.  This is "safer" in the event that the robot will
// always end the motion as soon as possible.
// However, if you require that BOTH motors have finished their moves before the robot continues
// onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && robot.frontLeftDrive.isBusy() && robot.backRightDrive.isBusy()) {

// Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.frontLeftDrive.getCurrentPosition(),
                        robot.backRightDrive.getCurrentPosition());
                telemetry.update();
            }

// Stop all motion;
            robot.frontRightDrive.setPower(0);
            robot.frontLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.armMotor.setPower(0);
// Turn off RUN_TO_POSITION
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//  sleep(250);   // optional pause after each move
        }
    }

    public void encoderArm(double speed, double degree) {

        if (opModeIsActive()) {
            robot.armMotor.setTargetPosition(robot.armMotor.getCurrentPosition() + (int) (degree / ARM_COUNTS_PER_DEGREE));
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && robot.armMotor.isBusy()) {

                telemetry.addData("Position", robot.armMotor.getCurrentPosition() / ARM_COUNTS_PER_DEGREE);
                telemetry.addData("Target", robot.armMotor.getCurrentPosition() + (int) (degree / ARM_COUNTS_PER_DEGREE));
                telemetry.update();

            }

            robot.armMotor.setPower(0);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
