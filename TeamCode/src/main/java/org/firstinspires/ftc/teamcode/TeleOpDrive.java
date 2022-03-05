package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name="DRIVERED", group="Linear Opmode")

public class TeleOpDrive extends LinearOpMode {

    // DEFINE robot
    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.7 ; // eg: TETRIX Motor Encoder
    static final double ARM_COUNTS_PER_DEGREE = (1440 / 360 * 0.5);
    static final double DRIVE_GEAR_REDUCTION = 1.0 ; // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES   = 4.0 ; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DS = 0.6;
    static final double TURN_SPEED  = 0.6;

    @Override
    public void runOpMode() {
        double x1 = 0;
        double y1 = 0;

        double fortyFiveInRads = -Math.PI/4;
        double cosine45 = Math.cos(fortyFiveInRads);
        double sine45 = Math.sin(fortyFiveInRads);

        double x2 = 0;
        double y2 = 0;

        robot.init(hardwareMap);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.frontLeftDrive.getCurrentPosition(),
                robot.backRightDrive.getCurrentPosition());
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Driver on Gamepad 1
            double G1LT = gamepad1.left_trigger;
            double G1RT = gamepad1.right_trigger;
            double G2leftStickY = gamepad2.left_stick_y;
            double G2RT = gamepad2.right_trigger;
            boolean G1a = gamepad1.a; boolean G1b = gamepad1.b;
            boolean G1x = gamepad1.x; boolean G1y = gamepad1.y;
            boolean G1dp4 = gamepad1.dpad_up; boolean G1dp1 = gamepad1.dpad_down;
            boolean G1dp2 = gamepad1.dpad_left; boolean G1dp3 = gamepad1.dpad_right;

            double G2LSX = gamepad2.left_stick_x;
            robot.spinnerDuck.setPower(G2LSX);

            x1 = gamepad1.left_stick_y;
            y1 = -gamepad1.right_stick_y;

            double x3 = gamepad1.left_stick_x;
            double y3 = gamepad1.right_stick_x;

            y2 = y1*cosine45 + x1*sine45;
            x2 = x1*cosine45 - y1*sine45;

            //Turn Right
            if(G1dp2){
                encoderDrive(1, 20.75, 20.75, 20.75, 20.75);
                sleep(100);
            }
            //Turn Left
            else if(G1dp3){
                encoderDrive(1, -20.75, -20.75, -20.75, -20.75);
                sleep(100);
            }
            //Drive Forward
            else if(G1dp4){
                encoderDrive(1, 55.75, -55.75, -55.75, 55.75);
                sleep(100);
            }
            //Drive Backwards
            else if(G1dp1){
                encoderDrive(1, -55.75, 55.75, 55.75, -55.75);
                sleep(100);
            }
            //Cycle In
            if(G1a){
                encoderDrive(1, -20.75, -20.75, -20.75, -20.75);
                sleep(100);
                encoderDrive(1, -27, 27, -27, 27);
                sleep(100);
                encoderDrive(1, 55.75, -55.75, -55.75, 55.75);
                sleep(100);
            }
            //Cycle Out
            else if(G1b){
                encoderDrive(1, -55.75, 55.75, 55.75, -55.75);
                sleep(100);
                encoderDrive(1, 23, -25, 25, -25);
                sleep(100);
                encoderDrive(1, 20.75, 20.75, 20.75, 20.75);
                sleep(100);
            }
            //Tank Driving
            else if(x1 < -0.3 || x1 > 0.3 || y1 > 0.3 || y1 < -0.3){
                robot.frontLeftDrive.setPower(y1);
                robot.frontRightDrive.setPower(x1);
                robot.backLeftDrive.setPower(x1);
                robot.backRightDrive.setPower(y1);
            }
            //Left Translate
            else if(G1LT > 0.1){
                robot.frontLeftDrive.setPower(G1LT);
                robot.frontRightDrive.setPower(G1LT);
                robot.backLeftDrive.setPower(-G1LT);
                robot.backRightDrive.setPower(-G1LT);
            }
            //Right Translate
            else if(G1RT > 0.1){
                robot.frontLeftDrive.setPower(-G1RT);
                robot.frontRightDrive.setPower(-G1RT);
                robot.backLeftDrive.setPower(G1RT);
                robot.backRightDrive.setPower(G1RT);
            }
            //Safety
            else if(G1y){
                encoderDrive(0, 0, 0, 0, 0);
                sleep(100);
                encoderArm(0, 0);
                sleep(100);
            }
            //Defualt Power
            else{
                robot.frontLeftDrive.setPower(0);
                robot.frontRightDrive.setPower(0);
                robot.backLeftDrive.setPower(0);
                robot.backRightDrive.setPower(0);
            }

            //Driver 2

            double G2TT = gamepad2.left_trigger;
            robot.spinnerBlock.setPower(-G2leftStickY);
            boolean G2a = gamepad2.a;
            boolean G2b = gamepad2.b;
            boolean G2y = gamepad2.y;
            boolean G2dp4 = gamepad2.dpad_up;
            boolean G2dp1 = gamepad2.dpad_down;
            boolean G2dp2 = gamepad2.dpad_left;


            /*if(gamepad2.right_stick_y < 0)robot.armMotor.setPower(gamepad2.right_stick_y * 0.5);
            else if(gamepad2.right_stick_y > 0)robot.armMotor.setPower(-0.0005);
            else if(gamepad2.right_trigger > 0)robot.armMotor.setPower(0.3);
            else if(gamepad2.left_trigger > 0)robot.armMotor.setPower(G2TT);
            else robot.armMotor.setPower(-0.0012);*/

            //Arm Cycle Level 3
            if(G2dp4){
                encoderArm(1, -1120);
                sleep(100);

                robot.spinnerBlock.setPower(-1);
                robot.armMotor.setPower(-0.08);
                sleep(750);

                robot.spinnerBlock.setPower(0);
                robot.armMotor.setPower(0);
                sleep(100);
            }
            //Arm Cycle Level 2
            else if(G2dp2){
                encoderArm(1, -770);
                sleep(100);

                robot.spinnerBlock.setPower(-1);
                robot.armMotor.setPower(-0.08);
                sleep(750);

                robot.spinnerBlock.setPower(0);
                robot.armMotor.setPower(0);
                sleep(100);
            }
            //Arm Cycle Level 1
            else if(G2dp1){
                encoderArm(1, -480);
                sleep(100);

                robot.spinnerBlock.setPower(-1);
                robot.armMotor.setPower(-0.08);
                sleep(750);

                robot.spinnerBlock.setPower(0);
                robot.armMotor.setPower(0);
                sleep(100);
            }
            else if(G2y){
                robot.frontLeftDrive.setPower(0);
                robot.frontRightDrive.setPower(0);
                robot.backLeftDrive.setPower(0);
                robot.backRightDrive.setPower(0);
                robot.armMotor.setPower(0);
            }





            telemetry.addData("frontLeftDrive Power", robot.frontLeftDrive.getPower());

            telemetry.addData("backLeftDrive Power", robot.backLeftDrive.getPower());

            telemetry.addData("frontRightDrive Power", robot.frontRightDrive.getPower());

            telemetry.addData("backRightDrive Power", robot.backRightDrive.getPower());


            telemetry.addData("spinnerDuck Power", robot.spinnerDuck.getPower());

            telemetry.addData("Block Spinner Power", robot.spinnerBlock.getPower());

            telemetry.addData("Arm Motor Power", robot.armMotor.getPower());

            telemetry.update();


        }
    }
    public void encoderDrive(double speed,
                             double frontLeftInches, double backLeftInches, double frontRightInches, double backRightInches) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

// Ensure that the opmode is still active
        if (opModeIsActive()) {

// Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRightDrive.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = robot.backLeftDrive.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.backRightDrive.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);
            robot.frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            robot.frontRightDrive.setTargetPosition(newFrontRightTarget);
            robot.backLeftDrive.setTargetPosition(newBackLeftTarget);
            robot.backRightDrive.setTargetPosition(newBackRightTarget);

// Turn On RUN_TO_POSITION
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            while (opModeIsActive() && robot.frontLeftDrive.isBusy() && robot.backRightDrive.isBusy() ){

// Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
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

//  sleep(250);   // optional pause after each move
        }
    }

    public void encoderArm(double speedA, double degree) {

        int newdegree;

        if (opModeIsActive()) {
            robot.armMotor.setTargetPosition(robot.armMotor.getCurrentPosition() + (int)(degree / ARM_COUNTS_PER_DEGREE));
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setPower(Math.abs(speedA));

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
