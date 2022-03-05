package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

//imports

@Autonomous(name="Gyro", group="Auto")
public class Gyro extends LinearOpMode{

    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    private BNO055IMU imu;
    double firstAngles = 0;

    @Override
    public void runOpMode(){


        robot.init(hardwareMap);
        //Check how hardware is init in my classes

        waitForStart();

        turn(90);

        sleep(3000);

        //forward(1);

        //sleep(1500);



        //turnTo(-90);

    }

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

    //public void forward(double time){

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

    }

}
