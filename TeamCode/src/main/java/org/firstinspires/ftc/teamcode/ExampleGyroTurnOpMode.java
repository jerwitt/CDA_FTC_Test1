
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous

public class ExampleGyroTurnOpMode extends LinearOpMode {
    private Blinker control_Hub;
    private Servo intakeServo;
    private DcMotor leftLift;
    private DcMotor rightLift;
    private DcMotor testMotor1;
    private DcMotor testMotor2;
    private BNO055IMU imu;

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    
    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        intakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        leftLift = hardwareMap.get(DcMotor.class, "LeftLift");
        rightLift = hardwareMap.get(DcMotor.class, "RightLift");
        testMotor1 = hardwareMap.get(DcMotor.class, "TestMotor1");
        testMotor2 = hardwareMap.get(DcMotor.class, "TestMotor2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        //initialize the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO005IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
        imu.initialize(parameters);
        

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        turn(90);
        
        telemetry.addData("Status", "turned 90");
        telemetry.update();
        
        sleep(2000);

        turnTo(-90);
        
        telemetry.addData("Status", "turned to -90");
        telemetry.update();

        sleep(10000);

    }
    
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0.0;
    }
    
    private double getAngle() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
        
        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle <= -180) {
            deltaAngle += 360;
        }
        
        currAngle += deltaAngle;
        lastAngles = orientation;
        
 //       telemetry.addData("gyro", orientation.firstAngle);
 //       telemetry.update();
        
        return currAngle;
    }
    
    public void turn(double degrees) {
        resetAngle();
        double error = degrees;
        
        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            setMotorPower(motorPower, motorPower);
            error = degrees - getAngle();
            
            telemetry.addData("error", error);
            telemetry.update();
        
        }
        
        setMotorPower(0.0,0.0);
    }
    
    private void turnTo(double degrees) {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = degrees - orientation.firstAngle;
        
        if (error > 180) {
            error -= 360;
        } else if (error <= -180) {
            error += 360;
        }
        
        turn(error);
    }
    
    private void setMotorPower(double leftPower, double rightPower) {
        testMotor1.setPower(leftPower);
        testMotor2.setPower(rightPower);
    }
}
