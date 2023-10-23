package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
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

public class JackAutonomousModeParking extends LinearOpMode {
    private Blinker control_Hub;
    private Servo intakeServo;
    private DcMotor leftLift;
    private DcMotor rightLift;
    private DcMotor testMotor1;
    private DcMotor testMotor2;
    private DcMotor testMotor3;
    private DcMotor testMotor4;
    private Servo armservo;
    private BNO055IMU imu;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    
    @Override
    
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        testMotor1 = hardwareMap.get(DcMotor.class, "TestMotor1");
        testMotor2 = hardwareMap.get(DcMotor.class, "TestMotor2");
        testMotor3 = hardwareMap.get(DcMotor.class, "TestMotor3");
        testMotor4 = hardwareMap.get(DcMotor.class, "TestMotor4");
        armservo = hardwareMap.get(Servo.class, "armservo");
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
        
        //Park in spot 1
        //forward(23.5);

        turn(90);
        
        telemetry.addData("Status", "turned 90");
        telemetry.update();
        
        //sleep(2000);


        telemetry.addData("Status", "turned to -90");
        telemetry.update();

        sleep(1000);

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
    
    public void forward(int distance) {
        testMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        int ticks = distance / 10 * 560;
        
        testMotor1.setTargetPosition(ticks);
        testMotor2.setTargetPosition(ticks);
        
        testMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        testMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        testMotor1.setPower(0.3);
        testMotor2.setPower(0.3);
        
        while(testMotor1.isBusy() || testMotor2.isBusy()); {
            telemetry.addData("Status", "Moving Forward");
            telemetry.update();
        }
        
    }

}

