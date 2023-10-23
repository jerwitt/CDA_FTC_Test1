
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import android.graphics.Color;

@Autonomous

public class AutoParkOpMode extends LinearOpMode {
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private Blinker control_Hub;
    private DcMotor testMotor1;
    private DcMotor testMotor2;
    private DcMotor testMotor3;
    private DcMotor testMotor4;
    private Servo armservo;
    private BNO055IMU imu;

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    
    private double targetAngle;
    private double Kp = 0.01;
    private double Ki = 0.0;
    private double Kd = 0.03;
    private double acculumatedError = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError;
    private double lastTime;
    private static final int signalHue1 = 213;
    private static final int signalHue2 = 93;
    private static final int signalHue3 = 211;
    private static final int signalRed1 = 438;
    //private static final int signalRed2 = 0;
    private static final int signalRed3 = 150;
    private static final double colerReadDistance = 18.5;
    
    //an array that hols hue, saturation, and value info
    float hsvValues [] = {0F, 0F, 0F};
    //values is a reference to the hsvValues array
    final float values[] = hsvValues;
    //scale factor
    final double SCALE_FACTOR = 255;
        
    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ColorSensor");
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
        
        testMotor1.setPower(0);
        testMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor2.setPower(0);
        testMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor3.setPower(0);
        testMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor4.setPower(0);
        testMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // hold on to the cone at the beginning of Autonomous mode
        // otherwise, the first cone will be dropped4
        armservo.setPosition(1);
      
        telemetry.addData("Distance (inch)", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
 /*       while (distanceSensor.getDistance(DistanceUnit.CM) > 2.1) {
            testMotor1.setPower(0.1);
            testMotor2.setPower(-0.1);
            telemetry.addData("Distance (inch)", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
         }*/
        
        
        forward(colerReadDistance, 0.3);
        
        Color.RGBToHSV((colorSensor.red()),
                        (colorSensor.green()),
                        (colorSensor.blue()),
                        hsvValues);
                            
        telemetry.addData("Distance (cm)", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance (inch)", distanceSensor.getDistance(DistanceUnit.INCH));           telemetry.addData("Alpha", colorSensor.alpha());
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("hue", hsvValues[0]);
            
        int zone = signalZone(hsvValues[0], colorSensor.red()); 
        telemetry.addData("zone", zone);
        telemetry.update();                    

        // define movement base on zone value 
            
        if (zone == 1){
            telemetry.addData("executing zone", zone);
            telemetry.update();                    
            //park in zone 1
            forward(10, 0.3);
            forward(-7, 0.3);
            forward(0, 0.3);
            turn(90);
            forward(19.3, 0.3);
           
        }
        else if(zone == 2){
            telemetry.addData("executing zone", zone);
            telemetry.update();                    
            //park in zone 2, park in place
            forward(10, 0.3);
            forward(-6.5, 0.3);

        }
        else if(zone == 3){
            telemetry.addData("executing zone", zone);
            telemetry.update();    
            //park in zone 3
            forward(11, 0.3);
            forward(-7, 0.3);
            turn(-90);
            forward(20, 0.3);
        }
        else
        {
            telemetry.addData("zone reading error ", zone);
            telemetry.update();                    
            // do nothing, just park where you are   
            forward(2, 0.2);
        }
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Distance (cm)", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance (inch)", distanceSensor.getDistance(DistanceUnit.INCH));           telemetry.addData("Alpha", colorSensor.alpha());
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("hue", hsvValues[0]);
            telemetry.addData("parked at zone", zone);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

        
    }
    
    public void forward(double distance, double speed) {
        testMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        testMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        int ticks = (int)Math.round(distance / 10 * 560);
        
        testMotor1.setTargetPosition(ticks);
        testMotor2.setTargetPosition(-ticks);
        
        testMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        testMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        testMotor1.setPower(speed);
        testMotor2.setPower(speed);
        
        while(testMotor1.isBusy() || testMotor2.isBusy()); {
            telemetry.addData("Status", "Moving Forward");
            telemetry.update();
        }
        
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
        
        telemetry.addData("gyro", orientation.firstAngle);
        telemetry.update();
        
        return currAngle;
    }
    
    public void turn(double degrees) {
        resetAngle();
        double error = degrees;
        
        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            // run without decoder
            testMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            testMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // set motor zeroPowerBehavior
            testMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
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
    
    private double getAbsoluteAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }        
    
    void turnToPID(double degrees) {
        targetAngle = degrees;
        
        while (opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 1) {
            
            double motorPower = update(getAbsoluteAngle());
            // run without decoder
            testMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            testMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // set motor zeroPowerBehavior
            testMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            setMotorPower(motorPower, motorPower);
        }
        setMotorPower(0.0,0.0);
    }
    
    void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }
    
    private double update(double currentAngle) {
        //P
        double error = targetAngle - currentAngle;
        error %= 360;
        error += 360;
        error %= 360;
        
        if (error > 180) {
            error -= 360;
        } 
  
        //I
        acculumatedError += error;
        if (Math.abs(error) < 1) {
            acculumatedError = 0;
        }
        acculumatedError = Math.abs(acculumatedError * Math.signum(error));
        
        //D
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastError = error;
        
        //motor power calculation 
        double motorPower = 0.1 * Math.signum(error) + 0.9 * Math.tanh(
            Kp * error + Ki * acculumatedError + Kd * slope);
            
        return motorPower;
    }
    
    private void setMotorPower(double leftPower, double rightPower) {
        testMotor1.setPower(leftPower);
        testMotor2.setPower(rightPower);
    }
    
    private int signalZone (float hueValue, float redValue) {
        
    if ( ( hueValue > (signalHue1 - 30)) && (hueValue < (signalHue1 + 30) )){
          if  ( redValue > 235){
            return 1;
          }
          else {
            return 3;
          }
    }
    else if ( ( hueValue > (signalHue2 - 30) ) && (hueValue < (signalHue2 + 30) )){
        return 2;
    }
    else if ( (hueValue > (signalHue3 - 30) ) && (hueValue < (signalHue3 + 30) )){
        return 3;
    }
    else {
        return 0;
    }
    
}
}