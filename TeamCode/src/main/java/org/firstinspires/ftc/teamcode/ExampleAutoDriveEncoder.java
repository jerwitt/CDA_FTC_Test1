package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (group="z2022")

public class ExampleAutoDriveEncoder extends LinearOpMode {
    private Blinker control_Hub;
    private Servo intakeServo;
    private DcMotor leftLift;
    private DcMotor rightLift;
    private DcMotor testMotor1;
    private DcMotor testMotor2;
    private BNO055IMU imu;

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 20.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.5433 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
 
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
        
        // Send telemetry message;
        //telemetry.addData("Status", "Resetting Encoders");    //
        //telemetry.update();

        // Set all motors to zero power
        testMotor1.setPower(0);
        testMotor2.setPower(0);
        
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          testMotor1.getCurrentPosition(),
                          testMotor2.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // forward 20 inches
        encoderDrive(DRIVE_SPEED,  16,  16, 10.0);  
        // turn 90 degrees
        turn(90);
        sleep(2000);
        //back 16 inches
        encoderDrive(DRIVE_SPEED, 16, 16, 10.0);  

        sleep(10000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
        
        
    }
    
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        testMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        testMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = testMotor1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = testMotor2.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            testMotor1.setTargetPosition(newLeftTarget);
            testMotor2.setTargetPosition(-newRightTarget);

            // Turn On RUN_TO_POSITION
            testMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            testMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            testMotor1.setPower(Math.abs(speed));
            testMotor2.setPower(-Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (testMotor1.isBusy() && testMotor2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            testMotor1.getCurrentPosition(),
                                            testMotor2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            testMotor1.setPower(0);
            testMotor2.setPower(0);

            // Turn off RUN_TO_POSITION
            testMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            testMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            testMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            testMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(1000);   // optional pause after each move
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
        testMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        testMotor1.setPower(leftPower);
        testMotor2.setPower(rightPower);
    }
}
