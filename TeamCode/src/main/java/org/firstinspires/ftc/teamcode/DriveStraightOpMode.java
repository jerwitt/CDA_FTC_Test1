package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (group="z2022")

public class DriveStraightOpMode extends LinearOpMode {
    private Blinker control_Hub;
    private Servo intakeServo;
    private DcMotor leftLift;
    private DcMotor rightLift;
    private DcMotor testMotor1;
    private DcMotor testMotor2;
    private BNO055IMU imu;
    Orientation angles;
    double KP = 0;
    double KI = 0;
    double KD = 0;
    double error = 0;
    double prevError = 0;
    double integral = 0;
    double derivative = 0;


    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        intakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        leftLift = hardwareMap.get(DcMotor.class, "LeftLift");
        rightLift = hardwareMap.get(DcMotor.class, "RightLift");
        testMotor1 = hardwareMap.get(DcMotor.class, "TestMotor1");
        testMotor2 = hardwareMap.get(DcMotor.class, "TestMotor2");
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        resetPID();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            drivePID(0.0);
            telemetry.addData("Heading Z", angles.firstAngle);
            telemetry.addData("Roll Y", angles.secondAngle);
            telemetry.addData("Pitch X", angles.thirdAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Previous Error", prevError);
            telemetry.addData("Integral", integral);
            telemetry.addData("Derivative", derivative);
            telemetry.update();

        }
    }
    public void drivePID(double target) {
        prevError = error;
        error = target - angles.firstAngle;
        integral += error;
        derivative = prevError - error;
        
        
    }
    public void resetPID() {
        KP = 0.1;
        KI = 0.1;
        KD = 0.1;
        error = 0;
        prevError = 0;
        integral = 0;
        derivative = 0;
    }
}
