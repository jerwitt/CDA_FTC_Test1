package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class JonathanWittTankDriveOpMode extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor testMotor1;
    private DcMotor testMotor2;
    private Gyroscope imu;


    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        testMotor1 = hardwareMap.get(DcMotor.class, "TestMotor1");
        testMotor2 = hardwareMap.get(DcMotor.class, "TestMotor2");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        
        testMotor1.setPower(0);
        testMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        testMotor2.setPower(0);
        testMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double leftStick;
            double rightStick;
            
            leftStick = gamepad1.left_stick_y;
            rightStick = -gamepad1.right_stick_y;
            
            telemetry.addData("Status", "left (%.2f) right (%.2f)", leftStick, rightStick);
            telemetry.update();
            
            testMotor1.setPower(leftStick);
            testMotor2.setPower(rightStick);
            
            

        }
    }
}
