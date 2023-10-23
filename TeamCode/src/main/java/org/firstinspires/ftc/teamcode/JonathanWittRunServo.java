package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class JonathanWittRunServo extends LinearOpMode {
    private Blinker control_Hub;
    private Servo intakeServo;
    private DcMotor testMotor1;
    private DcMotor testMotor2;
    private Gyroscope imu;

    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;
    double position = (MAX_POS + MIN_POS) / 2;
    

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        intakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        testMotor1 = hardwareMap.get(DcMotor.class, "TestMotor1");
        testMotor2 = hardwareMap.get(DcMotor.class, "TestMotor2");
        imu = hardwareMap.get(Gyroscope.class, "imu");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        intakeServo.setPosition(position);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            
            if (gamepad1.left_bumper) {
                intakeServo.setPosition(0);
            }
            if (gamepad1.right_bumper) {
                intakeServo.setPosition(1);
            }
        }
    }
}
