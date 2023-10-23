package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

public class JonathanAutonomusOpMode extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor testMotor1;
    private DcMotor testMotor2;
    private DcMotor testMotor3;
    private DcMotor testMotor4;
    private Servo armservo;
    private Gyroscope imu;


    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        testMotor1 = hardwareMap.get(DcMotor.class, "TestMotor1");
        testMotor2 = hardwareMap.get(DcMotor.class, "TestMotor2");
        testMotor3 = hardwareMap.get(DcMotor.class, "TestMotor3");
        testMotor4 = hardwareMap.get(DcMotor.class, "TestMotor4");
        armservo = hardwareMap.get(Servo.class, "armservo");
        imu = hardwareMap.get(Gyroscope.class, "imu");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        //forward(23.5);
        

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
