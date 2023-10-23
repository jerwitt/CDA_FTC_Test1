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

@TeleOp (group="Primary")

public class ArmServoTestOpMode extends LinearOpMode {
    private Blinker control_Hub;
    private Servo bottomArmServo;
    private Gyroscope imu;
    private Servo topArmServo;


    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        bottomArmServo = hardwareMap.get(Servo.class, "bottomArmServo");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        topArmServo = hardwareMap.get(Servo.class, "topArmServo");
        
        double lTrigger;
        double rTrigger;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
    
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            rTrigger = gamepad1.right_trigger;
            lTrigger = gamepad1.left_trigger;
            
            bottomArmServo.setPosition((1-lTrigger)*0.15);
            topArmServo.setPosition(rTrigger*0.15+0.45);
            
            //close:
            //bottomArmServo.setPosition(0);
            //topArmServo.setPosition(0.6);
            
            //open:
            //bottomArmServo.setPosition(0.15);
            //topArmServo.setPosition(0.45);
            
            
            
            telemetry.addData("Status", "Running");
            //telemetry.addData("Servos", "Bottom: %.3f, Top: %.3f", lTrigger, rTrigger);
            telemetry.update();

        }
    }
}
