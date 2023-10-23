/*
Copyright 2022 FIRST Tech Challenge Team 21514

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
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

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class MicahDriverOpMode extends LinearOpMode {
    private Blinker control_Hub;
    private Servo intakeServo;
    private DcMotor leftLift;
    private DcMotor rightLift;
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
        leftLift = hardwareMap.get(DcMotor.class, "LeftLift");
        rightLift = hardwareMap.get(DcMotor.class, "RightLift");
        testMotor1 = hardwareMap.get(DcMotor.class, "TestMotor1");
        testMotor2 = hardwareMap.get(DcMotor.class, "TestMotor2");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        
        //initialize testMotor1 
        testMotor1.setPower(0);
        // set motor mode     
        testMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // set motor zeroPowerBehavior
        testMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initialize testMotor2
        testMotor2.setPower(0);
        // set motor mode     
        testMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // set motor zeroPowerBehavior
        testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        intakeServo.setPosition(position);
        
        


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //right trigger initialization
            float rightTrigger = gamepad1.right_trigger;
            //left trigger initialization
            float leftTrigger = gamepad1.left_trigger;
            //right bumper initialization
            boolean rightBumper = gamepad1.right_bumper;
            //left bumper initialization
            boolean leftBumper = gamepad1.left_bumper;
            //left stick init
            float leftStickX = gamepad1.left_stick_x;
            //right stick init
            float rightStickY = gamepad1.right_stick_y;
            
            
            
            
            if (leftBumper) {
                intakeServo.setPosition(0);
            }
            if (rightBumper) {
                intakeServo.setPosition(1);
            }
            
            float motor1Power = rightTrigger - leftTrigger;
            float motor2Power = rightTrigger - leftTrigger;
            float rightLiftPower = rightStickY;
            float leftLiftPower = rightStickY;

            
           
            leftLift.setPower(rightLiftPower);
            rightLift.setPower(-leftLiftPower);
            
            
            
            testMotor1.setPower(motor1Power);
            testMotor2.setPower(-motor2Power);
            
                       
            telemetry.addData("Motor 1 Value", motor1Power);
            telemetry.addData("Motor 2 Value", motor2Power);
            telemetry.addData("Left Lift Value", leftLiftPower);
            telemetry.addData("Right Lift Value", rightLiftPower);
            
            telemetry.update();

        }
    }
}
