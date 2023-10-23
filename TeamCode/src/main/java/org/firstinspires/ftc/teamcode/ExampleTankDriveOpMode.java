package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
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

public class ExampleTankDriveOpMode extends LinearOpMode {
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


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //define a variable for the gamepad left stick push position
            double leftStick;
            //define a variable for the gamepad right stick push position
            double rightStick;
            
            //retrieve the left stick value
            leftStick = gamepad1.left_stick_y;
            //retrieve the left stick value
            rightStick = -gamepad1.right_stick_y;
            //display the left stick value on driver station
            telemetry.addData("Status", "left (%.2f) right (%.2f)", leftStick, rightStick);
            telemetry.update();
            
            //run motor according to left stick push position
            testMotor1.setPower(leftStick);
            testMotor2.setPower(rightStick);
 
        }
    }
}
