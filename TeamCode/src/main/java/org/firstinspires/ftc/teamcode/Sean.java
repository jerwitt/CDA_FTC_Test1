
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

public class Sean extends LinearOpMode {
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
                //initialize testMotor1
    testMotor1.setPower(0);
    //set motor mode
    testMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //set motor zeroPowerBehavior
    testMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    double leftStick;
    leftStick=gamepad1.left_stick_y;
    telemetry.addData("Status", "left(%.2f)",leftStick);
    telemetry.update();
    testMotor1.setPower(leftStick *10);
    

        }
    }
}
