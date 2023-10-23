
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous

public class ExampleDriveSquare extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor testMotor1;
    private DcMotor testMotor2;
    private Gyroscope imu;
    private ElapsedTime     runtime = new ElapsedTime();


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

        //initialize testMotor1 
        testMotor2.setPower(0);
        // set motor mode     
        testMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // set motor zeroPowerBehavior
        testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)        while (opModeIsActive()) {
        for (int i = 0; i < 4; i++) {
            testMotor1.setPower(0.5);
            testMotor2.setPower(-0.5);
            runtime.reset();
            while (opModeIsActive() && runtime.milliseconds() < 1000) {
                telemetry.addData("moving forward", "");
                telemetry.update();
            }
            testMotor1.setPower(0);
            testMotor2.setPower(0);
            sleep(1000);

            testMotor1.setPower(0.5);
            testMotor2.setPower(0.5);
            runtime.reset();
            while (opModeIsActive() && runtime.milliseconds() < 500) {
                telemetry.addData("turning", "");
                telemetry.update();
            }
            testMotor1.setPower(0);
            testMotor2.setPower(0); 
            sleep(1000);
        }
    }
}
