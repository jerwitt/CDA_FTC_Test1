
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

public class JackDaYakAutonomous extends LinearOpMode {
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
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//fordward
testMotor1.setPower(0.5);
testMotor2.setPower(-0.5);
sleep(1000);

testMotor1.setPower(0.0);
testMotor2.setPower(-0.0);
sleep(1000);

//turn
testMotor1.setPower(0.5);
testMotor2.setPower(0.5);
sleep(1000);

testMotor1.setPower(0.0);
testMotor2.setPower(-0.0);
sleep(1000);   
   
//backward
testMotor1.setPower(-0.5);
testMotor2.setPower(0.5);
sleep(1000);

testMotor1.setPower(-0.0);
testMotor2.setPower(0.0);  
sleep(500);

//forward on steriods (I cant spell) (or can I)
testMotor1.setPower(2.5);
testMotor2.setPower(-2.5);
sleep(1000);

testMotor1.setPower(0.0);
testMotor2.setPower(-0.0); 
sleep(500);

//360donuts lol
testMotor1.setPower(1.5);
testMotor2.setPower(-1.5);
sleep(5000);

testMotor1.setPower(0.0);
testMotor2.setPower(-0.0); 

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "if you can see this it works");
            telemetry.update();

        }
    }
}
