package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.graphics.Color;

@Autonomous

public class ExampleColorDistanceSensor extends LinearOpMode {
    private Blinker control_Hub;
    private Servo intakeServo;
    private DcMotor leftLift;
    private DcMotor rightLift;
    private DcMotor testMotor1;
    private DcMotor testMotor2;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private static final int signalHue1 = 336;
    private static final int signalHue2 = 35;
    private static final int signalHue3 = 219;
    

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        intakeServo = hardwareMap.get(Servo.class, "armservo");
        leftLift = hardwareMap.get(DcMotor.class, "TestMotor3");
        rightLift = hardwareMap.get(DcMotor.class, "TestMotor4");
        testMotor1 = hardwareMap.get(DcMotor.class, "TestMotor1");
        testMotor2 = hardwareMap.get(DcMotor.class, "TestMotor2");
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ColorSensor");
        
        //an array that hols hue, saturation, and value info
        float hsvValues [] = {0F, 0F, 0F};
        //values is a reference to the hsvValues array
        final float values[] = hsvValues;
        //scale factor
        final double SCALE_FACTOR = 255;

    
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //convert RGV vlues to HSV values
            /*Color.RGBToHSV((int)(colorSensor.red() * SCALE_FACTOR),
                            (int)(colorSensor.green() * SCALE_FACTOR),
                            (int)(colorSensor.blue() * SCALE_FACTOR),
                            hsvValues);*/

            Color.RGBToHSV((colorSensor.red()),
                            (colorSensor.green()),
                            (colorSensor.blue()),
                            hsvValues);
                           
            telemetry.addData("Distance (cm)", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance (inch)", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Alpha", colorSensor.alpha());
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Green", colorSensor.green());
//            telemetry.addData("Scaled Red", (colorSensor.red() * SCALE_FACTOR));
//            telemetry.addData("Scaled Blue", (colorSensor.blue() * SCALE_FACTOR));
//            telemetry.addData("Scaled green", (colorSensor.green() * SCALE_FACTOR));
            telemetry.addData("hue", hsvValues[0]);
            
           // if ( ( hsvValues[0] > (signalHue1 -10) ) && (hsvValues[0] < (signalHue1 +10) )){
            int zone = signalZone(hsvValues[0]); 
            telemetry.addData("zone", zone);
            telemetry.update();
            }
 
        }
        
   private int signalZone (float hueValue) {
        
      if ( ( hueValue > (signalHue1 - 20)) && (hueValue < (signalHue1 + 20) )){
        return 1;
    }
    else if ( ( hueValue > (signalHue2 -20) ) && (hueValue < (signalHue2 + 20) )){
        return 2;
    }
    else if ( (hueValue > (signalHue3 - 20) ) && (hueValue < (signalHue3 + 20) )){
        return 3;
    }
    else {
        return 0;
    }
    }
        
    }



