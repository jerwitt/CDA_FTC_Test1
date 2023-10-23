package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Disabled
@TeleOp (group="Primary")

public class Test extends LinearOpMode
{
    private VoltageSensor batteryVoltageSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {}
        waitForStart();
        while (opModeIsActive()){}
    }
    public void initHardware() {}


}
