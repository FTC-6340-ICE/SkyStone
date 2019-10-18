package org.firstinspires.ftc.teamcode;

//TODO further testing of rev robotics hex motor encoders

//Import FTC modules

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

//Define as Autonomous
@Disabled
@Autonomous(name = "AutoTemplate", group = "Linear Opmode")
public class
Autonamouse_Layout extends ICE_Controls {

    @Override
    public void runOpMode() {
        // Initialize the hardware
        initializeHardware();
        initVuforia();
        initTfod();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Waiting for Start.");
        telemetry.update();
        waitForStart();

        // Insert Autonomous Code Here

    }
}