package org.firstinspires.ftc.teamcode;

//TODO further testing of rev robotics hex motor encoders

//Import FTC modules

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

//Define as Autonomous
@Disabled
@Autonomous(name = "Name of Auto", group = "Linear Opmode")
public class AutoTemplate extends ICE_Controls_2Motors {

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
        //Put in position on field for when setting up on field.
        telemetry.addData("Position", "In the triangle corner.");
        telemetry.update();
        waitForStart();

        // Insert Autonomous Code Here

    }
}