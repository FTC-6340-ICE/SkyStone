package org.firstinspires.ftc.teamcode;

//TODO further testing of rev robotics hex motor encoders

//Import FTC modules

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

//Define as Autonomous
@Disabled
@Autonomous(name = "Stone+Foundation", group = "Linear Opmode")
public class StoneFoundation extends ICE_Controls_2Motors {

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
        telemetry.addData("Position", "In the triangle corner.");
        telemetry.update();
        waitForStart();

        // Insert Autonomous Code Here

        gyroDrive(.7, 30, 0,15);
        gyroDrive(-1, -30,0,15);
        gyroTurn(.3, -90,10);
        gyroDrive(1,100, -90,15);
        gyroTurn(.3, 0, 10);
        //Todo add adrians stone detection method.



    }
}