import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.ICE_Controls;

import java.sql.Driver;
import java.util.List;

;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//import all assets neccesary//

//name of program that shows up on phone,group linear of iterative//
@Autonomous(name="DetectAndDriveSkystone_Path1", group="Linear Opmode")
@Disabled
public class DetectAndDriveSkystone_Path1 extends ICE_Controls {

    private ElapsedTime runtime = new ElapsedTime();
    @Override
//what shows up on your phone//
    public void runOpMode() {
        initializeHardware();
        telemetry.addData("On Our Way To the Stone", "SkyStone");
        telemetry.update();
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        //If phone is comapatible start: if not say text on phone//
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        //What shows up on your phone//
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
//Wait for start
        waitForStart();
        runtime.reset();
        double currentAngle = 0;

        //If opmode is active,turn until you find stone or skystone//
        if (opModeIsActive()) {
            double TURN_SPEED=1.0;
            double DRIVE_SPEED=0.7;

            gyroHold(TURN_SPEED,0,2);
            gyroDrive(DRIVE_SPEED,12,0);
            gyroTurn(TURN_SPEED,90,5);
            gyroHold(TURN_SPEED,90,1);
            gyroDrive(DRIVE_SPEED,15,90);
            gyroTurn(TURN_SPEED,0, 5);
            gyroHold(TURN_SPEED,0,1);
            //gyroDrive(DRIVE_SPEED,8,0);


            while (opModeIsActive()) {
                Recognition stonerecognition = null;
                boolean skystoneFound = false;
                //keep attempting to find stone//
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//If you find stone then say on DS Object detected//
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                        //Found Object.....Now break the while loop
                        skystoneFound = false;

                        if (updatedRecognitions.size() > 0) {
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel() == LABEL_SECOND_ELEMENT) {
                                    skystoneFound = true;
                                    stonerecognition = recognition;
                                    break;
                                }


                            }
                            //if (skystoneFound)
                            //    break;
                        }
                    }
                }
                if (skystoneFound) {
//shows us angle to turn to//
                    double angleToTurnTo = -1 * stonerecognition.estimateAngleToObject(AngleUnit.DEGREES);
                    telemetry.addData(">", "Skystone FOUND!!!!!");
                    telemetry.addData("AngleToTurn =",angleToTurnTo);
                    telemetry.update();
                    sleep(5000);

                    /*

                    float MidpointOfStone = (stonerecognition.getLeft() + stonerecognition.getRight()) / 2;

                        while(true) {
                            if (640 - MidpointOfStone > 10) {

                                gyroHold(TURN_SPEED,-15,5);
                                telemetry.addData(">", "left turning");
                                telemetry.update();
                            } else if (640 - MidpointOfStone < 10) {
                                 gyroHold(TURN_SPEED,10,5);
                                 telemetry.addData(">", "right turning");
                                telemetry.update();
                            } else {
                                gyroDrive(DRIVE_SPEED,-2,0,5);
                                telemetry.addData(">", "going forward");
                                telemetry.update();
                                break;
                            }
                        }
                */
//turns to the angle to turn to and holds for 5 seconds
                    TURN_SPEED=0.5;
                gyroTurn(TURN_SPEED,angleToTurnTo,10);
                gyroHold(TURN_SPEED,angleToTurnTo,1);
                //Drives forward for 2 feet or one tile at the angleToTurnTo
                gyroDrive(DRIVE_SPEED,30,angleToTurnTo);
              servo.setPosition(1.0);
              sleep(2000);
              //turns -90 degrees  and holds there for 5 seconds
              gyroDrive(DRIVE_SPEED,-30,angleToTurnTo);
              TURN_SPEED=1.0;
                gyroTurn(TURN_SPEED,-90,5);
                gyroHold(TURN_SPEED,-90,1);
                //goes Forward for 2 feet at -90
                gyroDrive(DRIVE_SPEED,50,-90);
               //Turns -180 and holds for 5
               //goes forward for 1 foot
                //turns to -90 and holds for 5




                    break;
                }

                }
            }
        }

//end of code
  /**
     *
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        //what shows on screen//
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
