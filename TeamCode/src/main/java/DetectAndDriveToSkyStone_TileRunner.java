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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

 //name of program that shows up on phone,group linear of iterative//
 @TeleOp(name="DetectAndDriveToSkyStone", group="Linear Opmode")
 //@Disabled
 public class DetectAndDriveToSkyStone_TileRunner extends LinearOpMode {

     //Declare TensorFlow variables
     private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
     private static final String LABEL_FIRST_ELEMENT = "Stone";
     private static final String LABEL_SECOND_ELEMENT = "Skystone";
 //Entered Vuforia Key//
     private static final String VUFORIA_KEY =
             " AZWzerv/////AAABmZeKo4MkD08MoSz5oHB/JU6N1BsUWpfHgQeAeVZemAypSUGVQhvAHo6+v7kJ3MITd8530MhwxRx7GjRtdCs1qjPmdKiJK66dv0yN4Zh4NvKBfP5p4TJjM+G0GoMVgVK0pItm2U56/SVqQH2AYtczQ+giw6zBe4eNhHPJCMY5C2t5Cs6IxxjZlMkRF85l8YAUlKGLipnoZ1T/mX8DNuThQA57qsIB2EN6pGWe8GI64hcPItQ0j7Oyjp82lEN13rYQYsS3Ur4a6//D6yhwa0rogXAysG68G+VgC1mNlj1CjX60qDI84ZN0b/A081xXqjeyFqZK8A/jO8y7BGz9ZuuZNxxXIon6xRNeKYudpfTD23+5";

     /**
      * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
      * localization engine.
      */
     private VuforiaLocalizer vuforia;
 //Not sure what this line means //
     /**
      * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
      * Detection engine.
      */
     private TFObjectDetector tfod;
 //Pretty sure this mean that it is turning on the object detector//

     // Declare OpMode members.
     private ElapsedTime runtime = new ElapsedTime();
    // private DcMotor leftDrive = null;
     //private DcMotor rightDrive = null;
 //defining variables in class for motors//
     DcMotor leftFrontMotor;
     DcMotor rightFrontMotor;
     DcMotor leftBackMotor;
     DcMotor rightBackMotor;
 //Setting power variable//
     double power = 0.2;
 //new instance//

     @Override
 //what shows up on your phone//
     public void runOpMode() {
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
 //mapping and finding the motors//
         leftFrontMotor  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
         rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
         leftBackMotor  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
         rightBackMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
 //setting direction of motors//
         leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
         rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
         leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
         rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

         /** Wait for the game to begin */
        //What shows up on your  phone//
         telemetry.addData(">", "Press Play to start op mode");
         telemetry.update();
 //Wait for start
         waitForStart();
         runtime.reset();
 //If opmode is active,turn until you find stone or skystone//
         if (opModeIsActive()) {
             while (opModeIsActive()) {
                 power = 0.5;
                 leftFrontMotor.setPower(power);
                 rightFrontMotor.setPower(power);
                 leftBackMotor.setPower(power);
                 rightBackMotor.setPower(power);
                 sleep(250);
                 power=0.0;
                 leftFrontMotor.setPower(power);
                 rightFrontMotor.setPower(power);
                 leftBackMotor.setPower(power);
                 rightBackMotor.setPower(power);

                 sleep(1000);
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
                         boolean skystoneFound = false;

                         if (updatedRecognitions.size()>0) {
                             for (Recognition recognition : updatedRecognitions) {
                                if( recognition.getLabel()== LABEL_SECOND_ELEMENT)
                                {
                                skystoneFound=true;
                                 break;
                                }



                             }
                             if (skystoneFound)
                                 break;
                         }
                     }
                 }


             }
 //object detected... Now move to the object//
 //Go forward for 1 second and then stop for ten and shut off//


         }

         leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
         leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
         rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
         rightBackMotor.setDirection(DcMotor.Direction.FORWARD);


         power = 0.5;
         leftFrontMotor.setPower(power);
         rightFrontMotor.setPower(power);
         leftBackMotor.setPower(power);
         rightBackMotor.setPower(power);

         sleep(1000);
          power=0.0;
          leftFrontMotor.setPower(power);
          rightFrontMotor.setPower(power);
          leftBackMotor.setPower(power);
          rightBackMotor.setPower(power);
          sleep(10000);


     }

     /**
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
