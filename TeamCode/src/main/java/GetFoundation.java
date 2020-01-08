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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.ICE_Controls_2_Motors;

import java.util.List;


@Autonomous(name="GetFoundation", group="Linear Opmode")
//@Disabled
    public abstract class GetFoundation extends ICE_Controls_2_Motors {
        // Declare OpMode members.
        public GetFoundation(int TeamColor)
        {
            super();
            teamColor = TeamColor;
        }
    private int teamColor;
    private ElapsedTime runtime = new ElapsedTime();

    private final double SKYSTONE_DETECT_MAX_TIME_SECONDS = 5.0;
    private double CameraOffSetDistaceFromMiddle =1;
    private double CameraDistanceAwayFromBack =10;
    private double CameraDistanceToStones = 35;
    private double DistanceToMoveForwardFromBackWall = 15;//18
    private double DistanceToGoForwardForStoneIntake = 36;
    private double DistanceToComeBackAfterStoneIntake = 25;
    private double DistanceToGoBackBeforeDetectingSecondSkyStone =5;
    private double AngleToTurnToForTelemetry = 0;
    // private DcMotor leftDrive = null;
        //private DcMotor rightDrive = null;
        //DcMotor leftMotor;
        //DcMotor rightMotor;
        @Override
        public void runOpMode() {
        initializeHardware();
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            double adjacentSide = CameraDistanceToStones - DistanceToMoveForwardFromBackWall;
            double oppositeSide = 8;//Width of a Stone
            double angleOffsetToSideStoneThreshold = Math.toDegrees(Math.atan(oppositeSide/adjacentSide))/2;

            double angleToAdjustDueToPhoneOffset = Math.toDegrees(Math.atan(CameraOffSetDistaceFromMiddle/adjacentSide));
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


            waitForStart();

            runtime.reset();
            if (digitalTouch.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {

            }

            servoleft.setPosition(1);
            servoright.setPosition(0);
           // gyroHold(DRIVE_SPEED,0,1);
            if(teamColor == 1) {
                intakeMotorLeft.setPower(0.5);
                intakeMotorRight.setPower(0.0);
            }
            else{
                intakeMotorLeft.setPower(0.0);
                intakeMotorRight.setPower(0.5);

            }


            gyroTurn(TURN_SPEED_BOOST,-35*teamColor,1);
            gyroDrive(DRIVE_SPEED_BOOST,20*-1,-35*teamColor,2);
            gyroTurn(TURN_SPEED_BOOST,0,1 );
            gyroHold(DRIVE_SPEED,0,.5);
            gyroDrive(.3,16*-1,0,2);

            servoleft.setPosition(0);
            servoright.setPosition(1);
            gyroHold(DRIVE_SPEED,0,1);

            gyroDrive(DRIVE_SPEED_BOOST,33,0,5);

            servoleft.setPosition(1);
            servoright.setPosition(0);
            gyroHold(DRIVE_SPEED,0,1);

            if(teamColor == 1) {
                intakeMotorLeft.setPower(1.0);
                intakeMotorRight.setPower(0.0);
            }
            else{
                intakeMotorLeft.setPower(0.0);
                intakeMotorRight.setPower(1.0);

            }
            sleep(500);
            gyroTurn(TURN_SPEED_BOOST,-90*teamColor,5);
            gyroDrive(DRIVE_SPEED,25,-90*teamColor,2);
            intakeMotorLeft.setPower(0.0);
            intakeMotorRight.setPower(0.0);

/*WOrking code
            gyroTurn(TURN_SPEED_BOOST,-180*teamColor,5);
            gyroDrive(DRIVE_SPEED,12,-180*teamColor,2);
            gyroTurn(TURN_SPEED_BOOST,-270*teamColor,5);
            gyroDrive(DRIVE_SPEED_BOOST,15,-270*teamColor,2);

            //Skystone detect code start
            gyroDrive(DRIVE_SPEED_BOOST,-62,-260*teamColor);
            */
//testing code start

            gyroTurn(TURN_SPEED_BOOST,-135*teamColor,5);
            gyroDrive(DRIVE_SPEED_BOOST,20,-135*teamColor,2);
            gyroTurn(TURN_SPEED_BOOST,-90*teamColor,5);

            //Skystone detect code start
            gyroDrive(DRIVE_SPEED_BOOST,36,-80*teamColor);
//teting code end
            gyroTurn(TURN_SPEED_BOOST,-180,3);
            gyroHold(HOLD_SPEED,-180,0.5);
            gyroDrive(DRIVE_SPEED_BOOST,-1*DistanceToGoBackBeforeDetectingSecondSkyStone,-180,2);

            ElapsedTime timeToLookForSkyStone = new ElapsedTime();
            timeToLookForSkyStone.reset();
//Find and Move Second Skystone
            while (opModeIsActive()) {
                //keep attempting to find stone//
                RecognizedObject recognizedObject = DetectSkyStoneAndReturnAngle();
                if (recognizedObject.skystoneFound||(timeToLookForSkyStone.seconds()> SKYSTONE_DETECT_MAX_TIME_SECONDS)) {
//shows us angle to turn to//
                    //double angleToTurnTo = -1 * stonerecognition.estimateAngleToObject(AngleUnit.DEGREES);
                    double angleToTurnTo;
                    if(recognizedObject.skystoneFound) {
                        angleToTurnTo = -1 * (recognizedObject.skystoneFoundAngle-angleToAdjustDueToPhoneOffset);
                    }
                    else
                    {
                        angleToTurnTo = 0;
                    }

                    telemetry.addData(">", "Skystone FOUND!!!!!");
                    telemetry.addData("AngleToTurnTo =", angleToTurnTo);
                    telemetry.update();

                //  sleep(5000);
                    double angleToAddToAngle = -1 * imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    angleToTurnTo -= angleToAddToAngle;
                    /*
                    telemetry.addData(">", "Skystone FOUND 2");
                    telemetry.addData("AngleToTurnTo =", angleToTurnTo);
                    telemetry.addData("angleToAddToAngle =", angleToAddToAngle);

                    telemetry.update();
                    */

                  //  sleep(5000);

                    double distanceToDropOffSkystone = 0;
                    double distanceBackToCenterLine = 0;
                    double distanceBackToSecondStone = 0;
                    boolean turnOnlyOneAtIntake = false;
                    if(angleToTurnTo<-1*angleOffsetToSideStoneThreshold){
                        if(teamColor==1) {
                            //red side
                            distanceToDropOffSkystone = 50;
                            distanceBackToCenterLine = -20;
                            distanceBackToSecondStone = -50;
                        }
                        else
                        {
                            //blue side
                            distanceToDropOffSkystone = 50;
                            distanceBackToCenterLine = -20;
                            distanceBackToSecondStone = -50;
                        }
                        //angleToTurnTo -= 4;
                        /*
                        if(teamColor==-1) {
                            angleToTurnTo = 30 * teamColor;
                            turnOnlyOneAtIntake = true;
                        }

                         */

                    }
                    else if(angleToTurnTo>angleOffsetToSideStoneThreshold){
                        if(teamColor==1) {
                            distanceToDropOffSkystone = 50;
                            distanceBackToCenterLine = -25;
                            distanceBackToSecondStone = -55;
                        }
                        else {
                            distanceToDropOffSkystone = 50;
                            distanceBackToCenterLine = -25;
                            distanceBackToSecondStone = -55;
                        }
                        // angleToTurnTo += 4;
                        /*
                        if(teamColor==1) {
                            angleToTurnTo = 30 * teamColor;
                            turnOnlyOneAtIntake=true;
                        }
                        */
                    }
                    else{
                        distanceToDropOffSkystone=50;
                        distanceBackToCenterLine=-25;
                        distanceBackToSecondStone=-50;
                    }

                    /*
                    if(angleToTurnTo<-5) {
                        gyroTurn(TURN_SPEED,angleToTurnTo,5);
                        gyroHold(HOLD_SPEED,angleToTurnTo,0.5);

                        gyroDrive(DRIVE_SPEED, 8, angleToTurnTo,5);
                        gyroTurn(TURN_SPEED,0,5);
                        gyroHold(HOLD_SPEED,0,0.5);
                        angleToTurnTo =0;
                    }
                    else {
                        //TURN_SPEED = 0.5;
                        gyroTurn(TURN_SPEED, angleToTurnTo, 5);
                        gyroHold(HOLD_SPEED, angleToTurnTo, 0.5);
                    }*/
                    AngleToTurnToForTelemetry = angleToTurnTo;
                    telemetry.addData(">", "Skystone FOUND!!!!!");
                    telemetry.addData("AngleToTurnTo =", angleToTurnTo);
                    telemetry.update();
                    //sleep(5000);

                    gyroTurn(TURN_SPEED, angleToTurnTo, 5);
                    gyroHold(HOLD_SPEED, angleToTurnTo, 0.5);
                    inTakeStone(turnOnlyOneAtIntake,teamColor);
                    gyroDrive(DRIVE_SPEED, 24, angleToTurnTo,2);
                    inTakeStone();

                    gyroDrive(DRIVE_SPEED, DistanceToGoForwardForStoneIntake-24, angleToTurnTo,2);
                    servoleft.setPosition(0.25);
                    servoright.setPosition(1.0);

                    //sleep(1350);
                    stopInTakeStone();
                    /*
                    if(servoleft.getPosition() >0.25)
                    {
                        servoleft.setPosition(servoleft.getPosition() +0.01);
                        sleep(1000);
                    }

                     */
                    //turns -90 degrees  and holds there for 5 seconds
                    gyroDrive(DRIVE_SPEED_BOOST, -1*DistanceToComeBackAfterStoneIntake, angleToTurnTo,5);
                    //turning right
                    gyroTurn(TURN_SPEED_BOOST,-270*teamColor,5);
                    gyroHold(HOLD_SPEED,-270*teamColor,0.5);

                    gyroDrive(DRIVE_SPEED_BOOST,distanceToDropOffSkystone, -270*teamColor,5);
                    //Put's servo up to deliver stone
                  // gyroTurn(TURN_SPEED,-235,2);
                    ouTakeStoneForAutonomous(-270*teamColor,2,2);
                    servoleft.setPosition(1.0);
                    servoright.setPosition(0.0);
                    sleep(1000);
                    //TURN_SPEED=1.0;
                    stopInTakeStone();
                    gyroDrive(1.0,distanceBackToCenterLine,-270*teamColor,5);
                    //gyroTurn(TURN_SPEED,0,5);
                    //gyroDrive(DRIVE_SPEED,5,0);
                    break;

                }
            }

            telemetry.addData("AngleToTurnTo =", AngleToTurnToForTelemetry);
            //telemetry.update();
            sleep(5000);
        }


    protected RecognizedObject DetectSkyStoneAndReturnAngle(){
        RecognizedObject recognizedObjectToReturn = new RecognizedObject(false,0,null);
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
                Recognition stonerecognition = null;
                // skystoneFound = false;

                if (updatedRecognitions.size() > 0) {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == LABEL_SECOND_ELEMENT) {
                            skystoneFound = true;
                            stonerecognition = recognition;
                            break;
                        }


                    }
                    if (skystoneFound){
                        recognizedObjectToReturn.skystoneFound = skystoneFound;
                        recognizedObjectToReturn.skystoneFoundAngle = stonerecognition.estimateAngleToObject(AngleUnit.DEGREES);
                        recognizedObjectToReturn.stoneRecognition = stonerecognition;
                        return recognizedObjectToReturn;
                        //return stonerecognition.estimateAngleToObject(AngleUnit.DEGREES);

                    }
                    //    break;
                }
            }
        }
        return recognizedObjectToReturn;
    }
//end of code/
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
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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
        tfodParameters.minimumConfidence = 0.7;
        //what shows on screen//
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    protected class RecognizedObject{

        public boolean skystoneFound;
        public double skystoneFoundAngle;
        public Recognition stoneRecognition;

        public RecognizedObject(boolean skystoneFound,double skystoneFoundAngle,Recognition stoneRecognition){
            this.skystoneFound=skystoneFound;
            this.skystoneFoundAngle=skystoneFoundAngle;
            this.stoneRecognition=stoneRecognition;
        }


    }
}
