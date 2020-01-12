import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.ICE_Controls_2_Motors;
import org.firstinspires.ftc.teamcode.VuforiaStuff;

import java.util.List;

//name of program that shows up on phone,group linear of iterative//
@Autonomous(name="TwoStoneV4WithVuforia", group="Linear Opmode")
//@Disabled
public abstract class Two_Stone_V4_WithVuforia extends ICE_Controls_2_Motors {

    public Two_Stone_V4_WithVuforia(int TeamColor)
    {
       super();
       teamColor = TeamColor;
    }
    private ElapsedTime runtime = new ElapsedTime();
    //if(public int ElapsedTime)
    //teamColor  = 1 for Red an -1 or Blue team
    private int teamColor;
    private final double SKYSTONE_DETECT_MAX_TIME_SECONDS = 5.0;
    private double CameraOffSetDistaceFromMiddle =1;
    private double CameraDistanceAwayFromBack =10;
    private double CameraDistanceToStones = 37;
   // private double DistanceToMoveForwardFromBackWall = 15;//18
    private double DistanceToGoForwardForFirstStoneIntake = 40;
    private double DistanceToComeBackAfterFirstStoneIntake = 20;
    private double DistanceToGoForwardForSecondStoneIntake = 28;
    private double DistanceToComeBackAfterSecondStoneIntake = 26;

    private double DistanceToGoBackBeforeDetectingSecondSkyStone = 6;
    private boolean inDebugMode = false;
    private VuforiaLocalizer vuforia;
    public VuforiaStuff vuforiaStuff;


    @Override
//what shows up on your phone//
    public void runOpMode() {
        initializeHardware();
       // initVuforia();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforiaStuff = new VuforiaStuff(vuforia);

        //servoleft.setPosition(0.5);
        telemetry.addData("On Our Way To the Stone", "SkyStone");
        telemetry.update();
        double adjacentSide = CameraDistanceToStones;
        double oppositeSide = 8;//Width of a Stone
       // double angleOffsetToSideStoneThreshold = Math.toDegrees(Math.atan(oppositeSide/adjacentSide))/2;
        double angleToTurnToForFirstStone = Math.toDegrees(Math.atan(oppositeSide / adjacentSide)) ;
     //   double angleToAdjustDueToPhoneOffset = Math.toDegrees(Math.atan(CameraOffSetDistaceFromMiddle/adjacentSide));
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
       // initVuforia();
        //If phone is comapatible start: if not say text on phone//
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
            //double TURN_SPEED=1.0;
            //double DRIVE_SPEED=0.7;

//            gyroHold(HOLD_SPEED,0,2);
            gyroHold(HOLD_SPEED,0,1);
            //gyroDrive(DRIVE_SPEED,DistanceToMoveForwardFromBackWall,0,2);
            //gyroHold(HOLD_SPEED,0,2);
            VuforiaStuff.skystonePos pos = null;
            if(teamColor == 1)
                pos = vuforiaStuff.vuforiascan(true, true);
            else
                pos = vuforiaStuff.vuforiascan(true, false);



           // CameraDevice.getInstance().setFlashTorchMode(true);
            ElapsedTime timeToLookForSkyStone = new ElapsedTime();
            timeToLookForSkyStone.reset();

//Fird and Move First Skystone
            while (opModeIsActive()) {
             //   RecognizedObject recognizedObject = DetectSkyStoneAndReturnAngle();
             //   if ((recognizedObject.skystoneFound)||(timeToLookForSkyStone.seconds()> SKYSTONE_DETECT_MAX_TIME_SECONDS)) {


                    if (pos == VuforiaStuff.skystonePos.CENTER) {
                        angleToTurnToForFirstStone = 0;
                    }
                    if (pos == VuforiaStuff.skystonePos.RIGHT) {
                        angleToTurnToForFirstStone = (angleToTurnToForFirstStone * -1);


                    } else {
                        angleToTurnToForFirstStone = angleToTurnToForFirstStone;


                    }






                    double distanceToDropOffSkystone = 0;
                    double distanceBackToCenterLine = 0;
                    double distanceBackToSecondStone = 0;
                   //right side664

                    if (pos == VuforiaStuff.skystonePos.RIGHT) {
                        if(teamColor==1) {
                            //red side
                            distanceToDropOffSkystone = 35;
                            distanceBackToCenterLine = -10;
                            distanceBackToSecondStone = -68;
                        }
                        else {
                            //blue side
                            distanceToDropOffSkystone = 50;
                            distanceBackToCenterLine = -10;
                            distanceBackToSecondStone = -71;
                        }
                        //angleToTurnTo -= 4;
                    }
        //Left Side
                    else if (pos == VuforiaStuff.skystonePos.LEFT) {
                        if(teamColor==1) {
                            //red side
                            distanceToDropOffSkystone = 35;
                            distanceBackToCenterLine = -15;
                            distanceBackToSecondStone = -72;
                        }
                        else
                        {
                            //blue sides
                            distanceToDropOffSkystone = 54;
                            distanceBackToCenterLine = -15;
                            distanceBackToSecondStone = -72;
                        }

                    }
//Middle
                    else{
                        distanceToDropOffSkystone=54;
                        distanceBackToCenterLine=-10;
                        distanceBackToSecondStone=-75;
                        DistanceToGoForwardForFirstStoneIntake = 45;
                    }

                    telemetry.addData(">", "Skystone FOUND!!!!!");
                    telemetry.addData("NEW AngleToTurnTo =", angleToTurnToForFirstStone);
                    telemetry.update();
  //                  sleep(5000);

                    gyroTurn(TURN_SPEED,angleToTurnToForFirstStone,5);
                    gyroHold(HOLD_SPEED,angleToTurnToForFirstStone,0.5);
                    inTakeStone();
                    gyroDrive(DRIVE_SPEED, DistanceToGoForwardForFirstStoneIntake, angleToTurnToForFirstStone,2);
                    stopInTakeStone();
                    gyroTurn(TURN_SPEED,0,5);

                    gyroHold(HOLD_SPEED,0,0.5);

                    gyroDrive(DRIVE_SPEED_BOOST, -1*DistanceToComeBackAfterFirstStoneIntake, 0,5);
                    //turning right
                   // stopInTakeStone();
                    gyroTurn(TURN_SPEED,-90*teamColor,5);
                    gyroHold(HOLD_SPEED,-90*teamColor,0.5);
                    gyroDrive(DRIVE_SPEED_BOOST_AUTONOMOUS,distanceToDropOffSkystone, -90*teamColor,5);
                    ouTakeStoneForAutonomous(-90*teamColor,2,-5);
                    //Put's servo up to deliver stone
                    //servoleft.setPosition(1.0);
                    //servoright.setPosition(0.0);
                    sleep(500);

                    stopInTakeStone();
                    gyroHold(HOLD_SPEED,-80*teamColor,0.5);

                gyroTurn(HOLD_SPEED,-80*teamColor,0.5);

                    gyroDrive(DRIVE_SPEED_BOOST,distanceBackToSecondStone,-80*teamColor);
                    gyroTurn(TURN_SPEED,0,3);
                    gyroHold(HOLD_SPEED,0,0.5);
                    //gyroDrive(DRIVE_SPEED,-1*DistanceToGoBackBeforeDetectingSecondSkyStone,0,2);
                    //gyroDrive(DRIVE_SPEED,5,0);
                    break;

              // }
            }

            timeToLookForSkyStone.reset();
//Find and Move Second Skystone
            while (opModeIsActive()) {
                //keep attempting to find stone//
               // RecognizedObject recognizedObject = DetectSkyStoneAndReturnAngle();
//                if (recognizedObject.skystoneFound||(timeToLookForSkyStone.seconds()> SKYSTONE_DETECT_MAX_TIME_SECONDS)) {
//shows us angle to turn to//
                    //double angleToTurnTo = -1 * stonerecognition.estimateAngleToObject(AngleUnit.DEGREES);
                    double angleToTurnToForSecondStone;
                      angleToTurnToForSecondStone = 23;

                    double distanceToDropOffSkystone = 0;
                    double distanceBackToCenterLine = 0;
                    double distanceBackToSecondStone = 0;
                    boolean turnOnlyOneAtIntake = false;
                    if (pos == VuforiaStuff.skystonePos.CENTER) {
                        angleToTurnToForSecondStone = 0;
                    }
                    if (pos == VuforiaStuff.skystonePos.RIGHT) {
                        angleToTurnToForSecondStone = (angleToTurnToForSecondStone * -1);


                    } else {
                        angleToTurnToForSecondStone = angleToTurnToForSecondStone;


                    }



                if (pos == VuforiaStuff.skystonePos.RIGHT) {

                    if(teamColor==1) {
                            //red side
                            distanceToDropOffSkystone = 65;
                            distanceBackToCenterLine = -25;
                            distanceBackToSecondStone = -50;
                        }
                        else
                        {
                            //blue side
                            distanceToDropOffSkystone = 71;
                            distanceBackToCenterLine = -20;
                            distanceBackToSecondStone = -50;
                        }
                        if(teamColor==-1) {
                            angleToTurnToForSecondStone = 0 * teamColor;
                         //   turnOnlyOneAtIntake = true;
                        }

                    }
                else if (pos == VuforiaStuff.skystonePos.LEFT) {
                        if(teamColor==1) {
                            distanceToDropOffSkystone = 55;
                            distanceBackToCenterLine = -20;
                            distanceBackToSecondStone = -55;
                        }
                        else {
                            distanceToDropOffSkystone = 71;
                            distanceBackToCenterLine = -30;
                            distanceBackToSecondStone = -55;
                        }
                    if(teamColor==1) {
                        angleToTurnToForSecondStone = 0 * teamColor;
                    }

                    }
                    else{
                        distanceToDropOffSkystone=71;
                        distanceBackToCenterLine=-30;
                        distanceBackToSecondStone=-50;
                    }


                    gyroTurn(TURN_SPEED, angleToTurnToForSecondStone, 5);
                    gyroHold(HOLD_SPEED, angleToTurnToForSecondStone, 0.5);
                    inTakeStone(turnOnlyOneAtIntake,teamColor);
                    inTakeStone();

                    gyroDrive(DRIVE_SPEED, DistanceToGoForwardForSecondStoneIntake, angleToTurnToForSecondStone,2);
                    stopInTakeStone();
                    //turns -90 degrees  and holds there for 5 seconds
                    gyroTurn(TURN_SPEED,0,5);

                    gyroHold(HOLD_SPEED,0,0.5);

                    gyroDrive(DRIVE_SPEED_BOOST, -1*DistanceToComeBackAfterSecondStoneIntake, 0,5);


                    gyroTurn(TURN_SPEED,-90*teamColor,5);
                    gyroHold(HOLD_SPEED,-90*teamColor,0.5);

                    gyroDrive(DRIVE_SPEED_BOOST_AUTONOMOUS,distanceToDropOffSkystone, -90*teamColor,5);
                    //Put's servo up to deliver stone
                    ouTakeStoneForAutonomous(-90*teamColor,2,2);
                   // servoleft.setPosition(1.0);
                    //servoright.setPosition(0.0);
                    sleep(500);
                    //TURN_SPEED=1.0;
                    stopInTakeStone();
                    gyroDrive(1.0,distanceBackToCenterLine,-90*teamColor,5);
                    //gyroTurn(TURN_SPEED,0,5);
                    //gyroDrive(DRIVE_SPEED,5,0);
                    break;

                //}
            }
            //CameraDevice.getInstance().setFlashTorchMode(false);

        }
    }


  //  protected RecognizedObject DetectSkyStoneAndReturnAngle(){
    //    RecognizedObject recognizedObjectToReturn = new RecognizedObject(false,0,null);
      //  if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
        //    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//If you find stone then say on DS Object detected//
          //  if (updatedRecognitions != null) {
            //    telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
              //  int i = 0;
                //for (Recognition recognition : updatedRecognitions) {
                  //  telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    //telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                      //      recognition.getLeft(), recognition.getTop());
                    //telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                      //      recognition.getRight(), recognition.getBottom());
                //}
              //  telemetry.update();
                //Found Object.....Now break the while loop
//                Recognition stonerecognition = null;
               // skystoneFound = false;

  //              if (updatedRecognitions.size() > 0) {
    //                for (Recognition recognition : updatedRecognitions) {
      //                  if (recognition.getLabel() == LABEL_SECOND_ELEMENT) {
        ////                    skystoneFound = true;
            //                stonerecognition = recognition;
              //              break;
                //        }


                  //  }
                    //if (skystoneFound){
                      //  recognizedObjectToReturn.skystoneFound = skystoneFound;
                        //recognizedObjectToReturn.skystoneFoundAngle = stonerecognition.estimateAngleToObject(AngleUnit.DEGREES);
                        //recognizedObjectToReturn.stoneRecognition = stonerecognition;
                       // return recognizedObjectToReturn;
                        //return stonerecognition.estimateAngleToObject(AngleUnit.DEGREES);

                    //}
                    //    break;
               // }
            //}
        //}
   //     return recognizedObjectToReturn;
    //}
//end of code/
  /**
     *
     * Initialize the Vuforia localization engine.
     */


}
