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
import org.firstinspires.ftc.teamcode.VuforiaStuff;

import java.util.List;


@Autonomous(name="GetFoundationV2WithVuforia", group="Linear Opmode")
//@Disabled
    public abstract class GetFoundationV2WithVufroria extends ICE_Controls_2_Motors {
    // Declare OpMode members.
    public GetFoundationV2WithVufroria(int TeamColor) {
        super();
        teamColor = TeamColor;
    }

    private int teamColor;
    private ElapsedTime runtime = new ElapsedTime();

    private final double SKYSTONE_DETECT_MAX_TIME_SECONDS = 5.0;
    private double CameraOffSetDistaceFromMiddle = 1;
    private double CameraDistanceAwayFromBack = 10;
    private double CameraDistanceToStones = 35;
    private double DistanceToMoveForwardFromBackWall = 15;//18
    private double DistanceToGoForwardForStoneIntake = 36;
    private double DistanceToComeBackAfterStoneIntake = 25;
    private double DistanceToGoBackBeforeDetectingSecondSkyStone = 10;
    private double AngleToTurnToForTelemetry = 0;
    // private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;
    //DcMotor leftMotor;
    //DcMotor rightMotor;
    private VuforiaLocalizer vuforia;
    public VuforiaStuff vuforiaStuff;

    @Override
    public void runOpMode() {
        initializeHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforiaStuff = new VuforiaStuff(vuforia);

        //  Instantiate the Vuforia engine


        double adjacentSide = CameraDistanceToStones - DistanceToMoveForwardFromBackWall;
        double oppositeSide = 8;//Width of a Stone
        double angleToTurnTo = Math.toDegrees(Math.atan(oppositeSide / adjacentSide)) / 2;

        double angleToAdjustDueToPhoneOffset = Math.toDegrees(Math.atan(CameraOffSetDistaceFromMiddle / adjacentSide));
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        //initVuforia();
        //If phone is comapatible start: if not say text on phone//
        //if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
        //initTfod();
        // } else {


        waitForStart();

        runtime.reset();
        if (digitalTouch.getState() == true) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {

        }

        servoleft.setPosition(1);
        servoright.setPosition(0);
        // gyroHold(DRIVE_SPEED,0,1);
        gyroTurn(TURN_SPEED_BOOST, -35 * teamColor, 1);
        gyroDrive(DRIVE_SPEED_BOOST, 20 * -1, -35 * teamColor, 2);
        // gyroDrive(.3,15*-1,,2);
        //gyroDriveStopOnTouchSensor(.3,15*-1,0,2);

        //gyroHold(DRIVE_SPEED,0,1);
        gyroTurn(TURN_SPEED_BOOST, 0, 1);
        gyroHold(DRIVE_SPEED, 0, .5);
        gyroDrive(.3, 16 * -1, 0, 2);

        servoleft.setPosition(0);
        servoright.setPosition(1);
        gyroHold(DRIVE_SPEED, 0, 1);

        gyroDrive(DRIVE_SPEED_BOOST, 33, 0, 5);

        servoleft.setPosition(1);
        servoright.setPosition(0);
        gyroHold(DRIVE_SPEED, 0, 1);

        if (teamColor == 1) {
            intakeMotorLeft.setPower(1.0);
            intakeMotorRight.setPower(0.0);
        } else {
            intakeMotorLeft.setPower(0.0);
            intakeMotorRight.setPower(1.0);

        }
        sleep(500);
        gyroTurn(TURN_SPEED_BOOST, -90 * teamColor, 5);
        gyroDrive(DRIVE_SPEED, 25, -90 * teamColor, 2);
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

        gyroTurn(TURN_SPEED_BOOST, -135 * teamColor, 5);
        gyroDrive(DRIVE_SPEED_BOOST, 20, -135 * teamColor, 2);
        gyroTurn(TURN_SPEED_BOOST, -90 * teamColor, 5);

        //Skystone detect code start
        gyroDrive(DRIVE_SPEED_BOOST, 40, -80 * teamColor);
//teting code end
        gyroTurn(TURN_SPEED_BOOST, -180, 3);
        gyroHold(HOLD_SPEED, -180, 0.5);
        gyroDrive(DRIVE_SPEED_BOOST, -1 * DistanceToGoBackBeforeDetectingSecondSkyStone, -180, 2);

        ElapsedTime timeToLookForSkyStone = new ElapsedTime();
        timeToLookForSkyStone.reset();
//Find and Move Second Skystone


        while (opModeIsActive()) {

            VuforiaStuff.skystonePos pos = null;
            pos = vuforiaStuff.vuforiascan(true, true);
            double distanceToDropOffSkystone = 0;
            double distanceBackToCenterLine = 0;
            double distanceBackToSecondStone = 0;
            boolean turnOnlyOneAtIntake = false;


            if (pos == VuforiaStuff.skystonePos.CENTER) {
                angleToTurnTo = 0;
            }
            if (pos == VuforiaStuff.skystonePos.RIGHT) {
                angleToTurnTo = (angleToTurnTo * -1)-180;


            } else {
                angleToTurnTo = angleToTurnTo-180;


            }




            if (pos == VuforiaStuff.skystonePos.RIGHT) {
                if (teamColor == 1) {
                    //red side
                    distanceToDropOffSkystone = 54;
                    distanceBackToCenterLine = -20;
                    distanceBackToSecondStone = -50;
                } else {
                    //blue side
                    distanceToDropOffSkystone = 54;
                    distanceBackToCenterLine = -20;
                    distanceBackToSecondStone = -50;
                }


            } else if (pos == VuforiaStuff.skystonePos.LEFT) {
                if (teamColor == 1) {
                    distanceToDropOffSkystone = 54;
                    distanceBackToCenterLine = -25;
                    distanceBackToSecondStone = -55;
                } else {
                    distanceToDropOffSkystone = 54;
                    distanceBackToCenterLine = -25;
                    distanceBackToSecondStone = -55;
                }
            } else {
                distanceToDropOffSkystone = 54;
                distanceBackToCenterLine = -25;
                distanceBackToSecondStone = -50;
            }


            //sleep(5000);

            gyroTurn(TURN_SPEED, angleToTurnTo, 5);
            gyroHold(HOLD_SPEED, angleToTurnTo, 0.5);
            inTakeStone(turnOnlyOneAtIntake, teamColor);
            gyroDrive(DRIVE_SPEED, 24, angleToTurnTo, 2);
            inTakeStone();

            gyroDrive(DRIVE_SPEED, DistanceToGoForwardForStoneIntake - 24, angleToTurnTo, 2);
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
            gyroDrive(DRIVE_SPEED_BOOST, -1 * DistanceToComeBackAfterStoneIntake, angleToTurnTo, 5);
            //turning right
            gyroTurn(TURN_SPEED_BOOST, -270 * teamColor, 5);
            gyroHold(HOLD_SPEED, -270 * teamColor, 0.5);

            gyroDrive(DRIVE_SPEED_BOOST, distanceToDropOffSkystone, -270 * teamColor, 5);
            //Put's servo up to deliver stone
            // gyroTurn(TURN_SPEED,-235,2);
            ouTakeStoneForAutonomous(-270 * teamColor, 2, 2);
            servoleft.setPosition(1.0);
            servoright.setPosition(0.0);
            sleep(1000);
            //TURN_SPEED=1.0;
            stopInTakeStone();
            gyroDrive(1.0, distanceBackToCenterLine, -270 * teamColor, 5);
            gyroHold(1.0, -270 * teamColor, 5);

            }
        }

}



