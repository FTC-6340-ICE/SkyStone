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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.VuforiaStuff;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ICE_Controls;
import org.firstinspires.ftc.teamcode.ICE_Controls_2_Motors;


@Autonomous(name="Gyrotest", group="Linear Opmode")
//@Disabled
    public class Gyrotest extends ICE_Controls_2_Motors {
        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        // private DcMotor leftDrive = null;
        //private DcMotor rightDrive = null;
        //DcMotor leftMotor;
        //DcMotor rightMotor;
        private VuforiaLocalizer vuforia;
    public VuforiaStuff vuforiaStuff;
        @Override
        public void runOpMode() {
        //initializeHardware();
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
            vuforiaStuff = new VuforiaStuff(vuforia);
            VuforiaStuff.skystonePos pos;
            pos = vuforiaStuff.vuforiascan(true, true);

            telemetry.addData("Position of Stone" , pos );
            telemetry.update();





            waitForStart();

            runtime.reset();
            //leftMotor.setTargetPositionTolerance(1);
            //rightMotor.setTargetPositionTolerance(1);
            //telemetry.addData("Status", "G1");
            //telemetry.update();
            //gyroDrive(DRIVE_SPEED,24);

          // telemetry.addData("Status", "G2");
            //elemetry.update();
            //double TURN_SPEED=0.8;
//gyroDrive(DRIVE_SPEED,30,0);
//servoright.setPosition(0);
//ervoleft.setPosition(1);
//sleep(1500);
  //        servoright.setPosition(1);
    //gyroDrive(DRIVE_SPEED,12,0,5);
            while(opModeIsActive()) {
                gyroHold(DRIVE_SPEED, 0, 2);
                telemetry.addData("movecountsintakemotorRIght", intakeMotorRight.getCurrentPosition());
                telemetry.addData("movecountsIntakeMotorLeft", intakeMotorLeft.getCurrentPosition());
                telemetry.addData("movecountsLeftMotor", leftMotor.getCurrentPosition());
                telemetry.addData("movecountsRightMotor", rightMotor.getCurrentPosition());

                telemetry.update();
                sleep(3000);

            }


            //        servoleft.setPosition(0);
    // servoleft.setPostio
      //      sleep(1500);
//gyroDrive(DRIVE_SPEED,-30,0);
            //telemetry.addData("Current Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            //telemetry.update();
/*




            gyroHold(TURN_SPEED,0,2);
            gyroDrive(DRIVE_SPEED,12,0);
            gyroTurn(TURN_SPEED,90,5);
            gyroHold(TURN_SPEED,90,5);
            gyroDrive(DRIVE_SPEED,18,90);
            gyroTurn(TURN_SPEED,0, 5);
            gyroHold(TURN_SPEED,0,5);
            gyroDrive(DRIVE_SPEED,8,0);
*/


          /*  telemetry.addData("Status", "G3");
            telemetry.update();
            gyroDrive(DRIVE_SPEED,12,0,10);







  /*


       //gyroHold(TURN_SPEED,0,3);
            //gyroDrive(0.2,17,0);
            int a = 1;
            leftMotor.setTargetPositionTolerance(0);
            rightMotor.setTargetPositionTolerance(0);
            leftMotor.setTargetPosition(a);
            rightMotor.setTargetPosition(a);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("Current Position", leftMotor.getCurrentPosition());
            telemetry.update();

            leftMotor.setPower(0.2);
            rightMotor.setPower(0.2);
*/
            /*
            sleep(2000);
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
*/
/*            sleep(5000);

            telemetry.addData("Current New Position", leftMotor.getCurrentPosition());
            telemetry.update();

            sleep(5000);

            telemetry.addData("Status", "going forward and turning");

            telemetry.update();
            sleep(5000);
            //gyroDrive(0.2,10,90);
*/

        }
    }
