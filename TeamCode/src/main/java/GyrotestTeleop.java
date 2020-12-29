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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ICE_Controls_4_Motors;


@TeleOp(name="GyrotestTeleOp", group="Linear Opmode")
//@Disabled
    public class GyrotestTeleop extends ICE_Controls_4_Motors {
        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();

        // private DcMotor leftDrive = null;
        //private DcMotor rightDrive = null;
        //DcMotor leftMotor;
        //DcMotor rightMotor;
     //   private VuforiaLocalizer vuforia;
   // public VuforiaStuff vuforiaStuff;
        @Override
        public void runOpMode() {
        initializeHardware();
     /*       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
*/




            waitForStart();

            runtime.reset();
            while (opModeIsActive()) {

                if(gamepad1.right_bumper) {
                    rightMotorFront.setPower(-1.0);
                    rightMotorBack.setPower(1.0);
                    leftMotorBack.setPower(-1.0);
                    leftMotorFront.setPower(1.0);
                }

                if(gamepad1.left_bumper) {
                    rightMotorFront.setPower(1.0);
                    rightMotorBack.setPower(-1.0);
                    leftMotorBack.setPower(1.0);
                    leftMotorFront.setPower(-1.0);
                }
                if(!gamepad1.left_bumper && !gamepad1.right_bumper) {

                    rightMotorFront.setPower(0);
                    rightMotorBack.setPower(0);
                    leftMotorBack.setPower(0);
                    leftMotorFront.setPower(0);

                }
            /*
                else {
                rightMotorFront.setPower(0);
                rightMotorBack.setPower(0);
                leftMotorBack.setPower(0);
                leftMotorFront.setPower(0);



                }
*/
            }

        }
    }
