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


@Autonomous(name="GetFoundationForCircutBreakers", group="Linear Opmode")
@Disabled
    public abstract class GetFoundationForCircutBreakers extends ICE_Controls_2_Motors {
    // Declare OpMode members.
    public GetFoundationForCircutBreakers(int TeamColor) {
        super();
        teamColor = TeamColor;
    }

    private int teamColor;
    private ElapsedTime runtime = new ElapsedTime();

    // private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;
    //DcMotor leftMotor;
    //DcMotor rightMotor;
    @Override
    public void runOpMode() {
        initializeHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
       // first.
        waitForStart();

        runtime.reset();

        servoleft.setPosition(1);
        servoright.setPosition(0);
        // gyroHold(DRIVE_SPEED,0,1);
       if(teamColor == 1) {
           gyroTurn(TURN_SPEED_BOOST, -35 * teamColor, 1);
           gyroDrive(DRIVE_SPEED_BOOST, 22 * -1, -35 * teamColor, 2);
       }
       else {
           gyroTurn(TURN_SPEED_BOOST, -45 * teamColor, 1);

           gyroDrive(DRIVE_SPEED_BOOST, 22 * -1, -45 * teamColor, 2);
       }
        // gyroDrive(.3,15*-1,,2);
        //gyroDriveStopOnTouchSensor(.3,15*-1,0,2);

        //gyroHold(DRIVE_SPEED,0,1);
        gyroTurn(TURN_SPEED_BOOST, 0, 1);
        gyroHold(DRIVE_SPEED, 0, .5);
        gyroDrive(.3, 18 * -1, 0, 2);

        servoleft.setPosition(0);
        servoright.setPosition(1);
        gyroHold(DRIVE_SPEED, 0, 1);

        gyroDrive(DRIVE_SPEED_BOOST, 37, 0, 5);

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
        gyroDrive(DRIVE_SPEED, 35, -90 * teamColor, 2);
        intakeMotorLeft.setPower(0.0);
        intakeMotorRight.setPower(0.0);

    }
}
