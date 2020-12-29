import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ICE_Controls_2_Motors;
import org.firstinspires.ftc.teamcode.ICE_Controls_4_Motors;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MecanumPOV", group="Linear Opmode")
//@Disabled
public class MecanumPoV extends ICE_Controls_4_Motors {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        initializeHardware();
        servoLeftRight.setPosition(0.0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double leftTurboPower;
            double rightTurboPower;

            double turboMaxSpeed = 0.9;
            double maxSpeed = 0.5;
            // Choose to drive using either Tank Mode, or org.firstinspires.ftc.teamcode.POV Mode
            // Comment out the method that's not used.  The default below is org.firstinspires.ftc.teamcode.POV.

            // org.firstinspires.ftc.teamcode.POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1 * maxSpeed, maxSpeed);
            rightPower = Range.clip(drive - turn, -1 * maxSpeed, maxSpeed);
            leftTurboPower = Range.clip(drive + turn, -1 * turboMaxSpeed, turboMaxSpeed);
            rightTurboPower = Range.clip(drive - turn, -1 * turboMaxSpeed, turboMaxSpeed);









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
            if(!gamepad1.left_bumper && !gamepad1.right_bumper && gamepad1.right_trigger < .5   ) {

                rightMotorFront.setPower(rightPower);
                rightMotorBack.setPower(rightPower);
                leftMotorBack.setPower(leftPower);
                leftMotorFront.setPower(leftPower);

            }












            if (gamepad2.right_bumper){
                servoleft.setPosition(1);
                servoright.setPosition(0);

            }

            if (gamepad2.left_bumper){
                servoleft.setPosition(0);
                servoright.setPosition(1);

            }

            if (gamepad1.right_trigger > .5) {
                leftMotorBack.setPower(leftTurboPower);
                rightMotorBack.setPower(rightTurboPower);
                leftMotorFront.setPower(leftTurboPower);
                rightMotorFront.setPower(rightTurboPower);

            }
            if (gamepad2.dpad_down) {
                servoLeftRight.setPosition(-0.5);

            }
            if (gamepad2.dpad_up) {
                servoLeftRight.setPosition(1.0);

            }



            if (gamepad2.dpad_down) {
                servoLeftRight.setPosition(0.0);

            }
            if (gamepad2.dpad_left) {
                servoLeftRight.setPosition(0.3);

            }

            if (gamepad2.back) {
                servoDrop.setPosition(0.0);

            }
            else{
                servoDrop.setPosition(1.0);

            }

            if (gamepad1.dpad_down) {
                servoGiddyUp.setPosition(0.0);

            }
            if (gamepad1.dpad_up) {
              servoGiddyUp.setPosition(1.0);

            }







            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}

