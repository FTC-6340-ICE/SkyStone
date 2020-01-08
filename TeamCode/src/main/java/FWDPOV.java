import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ICE_Controls_2Motors;
import org.firstinspires.ftc.teamcode.ICE_Controls_2_Motors;


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

@TeleOp(name="FWDPOV", group="Linear Opmode")
//@Disabled
public class FWDPOV extends ICE_Controls_2_Motors {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        initializeHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeMotorRight.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
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

            //  if (gamepad1.a) {
           // servoleft.setPosition(0.25);

            //}
            //if (gamepad1.b) {
            //servoleft.setPosition(1);


            //

            if (digitalTouch.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                stopInTakeStone();
            }

            if (gamepad2.a) {
                inTakeStone();

            }
            if (gamepad2.y) {
                ouTakeStone();
            }

            if (gamepad2.x) {
                stopInTakeStone();
            }

            if (gamepad2.right_bumper){
                servoleft.setPosition(1);
                servoright.setPosition(0);

            }

            if (gamepad2.left_bumper){
                servoleft.setPosition(0);
                servoright.setPosition(1);

            }

            if (gamepad1.right_trigger < .5) {
                    // Send calculated power to wheels
                    leftMotor.setPower(leftPower);
                    rightMotor.setPower(rightPower);
            } else {
                    leftMotor.setPower(leftTurboPower);
                    rightMotor.setPower(rightTurboPower);

            }
            if (gamepad1.b) {
                servoCapStone.setPosition(0.0);

            }
            if (gamepad1.a) {
                servoCapStone.setPosition(1.0);

            }
            if (gamepad1.y) {
                servoCapStone2.setPosition(0.0);

            }
            if (gamepad1.x) {
                servoCapStone2.setPosition(1.0);

            }





            // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.update();
            }
        }
    }

