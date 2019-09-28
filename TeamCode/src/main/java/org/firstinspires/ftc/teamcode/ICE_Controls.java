package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone

import java.util.List;
import java.util.Locale;

import static java.lang.Math.abs;

/**
 * Created by Team ICE 6340 on 9.23.2019!
 */


public abstract class ICE_Controls<TODO> extends LinearOpMode {


    //Initialize and instantiate vuforia variables
    OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = "AZWzerv/////AAABmZeKo4MkD08MoSz5oHB/JU6N1BsUWpfHgQeAeVZemAypSUGVQhvAHo6+v7kJ3MITd8530MhwxRx7GjRtdCs1qjPmdKiJK66dv0yN4Zh4NvKBfP5p4TJjM+G0GoMVgVK0pItm2U56/SVqQH2AYtczQ+giw6zBe4eNhHPJCMY5C2t5Cs6IxxjZlMkRF85l8YAUlKGLipnoZ1T/mX8DNuThQA57qsIB2EN6pGWe8GI64hcPItQ0j7Oyjp82lEN13rYQYsS3Ur4a6//D6yhwa0rogXAysG68G+VgC1mNlj1CjX60qDI84ZN0b/A081xXqjeyFqZK8A/jO8y7BGz9ZuuZNxxXIon6xRNeKYudpfTD23+5";

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    //Initialize elapsed time object
    private ElapsedTime runtime = new ElapsedTime();

    // The IMU sensor object
    protected BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double absoluteAngle, power = .30, correction;

    // Orientation and acceleration variables from the built in 9-axis accelerometer
    protected Orientation angles;
    protected Acceleration gravity;

    //ROBOT HARDWARE
    //Instantiate chassis motors

    protected DcMotorEx leftMotor;
    protected DcMotorEx rightMotor;

    //Instantiate servos
    //protected Servo ????;

    //Instantiate sensors
    //ColorSensor ?????;

    //Initlize encoder variables
    protected double COUNTS_PER_MOTOR_DRIVE = 1120;    // Rev Hex Motor 288
    protected double COUNTS_PER_MOTOR_LIFT = 1120;    // andyMark 40
    protected double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    protected double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    protected double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_DRIVE * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    protected double COUNTS_PER_INCH_LIFT = (COUNTS_PER_MOTOR_LIFT) / (.677 * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = .3;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = .3;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 2.5;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = .002;     // .02 Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = .05;     // .05 Larger is more responsive, but also less stable

    //

    //Initialize Vuforia variables
     VuforiaTrackables VuforiaSkyStone;
    //VuforiaTrackable relicTemplate;


    protected void initializeHardware() {

        //Give the OK message
        telemetry.addData("Status", "Initializing hardware");
        telemetry.update();


        //Initialize robot hardware
        //Begin with the chassis
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor");

        //Reset the encoders on the chassis to 0
        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //Set the motor modes
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //Reverse the left motor so all motors move forward when set to a positive speed.
        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        //When the motors are told to stop this makes sure you now that there stoped on the screen
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initialize the servos
        // ??? = hardwareMap.get(Servo.class, "???");

        //Initialize sensors
        //??? = hardwareMap.get(ColorSensor.class, "???"); //for colorsensor

        //TODO add gyro telemetry

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();

        }

        public void gyroDrive ( double speed, double distance, double heading, double timeout){

            int newLeftTarget;
            int newRightTarget;
            int moveCounts;
            double max;
            double error;
            double steer;
            double leftSpeed;
            double rightSpeed;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                moveCounts = (int) (distance * COUNTS_PER_INCH);
                newLeftTarget = leftMotor.getCurrentPosition() + moveCounts;
                newRightTarget = rightMotor.getCurrentPosition() + moveCounts;

                // Set Target and Turn On RUN_TO_POSITION
                leftMotor.setTargetPosition(newLeftTarget);
                rightMotor.setTargetPosition(newRightTarget);

                // Turn On RUN_TO_POSITION
                leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                leftMotor.setTargetPositionTolerance(100);
                rightMotor.setTargetPositionTolerance(100);
                // start motion.
                speed = Range.clip(speed, -1.0, 1.0);
                leftDrive(speed);
                rightDrive(speed);

                double timeoutTime = runtime.seconds() + timeout;
                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive() && (leftMotor.isBusy() && rightMotor.isBusy()) && runtime.seconds() <= timeoutTime) {

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0) steer *= -1.0;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;
                    telemetry.addData("left: ", leftSpeed);
                    telemetry.addData("right", rightSpeed);
                    telemetry.update();
                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(abs(leftSpeed), abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    leftDrive(leftSpeed);
                    rightDrive(rightSpeed);

                    // Display drive status for the driver.
                    telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Actual", "%7d:%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                    telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                    telemetry.update();
                }

                // Stop all motion;
                leftDrive(0);
                rightDrive(0);
            }

        }

        public void gyroTurn ( double speed, double heading, double timeout){

//Ensure the motors are in the right configuration
            rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            double timeoutTime = runtime.seconds() + timeout;
            // keep looping while we are still active, and not on heading.
            while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF) && runtime.seconds() < timeoutTime) {
                // Update telemetry & Allow time for other processes to run.
                telemetry.update();

            }


        }
        public void gyroHold ( double speed, double heading, double holdTime){

            ElapsedTime holdTimer = new ElapsedTime();

            // keep looping while we have time remaining.
            holdTimer.reset();
            while (opModeIsActive() && (holdTimer.time() < holdTime)) {
                // Update telemetry & Allow time for other processes to run.
                onHeading(speed, angle, P_TURN_COEFF);
                telemetry.update();
            }

            // Stop all motion;
            leftDrive(0);
            rightDrive(0);

        }



    }