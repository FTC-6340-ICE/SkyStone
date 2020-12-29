package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static java.lang.Math.abs;

/**
 * Created by Team ICE 6340 on 9.23.2019!
 */


public abstract class ICE_Controls_4_Motors extends LinearOpMode {
    //Initialize and instantiate vuforia variables
    OpenGLMatrix lastLocation = null;
    protected VuforiaLocalizer vuforia;
    protected static final String VUFORIA_KEY = "AZWzerv/////AAABmZeKo4MkD08MoSz5oHB/JU6N1BsUWpfHgQeAeVZemAypSUGVQhvAHo6+v7kJ3MITd8530MhwxRx7GjRtdCs1qjPmdKiJK66dv0yN4Zh4NvKBfP5p4TJjM+G0GoMVgVK0pItm2U56/SVqQH2AYtczQ+giw6zBe4eNhHPJCMY5C2t5Cs6IxxjZlMkRF85l8YAUlKGLipnoZ1T/mX8DNuThQA57qsIB2EN6pGWe8GI64hcPItQ0j7Oyjp82lEN13rYQYsS3Ur4a6//D6yhwa0rogXAysG68G+VgC1mNlj1CjX60qDI84ZN0b/A081xXqjeyFqZK8A/jO8y7BGz9ZuuZNxxXIon6xRNeKYudpfTD23+5";
    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    protected TFObjectDetector tfod;
    protected static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    protected static final String LABEL_FIRST_ELEMENT = "Stone";
    protected static final String LABEL_SECOND_ELEMENT = "Skystone";


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
    protected DcMotorEx leftMotorBack;
    protected DcMotorEx leftMotorFront;
    protected DcMotorEx rightMotorBack;
    protected DcMotorEx rightMotorFront;

    protected DcMotorEx intakeMotorRight;
    protected DcMotorEx intakeMotorLeft;
   // protected DcMotorEx stackingArm;
    protected DigitalChannel digitalTouch;
    protected DigitalChannel touchSensorBack;
    //Instantiate servos
    //protected Servo ????;
    protected Servo   servoleft;
    protected Servo servoright;
    protected Servo servoCapStone;
    protected Servo servoCapStone2;

   // protected Servo servoUpDown;
    protected Servo servoLeftRight;
    protected Servo servoDrop;
    protected Servo servoGiddyUp;


//
//    protected Servo servoUp;


    //Instantiate sensors
    //ColorSensor ?????;

    //Initlize encoder variables
    protected double COUNTS_PER_MOTOR_DRIVE = 1120;    // Rev Hex Motor 288
    protected double COUNTS_PER_MOTOR_LIFT = 1120;    // andyMark 40
    protected double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    protected double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    protected double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_DRIVE * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    protected double COUNTS_PER_INCH_LIFT = (COUNTS_PER_MOTOR_LIFT) / (.677 * 3.1415);

    //static final double     COUNTS_PER_MOTOR_REV_CORE_HEX    = 1120 ;    // eg: Rev Core Hex Motor Encoder
    static final double     COUNTS_PER_MOTOR_REV_CORE_HEX    = 560 ;    // eg: Rev Core Hex Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION_CORE_HEX    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES_CORE_HEX   = 3.36 ;     // For figuring circumference 3.54
    static final double     COUNTS_PER_INCH_CORE_HEX         = (COUNTS_PER_MOTOR_REV_CORE_HEX * DRIVE_GEAR_REDUCTION_CORE_HEX) /
            (WHEEL_DIAMETER_INCHES_CORE_HEX * 3.1416);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public final double DRIVE_SPEED_FOR_EDGE_STONE = 0.3;
    public final double DRIVE_SPEED = .55;     // Nominal speed for better accuracy.
    public final double TURN_SPEED = .5;     // Nominal half speed for better accuracy.
    public final double TURN_SPEED_BOOST = .6;     // Nominal half speed for better accuracy.
    public final double TURN_SPEED_BOOST_AUTONOMOUS = 0.8;     // Nominal half speed for better accuracy.

    public final double HOLD_SPEED = .3;     // Nominal half speed for better accuracy.
    public final double DRIVE_SPEED_BOOST = 0.8;
    public final double DRIVE_SPEED_BOOST_AUTONOMOUS = 1.0;

    static final double HEADING_THRESHOLD = 1.5;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = .009;     // .02 Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = .007;     // .009 Larger is more responsive, but also less stable
    static final double P_HOLD_COEFF = .009;
    static final double MIN_TURN_SPEED = .15;
    //

    //Initialize Vuforia variables
     VuforiaTrackables VuforiaSkyStone;
    //VuforiaTrackable relicTemplate;

    protected void rightFrontDrive(double power) {
        rightMotorFront.setPower(power);

    }
    protected void rightBackDrive(double power) {
        rightMotorBack.setPower(power);
    }
    protected void leftFrontDrive(double power) {
        leftMotorFront.setPower(power);
    }

    protected void leftBackDrive(double power) {
        leftMotorBack.setPower(power);
    }

    protected void initializeHardware() {

        //Give the OK message
        telemetry.addData("Status", "Initializing hardware");
        telemetry.update();
               //Initialize robot hardware
        //Begin with the chassis
        leftMotorBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotorBack");
        leftMotorFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotorFront");
        rightMotorBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotorBack");
        rightMotorFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotorFront");

       // intakeMotorRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "intakeMotorRight");
        //intakeMotorLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "intakeMotorLeft");
       // stackingArm = (DcMotorEx) hardwareMap.get(DcMotor.class, "stackingArm3");

        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        touchSensorBack = hardwareMap.get(DigitalChannel.class, "touchSensor23");



        //Reset the encoders on the chassis to 0
        leftMotorBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

       // stackingArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //Set the motor modes
        leftMotorBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftMotorFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        //stackingArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //Reverse the left motor so all motors move forward when set to a positive speed.
        leftMotorBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftMotorFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotorBack.setDirection(DcMotorEx.Direction.FORWARD) ;
        rightMotorFront.setDirection(DcMotorEx.Direction.FORWARD) ;

        //When the motors are told to stop this makes sure you now that there stoped on the screen
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // stackingArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initialize the servos
        // ??? = hardwareMap.get(Servo.class, "???");
        servoright = hardwareMap.get(Servo.class, "rightFoundation");
        servoleft = hardwareMap.get(Servo.class, "leftFoundation");
       //servoUpDown = hardwareMap.get(Servo.class,"servoud");
         servoLeftRight = hardwareMap.get(Servo.class,"servolr");
         servoDrop = hardwareMap.get(Servo.class,"servodrop");
      //  servoGiddyUp = hardwareMap.get(Servo.class,"servoGiddyUp");

        //servoCapStone = hardwareMap.get(Servo.class,"servoCapstone3");
       // servoCapStone2 = hardwareMap.get(Servo.class,"servoCapstone4");

        //
        //  servoCapStone =hardwareMap.get(Servo.class, "CapStoneServo3");
        //intakeMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        //intakeMotorRight.setDirection(DcMotor.Direction.FORWARD);
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        touchSensorBack.setMode(DigitalChannel.Mode.INPUT);

        //stopIntake = hardwareMap.get(DigitalChannel.class, "stopMotorIntake01");

        //Initialize sensors
        //??? = hardwareMap.get(ColorSensor.class, "???"); //for colorsensor

        //TODO add gyro telemetry
        BNO055IMU.Parameters parametersG = new BNO055IMU.Parameters();
        parametersG.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersG.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersG.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersG.loggingEnabled = true;
        parametersG.loggingTag = "IMU";
        parametersG.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersG);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();

        }

    }
    public void gyroDrive ( double speed, double distance) {
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH_CORE_HEX);
            newLeftTarget = leftMotorBack.getCurrentPosition() + moveCounts;
            newRightTarget = rightMotorBack.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftMotorBack.setTargetPosition(newLeftTarget);
            leftMotorFront.setTargetPosition(newLeftTarget);
            rightMotorBack.setTargetPosition(newRightTarget);
            rightMotorFront.setTargetPosition(newRightTarget);

            leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftMotorBack.setPower(speed);
            leftMotorFront.setPower(speed);

            rightMotorBack.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            /*
            while (opModeIsActive() &&
                    (leftMotorBack.isBusy() && rightMotorBack.isBusy())) {
                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);
                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;
                leftSpeed = speed - steer;
                rightSpeed = speed + steer;
                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
                leftMotorBack.setPower(leftSpeed);
                rightMotorBack.setPower(rightSpeed);
                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftMotorBack.getCurrentPosition(),
                        rightMotorBack.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }*/

            // Stop all motion;
            leftMotorBack.setPower(0);
            leftMotorFront.setPower(0);

            rightMotorBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void gyroDrive ( double speed, double distance, double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH_CORE_HEX);
            newLeftTarget = leftMotorBack.getCurrentPosition() + moveCounts;
            newRightTarget = rightMotorBack.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftMotorBack.setTargetPosition(newLeftTarget);
            leftMotorFront.setTargetPosition(newLeftTarget);

            rightMotorBack.setTargetPosition(newRightTarget);

            leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftMotorBack.setPower(speed);
            rightMotorBack.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftMotorBack.isBusy() && rightMotorBack.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftMotorBack.setPower(leftSpeed);
                rightMotorBack.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftMotorBack.getCurrentPosition(),
                        rightMotorBack.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftMotorBack.setPower(0);
            rightMotorBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

          }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     // @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle Fs required, add/subtract from current heading.
     */








    public void gyroDrive ( double speed, double distance, double heading, double timeout ){
        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftFrontSpeed;
        double rightFrontSpeed;
        double leftBackSpeed;
        double rightBackSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = leftMotorBack.getCurrentPosition() + moveCounts;
            newRightTarget = rightMotorBack.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftMotorBack.setTargetPosition(newLeftTarget);
            rightMotorBack.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotorBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightMotorBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            leftMotorBack.setTargetPositionTolerance(100);
            rightMotorBack.setTargetPositionTolerance(100);
            // start motion.
            speed = Range.clip(speed, -1.0, 1.0);
            leftFrontDrive(speed);
            rightBackDrive(speed);
            leftBackDrive(speed);
            rightFrontDrive(speed);

            double timeoutTime = runtime.seconds() + timeout;
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (leftMotorBack.isBusy() && rightMotorBack.isBusy()) && runtime.seconds() <= timeoutTime) {

                // adjust relative speed based on heading error.
                error = getError(heading);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) steer *= -1.0;

                leftFrontSpeed = speed - steer;
                rightFrontSpeed = speed + steer;
                leftBackSpeed = speed + steer;
                rightBackSpeed = speed + steer;

                telemetry.addData("leftFront: ", leftFrontSpeed);
                telemetry.addData("rightFront", rightFrontSpeed);
                telemetry.addData("leftBack: ", leftBackSpeed);
                telemetry.addData("rightBack", rightBackSpeed);
                telemetry.update();
                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(abs(leftFrontSpeed), abs(rightFrontSpeed));
                if (max > 1.0) {
                    leftFrontSpeed /= max;
                    rightFrontSpeed /= max;
                    leftBackSpeed /= max;
                    rightBackSpeed /= max;
                }

                leftFrontDrive(leftFrontSpeed);
                rightFrontDrive(rightFrontSpeed);
                leftBackDrive(leftBackSpeed);
                rightBackDrive(rightBackSpeed);


                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", leftMotorBack.getCurrentPosition(), rightMotorBack.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftFrontSpeed, rightFrontSpeed,rightBackSpeed,leftBackSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive(0);
            rightFrontDrive(0);
            leftBackDrive(0);
            rightBackDrive(0);
        }

    }

    public void gyroTurn ( double speed, double heading, double timeout){

//Ensure the motors are in the right configuration

//Ensure the motors are in the right configuration
        rightMotorBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        double timeoutTime = runtime.seconds() + timeout;
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, heading, P_TURN_COEFF) && runtime.seconds() < timeoutTime) {
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
                onHeading(speed, heading, P_HOLD_COEFF);
                telemetry.update();
            }

            // Stop all motion;
        leftFrontDrive(0);
        rightFrontDrive(0);
        leftBackDrive(0);
        rightBackDrive(0);


    }
    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    private double getSteer(double error, double PCoeff) {
        return Range.clip(PCoeff * error, -1, 1);


    }
    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    private boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftFrontSpeed;
        double rightFrontSpeed;
        double leftBackSpeed;
        double rightBackSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftFrontSpeed = 0.0;
            rightFrontSpeed = 0.0;
            leftBackSpeed = 0.0;
            rightBackSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightFrontSpeed = Range.clip(speed * steer,-1,1);
            leftFrontSpeed = -rightFrontSpeed;
            rightBackSpeed = Range.clip(speed * steer,-1,1);
            leftBackSpeed = -rightBackSpeed;
        }

        // Send desired speeds to motors.
        leftFrontDrive(leftFrontSpeed);
        rightFrontDrive(rightFrontSpeed);
        leftBackDrive(leftBackSpeed);
        rightBackDrive(rightBackSpeed);


        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed);

        return onTarget;
    }

    public void inTakeStone(){
        intakeMotorLeft.setPower(-1.0);
        intakeMotorRight.setPower(-1.0);




    }
    public void inTakeStoneAutonomous(){
        intakeMotorLeft.setPower(-1.0);
        intakeMotorRight.setPower(-1.0);



    }






    public void inTakeStone(boolean turnOnlyOneAtIntake,int teamColor){
        if(turnOnlyOneAtIntake)
        {
            if(teamColor==1)
            {
                intakeMotorRight.setPower(-1.0);

            }
            else
            {
                intakeMotorLeft.setPower(-1.0);

            }
        }
        else {
            intakeMotorLeft.setPower(-1.0);
            intakeMotorRight.setPower(-1.0);
        }

    }

    public void ouTakeStone(){
        intakeMotorLeft.setPower(0.3);
        intakeMotorRight.setPower(0.3);

    }
    public void stopInTakeStone(){
        intakeMotorLeft.setPower(0.0);
        intakeMotorRight.setPower(0.0);

    }

    public void ouTakeStoneForAutonomous(double heading,double timeout,double distance){
        gyroDrive(DRIVE_SPEED,distance,heading,timeout);
        intakeMotorLeft.setPower(0.4);
        intakeMotorRight.setPower(0.4);

    }



}