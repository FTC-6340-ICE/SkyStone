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
    static final double P_TURN_COEFF = .010;     // .02 Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = .010;     // .05 Larger is more responsive, but also less stable

    //Initialize Vuforia variables
    //VuforiaTrackables relicTrackables;
    //VuforiaTrackable relicTemplate;

    //TODO Add public methods here ie. public void gyroDrive

    public void gyroDrive(double speed, double distance, double heading, double timeout) {


    }

    public void gyroTurn(double speed, double heading, double timeout) {


    }

    public void gyroHold(double speed, double heading, double holdTime) {


    }
}