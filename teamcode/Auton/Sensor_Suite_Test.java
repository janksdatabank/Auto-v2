package org.firstinspires.ftc.teamcode.Auton;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import java.util.concurrent.SynchronousQueue;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.multiGlyph.IMU;

/**
 * Created by Janker on 10/2/2017.
 */
@Autonomous(name="Sensor Suite Test",group="Auton")
//@Disabled
public class Sensor_Suite_Test extends OpMode {

    //* Everything before next comment is defining variables to be accessed
    //* for the the rest of the time

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final int HEADING_THRESHOLD = 1;

    static final double P_C = 0.01;
    static final double I_C = 0.00000125;
    static final double D_C = 0;

    int iError = 0;
    int previousError = 0;

    public ElapsedTime runtime = new ElapsedTime();

    int state = 0;
    int previousState = 0;

    int i = 1;

    boolean Center = false;
    boolean Right = false;
    boolean Left = false;
    String currentCol = "Center";

    DcMotor rearLeftMotor;
    DcMotor frontLeftMotor;
    DcMotor rearRightMotor;
    DcMotor frontRightMotor;

    DcMotor Lift;
    DcMotor Extension;
    DcMotor leftIntake;
    DcMotor rightIntake;

    Servo jewelPivot;
    Servo jewelArm;
    Servo relicClamp;
    Servo relicArm;
    Servo glyphClamp;
    Servo glyphDump;
    Servo alignArm;

    IMU imu;





    ColorSensor bColor;
    ColorSensor fColor;
    ColorSensor blueColorSensor;
    DistanceSensor blueDistanceSensor;
    DistanceSensor aDist;
    DistanceSensor bDist;
    DistanceSensor fDist;
    DistanceSensor gDist;

    String measuredColor = "None";

    DigitalChannel digitalTouch;

    VuforiaLocalizer vuforia;
    int cameraMonitorViewId;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;

    static final String        ALLIANCE_COLOR       = "Blue";
    static final String        OPPONENT_COLOR       = "Red";

    // *********************************************
    //          Navigation Settings
    //              - Distance in inches
    //              - Angles in degrees
    //              - Strafe angles in radians
    //
    //           FINE TUNE AUTONOMOUS NAVIGATION HERE
    // *********************************************
    static final double PIT_TO_BOX_DIST                             = -6;               // How far the bot moves forward b4 dumping
    static final double GLYPH_PUSH_DIST                             = -3;            // How far the bot pushes the glyphs forward after dumping
    static final double REVERSE_DIST                                = 6;                // How far the bot reverses after pushing the glyphs in
    static final double REVERSE_TO_PIT_DIST                          = 7;
    static final double FAST_DRIVE_SPEED                            = -.7;
    static final double SLOW_DRIVE_SPEED                            = -.2;
    static final double STRAFE_SPEED                                = .5;
    static final double STRAFE_LEFT                                 = Math.PI/2;
    static final double STRAFE_RIGHT                                = -Math.PI/2;
    static final double STRAFE_DIST_LEFT_CENTER                     = -3.75;                // The distance when strafing from the left to center column
    static final double STRAFE_DIST_LEFT_RIGHT                      = -8;
    static final double STRAFE_DIST_CENTER_LEFT                     = 3.75;
    static final double STRAFE_DIST_CENTER_RIGHT                    = -3.75;
    static final double STRAFE_DIST_RIGHT_CENTER                    = 3.75;
    static final double STRAFE_DIST_RIGHT_LEFT                      = 8;

    // Settings dependent on alliance color
    static final double CENTER_COL_DIST_BLUE                        = 24;               // The dist the bot runs off platform to align w/ center col (Blue)
    static final double RIGHT_COL_DIST_BLUE                         = 24 + 5.75;        
    static final double LEFT_COL_DIST_BLUE                          = 24 - 5.25;
    static final int    ANGLE_OF_BOX_BLUE                           = -88;              // Angle to face the blue cryptobox
    static final double CENTER_COL_DIST_RED                         = -24;
    static final double RIGHT_COL_DIST_RED                          = -(24 - 5.25);
    static final double LEFT_COL_DIST_RED                           = -(24 + 5.75);
    static final int    ANGLE_OF_BOX_RED                            = 88;

    // *********************************************
    //          Servo Positions
    // *********************************************
    static final double GLYPH_CLAMP_LOCK                            = .8;
    static final double GLYPH_CLAMP_UNLOCK                          = .45;
    static final double GLYPH_DUMP_RAISED                           = .94;
    static final double GLYPH_DUMP_LOWERED                          = .45;
    static final double JEWEL_ARM_IN                                = .85;
    static final double JEWEL_ARM_RAISED                            = .70;
    static final double JEWEL_ARM_LOWERED                           = .18;
    static final double JEWEL_PIVOT_RAISED                          = .50;
    static final double JEWEL_PIVOT_LOWERED                         = .62;
    static final double JEWEL_PIVOT_LEFT                            = .8;               //Position for the jewel arm to hit the left jewel
    static final double JEWEL_PIVOT_RIGHT                           = .2;
    static final double ALIGN_ARM_RAISED                            = 1;
    static final double ALIGN_ARM_DETECT                            = .35;
    static final double ALIGN_ARM_LOWERED                           = .40;
    static final double ALIGN_ARM_IN                                = .1;
    static final double RELIC_ARM_LOWERED                           = .50;
    static final double RELIC_ARM_RAISED                            = .05;
    static final double RELIC_ARM_IN                                = .80;
    static final double RELIC_CLAMP_LOCK                            = 0;
    static final double RELIC_CLAMP_UNLOCK                          = 1;

    double centerDist;
    double rightDist;
    double leftDist;
    int boxAngle;
    double strafeDist;
    int encoderCount;
    double strafeDir;
    boolean atWall = false;

    @Override
    public void init() {

    // *********************************************
    //          FInding & Accessing Hardware
    // *********************************************
        rearLeftMotor = hardwareMap.dcMotor.get("rL");
        frontLeftMotor = hardwareMap.dcMotor.get("fL");
        rearRightMotor = hardwareMap.dcMotor.get("rR");
        frontRightMotor = hardwareMap.dcMotor.get("fR");

        Lift = hardwareMap.dcMotor.get("L");
        Extension = hardwareMap.dcMotor.get("E");
        leftIntake = hardwareMap.dcMotor.get("lI");
        rightIntake = hardwareMap.dcMotor.get("rI");

        jewelPivot = hardwareMap.servo.get("lP");
        relicArm = hardwareMap.servo.get("rA");
        relicClamp = hardwareMap.servo.get("rC");
        jewelArm = hardwareMap.servo.get("lJA");
        glyphClamp = hardwareMap.servo.get("fC");
        glyphDump = hardwareMap.servo.get("fD");
        alignArm = hardwareMap.servo.get("aA");

        blueColorSensor = hardwareMap.get(ColorSensor.class, "bColorDistance");
        blueDistanceSensor = hardwareMap.get(DistanceSensor.class, "bColorDistance");

      //  bColor = hardwareMap.get(ColorSensor.class, "bDist");
       // fColor = hardwareMap.get(ColorSensor.class, "fDist");


        aDist = hardwareMap.get(DistanceSensor.class, "aDist");
        gDist = hardwareMap.get(DistanceSensor.class, "gDist");
       // bDist = hardwareMap.get(DistanceSensor.class, "bDist");
       // fDist = hardwareMap.get(DistanceSensor.class, "fDist");


        // get a reference to our digitalTouch object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

    // *********************************************
    //          Setting Motor Directions and Modes
    // *********************************************
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);

        Lift.setDirection(DcMotor.Direction.REVERSE);
        Extension.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);

        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    // *********************************************
    //          Setting Initial Servo Positions
    // *********************************************
        relicClamp.setPosition(RELIC_CLAMP_LOCK);
        relicArm.setPosition(RELIC_ARM_RAISED);
        glyphDump.setPosition(GLYPH_DUMP_RAISED);
        glyphClamp.setPosition(GLYPH_CLAMP_UNLOCK);
        alignArm.setPosition(ALIGN_ARM_LOWERED);
        jewelPivot.setPosition(JEWEL_PIVOT_RAISED);
        jewelArm.setPosition(JEWEL_ARM_RAISED);

    // *********************************************
    //          Alliance Setup Stuff
    // *********************************************

    // Depending on the alliance color, use the specified navigation settings for the autonomous
        if (ALLIANCE_COLOR ==  "Blue"){
            centerDist = CENTER_COL_DIST_BLUE;
            rightDist = RIGHT_COL_DIST_BLUE;
            leftDist = LEFT_COL_DIST_BLUE;
            boxAngle = ANGLE_OF_BOX_BLUE;
        } else {
            centerDist = CENTER_COL_DIST_RED;
            rightDist = RIGHT_COL_DIST_RED;
            leftDist = LEFT_COL_DIST_RED;
            boxAngle = ANGLE_OF_BOX_RED;
        }

    // *********************************************
    //          IMU Setup
    // *********************************************

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        //BNO055IMU.Parameters gParameters = new BNO055IMU.Parameters();
       // gParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
       // gParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
       // gParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
       // gParameters.loggingEnabled      = true;
       // gParameters.loggingTag          = "IMU";
       // gParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMU imu = new IMU();
        imu.initIMU(hardwareMap,"imu");
        this.imu = imu;
        // Retrieve and initialize the IMU. The imu is built into the Rev Expansion Hub
        // The imu is accessible through the I2C port 0

       // imu.initialize(gParameters);

        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

     // *********************************************
    //          Vuforia Setup
    // *********************************************
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AavXRvD/////AAAAGYbu90CPGkoGh7X1nRUxwjsWz20nEAcBDAXkRzK4SjWSsBKxl/Ixw/qo0zdSsFvZgQFU7omaLYZ6zjjPIXUWGf8C0f/KcZcbmwc2MrPRO6fI0i7rXSdI1pn9thjJOsjQ5imtpAFm2nT5dkorcWt1Blevx5LjpU99u2ysu8g92vHO6JeTD3g7V2r9Zm4Fo6K/bbfvqmywG9oqMJRGAq27hvgXhvwzORdSdO+TtDBXrFgAvDuQozaD9FqeRa2bwFMLZrJM7YCthG9RT9ENYSRdbrHLLUXgk43tOZhK2N4ec9JP0Xqj8HhKs3pAjtvq5f9Q5kPjQwkd6w40wJmXoULHy8B6vgjI7F8XYdPT2QpCTGPO";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        relicTrackables.activate();


    }


    @Override
    public void init_loop() {
        runtime.reset();

    }

    @Override
    public void loop() {
        alignArm.setPosition(ALIGN_ARM_DETECT);
      //  angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("IMU Heading: ", imu.getHeading());

        telemetry.addData("distance", gDist.getDistance(DistanceUnit.CM));

        telemetry.addData("distanceJew", blueDistanceSensor.getDistance(DistanceUnit.CM));

        telemetry.addData("Touch Sensor Pressed: ", !digitalTouch.getState());

        telemetry.addData("Color Sensor Readings: ", "Red[%d], Blue[%d]", blueColorSensor.red(), blueColorSensor.blue());

        //telemetry.addData("Color Sensor Readings: ", "Alpha[%d]", fColor.alpha());

        //telemetry.addData("Color Sensor Readings: ", "Alpha[%d]", bColor.alpha());

        telemetry.addData("Distance Sensor Readings (cm): ", String.format(Locale.US, "%.02f", aDist.getDistance(DistanceUnit.CM)));

        
        telemetry.addData("encoder", Extension.getCurrentPosition());

      //  telemetry.addData("Distance Sensor Readings (cm): ", String.format(Locale.US, "%.02f", bDist.getDistance(DistanceUnit.CM)));

       // telemetry.addData("Distance Sensor Readings (cm): ", String.format(Locale.US, "%.02f", fDist.getDistance(DistanceUnit.CM)));

        //telemetry.addData("Distance Sensor Readings (cm): ", String.format(Locale.US, "%.02f", dDist.getDistance(DistanceUnit.CM)));


    }


    // *********************************************
    //          Navigation Functions
    // *********************************************

    /**
     * Move the bot a user defined amount in inches. To move backwards the speed must be reversed.
     */


    public void resetEncoder(){
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }






    /**
     * Get the maximum velocity of the four wheel in order to scale everything else by it
     */

    public double getMaxVelocity(double currentMax, double wheelVelocity){

        if (Math.abs(currentMax) > Math.abs(wheelVelocity)){
            return Math.abs(currentMax);
        } else{
            return Math.abs(wheelVelocity);
        }
    }

    /**
     *  Determines if bot is aligned with the beacon by seeing if the optical distance sensor is over the
     *  line
     * @return If the optical distance sensor reads a threshold value, indicating the bot is over the line
     */
    /*
    public boolean lineDetected(){
        telemetry.addData("Light Level: ", lineSensor.getLightDetected());

        if (lineSensor.getLightDetected() > .4){
            return true;
        }
        return false;
    }
    */


    // *********************************************
    //          State Machine Functions
    // *********************************************


    /**
     *  Check if the encoders are reset and adds in a small delay to account for hardware cycles
     *  Sets the motors to run using encoders
     */
    public boolean resetDelay(){
        if (frontLeftMotor.getCurrentPosition() == 0||
                frontRightMotor.getCurrentPosition() == 0 ||
                rearLeftMotor.getCurrentPosition() == 0  ||
                rearRightMotor.getCurrentPosition() == 0 || runtime.time() > .5){

            rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            previousError = 0;
            iError = 0;

            return true;
        }
        return false;
    }

    /**
     * Increments into the next state but first loops to the delay function
     * The current state is bookmarked in order to move to the next state after
     * calling the delay.
     */
    public void nextState() {
        previousState = state;
        state = -1;
        this.resetEncoder();
        runtime.reset();

        return;

    }


    /**
     * Preps the state machine to change to the designated state
     * @param dstState The next state
     */
    public void goToState(int dstState){
        previousState = dstState - 1;
        state = -1;
        runtime.reset();
        this.resetEncoder();
        return;
    }

    /**
     * Function that is called to end the state machine
     */
    public void stopStateMachine() {
        state = 7261;
        return;
    }
}
