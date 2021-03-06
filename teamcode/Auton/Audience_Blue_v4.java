package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by Janker on 10/2/2017.
 */
@Autonomous(name="Audience_Blue_Four",group="Auton")
@Disabled
public class Audience_Blue_v4 extends OpMode {

    //* Everything before next comment is defining variables to be accessed
    //* for the the rest of the time

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 5.5;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double HEADING_THRESHOLD = .5;

    static final double P_C_TURN = 0.013;
    static final double I_C_TURN = 0.0000125;
    static final double D_C_TURN = 0;

    static final double P_C_AD = 0.013;
    static final double I_C_AD = 0.00000125;
    static final double D_C_AD = 0;

    static final double P_C_EI = 0.04;
    static final double I_C_EI = 0.00000125;
    static final double D_C_EI = 0;

    double iError = 0;
    double previousError = 0;

    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime suckTime = new ElapsedTime();

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

    BNO055IMU imu;

    Orientation angles;

    VoltageSensor vs;

    ColorSensor blueColorSensor;
    ColorSensor brColor;
    ColorSensor frColor;
    ColorSensor blColor;
    ColorSensor flColor;
    DistanceSensor blueDistanceSensor;
    DistanceSensor aDist;
    DistanceSensor bDist;
    DistanceSensor fDist;
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
    static final double PIT_TO_BOX_DIST                             = -8;               // How far the bot moves forward b4 dumping
    static final double GLYPH_PUSH_DIST                             = -1.5;            // How far the bot pushes the glyphs forward after dumping
    static final double REVERSE_DIST                                = 8;                // How far the bot reverses after pushing the glyphs in
    static final double REVERSE_TO_PIT_DIST                         = 8;
    static final double PARKING_DIST                                = 4;
    static final double FAST_DRIVE_SPEED                            = 1;
    static final double SLOW_DRIVE_SPEED                            = .2;
    static final double STRAFE_SPEED                                = .35;
    static final double STRAFE_LEFT                                 = Math.PI/2;
    static final double STRAFE_RIGHT                                = -Math.PI/2;
    static final double STRAFE_DIST_LEFT_CENTER                     = -4.75;                // The distance when strafing from the left to center column
    static final double STRAFE_DIST_LEFT_RIGHT                      = -9;
    static final double STRAFE_DIST_CENTER_LEFT                     = 4.75;
    static final double STRAFE_DIST_CENTER_RIGHT                    = -4.75;
    static final double STRAFE_DIST_RIGHT_CENTER                    = 4.75;
    static final double STRAFE_DIST_RIGHT_LEFT                      = 9;

    // Settings dependent on alliance color
    static final double CENTER_COL_DIST_BLUE                        = 23.6;               // The dist the bot runs off platform to align w/ center col (Blue)
    static final double RIGHT_COL_DIST_BLUE                         = 23 + 6.5;
    static final double LEFT_COL_DIST_BLUE                          = 23 - 4.5;
    static final double ANGLE_OF_BOX_BLUE                           = -90;              // Angle to face the blue cryptobox
    static final double CENTER_COL_DIST_RED                         = -23;
    static final double RIGHT_COL_DIST_RED                          = -(23 - 6);
    static final double LEFT_COL_DIST_RED                           = -(23 + 5);
    static final double ANGLE_OF_BOX_RED                            = -90;

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
    static final double JEWEL_PIVOT_LOWERED                         = .45;
    static final double JEWEL_PIVOT_LEFT                            = .8;               //Position for the jewel arm to hit the left jewel
    static final double JEWEL_PIVOT_RIGHT                           = .2;
    static final double ALIGN_ARM_RAISED                            = 0;
    static final double ALIGN_ARM_DETECT                            = .37;
    static final double ALIGN_ARM_LOWERED                           = .65;
    static final double ALIGN_ARM_IN                                = .85;
    static final double RELIC_ARM_LOWERED                           = .50;
    static final double RELIC_ARM_RAISED                            = .05;
    static final double RELIC_ARM_IN                                = .80;
    static final double RELIC_CLAMP_LOCK                            = .15;
    static final double RELIC_CLAMP_UNLOCK                          = 1;

    boolean secondRun = false;
    boolean firstGrey = false;
    boolean firstBrown = false;
    boolean frontBrown = false;
    boolean frontGrey = false;
    boolean backBrown = false;
    boolean backGrey = false;
    double letsFindOut = 0;
    double centerDist;
    double rightDist;
    double leftDist;
    double boxAngle;
    double strafeDist;
    int encoderCount;
    double strafeDir;
    boolean atWall = false;
    double minV = 15;
    double maxV = 10;
    double dropV = 11;
    int blockCount = 0;
    boolean block = false;
    double checkDelay;
    double suckStart;
    int columns = 1;
    boolean sucked = false;

    @Override
    public void init() {

        // *********************************************
        //          Finding & Accessing Hardware
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
        frColor = hardwareMap.get(ColorSensor.class, "frdist");
        brColor = hardwareMap.get(ColorSensor.class, "brdist");
        flColor = hardwareMap.get(ColorSensor.class, "fldist");
        blColor = hardwareMap.get(ColorSensor.class, "bldist");

        aDist = hardwareMap.get(DistanceSensor.class, "aDist");
        bDist = hardwareMap.get(DistanceSensor.class, "brdist");
        fDist = hardwareMap.get(DistanceSensor.class, "frdist");

        // get a reference to our digitalTouch object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        vs = hardwareMap.get(VoltageSensor.class, "Left");

        // *********************************************
        //          Setting Motor Directions and Modes
        // *********************************************
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        //frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Lift.setDirection(DcMotor.Direction.REVERSE);
        Extension.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);

        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // *********************************************
        //          Setting Initial Servo Positions
        // *********************************************
        relicClamp.setPosition(RELIC_CLAMP_LOCK);
        relicArm.setPosition(1);
        glyphDump.setPosition(GLYPH_DUMP_RAISED);
        glyphClamp.setPosition(GLYPH_CLAMP_UNLOCK);
        alignArm.setPosition(ALIGN_ARM_IN);
        jewelPivot.setPosition(JEWEL_PIVOT_RAISED);
        jewelArm.setPosition(.9);

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
        BNO055IMU.Parameters gParameters = new BNO055IMU.Parameters();
        gParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gParameters.loggingEnabled      = true;
        gParameters.loggingTag          = "IMU";
        gParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. The imu is built into the Rev Expansion Hub
        // The imu is accessible through the I2C port 0
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gParameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

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
        //telemetry.addData("bDist: ", bDist.getDistance(DistanceUnit.CM));
        //telemetry.addData("State = ", state);

        //RobotLog.ii("PID_DRIVE", "FL Current Encoder: %s", frontLeftMotor.getCurrentPosition());
        //RobotLog.ii("PID_DRIVE", "FR Current Encoder: %s", frontRightMotor.getCurrentPosition());
        //RobotLog.ii("PID_DRIVE", "RL Current Encoder: %s", rearLeftMotor.getCurrentPosition());
        //RobotLog.ii("PID_DRIVE", "RR Current Encoder: %s", rearRightMotor.getCurrentPosition());

        //telemetry.addData("Direction", strafeDir);
        //telemetry.addData("Distance", strafeDist);
        telemetry.addData("Cipher", letsFindOut);

        telemetry.addData("Power fL", frontLeftMotor.getPower() );
        telemetry.addData("Power rL", rearLeftMotor.getPower() );
        telemetry.addData("Power fR", frontRightMotor.getPower() );
        telemetry.addData("Power rR", rearRightMotor.getPower() );
        telemetry.addData("Case:", state);
        //telemetry.addData("Current Column: ", currentCol);
        //telemetry.addData("Visited: ", "Center: %s, Left: %s, Right: %s", Center, Left, Right);


        switch (state) {

            // Give a small amount of time for hardware to reset after pressing play
            case -1:
                resetEncoder();
                RobotLog.ii("STATE", "Resetting");
                RobotLog.ii("RESET_INFO", "FL Current Encoder: %s", frontLeftMotor.getCurrentPosition());
                RobotLog.ii("RESET_INFO", "FR Current Encoder: %s", frontRightMotor.getCurrentPosition());
                RobotLog.ii("RESET_INFO", "RL Current Encoder: %s", rearLeftMotor.getCurrentPosition());
                RobotLog.ii("RESET_INFO", "RR Current Encoder: %s", rearRightMotor.getCurrentPosition());
                if (resetDelay()) {
                    RobotLog.ii("STATE", "Reset");
                    state = previousState + 1;
                }
                break;

            // Set positions for future actions and test for vuMark pictographs
            // If none, default center and continue process
            case 0:
                relicArm.setPosition(RELIC_ARM_LOWERED);
                alignArm.setPosition(ALIGN_ARM_LOWERED);

                vuMark = RelicRecoveryVuMark.from(relicTemplate);

                if (runtime.time() < 2){
                    RobotLog.ii("STATE", "Identifying Pictograph");
                    if (vuMark == RelicRecoveryVuMark.CENTER) {
                        RobotLog.ii("OUTPUT_INFO", "Center Pictograph");
                        //telemetry.addData("center", "true");
                        Center = true;
                        currentCol = "Center";
                        fastNextState();
                    } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                        RobotLog.ii("OUTPUT_INFO", "Left Pictograph");
                        //telemetry.addData("left", "true");
                        Left = true;
                        currentCol = "Left";
                        fastNextState();
                    } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        RobotLog.ii("OUTPUT_INFO", "Right Pictograph");
                        //telemetry.addData("right", "true");
                        Right = true;
                        currentCol = "Right";
                        fastNextState();
                    }
                } else{
                    RobotLog.ii("OUTPUT_INFO", "Default Pictograph");
                    Center = true;
                    currentCol = "Center";
                    fastNextState();
                }

                break;

            // Extend the color sensor, read the color of the jewel, and hit the jewel
            case 1:
                if (runtime.time() < .5) {
                    RobotLog.ii("STATE", "Lowering Jewel Arm");
                    jewelPivot.setPosition(JEWEL_PIVOT_LOWERED);
                    jewelArm.setPosition(JEWEL_ARM_LOWERED);
                    //telemetry.addData("Red Values: ", blueColorSensor.red());
                    //telemetry.addData("Blue Values: ", blueColorSensor.blue());

                } else if (runtime.time() < .75){
                    RobotLog.ii("STATE", "Identifying Color");
                    RobotLog.ii("SENSOR_INFO", "Red Values: %s", blueColorSensor.red());
                    RobotLog.ii("SENSOR_INFO", "Blue Values: %s", blueColorSensor.blue());
                    if (blueColorSensor.red() > blueColorSensor.blue()){
                        RobotLog.ii("OUTPUT_INFO", "Left Jewel Red");
                        measuredColor = "Red";
                    } else{
                        RobotLog.ii("OUTPUT_INFO", "Left Jewel Blue");
                        measuredColor = "Blue";
                    }
                } else if (runtime.time() < 1) {
                    RobotLog.ii("STATE", "Hitting Jewel");
                    //telemetry.addData("Measured Color: ", measuredColor);
                    if (measuredColor.equals(OPPONENT_COLOR)) {
                        RobotLog.ii("OUTPUT_INFO", "Pivot Left Position");
                        jewelPivot.setPosition(JEWEL_PIVOT_LEFT);
                        //telemetry.addData("Left Jewel: ", OPPONENT_COLOR);

                    } else {
                        RobotLog.ii("OUTPUT_INFO", "Pivot Right Position");
                        jewelPivot.setPosition(JEWEL_PIVOT_RIGHT);
                        //telemetry.addData("Left Jewel: ", ALLIANCE_COLOR);
                    }
                } else {
                    nextState();

                }
                break;

            // Use the front color sensor to determine color of placed glyph, then store
            // data for further use.
            case 2:
                if ((frColor.alpha() + flColor.alpha())/2  > 200) {
                    firstGrey = true;
                    RobotLog.ii("OUTPUT_INFO", "First Grey: %s", firstGrey);
                    fastNextState();
                }
                else {
                    firstBrown = true;
                    RobotLog.ii("OUTPUT_INFO", "First Brown: %s", firstBrown);
                    fastNextState();
                }
                break;

            // Drive forward to align with the pictograph column and retract the color sensor
            case 3:
                RobotLog.ii("STATE", "Driving to Pictograph Column");
                double distance;

                jewelArm.setPosition(JEWEL_ARM_RAISED);
                jewelPivot.setPosition(JEWEL_PIVOT_RAISED);

                if (Center) {
                    distance = centerDist;
                }
                else if (Left) {
                    distance = leftDist;
                }
                else  {
                    distance = rightDist;
                }

                if (ALLIANCE_COLOR.equals("Red")){
                    if (encoderIMUdrive(distance, -.5, 0)) {
                        fastNextState();//state++;
                    }
                } else{
                    if (encoderIMUdrive(distance, .5, 0)) {
                        fastNextState();//state++;
                    }
                }

                break;

            // Turn to face the box
            case 4:
                RobotLog.ii("STATE", "Turning to Face Cryptobox");
                if (turn(boxAngle)) {
                    nextState();
                }
                break;

            // Drive forward to the box and align itself within the columns
            case 5:
                if(!atWall) {
                    alignArm.setPosition(ALIGN_ARM_DETECT);
                    rightIntake.setPower(0);
                    leftIntake.setPower(0);
                    RobotLog.ii("STATE", "Driving to hit wall with Alignment Arm");
                    glyphClamp.setPosition(GLYPH_CLAMP_LOCK);
                    if (digitalTouch.getState() == true) {
                        IMUdrive(-SLOW_DRIVE_SPEED, boxAngle);
                    } else {
                        RobotLog.ii("STATE", "Hit Wall");
                        rightIntake.setPower(0);
                        leftIntake.setPower(0);
                        frontLeftMotor.setPower(0);
                        rearLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                        rearRightMotor.setPower(0);
                        alignArm.setPosition(ALIGN_ARM_DETECT);
                        atWall = true;
                    }
                } else {
                    RobotLog.ii("STATE", "Aligning with Strafe");
                    RobotLog.ii("SENSOR_INFO", "Alignment Arm Distance: %.02f", aDist.getDistance(DistanceUnit.CM));
                    if(aDist.getDistance(DistanceUnit.CM) > 10.5){
                        angleDrive(.1, STRAFE_RIGHT, boxAngle);
                        RobotLog.ii("OUTPUT_INFO", "Strafe Right");
                        //telemetry.addData("Strafing: ", "Left");
                    } else if (aDist.getDistance(DistanceUnit.CM) < 6.2){
                        RobotLog.ii("OUTPUT_INFO", "Strafe Left");
                        angleDrive(.1, STRAFE_LEFT, boxAngle);
                        //telemetry.addData("Strafing: ", "Right");
                    } else {
                        frontLeftMotor.setPower(0);
                        rearLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                        rearRightMotor.setPower(0);
                        RobotLog.ii("OUTPUT_INFO", "Aligned");
                        atWall = false;
                        fastNextState();
                    }
                }
                break;

            // Dump the glyph
            case 6:
                if (runtime.time() < .5) {
                    RobotLog.ii("STATE", "Dump Glyph");
                    alignArm.setPosition(ALIGN_ARM_RAISED);
                    glyphDump.setPosition(GLYPH_DUMP_LOWERED);
                } else {
                    RobotLog.ii("STATE", "Unlock Glyph");
                    glyphClamp.setPosition(GLYPH_CLAMP_UNLOCK);
                    nextState();
                }
                break;

            // Push the glyphs in
            case 7:
                RobotLog.ii("STATE", "Pushing Glyphs in");
                if (encoderIMUdrive(GLYPH_PUSH_DIST, -SLOW_DRIVE_SPEED, boxAngle)) {
                    nextState();
                }
                break;

            // Reverse from box to glyph pit, unless it has done the action before
            // In second case move to state 15 to park
            case 8:
                RobotLog.ii("STATE", "Reversing from Cryptobox");
                glyphDump.setPosition(GLYPH_DUMP_RAISED);
                alignArm.setPosition(ALIGN_ARM_LOWERED);
                rightIntake.setPower(-1);
                leftIntake.setPower(-1);
                if (secondRun) {
                    alignArm.setPosition(ALIGN_ARM_RAISED);
                    goToState(15);
                }
                if (encoderIMUdrive(REVERSE_DIST + 8, FAST_DRIVE_SPEED, boxAngle)) {
                    jewelPivot.setPosition(JEWEL_PIVOT_RAISED);
                    jewelArm.setPosition(JEWEL_ARM_RAISED);
                    alignArm.setPosition(ALIGN_ARM_LOWERED);
                    glyphClamp.setPosition(GLYPH_CLAMP_UNLOCK);
                    glyphDump.setPosition(GLYPH_DUMP_RAISED);
                    relicClamp.setPosition(RELIC_CLAMP_LOCK);
                    relicArm.setPosition(RELIC_ARM_LOWERED);
                    nextState();
                }
                break;

            // Reverse and suck up glyphs
            case 9:

                telemetry.addData("Block Count", blockCount);
                telemetry.addData("Voltage", vs.getVoltage());
                telemetry.addData("Distance Sensor Readings (cm): ", String.format(Locale.US, "%.02f", bDist.getDistance(DistanceUnit.CM)));

                telemetry.addData("Distance Sensor Readings (cm): ", String.format(Locale.US, "%.02f", fDist.getDistance(DistanceUnit.CM)));

                if(runtime.time() < .25) {
                    RobotLog.ii("STATE", "Starting up Intake");
                    rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightIntake.setPower(-1);
                    leftIntake.setPower(-1);
                } else if (runtime.time() < .325) {
                    suckTime.reset();
                    RobotLog.ii("STATE", "Recording Min and Max Voltage");
                    minV = Math.min(minV, vs.getVoltage());
                    maxV = Math.max(maxV, vs.getVoltage());
                    RobotLog.ii("SENSOR_INFO", "Min Voltage: %s", minV);
                    RobotLog.ii("SENSOR_INFO", "Max Voltage: %s", maxV);
                } else if (runtime.time() < 10) {

                    RobotLog.ii("STATE", "Sucking up Glyphs");
                    RobotLog.ii("SENSOR_INFO", "Voltage Difference: %s", minV - vs.getVoltage());


                    // Check for blocks using voltage
                    if (blockCount != 2) {
                        if (!block) {
                            rightIntake.setPower(-1);
                            leftIntake.setPower(-1);

                            // Perform a set of movement actions within the given time
                            // Suckstart is used as a bookmark for where the alg left off if a voltage drop was encountered
                            if (suckStart + suckTime.time() < 1.2) {
                                IMUdrive(.2, boxAngle);
                            } else if (suckStart + suckTime.time() < 1.8) {
                                if (i == 0){
                                    frontRightMotor.setPower(-.5);
                                    frontLeftMotor.setPower(.5);
                                    rearRightMotor.setPower(-.5);
                                    rearLeftMotor.setPower(.5);
                                } else{
                                    frontRightMotor.setPower(.5);
                                    frontLeftMotor.setPower(-.5);
                                    rearRightMotor.setPower(.5);
                                    rearLeftMotor.setPower(-.5);
                                }
                            } else if (turn(boxAngle)) {
                                if (i == 0) {
                                    i = 1;
                                } else {
                                    i = 0;
                                }
                            } else {
                                suckTime.reset();
                                suckStart = 0;
                            }

                            // Check for glyphs by looking if there is a voltage drop
                            // Check for glyphs by looking if there is a voltage drop
                            if (minV - vs.getVoltage() > .95) {
                                RobotLog.ii("STATE", "Encountered Voltage Drop/Glyph");
                                checkDelay = runtime.time() + 1;
                                suckStart = suckTime.time();
                                block = true;
                            }

                        } else {
                            RobotLog.ii("SENSOR_INFO", "Voltage During Glyph %s", vs.getVoltage());
                            RobotLog.ii("SENSOR_INFO", "Block Distance: %.02f", bDist.getDistance(DistanceUnit.CM));

                            // Check if glyphs is sucked up
                            if (minV - vs.getVoltage() < .4) {
                                if(sucked){
                                    RobotLog.ii("STATE", "Sucked in Glyph");
                                    blockCount=2;
                                    RobotLog.ii("OUTPUT_INFO", "Block Count: %s", blockCount);
                                    suckTime.reset();
                                    sucked = false;
                                    block = false;
                                } else{
                                    RobotLog.ii("STATE", "False Positive Voltage Return");
                                }
                            }

                            // To mitigate false positives uses a distance sensor
                            if (fDist.getDistance(DistanceUnit.CM) < 8 &&
                                    bDist.getDistance(DistanceUnit.CM) < 8){
                                sucked = true;
                            }

                            if (runtime.time() < checkDelay){
                                frontRightMotor.setPower(0);
                                frontLeftMotor.setPower(0);
                                rearRightMotor.setPower(0);
                                rearLeftMotor.setPower(0);
                            } else{
                                if (runtime.time() < checkDelay + .5) {
                                    if (i == 0){
                                        frontRightMotor.setPower(-.5);
                                        frontLeftMotor.setPower(.5);
                                        rearRightMotor.setPower(-.5);
                                        rearLeftMotor.setPower(.5);
                                    } else{
                                        frontRightMotor.setPower(.5);
                                        frontLeftMotor.setPower(-.5);
                                        rearRightMotor.setPower(.5);
                                        rearLeftMotor.setPower(-.5);
                                    }

                                } else if (turn(boxAngle)) {
                                    if (i == 0) {
                                        i = 1;
                                    } else {
                                        i = 0;
                                    }

                                    RobotLog.ii("STATE", "False Positive Voltage Drop");
                                    suckStart = 0;
                                    suckTime.reset();
                                    block = false;
                                }
                            }
                        }
                    } else{
                        rightIntake.setPower(0);
                        leftIntake.setPower(0);
                        frontRightMotor.setPower(0);
                        frontLeftMotor.setPower(0);
                        rearRightMotor.setPower(0);
                        rearLeftMotor.setPower(0);
                        encoderCount = frontLeftMotor.getCurrentPosition();
                        blockCount = 0;
                        fastNextState();
                    }

                } else {
                    RobotLog.ii("STATE", "Ran out of time to suck");
                    frontRightMotor.setPower(0);
                    frontLeftMotor.setPower(0);
                    rearRightMotor.setPower(0);
                    rearLeftMotor.setPower(0);
                    rightIntake.setPower(1);
                    leftIntake.setPower(1);
                    blockCount = 0;
                    fastNextState();
                }
                break;

            // Reposition to face the box, then determine color of current glyphs
            case 10:
                RobotLog.ii("STATE", "Turning to Face Cryptobox");
                if (turn(boxAngle)) {
                    int temp1;
                    int temp2;

                    temp1 = Math.min(frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition());
                    temp2 = Math.min(rearLeftMotor.getCurrentPosition(), rearRightMotor.getCurrentPosition());
                    //encoderCount = Math.min(temp1, temp2);
                    encoderCount = -600;

                }
                if ((frColor.alpha() + flColor.alpha())/2 > 200) {
                frontGrey = true;
                }
                else if ((frColor.alpha() + flColor.alpha())/2 <= 200) {
                frontBrown = true;
                }
                if ((brColor.alpha() + blColor.alpha())/2 > 200) {
                backGrey = true;
                }
                else if ((brColor.alpha() + blColor.alpha())/2  <= 200) {
                backBrown = true;
                }
                if (frontBrown || frontGrey ||
                    backBrown || backGrey ) {
                    RobotLog.ii("OUTPUT_INFO", "Front Brown: %s", frontBrown);
                    RobotLog.ii("OUTPUT_INFO", "Front Grey: %s", frontGrey);
                    RobotLog.ii("OUTPUT_INFO", "Back Brown: %s", backBrown);
                    RobotLog.ii("OUTPUT_INFO", "Back Grey: %s", backGrey);
                    fastNextState();
                }
                break;

            // Using information of all the glyphs collected so far, determine
            // column placement of the current glyphs to begin a cipher
            case 11:
                if (Center && firstBrown) {
                    if (frontBrown && backBrown) {
                        strafeDist = STRAFE_DIST_CENTER_LEFT ;
                        strafeDir = STRAFE_LEFT;
                        letsFindOut = 1;
                        fastNextState();
                    } if (frontGrey && backBrown) {
                        strafeDist = STRAFE_DIST_CENTER_LEFT;
                        strafeDir = STRAFE_LEFT;
                        letsFindOut = 2;
                        fastNextState();
                    } if (frontBrown && backGrey) {
                        strafeDist = STRAFE_DIST_CENTER_LEFT;
                        strafeDir = STRAFE_LEFT;
                        letsFindOut = 3;
                        fastNextState();
                    } if (frontGrey && backGrey){
                        strafeDist = STRAFE_DIST_CENTER_RIGHT;
                        strafeDir = STRAFE_RIGHT;
                        letsFindOut = 4;
                        fastNextState();
                    }

                } if (Center && firstGrey) {
                if (frontBrown && backBrown) {
                    strafeDist = STRAFE_DIST_CENTER_RIGHT;
                    strafeDir = STRAFE_RIGHT;
                    letsFindOut = 5;
                    fastNextState();

                } if (frontGrey && backBrown) {
                    strafeDist = STRAFE_DIST_CENTER_LEFT;
                    strafeDir = STRAFE_LEFT;
                    letsFindOut = 6;
                    fastNextState();

                } if (frontBrown && backGrey) {
                    strafeDist = STRAFE_DIST_CENTER_LEFT;
                    strafeDir = STRAFE_LEFT;
                    letsFindOut = 7;
                    fastNextState();

                } if (frontGrey && backGrey){
                    strafeDist = STRAFE_DIST_CENTER_LEFT;
                    strafeDir = STRAFE_LEFT;
                    letsFindOut = 8;
                    fastNextState();

                }

            } if (Left && firstBrown) {
                if (frontBrown && backBrown) {
                    strafeDist = STRAFE_DIST_LEFT_CENTER;
                    strafeDir = STRAFE_RIGHT;
                    letsFindOut = 9;
                    fastNextState();

                } if (frontGrey && backBrown) {
                    strafeDist = STRAFE_DIST_LEFT_CENTER;
                    strafeDir = STRAFE_RIGHT;
                    letsFindOut = 10;
                    fastNextState();

                } if (frontBrown && backGrey) {
                    strafeDist = STRAFE_DIST_LEFT_CENTER;
                    strafeDir = STRAFE_RIGHT;
                    letsFindOut = 11;
                    fastNextState();

                } if (frontGrey && backGrey){
                    strafeDist = STRAFE_DIST_LEFT_RIGHT;
                    strafeDir = STRAFE_RIGHT;
                    letsFindOut = 12;
                    fastNextState();

                }

            } if (Left && firstGrey) {
                if (frontBrown && backBrown) {
                    strafeDist = STRAFE_DIST_LEFT_RIGHT;
                    strafeDir = STRAFE_RIGHT;
                    letsFindOut = 13;
                    fastNextState();

                } if (frontGrey && backBrown) {
                    strafeDist = STRAFE_DIST_LEFT_CENTER;
                    strafeDir = STRAFE_RIGHT;
                    letsFindOut = 14;
                    fastNextState();

                } if (frontBrown && backGrey) {
                    strafeDist = STRAFE_DIST_LEFT_CENTER;
                    strafeDir = STRAFE_RIGHT;
                    letsFindOut = 15;
                    fastNextState();

                } if (frontGrey && backGrey){
                    strafeDist = STRAFE_DIST_LEFT_CENTER;
                    strafeDir = STRAFE_RIGHT;
                    letsFindOut = 16;
                    fastNextState();

                }

            } if (Right && firstBrown) {
                if (frontBrown && backBrown) {
                    strafeDist = STRAFE_DIST_RIGHT_LEFT;
                    strafeDir = STRAFE_LEFT;
                    letsFindOut = 17;
                    fastNextState();

                } if (frontGrey && backBrown) {
                    strafeDist = STRAFE_DIST_RIGHT_LEFT;
                    strafeDir = STRAFE_LEFT;
                    letsFindOut = 18;
                    fastNextState();

                } if (frontBrown && backGrey) {
                    strafeDist = STRAFE_DIST_RIGHT_CENTER;
                    strafeDir = STRAFE_LEFT;
                    letsFindOut = 19;
                    fastNextState();

                } if (frontGrey && backGrey){
                    strafeDist = STRAFE_DIST_RIGHT_LEFT;
                    strafeDir = STRAFE_LEFT;
                    letsFindOut = 20;
                    fastNextState();

                }

            } if (Right && firstGrey) {
                if (frontBrown && backBrown) {
                    strafeDist = STRAFE_DIST_RIGHT_LEFT;
                    strafeDir = STRAFE_LEFT;
                    letsFindOut = 21;
                    fastNextState();

                } if (frontGrey && backBrown) {
                    strafeDist = STRAFE_DIST_RIGHT_CENTER;
                    strafeDir = STRAFE_LEFT;
                    letsFindOut = 22;
                    fastNextState();

                } if (frontBrown && backGrey) {
                    strafeDist = STRAFE_DIST_RIGHT_LEFT;
                    strafeDir = STRAFE_LEFT;
                    letsFindOut = 23;
                    fastNextState();

                } if (frontGrey && backGrey){
                    strafeDist = STRAFE_DIST_RIGHT_LEFT;
                    strafeDir = STRAFE_LEFT;
                    letsFindOut = 24;
                    fastNextState();

                }

            }
                break;

            //


            // Fast approach the cryptobox
            case 12:
                RobotLog.ii("STATE", "Driving to Cryptobox from Glyph Pit");
                jewelPivot.setPosition(JEWEL_PIVOT_RAISED);
                jewelArm.setPosition(JEWEL_ARM_RAISED);
                alignArm.setPosition(ALIGN_ARM_LOWERED);
                glyphClamp.setPosition(GLYPH_CLAMP_UNLOCK);
                glyphDump.setPosition(GLYPH_DUMP_RAISED);
                relicClamp.setPosition(RELIC_CLAMP_LOCK);
                relicArm.setPosition(RELIC_ARM_LOWERED);
                firstBrown = false;
                frontBrown = false;
                frontGrey = false;
                backBrown = false;
                backGrey = false;
                if (encoderIMUdrive(-8, -FAST_DRIVE_SPEED, boxAngle)) {
                    nextState();
                }
                break;


            case 13:
                if (angleDrive(strafeDist, STRAFE_SPEED, strafeDir, boxAngle)) {
                    secondRun = true;
                    goToState(5);
                }
                break;


            case 15:
                RobotLog.ii("STATE", "Parking");

                    alignArm.setPosition(0);
                    stopStateMachine();

                break;


            case 7261:
                resetEncoder();
                requestOpModeStop();
                break;
            default:
                break;
        }
    }


    // *********************************************
    //          Navigation Functions
    // *********************************************

    public boolean PIDdrive(double distance, double speed) {

        int counts = (int) (COUNTS_PER_INCH * distance);

        frontRightMotor.setTargetPosition(counts);
        frontLeftMotor.setTargetPosition(counts);
        rearLeftMotor.setTargetPosition(counts);
        rearRightMotor.setTargetPosition(counts);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);
        rearRightMotor.setPower(speed);

        if (!frontLeftMotor.isBusy() ||
                !frontRightMotor.isBusy() ||
                !rearLeftMotor.isBusy()  ||
                !rearRightMotor.isBusy())  {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            rearLeftMotor.setPower(0);
            rearRightMotor.setPower(0);

            return true;
        }

        telemetry.addData("Encoder Target: ", counts);
        telemetry.addData("FL Current Encoder: ", frontLeftMotor.getCurrentPosition());
        telemetry.addData("FR Current Encoder: ", frontRightMotor.getCurrentPosition());
        telemetry.addData("RL Current Encoder: ", rearLeftMotor.getCurrentPosition());
        telemetry.addData("RR Current Encoder: ", rearRightMotor.getCurrentPosition());
        //telemetry.update();
        RobotLog.ii("PID_DRIVE", "Encoder Target: %s", counts);
        RobotLog.ii("PID_DRIVE", "FL Current Encoder: %s", frontLeftMotor.getCurrentPosition());
        RobotLog.ii("PID_DRIVE", "FR Current Encoder: %s", frontRightMotor.getCurrentPosition());
        RobotLog.ii("PID_DRIVE", "RL Current Encoder: %s", rearLeftMotor.getCurrentPosition());
        RobotLog.ii("PID_DRIVE", "RR Current Encoder: %s", rearRightMotor.getCurrentPosition());


        return false;

    }


    public boolean encoderIMUdrive(double distance, double speed, double angleTarget) {

        double angleError;
        double dError;
        double leftSpeed;
        double rightSpeed;
        double max;
        double speedCorrection;
        double angleCurrent;

        int counts = (int) (COUNTS_PER_INCH * distance);

        frontRightMotor.setTargetPosition(counts);
        frontLeftMotor.setTargetPosition(counts);
        rearLeftMotor.setTargetPosition(counts);
        rearRightMotor.setTargetPosition(counts);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Get the current angle
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleCurrent = angles.firstAngle;

        // PID caculations
        angleError = angleTarget - angleCurrent;
        dError = angleError - previousError;
        previousError = angleError;
        iError += angleError;

        // Correct the speed if the angle is off
        speedCorrection = (angleError * P_C_EI) + (iError * I_C_EI)+ (dError* D_C_EI);

        leftSpeed = speed - speedCorrection;
        rightSpeed = speed + speedCorrection;

        // normalize the speed
        max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        frontLeftMotor.setPower(leftSpeed);
        frontRightMotor.setPower(rightSpeed);
        rearLeftMotor.setPower(leftSpeed);
        rearRightMotor.setPower(rightSpeed);

        if (!frontLeftMotor.isBusy() ||
                !frontRightMotor.isBusy() ||
                !rearLeftMotor.isBusy()  ||
                !rearRightMotor.isBusy())  {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            rearLeftMotor.setPower(0);
            rearRightMotor.setPower(0);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            return true;
        }

        RobotLog.ii("encoderIMUdrive", "Target Angle %s", angleTarget);
        RobotLog.ii("encoderIMUdrive", "Current Angle %s", angleCurrent);
        RobotLog.ii("encoderIMUdrive", "Angle Error %s", angleError);
        RobotLog.ii("encoderIMUdrive", "I Error %s", iError);
        RobotLog.ii("encoderIMUdrive", "Encoder Target: %s", counts);
        RobotLog.ii("encoderIMUdrive", "FL Current Encoder: %s", frontLeftMotor.getCurrentPosition());
        RobotLog.ii("encoderIMUdrive", "FR Current Encoder: %s", frontRightMotor.getCurrentPosition());
        RobotLog.ii("encoderIMUdrive", "RL Current Encoder: %s", rearLeftMotor.getCurrentPosition());
        RobotLog.ii("encoderIMUdrive", "RR Current Encoder: %s", rearRightMotor.getCurrentPosition());
        RobotLog.ii("encoderIMUdrive", "Steer Correction: %s", speedCorrection);
        RobotLog.ii("encoderIMUdrive", "Left Speed: %s", leftSpeed);
        RobotLog.ii("encoderIMUdrive", "Right Speed: %s", rightSpeed);
        RobotLog.ii("encoderIMUdrive", "RL Motor Mode: %s", rearLeftMotor.getMode());


        return false;

    }


    public void IMUdrive(double speed, double angleTarget) {

        double angleError;
        double dError;
        double leftSpeed;
        double rightSpeed;
        double max;
        double speedCorrection;
        double angleCurrent;

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Get the current angle
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleCurrent = angles.firstAngle;

        // PID caculations
        angleError = angleTarget - angleCurrent;
        dError = angleError - previousError;
        previousError = angleError;
        iError += angleError;

        // Correct the speed if the angle is off
        speedCorrection = (angleError * P_C_EI) + (iError * I_C_EI)+ (dError* D_C_EI);

        leftSpeed = speed - speedCorrection;
        rightSpeed = speed + speedCorrection;

        // normalize the speed
        max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        frontLeftMotor.setPower(leftSpeed);
        frontRightMotor.setPower(rightSpeed);
        rearLeftMotor.setPower(leftSpeed);
        rearRightMotor.setPower(rightSpeed);

        RobotLog.ii("IMUdrive", "Target Angle %s", angleTarget);
        RobotLog.ii("IMUdrive", "Current Angle %s", angleCurrent);
        RobotLog.ii("IMUdrive", "Angle Error %s", angleError);
        RobotLog.ii("IMUdrive", "I Error %s", iError);
        RobotLog.ii("IMUdrive", "Steer Correction: %s", speedCorrection);
        RobotLog.ii("IMUdrive", "Left Speed: %s", leftSpeed);
        RobotLog.ii("IMUdrive", "Right Speed: %s", rightSpeed);
        RobotLog.ii("IMUdrive", "RL Motor Mode: %s", rearLeftMotor.getMode());

        return;

    }

    public boolean turn(double angleTarget) {
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        float angleCurrent;
        double angleError;
        double dError;
        double drivePower;

        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //telemetry.addData("Heading: ", angles.firstAngle);
        angleCurrent = angles.firstAngle;

        angleError = angleTarget - angleCurrent;
        dError = angleError - previousError;

        previousError = angleError;

        if ((Math.abs(angleError) > HEADING_THRESHOLD)) {
            iError += angleError;

            drivePower = (angleError * P_C_TURN) + (iError * I_C_TURN)+ (previousError* D_C_TURN);

            drivePower = Range.clip(drivePower, -1.0, 1.0);

            frontRightMotor.setPower(drivePower);
            frontLeftMotor.setPower(-drivePower);
            rearRightMotor.setPower(drivePower);
            rearLeftMotor.setPower(-drivePower);

            RobotLog.ii("TURN", "Target Angle %s", angleTarget);
            RobotLog.ii("TURN", "Current Angle %s", angleCurrent);
            RobotLog.ii("TURN", "Angle Error %s", angleError);
            RobotLog.ii("TURN", "I Error %s", iError);

            //telemetry.addData("Target Angle", angleTarget);
            //telemetry.addData("Current Angle", angleCurrent);
            //telemetry.addData("Angle Error", angleError);
            //telemetry.addData("Drive Power", drivePower);
            //telemetry.addData("Raw", odsSensor.getRawLightDetected());
            // telemetry.addData("Normal", odsSensor.getLightDetected());
            telemetry.update();

            return false;
        } else {
            RobotLog.ii("TURN", "Turn Finished");
            iError = 0;
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            rearRightMotor.setPower(0);
            rearLeftMotor.setPower(0);


            return true;
        }
    }

    public void resetEncoder(){
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    /**
     * Drives the robot at a specified angle and speed
     * @param speed The speed at which the bot moves
     * @param theta The angle at which the bot moves. Depending on the orientation of the gyro,
     *              the angle measured starts from the front of the robot and rotation clockwise or
     *              counterclockwise results in a positive or negative angle. Angles are measured in radians.
     */

    public boolean angleDrive(double distance, double speed, double theta, double fixedHeading){
        //Declare and/or intialize variables for the mecanum wheel velocity equations
        double raw_v_1, raw_v_2, raw_v_3, raw_v_4;
        double scaled_v_1, scaled_v_2, scaled_v_3, scaled_v_4;

        double max_v = 1;

        //Declare and/or initialize variables for maintaining the robot's current facing
        double angleCurrent;
        double angleError;

        //Declare and/or initialize variables for the PID control loop that corrects the robot's rotation
        double pError;
        double dError;
        double correctionOutput;

        // Calculates the encoder counts
        // Since the mecanum strafes the counts have to be scaled by a power of radical two
        int counts = (int) (COUNTS_PER_INCH * distance* 1.414);

        frontLeftMotor.setTargetPosition(counts);
        frontRightMotor.setTargetPosition(-counts);
        rearLeftMotor.setTargetPosition(-counts);
        rearRightMotor.setTargetPosition(counts);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Get the angle reading from the imu
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //telemetry.addData("Heading: ", angles.firstAngle);
        angleCurrent = angles.firstAngle;

        //Uses a PID control loop in order to make sure the heading of the bot doesn't change
        //Calculate the PID errors and the appropriate power levels
        pError = fixedHeading - angleCurrent;        //Finds the current difference between the target and current
        iError += pError;
        dError = pError - previousError;

        previousError = pError;

        correctionOutput = (pError * P_C_AD) + (iError *I_C_AD) + (dError* D_C_AD);

        RobotLog.ii("ANGLE_DRIVE", "Fixed Heading: %s", fixedHeading);
        RobotLog.ii("ANGLE_DRIVE", "Current Angle: %s", angleCurrent);
        RobotLog.ii("ANGLE_DRIVE", "Angle Error: %s", pError);
        RobotLog.ii("ANGLE_DRIVE", "I Error: %s", iError);
        RobotLog.ii("ANGLE_DRIVE", "Correction Output: %s", correctionOutput);



        //Calculate the raw motor output
        raw_v_1 = speed*Math.sin(theta + (Math.PI/4)) - correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_1);
        raw_v_2 = speed*Math.cos(theta + (Math.PI/4)) + correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_2);
        raw_v_3 = speed*Math.cos(theta + (Math.PI/4)) - correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_3);
        raw_v_4 = speed*Math.sin(theta + (Math.PI/4)) + correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_4);

        //If the raw motor outputs exceed the [-1, 1], the outputs are scaled
        if (max_v > 1.0){
            scaled_v_1 = raw_v_1/max_v;
            scaled_v_2 = raw_v_2/max_v;
            scaled_v_3 = raw_v_3/max_v;
            scaled_v_4 = raw_v_4/max_v;
        }
        else{
            scaled_v_1 = raw_v_1;
            scaled_v_2 = raw_v_2;
            scaled_v_3 = raw_v_3;
            scaled_v_4 = raw_v_4;
        }

        frontLeftMotor.setPower(scaled_v_1);
        frontRightMotor.setPower(scaled_v_2);
        rearLeftMotor.setPower(scaled_v_3);
        rearRightMotor.setPower(scaled_v_4);

        RobotLog.ii("ANGLE_DRIVE", "Front Left Velocity: %s", scaled_v_1);
        RobotLog.ii("ANGLE_DRIVE", "Front Right Velocity: %s", scaled_v_2);
        RobotLog.ii("ANGLE_DRIVE", "Rear Left Velocity: %s", scaled_v_3);
        RobotLog.ii("ANGLE_DRIVE", "Rear Right Velocity: %s", scaled_v_4);

        if (!frontLeftMotor.isBusy() ||
                !frontRightMotor.isBusy() ||
                !rearLeftMotor.isBusy()  ||
                !rearRightMotor.isBusy())  {
            RobotLog.ii("ANGLE_DRIVE", "Angle Drive/Strafe Finished");
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            rearLeftMotor.setPower(0);
            rearRightMotor.setPower(0);

            return true;
        }


        return false;
    }

    /**
     * Drives the robot at a specified angle and speed
     * @param speed The speed at which the bot moves
     * @param theta The angle at which the bot moves. Depending on the orientation of the gyro,
     *              the angle measured starts from the front of the robot and rotation clockwise or
     *              counterclockwise results in a positive or negative angle. Angles are measured in radians.
     */
    public void angleDrive(double speed, double theta, double fixedHeading){
        //Declare and/or intialize variables for the mecanum wheel velocity equations
        double raw_v_1, raw_v_2, raw_v_3, raw_v_4;
        double scaled_v_1, scaled_v_2, scaled_v_3, scaled_v_4;

        double max_v = 0;

        //Declare and/or initialize variables for maintaining the robot's current facing
        double angleCurrent;

        //Declare and/or initialize variables for the PID control loop that corrects the robot's rotation
        double pError;
        double dError;
        double correctionOutput;

        // Get the angle reading from the imu
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //telemetry.addData("Heading: ", angles.firstAngle);
        angleCurrent = angles.firstAngle;

        //Uses a PID control loop in order to make sure the heading of the bot doesn't change
        //Calculate the PID errors and the appropriate power levels
        pError = fixedHeading - angleCurrent;        //Finds the current difference between the target and current
        iError += pError;
        dError = pError - previousError;

        previousError = pError;

        correctionOutput = (pError * P_C_AD) + (iError *I_C_AD) + (dError* D_C_AD);

        RobotLog.ii("ANGLE_DRIVE", "Fixed Heading: %s", fixedHeading);
        RobotLog.ii("ANGLE_DRIVE", "Current Angle: %s", angleCurrent);
        RobotLog.ii("ANGLE_DRIVE", "Angle Error: %s", pError);
        RobotLog.ii("ANGLE_DRIVE", "I Error: %s", iError);
        RobotLog.ii("ANGLE_DRIVE", "Correction Output: %s", correctionOutput);

        //Calculate the raw motor output
        raw_v_1 = speed*Math.sin(theta + (Math.PI/4)) - correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_1);
        raw_v_2 = speed*Math.cos(theta + (Math.PI/4)) + correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_2);
        raw_v_3 = speed*Math.cos(theta + (Math.PI/4)) - correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_3);
        raw_v_4 = speed*Math.sin(theta + (Math.PI/4)) + correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_4);

        //If the raw motor outputs exceed the [-1, 1], the outputs are scaled
        if (max_v > 1.0){
            scaled_v_1 = raw_v_1/max_v;
            scaled_v_2 = raw_v_2/max_v;
            scaled_v_3 = raw_v_3/max_v;
            scaled_v_4 = raw_v_4/max_v;
        }
        else{
            scaled_v_1 = raw_v_1;
            scaled_v_2 = raw_v_2;
            scaled_v_3 = raw_v_3;
            scaled_v_4 = raw_v_4;
        }

        //Set the robot to run at the calculate power
        frontLeftMotor.setPower(scaled_v_1);
        frontRightMotor.setPower(scaled_v_2);
        rearLeftMotor.setPower(scaled_v_3);
        rearRightMotor.setPower(scaled_v_4);

        RobotLog.ii("ANGLE_DRIVE", "Front Left Velocity: %s", scaled_v_1);
        RobotLog.ii("ANGLE_DRIVE", "Front Right Velocity: %s", scaled_v_2);
        RobotLog.ii("ANGLE_DRIVE", "Rear Left Velocity: %s", scaled_v_3);
        RobotLog.ii("ANGLE_DRIVE", "Rear Right Velocity: %s", scaled_v_4);



        return;
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



    // *********************************************
    //          State Machine Functions
    // *********************************************


    /**
     *  Check if the encoders are reset and adds in a small delay to account for hardware cycles
     *  Sets the motors to run using encoders
     */
    public boolean resetDelay(){
        if (Math.abs(frontLeftMotor.getCurrentPosition()) < 2 &&
                Math.abs(frontRightMotor.getCurrentPosition()) < 2 &&
                Math.abs(rearLeftMotor.getCurrentPosition()) < 2  &&
                Math.abs(rearRightMotor.getCurrentPosition()) < 2){

            rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            previousError = 0;
            iError = 0;
            runtime.reset();

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
        this.resetEncoder();
        previousState = state;
        state = -1;
        runtime.reset();

        return;

    }


    public void fastNextState() {
        runtime.reset();
        state++;

        return;

    }


    /**
     * Preps the state machine to change to the designated state
     * @param dstState The next state
     */
    public void goToState(int dstState){
        this.resetEncoder();
        previousState = dstState - 1;
        state = -1;
        runtime.reset();
        return;
    }

    /**
     * Function that is called to end the state machine
     */
    public void stopStateMachine() {
        state = 7261;
        this.resetEncoder();
        return;
    }
}

