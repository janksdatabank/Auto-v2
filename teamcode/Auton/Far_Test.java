package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.multiGlyph.IMU;
import org.firstinspires.ftc.teamcode.multiGlyph.PID;
import org.firstinspires.ftc.teamcode.multiGlyph.WirelessPID;
import org.firstinspires.ftc.teamcode.multiGlyph.odemetry.PositionTracking;

import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.Locale;


/**
 * Created by Janker on 10/2/2017.
 */
@Autonomous(name="Far_Test",group="Auton")
//@Disabled
public class Far_Test extends OpMode {

    //* Everything before next comment is defining variables to be accessed
    //* for the the rest of the time

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 5.45;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double HEADING_THRESHOLD = .50;

    static final double P_C_TURN = 0.011;
    static final double I_C_TURN = 0.0000125;
    static final double D_C_TURN = 0;



    static final double P_C_EI = 0.04;
    static final double I_C_EI = 0.00000125;
    static final double D_C_EI = 0;



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

    CRServo rollerWheels;

    //BNO055IMU imu;

    IMU imu;

    double angles;
    PositionTracking positionTracking;
    VoltageSensor vs;
    boolean stop = false;
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
    static final double CENTER_COL_DIST_BLUE                        = 22.6;               // The dist the bot runs off platform to align w/ center col (Blue)
    static final double RIGHT_COL_DIST_BLUE                         = 22.6 + 5.5;
    static final double LEFT_COL_DIST_BLUE                          = 22.6 - 4.5;
    static final double ANGLE_OF_BOX_BLUE                           = -90;              // Angle to face the blue cryptobox
    static final double CENTER_COL_DIST_RED                         = -24.5;
    static final double RIGHT_COL_DIST_RED                          = -(24.5 - 6);
    static final double LEFT_COL_DIST_RED                           = -(24.5 + 5);
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
    static final double JEWEL_PIVOT_RAISED                          = .35;
    static final double JEWEL_PIVOT_LOWERED                         = .40;
    static final double JEWEL_PIVOT_LEFT                            = .60;               //Position for the jewel arm to hit the left jewel
    static final double JEWEL_PIVOT_RIGHT                           = .05;
    static final double ALIGN_ARM_RAISED                            = 0;
    static final double ALIGN_ARM_DETECT                            = .35;
    static final double ALIGN_ARM_LOWERED                           = .63;
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

        rollerWheels = hardwareMap.crservo.get("rW");

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

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        alignArm.setPosition(ALIGN_ARM_IN-.05);
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


        // Retrieve and initialize the IMU. The imu is built into the Rev Expansion Hub
        // The imu is accessible through the I2C port 0
        IMU imu = new IMU();
        imu.initIMU(hardwareMap,"imu");
        this.imu = imu;
        this.positionTracking = new PositionTracking(this,imu,rightIntake,Extension,frontLeftMotor,rearLeftMotor,frontRightMotor,rearRightMotor);

        this.positionTracking.startAbsoluteEncoderThread();

        positionTracking.zeroEncoders();

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

        //telemetry.addData("Direction", strafeDir);
        //telemetry.addData("Distance", strafeDist);

        telemetry.addData("Cipher", letsFindOut);
        telemetry.addData("positione", jewelArm.getPosition());
        telemetry.addData("Case:", state);
        telemetry.addData("xWheel",positionTracking.xPosition());
        telemetry.addData("xWheel inches",positionTracking.xPosition()/positionTracking.xPerInch);
        telemetry.addData("xWheel raw",rightIntake.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("yWheel",positionTracking.yPosition());
        telemetry.addData("yWheel inches",positionTracking.yPosition()/positionTracking.yPerInch);
        telemetry.addData("yWheel raw",Extension.getCurrentPosition());



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
                relicArm.setPosition(RELIC_ARM_RAISED);
                alignArm.setPosition(ALIGN_ARM_LOWERED);

                vuMark = RelicRecoveryVuMark.from(relicTemplate);

                if (runtime.time() < 2) {
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
                } else {
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

                } else if (runtime.time() < .75) {
                    RobotLog.ii("STATE", "Identifying Color");
                    RobotLog.ii("SENSOR_INFO", "Red Values: %s", blueColorSensor.red());
                    RobotLog.ii("SENSOR_INFO", "Blue Values: %s", blueColorSensor.blue());
                    if (blueColorSensor.red() > blueColorSensor.blue()) {
                        RobotLog.ii("OUTPUT_INFO", "Left Jewel Red");
                        measuredColor = "Red";
                    } else {
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
                jewelArm.setPosition(JEWEL_ARM_RAISED);
                jewelPivot.setPosition(JEWEL_PIVOT_RAISED);
                if ((frColor.alpha() + flColor.alpha()) / 2 > 200) {
                    firstGrey = true;
                    RobotLog.ii("OUTPUT_INFO", "First Grey: %s", firstGrey);
                    fastNextState();

                } else {
                    firstBrown = true;
                    RobotLog.ii("OUTPUT_INFO", "First Brown: %s", firstBrown);
                    fastNextState();

                }
                break;

            // Drive forward to align with the pictograph column and retract the color sensor
            case 3:
                if (PIDdrive(-21, -.3)) {
                    positionTracking.xPositionAbs=0;
                    positionTracking.yPositionAbs=0;
                    nextState();
                }

                break;
            case 4:
                positionTracking.autonomousDrive(-35*positionTracking.xPerInch, -36 * positionTracking.yPerInch, 0, 0);
                break;
            case 5:
                stopStateMachine();
            break;
        }
    }


    // *********************************************
    //          Navigation Functions
    // *********************************************

    public void stop() {
        positionTracking.stopTracking();


        positionTracking.stopDrive();

        stop = true;
    }



    private String formatVal(double val)
    {
        DecimalFormat df = new DecimalFormat("#.###");
        df.setRoundingMode(RoundingMode.CEILING);
        return df.format(val);
    }

    private double pError,iError,dError,previousError;
    private double P_C_AD = .009;
    private double I_C_AD = 0;
    private double D_C_AD = .009;

    private double pid (double fixedHeading, double angleCurrent) {
        pError = fixedHeading - angleCurrent;        //Finds the current difference between the target and current
        iError += pError;
        dError = pError - previousError;

        previousError = pError;

        return (pError * P_C_AD) + (iError *I_C_AD) + (dError* D_C_AD);
    }



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
        angles  = imu.getHeading();
        angleCurrent = imu.getHeading();

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
        angles  = imu.getHeading();
        angleCurrent = imu.getHeading();

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

        double angleCurrent;
        double angleError;
        double dError;
        double drivePower;

        angles  = imu.getHeading();
        //telemetry.addData("Heading: ", angles.firstAngle);
        angleCurrent = imu.getHeading();

        angleError = angleTarget - angleCurrent;
        dError = angleError - previousError;

        previousError = angleError;

        if ((Math.abs(angleError) > HEADING_THRESHOLD)) {
            iError += angleError;

            drivePower = (angleError * P_C_TURN) + (iError * I_C_TURN)+ (previousError* D_C_TURN);

            drivePower = Range.clip(drivePower, -1.0, 1.0);

            if (drivePower >= 0){
                drivePower = Math.max(drivePower, .07);
            } else{
                drivePower = Math.min(drivePower, -.07);
            }

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
        angles  = imu.getHeading();
        //telemetry.addData("Heading: ", angles.firstAngle);
        angleCurrent = imu.getHeading();

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
        angles  = imu.getHeading();
        //telemetry.addData("Heading: ", angles.firstAngle);
        angleCurrent = imu.getHeading();

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
    public void gyroTurn(double turnSpeed, int targetAngle, int allowedError) {
        if (targetAngle<0) {
            targetAngle += 360;
        }
        while (Math.abs(imu.getHeading()-targetAngle)>allowedError&&!stop) {
            double pidOffset = positionTracking.turnPID.run(targetAngle,(int)imu.getHeading());
            double power = -pidOffset * turnSpeed;
            robotCentric(0, 0, power);
        }
        robotCentric(0,0,0);
    }
    public void robotCentric(double forwards, double horizontal, double turning) {
        double leftFront = forwards + horizontal + turning;
        double leftBack = forwards - horizontal + turning;
        double rightFront = forwards - horizontal - turning;
        double rightBack = forwards + horizontal - turning;

        double[] wheelPowers = {Math.abs(rightFront), Math.abs(leftFront), Math.abs(leftBack), Math.abs(rightBack)};
        Arrays.sort(wheelPowers);
        double biggestInput = wheelPowers[3];
        if (biggestInput > 1) {
            leftFront /= biggestInput;
            leftBack /= biggestInput;
            rightFront /= biggestInput;
            rightBack /= biggestInput;
        }

        frontLeftMotor.setPower(leftFront);
        frontRightMotor.setPower(rightFront);
        rearLeftMotor.setPower(leftBack);
        rearRightMotor.setPower(rightBack);
    }
}




