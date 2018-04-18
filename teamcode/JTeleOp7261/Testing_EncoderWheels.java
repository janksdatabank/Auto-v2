package org.firstinspires.ftc.teamcode.JTeleOp7261;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.multiGlyph.IMU;
import org.firstinspires.ftc.teamcode.multiGlyph.PID;
import org.firstinspires.ftc.teamcode.multiGlyph.WirelessPID;
import org.firstinspires.ftc.teamcode.multiGlyph.odemetry.PositionTracking;

import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.Arrays;

/**
 * Created by Jankers on 9/22/2017.
 */
@TeleOp(name="Test Encoders TeleOp", group = "TeleOp")
public class Testing_EncoderWheels extends OpMode {
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

    PositionTracking positionTracking;
    IMU imu;

    //PID tuner
    private double p, i, d;
    private WirelessPID pidUdpReceiver;
    private double distance = 0;
    private double distance1 = 0;
    private double angle = 0;
    private double speed = .07;

    ModernRoboticsI2cGyro gyro;

    public ElapsedTime ejectTime = new ElapsedTime();
    boolean ejecting = false;
    boolean bCurrState_y = false;
    boolean bPrevState_y = false;

    boolean reversed;
    boolean bCurrState_x = false;
    boolean bPrevState_x = false;
    boolean precTurn;
    boolean bCurrState_b = false;
    boolean bPrevState_b = false;


    static final double PREC_DRIVE_SPEED                        = .8;
    static final double PREC_STRAFE_SPEED                       = .8;
    static final double PREC_STRAFE_SPEED_2                     = .4;
    static final double PREC_TURN_SPEED                         = .2;

    // *********************************************
    //          Servo Positions
    // *********************************************
    static final double GLYPH_CLAMP_LOCK                            = .8;
    static final double GLYPH_CLAMP_UNLOCK                          = .45;
    static final double GLYPH_DUMP_RAISED                           = .94;
    static final double GLYPH_DUMP_LOWERED                          = .42;
    static final double JEWEL_ARM_IN                                = .85;
    static final double JEWEL_ARM_RAISED                            = .70;
    static final double JEWEL_ARM_LOWERED                           = .18;
    static final double JEWEL_PIVOT_RAISED                          = .50;
    static final double JEWEL_PIVOT_LOWERED                         = .62;
    static final double JEWEL_PIVOT_LEFT                            = .8;               //Position for the jewel arm to hit the left jewel
    static final double JEWEL_PIVOT_RIGHT                           = .2;
    static final double ALIGN_ARM_RAISED                            = 0;
    static final double ALIGN_ARM_DETECT                            = .65;
    static final double ALIGN_ARM_LOWERED                           = .65;
    static final double ALIGN_ARM_IN                                = .9;
    static final double RELIC_ARM_LOWERED                           = .54;
    static final double RELIC_ARM_RAISED                            = .05;
    static final double RELIC_ARM_IN                                = 1;
    static final double RELIC_CLAMP_LOCK                            = 0;
    static final double RELIC_CLAMP_UNLOCK                          = 1;

    public PID xPID = new PID();

    boolean once = true;

    @Override
    public void init() {

        rearLeftMotor = hardwareMap.dcMotor.get("rL");
        frontLeftMotor = hardwareMap.dcMotor.get("fL");
        rearRightMotor = hardwareMap.dcMotor.get("rR");
        frontRightMotor = hardwareMap.dcMotor.get("fR");

        Lift = hardwareMap.dcMotor.get("L");
        Extension = hardwareMap.dcMotor.get("E");
        leftIntake = hardwareMap.dcMotor.get("lI");
        rightIntake = hardwareMap.dcMotor.get("rI");

        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Lift.setDirection(DcMotor.Direction.REVERSE);
        Extension.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);

        jewelPivot = hardwareMap.servo.get("lP");
        relicArm = hardwareMap.servo.get("rA");
        relicClamp = hardwareMap.servo.get("rC");
        jewelArm = hardwareMap.servo.get("lJA");
        glyphClamp = hardwareMap.servo.get("fC");
        glyphDump = hardwareMap.servo.get("fD");
        alignArm = hardwareMap.servo.get("aA");
        rollerWheels = hardwareMap.crservo.get("rW");

        Extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IMU imu = new IMU();
        imu.initIMU(hardwareMap,"imu");
        this.imu = imu;

        this.positionTracking = new PositionTracking(this,imu,rightIntake,Extension,frontLeftMotor,rearLeftMotor,frontRightMotor,rearRightMotor);

        this.positionTracking.startAbsoluteEncoderThread();

        positionTracking.zeroEncoders();


        //PID tuner
        pidUdpReceiver = new WirelessPID();
        pidUdpReceiver.beginListening();

        telemetry.setMsTransmissionInterval(50);


        // *********************************************
        //          Setting Initial Servo Positions
        // *********************************************
        // Now on controller 1 to prevent time loss on intialize.



    }

    @Override
    public void loop() {
        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);

        double forwBack = 0;
        double lefRight = 0;
        double spin = 0;
        int angleCurrent;
        double raw_FL_speed;
        double raw_FR_speed;
        double raw_RL_speed;
        double raw_RR_speed;
        double max_v = 0;
        double scaled_FL_speed;
        double scaled_FR_speed;
        double scaled_RL_speed;
        double scaled_RR_speed;


        // *********************************************
        //          Gamepad 1 Controls
        // *********************************************



        // Send telemetry message to signify robot running;
        //telemetry.addData("Forward Speed", "%.2f", forwBack);
        //telemetry.addData("Strafe Speed", "%.2f", lefRight);
        //telemetry.addData("Spin Speed", "%.2f", spin);
        telemetry.addData("FR", frontRightMotor.getPower());
        telemetry.addData("RL", rearLeftMotor.getPower());
        telemetry.addData("FL", frontLeftMotor.getPower());
        telemetry.addData("RR", rearRightMotor.getPower());


        double leftY = Math.signum(-gamepad1.left_stick_y) * Math.pow(-gamepad1.left_stick_y, 2);
        double leftX = Math.signum(gamepad1.left_stick_x) * Math.pow(gamepad1.left_stick_x, 2);
        double rightX = Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2);

        //robotCentric(leftY,leftX,rightX);

        telemetry.addLine();
        telemetry.addData("xWheel",positionTracking.xPosition());
        telemetry.addData("xWheel inches",positionTracking.xPosition()/positionTracking.xPerInch);
        telemetry.addData("xWheel raw",rightIntake.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("yWheel",positionTracking.yPosition());
        telemetry.addData("yWheel inches",positionTracking.yPosition()/positionTracking.yPerInch);
        telemetry.addData("yWheel raw",Extension.getCurrentPosition());

        telemetry.addData("Heading",imu.getHeading());

        telemetry.addData("Error",positionTracking.angleError);

        if (gamepad1.start) {
            positionTracking.zeroEncoders();
        }

        if (gamepad1.a) {
            distance = 0;
            distance1 = 0;
        }

        if (gamepad1.dpad_up) {
            distance1 += .05;
        } else if (gamepad1.dpad_down) {
            distance1 -= .05;
        }

        if (gamepad1.dpad_left) {
            distance += .05;
        } else if (gamepad1.dpad_right) {
            distance -= .05;
        }

        if (gamepad1.b) {
            angle += 1;
        } else if (gamepad1.x) {
            angle -= 1;
        }

        telemetry.addData("Distance Y",distance);
        telemetry.addData("Distance X",distance1);
        telemetry.addData("Angle",angle);

        telemetry.addData("x target counts", distance*positionTracking.xPerInch);
        telemetry.addData("y target counts", distance*positionTracking.yPerInch);

        telemetry.addLine();

        telemetry.addData("x pre",positionTracking.xStrafe);
        telemetry.addData("y pre",positionTracking.yForwrd);

        //telemetry.addData("error",positionTracking.xPID.error);

        telemetry.update();

        if (gamepad1.y) {
            positionTracking.drive((int)(distance1*positionTracking.xPerInch),-(int)(distance*positionTracking.yPerInch),(int)angle,(int)imu.getHeading());
            /*if (once) {
                positionTracking.driveToCoordinatesThread((int)(distance1*positionTracking.xPerInch),-(int)(distance*positionTracking.yPerInch),(int)angle,(int)imu.getHeading());
                once = false;
            }*/

            //positionTracking.driveToCordinates(distance*positionTracking.xPerInch,0,0,0);
            //distance*positionTracking.xPerInch,0,0,0
        } else {
            //positionTracking.fieldCentric(leftY,leftX,rightX,(int)imu.getHeading());
        }

        /*if (!positionTracking.driveThread) {
            robotCentric(leftY,leftX,rightX);
            once = true;
        }*/
//
        //bCurrState_x = gamepad1.x;

        // check for button-press state transitions.
        if ((bCurrState_x == true) && (bCurrState_x != bPrevState_x))  {
            reversed = !reversed;
        }

        // update previous state variable.
        bPrevState_x = bCurrState_x;

        //bCurrState_b = gamepad1.b;

        // check for button-press state transitions.
        if ((bCurrState_b == true) && (bCurrState_b != bPrevState_b))  {
            precTurn = !precTurn;
        }

        // update previous state variable.
        bPrevState_b = bCurrState_b;


        if (gamepad1.right_trigger > .1) {
            rollerWheels.setPower(.8);
        }
        else if (gamepad1.left_trigger > .1) {
            rollerWheels.setPower(-.8);
        } else {
            rollerWheels.setPower(0);
        }
        if (gamepad1.right_bumper) {
            rightIntake.setPower(1);
            leftIntake.setPower(-1);
        }else if (gamepad1.left_bumper) {
            rightIntake.setPower(-1);
            leftIntake.setPower(1);
        } else {
            rightIntake.setPower(0);
            leftIntake.setPower(0);
        }
        if (gamepad2.y) {
            relicClamp.setPosition(RELIC_CLAMP_LOCK+.20);
            relicArm.setPosition(RELIC_ARM_LOWERED);
            glyphDump.setPosition(GLYPH_DUMP_RAISED);
            glyphClamp.setPosition(GLYPH_CLAMP_UNLOCK);
            alignArm.setPosition(ALIGN_ARM_LOWERED);
            jewelPivot.setPosition(JEWEL_PIVOT_RAISED);
            jewelArm.setPosition(JEWEL_ARM_RAISED);

        }

        // *********************************************
        //          Gamepad 2 Controls
        // *********************************************

        Lift.setPower(gamepad2.left_stick_y);
        //if (Math.abs(gamepad2.left_stick_y) > .1){
        //    glyphDump.setPosition(PREC_DRIVE_SPEED);
        //}
        Extension.setPower(-gamepad2.right_stick_y);

        if (gamepad2.dpad_up) {
            relicClamp.setPosition(RELIC_CLAMP_UNLOCK);
        }
        if (gamepad2.dpad_down) {
            relicClamp.setPosition(RELIC_CLAMP_LOCK);
        }
        if (gamepad2.left_trigger > .1) {
            glyphDump.setPosition(GLYPH_DUMP_LOWERED);
        }
        if (gamepad2.right_trigger > .1) {
            glyphDump.setPosition(GLYPH_DUMP_RAISED);
            alignArm.setPosition(ALIGN_ARM_LOWERED);

        }
        if (gamepad2.left_bumper) {
            glyphClamp.setPosition(GLYPH_CLAMP_LOCK);
            alignArm.setPosition(ALIGN_ARM_RAISED);
        }
        if (gamepad2.right_bumper) {
            glyphClamp.setPosition(GLYPH_CLAMP_UNLOCK);
        }

        if (gamepad2.dpad_left) {
            relicArm.setPosition(RELIC_ARM_LOWERED);
        }
        if (gamepad2.dpad_right) {
            relicArm.setPosition(RELIC_ARM_RAISED);
        }
        if (gamepad2.x) {
            relicArm.setPosition(.55+.04);
            jewelArm.setPosition(JEWEL_ARM_RAISED);
        }
        if (gamepad2.a) {
            relicClamp.setPosition(RELIC_CLAMP_LOCK);
            relicArm.setPosition(RELIC_ARM_IN);
            jewelArm.setPosition(JEWEL_ARM_IN);
            alignArm.setPosition(ALIGN_ARM_IN);
        }



        //bCurrState_y = gamepad1.y;

        // check for button-press state transitions.
        if ((bCurrState_y == true) && (bCurrState_y != bPrevState_y))  {
            ejecting = true;
            ejectTime.reset();
        }

        // update previous state variable.
        bPrevState_y = bCurrState_y;

        if (ejecting){
            if (ejectTime.time() < .25){
                glyphDump.setPosition(GLYPH_DUMP_LOWERED+.3);
                relicArm.setPosition(RELIC_ARM_RAISED);
            }
            else if (ejectTime.time() < .5){
                alignArm.setPosition(1);
                glyphDump.setPosition(.8);
                relicArm.setPosition(RELIC_ARM_RAISED);
            } else if (ejectTime.time() < .525){
                glyphDump.setPosition(GLYPH_DUMP_RAISED);
                relicArm.setPosition(RELIC_ARM_RAISED);
            } else{
                alignArm.setPosition(ALIGN_ARM_LOWERED);
                relicArm.setPosition(RELIC_ARM_LOWERED);
                ejecting = false;
            }
        }
    }

    @Override
    public void stop() {
        positionTracking.stopTracking();
        pidUdpReceiver.shutdown();

        positionTracking.stopDrive();
    }

    public double getMaxVelocity(double currentMax, double wheelVelocity) {

        if (Math.abs(currentMax) > Math.abs(wheelVelocity)) {
            return Math.abs(currentMax);
        } else {
            return Math.abs(wheelVelocity);
        }
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
}












