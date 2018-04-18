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
@TeleOp(name="Test PID Turn", group = "Test")
public class Test_PIDTurn extends OpMode {
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

    boolean stop = false;

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
            gyroTurn(.1,(int)angle,3);
        }

    }

    @Override
    public void stop() {
        positionTracking.stopTracking();
        pidUdpReceiver.shutdown();

        positionTracking.stopDrive();

        stop = true;
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
}












