package org.firstinspires.ftc.teamcode.multiGlyph.odemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.multiGlyph.IMU;
import org.firstinspires.ftc.teamcode.multiGlyph.PID;

import java.util.Arrays;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class PositionTracking {

    final private int xOffset = -7940;
    final private int yOffset = -2176;

    public final int xPerInch = -233;
    public final int yPerInch = -233;

    final private int xTolerance = 50;
    final private int yTolerance = 50;
    final private int turnTolerance = 3;

    private int prevX = 0;
    private int prevY = 0;
    private double prevHeading = 0;

    public int xPositionAbs = 0;
    public int yPositionAbs = 0;

    private boolean positionThread = false;
    public boolean driveThread = false;

    public PID xPID = new PID();
    private PID yPID = new PID();
    public PID turnPID = new PID();

    private OpMode opMode;
    private IMU imu;
    private DcMotor xWheel, yWheel, frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;

    int xTarget;
    int yTarget;
    int headingTarget;
    int heading;

    private double P_C_AD = .009;
    private double I_C_AD = 0;
    private double D_C_AD = .009;

    private static final double P_C_TURN = 0.011;
    private static final double I_C_TURN = 0.0000125;
    private static final double D_C_TURN = 0;

    public double yForwrd;
    public double xStrafe;

    public double angleError;

    public boolean stop = false;

    public PositionTracking (OpMode opMode, IMU imu,
                             DcMotor xWheel, DcMotor yWheel,
                             DcMotor frontLeftMotor, DcMotor rearLeftMotor,
                             DcMotor frontRightMotor, DcMotor rearRightMotor) {
        this.opMode = opMode;
        this.imu = imu;
        this.xWheel = xWheel;
        this.yWheel = yWheel;
        this.frontLeftMotor = frontLeftMotor;
        this.rearLeftMotor = rearLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.rearRightMotor = rearRightMotor;

        turnPID.setVariables(.08,0,1);
    }

    public int xPosition() {
        return xPositionAbs;
    }

    public int yPosition() {
        return yPositionAbs;
    }

    public void zeroEncoders () {
        xPositionAbs = 0;
        yPositionAbs = 0;
        xWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int xInchCounts (double inches) {
        return (int)(inches * xPerInch);
    }

    public void fieldCentric(double forwards, double horizontal, double turning, int heading){
        //uses the orientation of the robot to offset the input powers.
        double forwrd = forwards * -1;
        double strafe = horizontal;

        double gyro_radians = heading * Math.PI/180;
        double temp = forwrd * cos(gyro_radians) + strafe * sin(gyro_radians);
        strafe = -forwrd * sin(gyro_radians) + strafe * cos(gyro_radians);
        forwrd = temp;

        robotCentric(-forwrd,strafe,turning);
    }

    private void robotCentric(double forwards, double horizontal, double turning) {
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

    public void absolutePositioning (int xInput, int yInput, double heading) {
        //double angleDelta = heading - prevHeading;
        angleError = (heading - prevHeading);
        angleError -= (360*Math.floor(0.5+((angleError)/360.0)));

        prevHeading = heading;

        int xDelta = xInput - prevX;
        prevX = xInput;

        int yDelta = yInput - prevY;
        prevY = yInput;

        double strafe = xDelta - (xOffset*angleError/360);
        double forwrd = yDelta - (yOffset*angleError/360);

        xStrafe += strafe;
        yForwrd += forwrd;

        double gyro_radians = heading * Math.PI/180;
        double temp = forwrd * cos(gyro_radians) + strafe * sin(gyro_radians);
        strafe = -forwrd * sin(gyro_radians) + strafe * cos(gyro_radians);
        forwrd = temp;

        yPositionAbs += (int)forwrd;
        xPositionAbs += (int)strafe;
    }

    public void stopTracking () {
        positionThread = false;
    }

    public void stopDrive () {
        driveThread = false;
        stop = true;
    }

    public void drive (int xTarget, int yTarget, int headingTarget, int heading) {
        double xPower = -0.05 * xPid(xTarget,xPositionAbs);
        double yPower = 0.03 * yPid(yTarget,yPositionAbs);
        double pidOffset = turnPID.run(headingTarget,heading);
        double turnPower = -pidOffset * .2;

        fieldCentric(yPower,xPower,turnPower,heading);
    }

    public boolean autonomousDrive (int xTarget, int yTarget, int headingTarget, int heading) {
        if(((Math.abs(yTarget-yPositionAbs) > yTolerance)
                )) {
            drive(-xTarget,yTarget,headingTarget,heading);
            return false;

        } else {
            return true;
        }

    }

    public void startAbsoluteEncoderThread() {
        positionThread = true;
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                Thread.currentThread().setPriority(9);
                //!opMode.isStopRequested()&&opMode.opModeIsActive()&&
                while(positionThread) {
                    int xPosition = xWheel.getCurrentPosition();
                    int yPosition = yWheel.getCurrentPosition();
                    double heading = imu.getHeading();

                    absolutePositioning(xPosition,yPosition,(360-heading));
                }
            }
        }).start();
    }
//
    //final int xTarget, final int yTarget, final int headingTarget, final int heading
    public void driveThreadLoop() {
        driveThread = true;
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                /*!opMode.isStopRequested()&&opMode.opModeIsActive()&&*/
                while(driveThread) {
                    double xPower = -0.05 * xPid(xTarget,xPositionAbs);
                    double yPower = 0.03 * yPid(yTarget,yPositionAbs);
                    //0;//yPID.runDistance(yTarget,yPositionAbs);
                    double turnPower = turnPid(headingTarget,heading);

                    robotCentric(yPower,xPower,turnPower);
                }
                robotCentric(0,0,0);

            }
        }).start();
    }

    public void driveToCordinates(final int xTarget, final int yTarget, final int headingTarget, final int heading) {
        while((xTarget-xPositionAbs) > xTolerance
                &&Math.abs(yTarget-yPositionAbs) > yTolerance
                &&Math.abs(headingTarget-heading) > turnTolerance) {

            double xPower = -0.05 * xPid(xTarget,xPositionAbs);
            double yPower = 0.03 * yPid(yTarget,yPositionAbs);
            double turnPower = turnPid(headingTarget,heading);

            fieldCentric(yPower,xPower,turnPower,heading);
        }
        robotCentric(0,0,0);
    }

    //final int xTarget, final int yTarget, final int headingTarget, final int heading
    public void driveToCoordinatesThread(final int xTarget, final int yTarget, final int headingTarget, final int heading) {
        driveThread = true;
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                while(driveThread) {
                    double xPower = -0.05 * xPid(xTarget,xPositionAbs);
                    double yPower = 0.04 * yPid(yTarget,yPositionAbs);
                    double pidOffset = turnPID.run(headingTarget,heading);
                    double turnPower = -pidOffset * .2;

                    robotCentric(0,0,turnPower);

                    //check if in tolerance and end thread if so end
                    if (Math.abs(xTarget-xPositionAbs) < xTolerance
                        &&Math.abs(yTarget-yPositionAbs) < yTolerance
                        &&Math.abs(headingTarget-heading) < turnTolerance) {

                        driveThread = false;
                    }
                }
                robotCentric(0,0,0);
            }
        }).start();
    }

    private double previousErrorY, iErrorY;
    private double yPid (double fixedHeading, double angleCurrent) {
        double pError = fixedHeading - angleCurrent;        //Finds the current difference between the target and current
        iErrorY += pError;
        double dError = pError - previousErrorY;

        previousErrorY = pError;

        return (pError * P_C_AD) + (iErrorY *I_C_AD) + (dError* D_C_AD);
    }

    private double previousErrorX, iErrorX;
    private double xPid (double fixedHeading, double angleCurrent) {
        double pError = fixedHeading - angleCurrent;        //Finds the current difference between the target and current
        iErrorX += pError;
        double dError = pError - previousErrorX;

        previousErrorX = pError;

        return (pError * P_C_AD) + (iErrorX *I_C_AD) + (dError* D_C_AD);
    }

    private double previousErrorTurn, iErrorTurn;
    private double turnPid (double fixedHeading, double angleCurrent) {
        double pError = fixedHeading - angleCurrent;        //Finds the current difference between the target and current
        pError-= (360*Math.floor(0.5+((pError)/360.0)));

        iErrorTurn += pError;
        double dError = pError - previousErrorTurn;

        previousErrorTurn = pError;

        return (pError * P_C_TURN) + (iErrorTurn *I_C_TURN) + (dError* D_C_TURN);
    }

}
