package org.firstinspires.ftc.teamcode.JTeleOp7261;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

/**
 * Created by Jankers on 9/22/2017.
 */
@TeleOp(name="Full TeleOp", group = "TeleOp")
public class Jankers_Mecanum_v2 extends OpMode {
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


    public ElapsedTime ejectTime = new ElapsedTime();
    boolean ejecting = false;
    boolean bCurrState_y = false;
    boolean bPrevState_y = false;
    public ElapsedTime jewTime = new ElapsedTime();
    boolean jewing = false;
    boolean aCurrState_y = false;
    boolean aPrevState_y = false;

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
    static final double JEWEL_ARM_RAISED                            = .73;
    static final double JEWEL_ARM_LOWERED                           = .18;
    static final double JEWEL_PIVOT_RAISED                          = .455;
    static final double JEWEL_PIVOT_LOWERED                         = .62;
    static final double JEWEL_PIVOT_LEFT                            = .8;               //Position for the jewel arm to hit the left jewel
    static final double JEWEL_PIVOT_RIGHT                           = .2;
    static final double ALIGN_ARM_RAISED                            = 0;
    static final double ALIGN_ARM_DETECT                            = .65;
    static final double ALIGN_ARM_LOWERED                           = .62;
    static final double ALIGN_ARM_IN                                = .9;
    static final double RELIC_ARM_LOWERED                           = .50;
    static final double RELIC_ARM_RAISED                            = .05;
    static final double RELIC_ARM_IN                                = 1;
    static final double RELIC_CLAMP_LOCK                            = 0;
    static final double RELIC_CLAMP_UNLOCK                          = 1;



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


        // *********************************************
        //          Setting Initial Servo Positions
        // *********************************************
        // Now on controller 1 to prevent time loss on intialize.




    }

    @Override
    public void loop() {

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

        if(!reversed){
            if (Math.abs(gamepad1.left_stick_y) > .1) {
                if(gamepad1.left_stick_y > 0) {
                    forwBack = -Math.pow(gamepad1.left_stick_y, 2);
                } else{
                    forwBack = Math.pow(gamepad1.left_stick_y, 2);
                }

            } else {
                forwBack = 0;
            }
            if (Math.abs(gamepad1.left_stick_x) > .1) {
                if(Math.abs(gamepad1.left_stick_x) > .7){
                    if(gamepad1.left_stick_x > 0) {
                        lefRight = Math.pow(gamepad1.left_stick_x, 2);
                    } else{
                        lefRight = -Math.pow(gamepad1.left_stick_x, 2);
                    }
                } else {
                    if(gamepad1.left_stick_x > 0) {
                        lefRight = PREC_STRAFE_SPEED_2;
                    } else{
                        lefRight = -PREC_STRAFE_SPEED_2;
                    }
                }
            } else {
                lefRight = 0;
            }
            if (Math.abs(gamepad1.right_stick_x) > .1) {
                if (!precTurn) {
                    if (gamepad1.right_stick_x > 0) {
                        spin = Math.pow(gamepad1.right_stick_x, 2);
                    } else {
                        spin = -Math.pow(gamepad1.right_stick_x, 2);
                    }
                } else{
                    if (gamepad1.right_stick_x > 0) {
                        spin = PREC_TURN_SPEED;
                    } else {
                        spin = -PREC_TURN_SPEED;
                    }
                }
            } else {
                spin = 0;
            }
        } else{
            if (Math.abs(gamepad1.left_stick_y) > .1) {
                if(gamepad1.left_stick_y > 0) {
                    forwBack = Math.pow(gamepad1.left_stick_y, 2);
                } else{
                    forwBack = -Math.pow(gamepad1.left_stick_y, 2);
                }
            } else {
                forwBack = 0;
            }
            if (Math.abs(gamepad1.left_stick_x) > .1) {
                if(Math.abs(gamepad1.left_stick_x) > .7){
                    if(gamepad1.left_stick_x > 0) {
                        lefRight = -Math.pow(gamepad1.left_stick_x, 2);
                    } else{
                        lefRight = Math.pow(gamepad1.left_stick_x, 2);
                    }
                } else {
                    if(gamepad1.left_stick_x > 0) {
                        lefRight = -PREC_STRAFE_SPEED_2;
                    } else{
                        lefRight = PREC_STRAFE_SPEED_2;
                    }
                }
            } else {
                lefRight = 0;
            }

            if (Math.abs(gamepad1.right_stick_x) > .1) {
                if (!precTurn) {
                    if (gamepad1.right_stick_x > 0) {
                        spin = Math.pow(gamepad1.right_stick_x, 2);
                    } else {
                        spin = -Math.pow(gamepad1.right_stick_x, 2);
                    }
                } else{
                    if (gamepad1.right_stick_x > 0) {
                        spin = PREC_TURN_SPEED;
                    } else {
                        spin = -PREC_TURN_SPEED;
                    }
                }
            } else {
                spin = 0;
            }
        }


        raw_FL_speed = forwBack + lefRight + spin;
        max_v = this.getMaxVelocity(max_v, raw_FL_speed);
        raw_FR_speed = forwBack - lefRight - spin;
        max_v = this.getMaxVelocity(max_v, raw_FR_speed);
        raw_RL_speed = forwBack - lefRight + spin;
        max_v = this.getMaxVelocity(max_v, raw_RL_speed);
        raw_RR_speed = forwBack + lefRight - spin;
        max_v = this.getMaxVelocity(max_v, raw_RR_speed);

        //If the raw motor outputs exceed the [-1, 1], the outputs are scaled
        if (max_v > 1.0) {
            scaled_FL_speed = raw_FL_speed / max_v;
            scaled_FR_speed = raw_FR_speed / max_v;
            scaled_RL_speed = raw_RL_speed / max_v;
            scaled_RR_speed = raw_RR_speed / max_v;
        } else {
            scaled_FL_speed = raw_FL_speed;
            scaled_FR_speed = raw_FR_speed;
            scaled_RL_speed = raw_RL_speed;
            scaled_RR_speed = raw_RR_speed;
        }



        // Send telemetry message to signify robot running;
        //telemetry.addData("Forward Speed", "%.2f", forwBack);
        //telemetry.addData("Strafe Speed", "%.2f", lefRight);
        //telemetry.addData("Spin Speed", "%.2f", spin);
        telemetry.addData("FR", frontRightMotor.getPower());
        telemetry.addData("RL", rearLeftMotor.getPower());
        telemetry.addData("FL", frontLeftMotor.getPower());
        telemetry.addData("RR", rearRightMotor.getPower());
        telemetry.update();

        if (gamepad1.dpad_down)
        {
            frontLeftMotor.setPower(-PREC_TURN_SPEED);
            frontRightMotor.setPower(-PREC_TURN_SPEED);
            rearLeftMotor.setPower(-PREC_TURN_SPEED);
            rearRightMotor.setPower(-PREC_TURN_SPEED);

        } else if (gamepad1.dpad_up) {
            frontLeftMotor.setPower(PREC_TURN_SPEED);
            frontRightMotor.setPower(PREC_TURN_SPEED);
            rearLeftMotor.setPower(PREC_TURN_SPEED);
            rearRightMotor.setPower(PREC_TURN_SPEED);
        } else if (gamepad1.dpad_left) {

            frontLeftMotor.setPower(-PREC_TURN_SPEED);
            frontRightMotor.setPower(PREC_TURN_SPEED);
            rearLeftMotor.setPower(-PREC_TURN_SPEED);
            rearRightMotor.setPower(PREC_TURN_SPEED);
        } else if (gamepad1.dpad_right) {
            frontLeftMotor.setPower(PREC_TURN_SPEED);
            frontRightMotor.setPower(-PREC_TURN_SPEED);
            rearLeftMotor.setPower(PREC_TURN_SPEED);
            rearRightMotor.setPower(-PREC_TURN_SPEED);
        } else {
            frontLeftMotor.setPower(scaled_FL_speed);
            frontRightMotor.setPower(scaled_FR_speed);
            rearLeftMotor.setPower(scaled_RL_speed);
            rearRightMotor.setPower(scaled_RR_speed);
            /*double leftY = Math.signum(-gamepad1.left_stick_y) * Math.pow(-gamepad1.left_stick_y, 2);
            double leftX = Math.signum(gamepad1.left_stick_x) * Math.pow(gamepad1.left_stick_x, 2);
            double rightX = Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2);

            robotCentric(leftY,leftX,rightX);*/
        }


        bCurrState_x = gamepad1.x;

        // check for button-press state transitions.
        if ((bCurrState_x == true) && (bCurrState_x != bPrevState_x))  {
            reversed = !reversed;
        }

        // update previous state variable.
        bPrevState_x = bCurrState_x;

        bCurrState_b = gamepad1.b;

        if (rightIntake.getPower() > 0 || rightIntake.getPower() < 0 ) {
            rollerWheels.setPower(.8);
        } else {
            rollerWheels.setPower(0);
        }

        // check for button-press state transitions.
        if ((bCurrState_b == true) && (bCurrState_b != bPrevState_b))  {
            precTurn = !precTurn;
        }

        // update previous state variable.
        bPrevState_b = bCurrState_b;


        if (gamepad1.right_trigger > .1) {
            relicArm.setPosition(RELIC_ARM_LOWERED);
        } if (gamepad1.left_trigger > .1) {
            relicArm.setPosition(RELIC_ARM_RAISED);
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
        aCurrState_y = gamepad2.y;

        // check for button-press state transitions.
        if ((aCurrState_y == true) && (aCurrState_y != aPrevState_y))  {
            jewing = true;
            jewTime.reset();
        }

        // update previous state variable.
        aPrevState_y = aCurrState_y;

        if (jewing){
            if (jewTime.time() < .25){
                relicClamp.setPosition(RELIC_CLAMP_UNLOCK);
                relicArm.setPosition(RELIC_ARM_RAISED);
                glyphDump.setPosition(GLYPH_DUMP_RAISED);
                glyphClamp.setPosition(GLYPH_CLAMP_UNLOCK);
                alignArm.setPosition(ALIGN_ARM_LOWERED);
                jewelPivot.setPosition(JEWEL_PIVOT_RAISED-.2);
                jewelArm.setPosition(JEWEL_ARM_RAISED);
            } else{
                jewelPivot.setPosition(JEWEL_PIVOT_RAISED);
                jewing = false;
            }

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



        bCurrState_y = gamepad1.y;

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
                glyphClamp.setPosition(GLYPH_CLAMP_UNLOCK);
            }
            else if (ejectTime.time() < .5){
                alignArm.setPosition(1);
                glyphDump.setPosition(.8);
                relicArm.setPosition(RELIC_ARM_RAISED);
                rollerWheels.setPower(.8);
            } else if (ejectTime.time() < .525){
                glyphDump.setPosition(GLYPH_DUMP_RAISED);
                relicArm.setPosition(RELIC_ARM_RAISED);
                rollerWheels.setPower(.8);
            } else{
                alignArm.setPosition(ALIGN_ARM_LOWERED);
                ejecting = false;
            }

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

    public double getMaxVelocity(double currentMax, double wheelVelocity) {

        if (Math.abs(currentMax) > Math.abs(wheelVelocity)) {
            return Math.abs(currentMax);
        } else {
            return Math.abs(wheelVelocity);
        }
    }
}
