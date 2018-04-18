package org.firstinspires.ftc.teamcode.JTeleOp7261;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jankers on 9/22/2017.
 */
@TeleOp(name="Jankers_Mecanum_v1", group = "TeleOp")
@Disabled
public class Jankers_Mecanum_v1 extends OpMode {
    DcMotor rearLeftMotor;
    DcMotor frontLeftMotor;
    DcMotor rearRightMotor;
    DcMotor frontRightMotor;

    DcMotor Lift;
    DcMotor Extension;
    DcMotor leftIntake;
    DcMotor rightIntake;

    Servo leftPivot;
    Servo relicClamp;
    Servo relicArm;
    Servo leftJewelArm;
    Servo flopperClamp;
    Servo flopperDick;

    ModernRoboticsI2cGyro gyro;



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


        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);

        Lift.setDirection(DcMotor.Direction.REVERSE);
        Extension.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);

        leftPivot = hardwareMap.servo.get("lP");
        relicArm = hardwareMap.servo.get("rA");
        relicClamp = hardwareMap.servo.get("rC");
        leftJewelArm = hardwareMap.servo.get("lJA");
        flopperClamp = hardwareMap.servo.get("fC");
        flopperDick = hardwareMap.servo.get("fD");


        //gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        //gyro.calibrate();

        // gyro.resetZAxisIntegrator();


    }

    @Override
    public void loop() {
        leftPivot.setPosition(.68);
        leftJewelArm.setPosition(.70);


        telemetry.addData("fR Direction", frontRightMotor.getDirection());
        telemetry.update();

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


        if (Math.abs(gamepad1.left_stick_y) > .1) {
            forwBack = -gamepad1.left_stick_y;
        } else {
            forwBack = 0;
        }
        if (Math.abs(gamepad1.left_stick_x) > .1) {
            lefRight = gamepad1.left_stick_x;
        } else {
            lefRight = 0;
        }
        if (Math.abs(gamepad1.right_stick_x) > .1) {
            spin = gamepad1.right_stick_x;
        } else {
            spin = 0;
        }


        //angleCurrent = gyro.getHeading();

        //max_v = forwBack * Math.cos(angleCurrent) - lefRight * Math.sin(angleCurrent);
        //lefRight = -forwBack * Math.sin(angleCurrent) + lefRight * Math.cos(angleCurrent);

        raw_FL_speed = forwBack + lefRight + spin;
        this.getMaxVelocity(max_v, raw_FL_speed);
        raw_FR_speed = forwBack - lefRight - spin;
        this.getMaxVelocity(max_v, raw_FR_speed);
        raw_RL_speed = forwBack - lefRight + spin;
        this.getMaxVelocity(max_v, raw_RL_speed);
        raw_RR_speed = forwBack + lefRight - spin;
        this.getMaxVelocity(max_v, raw_RR_speed);

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

        frontLeftMotor.setPower(scaled_FL_speed);
        frontRightMotor.setPower(scaled_FR_speed);
        rearLeftMotor.setPower(scaled_RL_speed);
        rearRightMotor.setPower(scaled_RR_speed);

        // Send telemetry message to signify robot running;
        telemetry.addData("Forward Speed", "%.2f", forwBack);
        telemetry.addData("Strafe Speed", "%.2f", lefRight);
        telemetry.addData("Spin Speed", "%.2f", spin);
        telemetry.update();


        /**
         * Get the maximum velocity of the four wheel in order to scale everything else by it
         */


        Lift.setPower(-gamepad2.left_stick_y);
        Extension.setPower(-gamepad2.right_stick_y);


       // leftPivot.setPosition(.68);
       // leftJewelArm.setPosition(.70);

        if (gamepad2.dpad_up) {
            relicClamp.setPosition(1);
        }
        if (gamepad2.dpad_down) {
            relicClamp.setPosition(0);
        }
        if (gamepad2.left_trigger > .1) {
            flopperDick.setPosition(.75);
        }
        if (gamepad2.right_trigger > .1) {
            flopperDick.setPosition(.45);
        }
        if (gamepad2.left_bumper) {
            flopperClamp.setPosition(1);
        }
        if (gamepad2.right_bumper) {
            flopperClamp.setPosition(.50);
        }
        if (gamepad1.b) {
            rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        if (gamepad1.a) {
            rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        }


            if (gamepad1.right_bumper) {
                rightIntake.setPower(-1);
                leftIntake.setPower(1);
            }else if (gamepad1.left_bumper) {
             rightIntake.setPower(1);
             leftIntake.setPower(-1);
          } else {
             rightIntake.setPower(0);
             leftIntake.setPower(0);
          }

         if (gamepad2.dpad_left) {
            relicArm.setPosition(.58);
        }
        if (gamepad2.dpad_right) {
            relicArm.setPosition(1);
        }
        if (gamepad2.x) {
            relicArm.setPosition(.62);
        } if (gamepad2.y) {
            relicArm.setPosition(0);
        }
        if (gamepad1.dpad_down) {
            rearLeftMotor.setPower(-.33);
            frontLeftMotor.setPower(-.33);
            rearRightMotor.setPower(-.33);
            frontRightMotor.setPower(-.33);
        }
        if (gamepad1.dpad_up) {
            rearLeftMotor.setPower(.33);
            frontLeftMotor.setPower(.33);
            rearRightMotor.setPower(.33);
            frontRightMotor.setPower(.33);
        }
        if (gamepad1.dpad_left) {
            rearLeftMotor.setPower(-.66);
            frontLeftMotor.setPower(-.66);
            rearRightMotor.setPower(.66);
            frontRightMotor.setPower(.66);
        }
        if (gamepad1.dpad_right) {
            rearLeftMotor.setPower(.66);
            frontLeftMotor.setPower(.66);
            rearRightMotor.setPower(-.66);
            frontRightMotor.setPower(-.66);
        }
    }

    public double getMaxVelocity(double currentMax, double wheelVelocity) {

        if (Math.abs(currentMax) > Math.abs(wheelVelocity)) {
            return Math.abs(currentMax);
        } else {
            return Math.abs(wheelVelocity);
        }
    }
}












