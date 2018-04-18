package org.firstinspires.ftc.teamcode.JTeleOp7261;

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
@TeleOp(name="Jankers TeleOp_v2", group = "TeleOp")
@Disabled
public class Jankers_Tele_v2 extends LinearOpMode {
    DcMotor rearLeftMotor;
    DcMotor frontLeftMotor;
    DcMotor rearRightMotor;
    DcMotor frontRightMotor;

    DcMotor Lift;
    DcMotor Extension;
    DcMotor Intake;

    Servo leftPivot;
    Servo rightPivot;
    Servo relicClamp;
    Servo relicArm;
    Servo leftJewelArm;
    Servo rightJewelArm;
    Servo bottomLeftClaw;
    Servo bottomRightClaw;
    Servo topRightClaw;
    Servo topLeftClaw;




    @Override
    public void runOpMode() throws InterruptedException {


        rearLeftMotor = hardwareMap.dcMotor.get("rearLeftMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        rearRightMotor = hardwareMap.dcMotor.get("rearRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");

        Lift = hardwareMap.dcMotor.get("Lift");
        Extension = hardwareMap.dcMotor.get("Extension");
      //  Intake = hardwareMap.dcMotor.get("Intake");

        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);

        Lift.setDirection(DcMotor.Direction.REVERSE);
        Extension.setDirection(DcMotorSimple.Direction.REVERSE);
       // Intake.setDirection(DcMotor.Direction.REVERSE);

        leftPivot = hardwareMap.servo.get("leftPivot");
        rightPivot = hardwareMap.servo.get("rightPivot");
        relicArm = hardwareMap.servo.get("relicArm");
        relicClamp = hardwareMap.servo.get("relicClamp");
        leftJewelArm = hardwareMap.servo.get("leftJewelArm");
        rightJewelArm = hardwareMap.servo.get("rightJewelArm");
       // leftIntake = hardwareMap.servo.get("leftIntake");
       // rightIntake = hardwareMap.servo.get("rightIntake");
        bottomLeftClaw = hardwareMap.servo.get("bottomLeftClaw");
        bottomRightClaw = hardwareMap.servo.get("bottomRightClaw");
        topLeftClaw = hardwareMap.servo.get("topLeftClaw");
        topRightClaw = hardwareMap.servo.get("topRightClaw");


        rightPivot.setPosition(.39);
        leftPivot.setPosition(.68);
        leftJewelArm.setPosition(.70);
        rightJewelArm.setPosition(.09);
        relicClamp.setPosition(1);
        relicArm.setPosition(0);
        topLeftClaw.setPosition(.30);
        topRightClaw.setPosition(.50);
        bottomLeftClaw.setPosition(.2);
        bottomRightClaw.setPosition(.8);
      //  leftIntake.setPosition(.59);
      //  rightIntake.setPosition(.23);

        waitForStart();

        while (opModeIsActive())
        {

            telemetry.addData("fR Direction", frontRightMotor.getDirection());
            telemetry.update();

            rearLeftMotor.setPower(-gamepad1.left_stick_y);
            frontLeftMotor.setPower(-gamepad1.left_stick_y);
            rearRightMotor.setPower(-gamepad1.right_stick_y);
            frontRightMotor.setPower(-gamepad1.right_stick_y);

            Lift.setPower(-gamepad2.left_stick_y);
            Extension.setPower(-gamepad2.right_stick_y);

            rightPivot.setPosition(.39);
            leftPivot.setPosition(.68);
            leftJewelArm.setPosition(.70);
            rightJewelArm.setPosition(.09);

            if (gamepad2.dpad_up) {
                relicClamp.setPosition(1);
            }  if (gamepad2.dpad_down){
            relicClamp.setPosition(0);
        }
            if (gamepad1.b) {
                rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
                frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
            }
            if (gamepad1.a)  {
                rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
                frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            }


        //    if (gamepad1.right_bumper) {
        //        Intake.setPower(-1);
       //     }if (gamepad1.left_bumper) {
       //     Intake.setPower(1);
      //  } else {
       //     Intake.setPower(0);
      //  }

          //  if (gamepad1.right_trigger >.2) {
          //      leftIntake.setPosition(.59);
          ///      rightIntake.setPosition(.23);
          //  }
           // if (gamepad1.left_trigger >.2) {
          //      leftIntake.setPosition(0);
          //      rightIntake.setPosition(.94);
          //  }
            if (gamepad2.dpad_left) {
                relicArm.setPosition(.65);
            } if (gamepad2.dpad_right) {
            relicArm.setPosition(1);
        } if ( gamepad2.left_bumper) {
            relicArm.setPosition(.68);
        }
        if (gamepad2.a) {
            bottomLeftClaw.setPosition(.8);
            bottomRightClaw.setPosition(.2);
        } if(gamepad2.b) {
            bottomLeftClaw.setPosition(.2);
            bottomRightClaw.setPosition(.8);
        } if (gamepad2.x) {
            topLeftClaw.setPosition(.85);
            topRightClaw.setPosition(.05);
        } if(gamepad2.y) {
            topLeftClaw.setPosition(.30);
            topRightClaw.setPosition(.50);
        }
            if(gamepad1.dpad_down){
                rearLeftMotor.setPower(-.33);
                frontLeftMotor.setPower(-.33);
                rearRightMotor.setPower(-.33);
                frontRightMotor.setPower(-.33);
            }
            if(gamepad1.dpad_up){
                rearLeftMotor.setPower(.33);
                frontLeftMotor.setPower(.33);
                rearRightMotor.setPower(.33);
                frontRightMotor.setPower(.33);
            }
            if(gamepad1.dpad_left){
                rearLeftMotor.setPower(-.66);
                frontLeftMotor.setPower(-.66);
                rearRightMotor.setPower(.66);
                frontRightMotor.setPower(.66);
            }
            if(gamepad1.dpad_right){
                rearLeftMotor.setPower(.66);
                frontLeftMotor.setPower(.66);
                rearRightMotor.setPower(-.66);
                frontRightMotor.setPower(-.66);
            }
        }
    }
}










