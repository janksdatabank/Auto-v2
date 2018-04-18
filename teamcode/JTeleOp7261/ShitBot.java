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
@TeleOp(name="ShitBot", group = "TeleOp")
@Disabled
public class ShitBot extends LinearOpMode {
    DcMotor LeftMotor;
    DcMotor Ruler;
    DcMotor RightMotor;
    DcMotor Arm;

    @Override
    public void runOpMode() throws InterruptedException {


        LeftMotor = hardwareMap.dcMotor.get("L");
        Ruler = hardwareMap.dcMotor.get("Ru");
        RightMotor = hardwareMap.dcMotor.get("Ri");
        Arm = hardwareMap.dcMotor.get("A");

        waitForStart();

        while (opModeIsActive()) {

            LeftMotor.setPower(-gamepad1.left_stick_y);
            if (gamepad1.dpad_left) {
                Arm.setPower(1);
            } else {
                Arm.setPower(0);
            }
            if (gamepad1.dpad_right) {
                Arm.setPower(-1);
            } else {
                Arm.setPower(0);
            }
            if (gamepad1.dpad_up) {
                Ruler.setPower(1);
            } else {
                Ruler.setPower(0);
            }
            if (gamepad1.dpad_down) {
                Ruler.setPower(-1);
            } else {
                Ruler.setPower(0);
            }
            RightMotor.setPower(gamepad1.right_stick_y);
        }
    }

}