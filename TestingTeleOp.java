package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="TestingTeleOp", group="TeamCode")

public class TestingTeleOp extends OpMode {
    DcMotor backLeft, backRight, frontLeft, frontRight;

    boolean canWrite = false;

    public void init() {

        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop() {
        canWrite = !canWrite;
        setDriveMotors();

        if (!canWrite) {
            telemetry.addData("frontLeft: ", frontLeft.getCurrentPosition());
        }
        telemetry.update();
    }

    private void setDriveMotors() {
        if (gamepad1.right_stick_y - gamepad1.right_stick_x > 1) {
            frontLeft.setPower(1 - gamepad1.left_stick_x / 2);
            backRight.setPower(1 + gamepad1.left_stick_x / 2);
        } else {
            frontLeft.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) - gamepad1.left_stick_x / 2);
            backRight.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) + gamepad1.left_stick_x / 2);
        }
        if (gamepad1.right_stick_y + gamepad1.right_stick_x > 1) {
            frontRight.setPower(1 + gamepad1.left_stick_x / 2);
            backLeft.setPower(1 - gamepad1.left_stick_x / 2);
        } else {
            frontRight.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) + gamepad1.left_stick_x / 2);
            backLeft.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) - gamepad1.left_stick_x / 2);
        }

        telemetry.addData("frontRight", frontRight.getPower());
        telemetry.addData("frontLeft", frontLeft.getPower());
        telemetry.addData("backRight", backRight.getPower());
        telemetry.addData("backLeft", backLeft.getPower());
        telemetry.update();
    }
}