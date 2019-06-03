package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "MyAutoOpMode", group = "Teamcode")

public class auto extends LinearOpMode {
    DcMotor backLeft, backRight, frontLeft, frontRight;
    final double TICKS_PER_INCH = 1120/(4*3.14);
    public void runOpMode() {
        initialize();
        waitForStart();
        driveForDistance(100.0, 0.75);
        sleep(500);
        driveForDistance(-100.0, 0.75);
    }


    public void initialize() {

        backLeft = hardwareMap.dcMotor.get("backLeft");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight = hardwareMap.dcMotor.get("backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight = hardwareMap.dcMotor.get("frontRight");

    }

    public void setModeAll(DcMotor.RunMode mode) {
        backLeft.setMode(mode);
        backRight.setMode(mode);
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
    }

    public void setPowerAll(double power){
        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
    }

    public void setTargetAll(int target){
        backLeft.setTargetPosition(target);
        backRight.setTargetPosition(target);
        frontLeft.setTargetPosition(target);
        frontRight.setTargetPosition(target);
    }

    public void driveForDistance(double inches, double power){
        int ticks = (int) (inches * TICKS_PER_INCH);
        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);
        setTargetAll(ticks);
        setPowerAll(power);
        while (backLeft.isBusy()&& opModeIsActive()){
            telemetry.addData("backLeft target: ", backLeft.getTargetPosition());
            telemetry.addData("backLeft (mod wire): ", backLeft.getCurrentPosition());
            telemetry.addData("frontRight (AM wire): ",frontRight.getCurrentPosition());
            telemetry.update();
        }
        setPowerAll(0);
    }

}