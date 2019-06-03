package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="ReverseWire", group="TeamCode")
public class ReverseWire extends LinearOpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    public void initialize() {
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int inchesToTicks(double inches) {
        double rotations = inches / (4 * 3.14159);
        return (int) rotations * 1120;
    }

    public void setModeAll(DcMotor.RunMode mode){
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void setTargetAll (int target){
        frontLeft.setTargetPosition(target);
        frontRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setTargetPosition(target);
    }

    public void setPowerAll(double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void driveStraight(double inches) {
        int ticks = inchesToTicks(inches);
        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);
        setTargetAll(inchesToTicks(inches));
        setPowerAll(0.75);

        while((frontRight.isBusy()) && opModeIsActive()) {
            telemetry.addData("frontRight target: ", frontRight.getTargetPosition());
            telemetry.addData("frontRight encoder (conversion kit): ", frontRight.getCurrentPosition());
            telemetry.addData("frontLeft encoder (AM cable): ", frontLeft.getCurrentPosition());
            telemetry.addData("backLeft encoder(mod AM cable): ", backLeft.getCurrentPosition());
            telemetry.update();
        }

        setPowerAll(0);


    }

    public void runOpMode() {
        initialize();
        waitForStart();
        driveStraight(48.0);
        driveStraight(-48.0);
    }
}
