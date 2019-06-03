package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import static java.lang.Math.abs;

@TeleOp(name="VectorTeleOp", group="TeamCode")

public class VectorTeleOp extends OpMode {
    Servo markerServo, dumperServo, cameraServo;
    CRServo collectorServo, collectorExtenderServo;
    DcMotor backLeft,backRight, frontLeft, frontRight, liftMotor, collectorMove, collectorExtenderDc,mineralLift;
    TouchSensor liftUpSensor, liftDownSensor;

    boolean canWrite = false;
    boolean gamepad2RightBumperPressed = false;
    boolean gamepad2LeftBumperPressed = false;
    int dumperServoPosition = 0;

    int currentCEVal = 0;

    final double TOLERANCE = 0.5;
    final double collectorMin = -10;
    final double collectorMax = 370;

    /*
    final double MAX_CM = 935;
    final double MIN_CM = 0;
    final double MAX_CE = 3525;
    final double MIN_CE = 0;
    */
    final double up = 0.85;
    final double mid = 0.3;
    final double low = 0;

    public void init() {
        markerServo = hardwareMap.servo.get("markerServo");
        dumperServo = hardwareMap.servo.get("dumperServo");
        cameraServo = hardwareMap.servo.get("cameraServo");

        collectorServo = hardwareMap.crservo.get("collectorServo");

        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        collectorMove = hardwareMap.dcMotor.get("collectorMove");
        collectorExtenderDc = hardwareMap.dcMotor.get("collectorExtenderDc");
        collectorExtenderServo = hardwareMap.crservo.get("collectorExtenderServo");
        mineralLift = hardwareMap.dcMotor.get("mineralLift");

        liftUpSensor = hardwareMap.touchSensor.get("liftUpSensor");
        liftDownSensor = hardwareMap.touchSensor.get("liftDownSensor");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorExtenderDc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMove.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        collectorServo.setDirection(CRServo.Direction.REVERSE);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        collectorMove.setDirection(DcMotor.Direction.FORWARD);
        collectorExtenderDc.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorMove.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorExtenderDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorExtenderDc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorMove.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        //set write mode and read mode
        canWrite = !canWrite;

        /*if (gamepad2.a) {
            //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            collectorExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            collectorMove.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else {
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            collectorExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            collectorMove.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }*/

        setDriveMotors();
        setLiftMotor();
        setCollectorMove();
        setCollectorExtender();
        setDumperServo();
        setCollectorServo();
        setMarkerServo();
        setMineralLift();

        //TODO: cut
        //get encoder value
        if (!canWrite) {
            //telemetry.addData("lift up sensor: ", liftUpSensor.getValue());
            //telemetry.addData("lift motor: ", liftMotor.getCurrentPosition());
            currentCEVal = collectorExtenderDc.getCurrentPosition();
        }
        telemetry.update();
    }

    private void setDriveMotors(){
        if(gamepad1.right_stick_y - gamepad1.right_stick_x > 1){
            frontLeft.setPower(1 - gamepad1.left_stick_x/2);
            backRight.setPower(1 + gamepad1.left_stick_x/2);
        }
        else{
            frontLeft.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) - gamepad1.left_stick_x/2);
            backRight.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) + gamepad1.left_stick_x/2);
        }
        if(gamepad1.right_stick_y + gamepad1.right_stick_x > 1){
            frontRight.setPower(1 + gamepad1.left_stick_x/2);
            backLeft.setPower(1 - gamepad1.left_stick_x/2);
        }
        else {
            frontRight.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) + gamepad1.left_stick_x / 2);
            backLeft.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) - gamepad1.left_stick_x / 2);
        }

        telemetry.addData("frontRight", frontRight.getPower());
        telemetry.addData("frontLeft", frontLeft.getPower());
        telemetry.addData("backRight", backRight.getPower());
        telemetry.addData("backLeft", backLeft.getPower());
        telemetry.update();
    }

    private void setLiftMotor() {
        if (gamepad2.a && liftDownSensor.getValue() == 0)
            liftMotor.setPower(-1);
        else if (gamepad2.y && liftUpSensor.getValue() == 0)
            liftMotor.setPower(+1);
        else
            liftMotor.setPower(0);

        telemetry.addData("lift motor", liftMotor.getCurrentPosition());
        telemetry.update();
    }

    private void setCollectorMove(){
        if (gamepad2.dpad_up /*&& (collectorMovePosition < MAX_CM)*/){
            collectorMove.setPower(-0.3);
        }
        else if (gamepad2.dpad_down /*&& (collectorMovePosition > MIN_CM)*/){
            collectorMove.setPower(0.2);
        }
        else{
            collectorMove.setPower(0);
        }
    }

    private void setCollectorExtender(){
        if(gamepad2.right_stick_y > TOLERANCE) {
            if(currentCEVal <= collectorMax) {
                collectorExtenderDc.setPower(gamepad2.right_stick_y / 5);
                collectorExtenderServo.setPower(gamepad2.right_stick_y);
            } else{
                collectorExtenderDc.setPower(0);
                collectorExtenderServo.setPower(gamepad2.right_stick_y);
            }
        } else if(gamepad2.right_stick_y < -TOLERANCE){
            if(collectorMin <= currentCEVal) {
                collectorExtenderDc.setPower(gamepad2.right_stick_y / 5);
                collectorExtenderServo.setPower(gamepad2.right_stick_y);
            } else{
                collectorExtenderDc.setPower(0);
                collectorExtenderServo.setPower(gamepad2.right_stick_y);
            }
        } else{
            collectorExtenderDc.setPower(0);
            collectorExtenderServo.setPower(0);
        }
    }

    private void setDumperServo(){
        if (gamepad2.right_bumper && !gamepad2RightBumperPressed){
            if(dumperServoPosition == 0){
                dumperServo.setPosition(mid);
                dumperServoPosition = 1;
            }
            else if(dumperServoPosition == 1){
                dumperServo.setPosition(up);
                dumperServoPosition = 2;
            }

            gamepad2RightBumperPressed = true;
        }
        else if(!gamepad2.right_bumper){
            gamepad2RightBumperPressed = false;
        }

        if(gamepad2.left_bumper && !gamepad2LeftBumperPressed){
            if(dumperServoPosition == 2){
                dumperServo.setPosition(mid);
                dumperServoPosition = 1;
            }
            else if(dumperServoPosition == 1){
                dumperServo.setPosition(low);
                dumperServoPosition = 0;
            }

            gamepad2LeftBumperPressed = true;
        }
        else if(!gamepad2.left_bumper){
            gamepad2LeftBumperPressed = false;
        }
    }

    private void setCollectorServo(){
        if (gamepad1.left_bumper){
            collectorServo.setPower(1);
        }
        else if (gamepad1.left_trigger>0.5) {
            collectorServo.setPower(-1);
        }
        else {
            collectorServo.setPower(0);
        }
    }

    private void setMarkerServo(){
        if (gamepad1.right_bumper){
            markerServo.setPosition(1);
        }
        else {
            markerServo.setPosition(0);
        }
    }

    private void setMineralLift() {
        if (gamepad2.left_trigger>0.5){
            mineralLift.setPower(-1);
        }
        else if(gamepad2.right_trigger>0.5){
            mineralLift.setPower(1);
        }
        else {
            mineralLift.setPower(0);
        }
    }
}
