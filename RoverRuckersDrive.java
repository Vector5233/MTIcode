package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RoverRuckersDrive extends Object {
    Servo markerServo, dumperServo, cameraServo;
    DcMotor backLeft, backRight, frontLeft, frontRight, liftMotor, collectorMove, collectorExtender;

    ModernRoboticsI2cGyro gyro;
    TouchSensor liftUpSensor;
    LinearOpMode opmode;

    final double TICKS_PER_INCH = 1120.0 / (4 * 3.14159265358979323846264);
    final double ROBOT_RADIUS = 10;
    final double TOLERANCE = 2;
    final double ROOT2 = 1.414;
    final int CAMERA_MIDPOINT = 400;
    final int DISTANCE_ROBOT_MINERAL_STRAIGHT = 19;
    final int DISTANCE_ROBOT_MINERAL_DIAGONAL = 20;
    final int SAMPLING_FORWARD = 4;

    final int RIGHT = 1;
    final int LEFT = -1;
    final int CENTER = 0;

    int distance = 0;
    double convertion = 0;

    private ElapsedTime strafeTimeout;
    private ElapsedTime driveTimeout;
    private ElapsedTime diagonalTimeout;
    private ElapsedTime turnTimeout;
    private ElapsedTime cmTimeout;
    private ElapsedTime lmTimeout;

    public RoverRuckersDrive(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR, DcMotor LM, DcMotor CM, DcMotor CE, Servo DS, Servo CS, Servo MS, ModernRoboticsI2cGyro G, TouchSensor TS, LinearOpMode L) {
        frontLeft = FL;
        frontRight = FR;
        backLeft = BL;
        backRight = BR;
        liftMotor = LM;
        collectorMove = CM;
        collectorExtender = CE;
        markerServo = MS;
        dumperServo = DS;
        cameraServo = CS;

        gyro = G;
        liftUpSensor = TS;
        opmode = L;
    }

    public void setModeAll(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void stopDriving() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    //movements

    public void driveDistance(double power, double distance) {
        int ticks = (int) (distance * TICKS_PER_INCH);

        /*if (power > 0.65) {
            power = 0.65;
        }*/

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while ((frontRight.isBusy() || frontLeft.isBusy()) && opmode.opModeIsActive()) ;

        telemetryDcMotor();

        stopDriving();
    }

    public void driveDistance(double power, double distance, int time) {
        driveTimeout = new ElapsedTime();
        final int DRIVE_TIMEOUT = time;

        int ticks = (int) (distance * TICKS_PER_INCH);

        /*if (power > 0.65) {
            power = 0.65;
        }*/

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);

        while ((frontRight.isBusy() || frontLeft.isBusy()) && opmode.opModeIsActive()){
            if (driveTimeout.milliseconds() > DRIVE_TIMEOUT)
                break;
        }

        frontLeft.setPower(0.0);
        telemetryDcMotor();
        stopDriving();
    }

    public void strafeDistance(double power, double distance) {
        int ticks = (int) (distance * TICKS_PER_INCH);

        /*if (power > 0.65) {
            power = 0.65;
        }*/

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        opmode.idle();

        opmode.telemetry.addData("BL initial", backLeft.getCurrentPosition());
        opmode.telemetry.addData("BR initial", backRight.getCurrentPosition());
        opmode.telemetry.addData("FL initial", frontLeft.getCurrentPosition());
        opmode.telemetry.addData("FR initial", frontRight.getCurrentPosition());
        opmode.telemetry.addData("tick:", ticks);
        opmode.telemetry.update();


        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while ((frontRight.isBusy() || frontLeft.isBusy()) && opmode.opModeIsActive()) {
            ;
        }

        stopDriving();
    }

    public void strafeDistance(double power, double distance, int time) {
        strafeTimeout = new ElapsedTime();
        final int STRAFE_TIMEOUT = time;

        int ticks = (int) (distance * TICKS_PER_INCH);

        /*if (power > 0.65) {
            power = 0.65;
        }*/

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        opmode.idle();

        opmode.telemetry.addData("BL initial", backLeft.getCurrentPosition());
        opmode.telemetry.addData("BR initial", backRight.getCurrentPosition());
        opmode.telemetry.addData("FL initial", frontLeft.getCurrentPosition());
        opmode.telemetry.addData("FR initial", frontRight.getCurrentPosition());
        opmode.telemetry.addData("tick:", ticks);
        opmode.telemetry.update();

        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while ((frontRight.isBusy() || frontLeft.isBusy()) && opmode.opModeIsActive()) {
            if (strafeTimeout.milliseconds() > STRAFE_TIMEOUT)
                break;
        }
        stopDriving();
    }

    public void diagonalDistance(int direction, double power, double distance){
        //direction should be 1 or 0
        //right: 1 left: 0

        int ticks = (int) (distance * TICKS_PER_INCH / ROOT2);

        /*if (power > 0.65) {
            power = 0.65;
        }*/

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(direction * ticks);
        backRight.setTargetPosition(direction * ticks);
        frontRight.setTargetPosition((1 - direction) * ticks);
        backLeft.setTargetPosition((1 - direction) * ticks);

        frontLeft.setPower(direction * power);
        backRight.setPower(direction * power);
        backLeft.setPower((1 - direction) * power);
        frontRight.setPower((1 - direction) * power);

        while ((frontRight.isBusy() || frontLeft.isBusy()) && opmode.opModeIsActive()) {
            telemetryDcMotor();
        }
        stopDriving();
    }

    public void diagonalDistance(int direction, double power, double distance, int time){
        diagonalTimeout = new ElapsedTime();
        final int DIAGONAL_TIMEOUT = time;

        //direction should be 1 or 0
        //right: 1 left: 0

        int ticks = (int) (distance * TICKS_PER_INCH / ROOT2);

        /*if (power > 0.65) {
            power = 0.65;
        }*/

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(direction * ticks);
        backRight.setTargetPosition(direction * ticks);
        frontRight.setTargetPosition((1 - direction) * ticks);
        backLeft.setTargetPosition((1 - direction) * ticks);

        frontLeft.setPower(direction * power);
        backRight.setPower(direction * power);
        backLeft.setPower((1 - direction) * power);
        frontRight.setPower((1 - direction) * power);

        while ((frontRight.isBusy() || frontLeft.isBusy()) && opmode.opModeIsActive()) {
            if (diagonalTimeout.milliseconds() > DIAGONAL_TIMEOUT)
                break;
        }
        stopDriving();
    }

    public void turnDegree(double power, double degrees) {
        // distance in inches
        //conjecture instead of moving 12", wheels will go 12"*cos(45)= 8.5"
        int ticks = (int) ((2 * 3.14159 / 360) * degrees * ROBOT_RADIUS * TICKS_PER_INCH);

        /*if (power > 0.65) {
            power = 0.65;
        }*/

        double target;
        opmode.telemetry.addData("Gyro", gyro.getIntegratedZValue());
        opmode.telemetry.update();
        target = gyro.getIntegratedZValue() + degrees;

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while ((frontRight.isBusy() || frontLeft.isBusy()) && opmode.opModeIsActive()) ;

        telemetryDcMotor();

        stopDriving();
        opmode.telemetry.addData("Gyro end of turn", gyro.getIntegratedZValue());
        opmode.telemetry.update();
    }

    public void turnDegree(double power, double degrees, int time) {
        turnTimeout = new ElapsedTime();
        final int TURN_TIMEOUT = time;

        // distance in inches
        //conjecture instead of moving 12", wheels will go 12"*cos(45)= 8.5"
        int ticks = (int) ((2 * 3.14159 / 360) * degrees * ROBOT_RADIUS * TICKS_PER_INCH);

        /*if (power > 0.65) {
            power = 0.65;
        }*/

        double target;
        opmode.telemetry.update();
        target = gyro.getIntegratedZValue() + degrees;

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while ((frontRight.isBusy() || frontLeft.isBusy()) && opmode.opModeIsActive()){
            if (turnTimeout.milliseconds() > TURN_TIMEOUT)
                break;
        }

        telemetryDcMotor();

        stopDriving();
        opmode.telemetry.addData("Gyro end of turn", gyro.getIntegratedZValue());
        opmode.telemetry.update();
    }

    public void turnCorrect (double target) {
        /* corrects absolute heading to be target in degrees counterclockwise from initial calibration.
         * Caller must know what final heading should be!
         */
        double g;
        g = gyro.getIntegratedZValue();
        opmode.telemetry.addData("Gyro start correct", g);
        opmode.telemetry.update();
        if (g > target + TOLERANCE){
            turnDegree(0.7, g - target);
        } else if (g < target - TOLERANCE){
            turnDegree(0.7, target - g);
        }
        else{

        }
        opmode.telemetry.addData("Gyro end correct", gyro.getIntegratedZValue());
        opmode.telemetry.update();
    }

    //set motor

    public void setCollectorMove (double CMpower, int CMtarget) {
        opmode.telemetry.addData("CM", collectorMove.getCurrentPosition());
        opmode.telemetry.update();
        collectorMove.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        collectorMove.setTargetPosition(CMtarget);
        collectorMove.setPower(CMpower);
        while(collectorMove.isBusy() && opmode.opModeIsActive()) {
            ;
        }
        opmode.telemetry.addData("CM", collectorMove.getCurrentPosition());
        opmode.telemetry.update();
    }

    public void setCollectorMove (double CMpower, int CMtarget, int time) {
        cmTimeout = new ElapsedTime();
        final int CM_TIMEOUT = time;

        opmode.telemetry.addData("CM", collectorMove.getCurrentPosition());
        opmode.telemetry.update();
        collectorMove.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        collectorMove.setTargetPosition(CMtarget);
        collectorMove.setPower(CMpower);
        while(collectorMove.isBusy() && opmode.opModeIsActive()) {
            if (cmTimeout.milliseconds() > CM_TIMEOUT)
                break;
        }
    }

    public void setCollectorMoveNoBlock (double CMpower, int CMtarget) {
        opmode.telemetry.addData("CM", collectorMove.getCurrentPosition());
        opmode.telemetry.update();
        collectorMove.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        collectorMove.setTargetPosition(CMtarget);
        collectorMove.setPower(CMpower);
    }

    public void setLiftMotor (double LMpower, int LMtarget) {
        opmode.telemetry.addData("LM", liftMotor.getCurrentPosition());
        opmode.telemetry.update();
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(LMtarget);
        liftMotor.setPower(LMpower);
        while(liftMotor.isBusy() && opmode.opModeIsActive()) {
            opmode.telemetry.addData("LM", liftMotor.getCurrentPosition());
            opmode.telemetry.update();
        }
    }

    public void setLiftMotor (double LMpower, int LMtarget, int time) {
        lmTimeout = new ElapsedTime();
        final int LM_TIMEOUT = time;

        opmode.telemetry.addData("LM", liftMotor.getCurrentPosition());
        opmode.telemetry.update();
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(LMtarget);
        liftMotor.setPower(LMpower);
        while(liftMotor.isBusy() && opmode.opModeIsActive()) {
            if (lmTimeout.milliseconds() > LM_TIMEOUT)
                break;
            if(liftUpSensor.getValue() > 0.1 )
                break;

            opmode.telemetry.addData("LM", liftMotor.getCurrentPosition());
            opmode.telemetry.update();
        }
    }

    public void setLiftMotorNoBlock (double LMpower, int LMtarget) {
        opmode.telemetry.addData("LM", liftMotor.getCurrentPosition());
        opmode.telemetry.update();
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(LMtarget);
        liftMotor.setPower(LMpower);
    }

    public void cameraMove(double degree){
        cameraServo.setPosition(degree);
    }

    public void telemetryDcMotor(){
        opmode.telemetry.addData("FR", frontRight.getPower());
        opmode.telemetry.addData("FB", frontLeft.getPower());
        opmode.telemetry.addData("BR", backRight.getPower());
        opmode.telemetry.addData("BL", backLeft.getPower());
        opmode.telemetry.update();
    }

    //AutoOp

    public void prepareToLand() {
        setCollectorMove(0.8, 650, 700);
        dumperServo.setPosition(0.1);
    }

    public void land() {
        setLiftMotor(1, 7650, 4000);
        opmode.sleep(500);
    }

    public void unhook() {
        driveDistance(0.3, -0.1, 500);
        opmode.sleep(300);
        strafeDistance(1, 4.3, 1000);
        opmode.sleep(50);
        //turnCorrect(0);
    }

    public void sampleCrater(int goldMineralPosition){
        //convertion = convertPixelToInch(goldMineralX, silverMineral1X, silverMineral2X, goldMineralPosition);
        if(goldMineralPosition == RIGHT){
            driveDistance(1, SAMPLING_FORWARD, 600);
            opmode.sleep(50);
            diagonalDistance(1, 1, DISTANCE_ROBOT_MINERAL_DIAGONAL - 2, 800);
            opmode.sleep(50);
            strafeDistance(1, 3, 1000);
            opmode.sleep(50);
            driveDistance(1, 12, 600);
            //diagonalDistance(diagonalDirection, 1, (abs(convertion)-SAMPLING_FORWARD)*ROOT2);
            //driveDistance(1, DISTANCE_ROBOT_MINERAL-abs(convertion)-SAMPLING_FORWARD);
            //driveDistance(1, DISTANCE_ROBOT_MINERAL_STRAIGHT - SAMPLING_FORWARD - DISTANCE_ROBOT_MINERAL_DIAGONAL / ROOT2);
        }
        else if(goldMineralPosition == LEFT){
            driveDistance(1, SAMPLING_FORWARD + 5, 600);
            opmode.sleep(50);
            strafeDistance(1, -13, 750);
            opmode.sleep(50);
            diagonalDistance(0, 1, DISTANCE_ROBOT_MINERAL_DIAGONAL - 6, 800);
            opmode.sleep(50);
            driveDistance(1, 8 , 600);
            //diagonalDistance(diagonalDirection, 1, (abs(convertion)-SAMPLING_FORWARD)*ROOT2);
            //driveDistance(1, DISTANCE_ROBOT_MINERAL-abs(convertion)-SAMPLING_FORWARD);
            //driveDistance(1, DISTANCE_ROBOT_MINERAL_STRAIGHT - SAMPLING_FORWARD - DISTANCE_ROBOT_MINERAL_DIAGONAL / ROOT2);
        }
        else{
            driveDistance(1, SAMPLING_FORWARD, 600);
            opmode.sleep(50);
            strafeDistance(1, -6, 500);
            opmode.sleep(50);
            driveDistance(1, DISTANCE_ROBOT_MINERAL_STRAIGHT - SAMPLING_FORWARD + 4, 1000);
            //driveDistance(1, convertion-SAMPLING_FORWARD);
        }
        opmode.sleep(50);
    }

    public void sampleBox(int goldMineralPosition){
        //convertion = convertPixelToInch(goldMineralX, silverMineral1X, silverMineral2X, goldMineralPosition);
        if(goldMineralPosition == RIGHT){
            driveDistance(1, SAMPLING_FORWARD, 600);
            opmode.sleep(50);
            diagonalDistance(1, 1, DISTANCE_ROBOT_MINERAL_DIAGONAL - 2, 800);
            opmode.sleep(50);
            strafeDistance(1, 3, 1000);
            opmode.sleep(50);
            driveDistance(1, 10, 600);
            //diagonalDistance(diagonalDirection, 1, (abs(convertion)-SAMPLING_FORWARD)*ROOT2);
            //driveDistance(1, DISTANCE_ROBOT_MINERAL-abs(convertion)-SAMPLING_FORWARD);
            //driveDistance(1, DISTANCE_ROBOT_MINERAL_STRAIGHT - SAMPLING_FORWARD - DISTANCE_ROBOT_MINERAL_DIAGONAL / ROOT2);
        }
        else if(goldMineralPosition == LEFT){
            /*driveDistance(1, SAMPLING_FORWARD, 600);
            opmode.sleep(50);
            strafeDistance(1, -6, 750);
            opmode.sleep(50);
            diagonalDistance(0, 1, DISTANCE_ROBOT_MINERAL_DIAGONAL, 800);
            opmode.sleep(50);
            driveDistance(1, 8, 600);*/
            driveDistance(1, SAMPLING_FORWARD + 5, 600);
            opmode.sleep(50);
            strafeDistance(1, -16, 750);
            opmode.sleep(50);
            diagonalDistance(0, 1, DISTANCE_ROBOT_MINERAL_DIAGONAL - 6, 800);
            opmode.sleep(50);
            driveDistance(1, 8, 600);
            //diagonalDistance(diagonalDirection, 1, (abs(convertion)-SAMPLING_FORWARD)*ROOT2);
            //driveDistance(1, DISTANCE_ROBOT_MINERAL-abs(convertion)-SAMPLING_FORWARD);
            //driveDistance(1, DISTANCE_ROBOT_MINERAL_STRAIGHT - SAMPLING_FORWARD - DISTANCE_ROBOT_MINERAL_DIAGONAL / ROOT2);
        }
        else{
            driveDistance(1, SAMPLING_FORWARD, 600);
            opmode.sleep(50);
            strafeDistance(1, -5, 500);
            opmode.sleep(50);
            driveDistance(1, DISTANCE_ROBOT_MINERAL_STRAIGHT - SAMPLING_FORWARD, 1000);
            //driveDistance(1, convertion-SAMPLING_FORWARD);
        }
        opmode.sleep(50);
    }

    /*
    public double convertPixelToInch(int goldMineralX, int silverMineral1X, int silverMineral2X, int goldMineralPosition){
        int leftMineral = min(goldMineralX, min(silverMineral1X, silverMineral2X));
        int rightMineral = max(goldMineralX, max(silverMineral1X, silverMineral2X));
        int centerMineral = -1;

        if(goldMineralPosition == 0)
            centerMineral = goldMineralX;
        else if(silverMineral1X == leftMineral && silverMineral1X == rightMineral)
            centerMineral = silverMineral2X;
        else
            centerMineral = silverMineral1X;

        distance = centerMineral - CAMERA_MIDPOINT;
        opmode.telemetry.addData("distance:", distance);

        if(goldMineralPosition == 1 || goldMineralX == -1){
            return (double) distance * (double) DISTANCE_MINERAL_MINERAL /(double) abs(centerMineral - goldMineralX);
        }
        else if(goldMineralPosition == 0){
            return (double) distance * (double) DISTANCE_MINERAL_MINERAL/(double) abs(centerMineral - goldMineralX);
        }
        else{
            return 0;
        }
    }
    */
}