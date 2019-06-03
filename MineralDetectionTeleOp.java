package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

@TeleOp(name="MineralDetectTeleOp", group="TeamCode")

public class MineralDetectionTeleOp extends OpMode {
    Servo markerServo, dumperServo, cameraServo;
    CRServo collectorServo;
    DcMotor backLeft,backRight, frontLeft, frontRight, liftMotor, collectorMove, collectorExtender;
    TouchSensor liftUpSensor, liftDownSensor;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    WebcamName webcamName;

    boolean canWrite = false;
    boolean gamepad2RightBumperPressed = false;
    boolean gamepad2LeftBumperPressed = false;
    int dumperServoPosition = 0;

    final double TOLERANCE = 0.5;
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
        collectorExtender = hardwareMap.dcMotor.get("collectorExtender");

        liftUpSensor = hardwareMap.touchSensor.get("liftUpSensor");
        liftDownSensor = hardwareMap.touchSensor.get("liftDownSensor");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMove.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        collectorServo.setDirection(CRServo.Direction.REVERSE);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        collectorMove.setDirection(DcMotor.Direction.FORWARD);
        collectorExtender.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorMove.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorMove.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final double FACE_FRONT = 0.425 ;
        cameraServo.setPosition(FACE_FRONT);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(/*cameraMonitorViewId*/ );
        parameters.cameraName = webcamName;
        parameters.vuforiaLicenseKey = "AbLVQDn/////AAABma+zV9cCqU+7pLBUXgQ3J6II1u1B8Vg4mrnGfVawPjc1l7C6GWoddOaL6Wqj5kXPBVUh3U3WND38234Tm0h3+LKmmTzzaVPRwOk3J+zBwKlOvv93+u7chctULk8ZYEyf0NuuEfsGwpgJx7xL9hIFBoaB2G1SpbJIt+n94wz6EvfRYSusBEiST/lUqgDISIlaeOLPWEipHh46axomcrGVRRl09pg6pCt2h7rU6us+guN5nKhupTXvM+BTUYW3kCO9YsUjz16jLr7GyFh8wVQbRS3dikSX7kzVsdkLjZnJdyinYaB5oDXfmmXtaC6ZXeD6vKs62vpaydAq9VGAlCtnSyq2J4NLI+LOIOvdtsCwarfS";
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        initTfod();
        tfod.activate();

        telemetry.addLine("initialize finished");
        telemetry.update();
    }

    private void activateTfod(){
        List<Recognition> updatedRecognitions = null;

        if (tfod != null) {
            updatedRecognitions = tfod.getUpdatedRecognitions();
            telemetry.addLine("recognition variable created");
            telemetry.update();
            if(updatedRecognitions == null)
                ;
            else if(updatedRecognitions.size() == 2 || updatedRecognitions.size() == 3)
                updatedRecognitions = sortList(updatedRecognitions);
        }
        if(updatedRecognitions != null) {
            int i = 0;
            for (Recognition r : updatedRecognitions) {
                telemetry.addData("recognition List" + Integer.toString(i), r.getLeft());
                telemetry.addData("width" + Integer.toString(i), r.getWidth());
                telemetry.addData("image width" + Integer.toString(i), r.getImageWidth());
                telemetry.addData("get label" + Integer.toString(i), r.getLabel());
                i ++;
            }
        }
        telemetry.update();
    }

    private void initTfod(){
        //int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(/*tfodMonitorViewId*/);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private List<Recognition> sortList(List<Recognition> r){
        List<Recognition> orderedList = new ArrayList<Recognition>();
        if(orderedList == null) {
            telemetry.addLine("list is null");
            telemetry.update();
            return null;
        }
        else if(r.get(0).getLeft() < r.get(1).getLeft()){
            orderedList.add(r.get(0));
            orderedList.add(r.get(1));
        }
        else{
            orderedList.add(r.get(1));
            orderedList.add(r.get(0));
        }

        if(r.size() == 3){
            if(r.get(2).getLeft() < r.get(0).getLeft())
                orderedList.add(0, r.get(2));
            else if(r.get(0).getLeft() < r.get(2).getLeft() || r.get(2).getLeft() < r.get(1).getLeft())
                orderedList.add(1, r.get(2));
            else
                orderedList.add(2, r.get(2));
        }

        return orderedList;
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

        if(gamepad1.x == true)
            activateTfod();
        else
            tfod.shutdown();
        //TODO: cut
        //get encoder value
        if (!canWrite) {
            telemetry.addData("lift up sensor: ", liftUpSensor.getValue());
            telemetry.addData("lift motor: ", liftMotor.getCurrentPosition());
        }
        telemetry.update();
    }

    private void setDriveMotors(){
        frontLeft.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x)/2 - gamepad1.left_stick_x);
        frontRight.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x)/2 + gamepad1.left_stick_x);
        backLeft.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x)/2 - gamepad1.left_stick_x);
        backRight.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x)/2 + gamepad1.left_stick_x);
    }

    private void setLiftMotor() {
        if (gamepad2.left_trigger >= 0.5 && liftDownSensor.getValue() == 0)
            liftMotor.setPower(- gamepad2.left_trigger);
        else if (gamepad2.right_trigger >= 0.5 && liftUpSensor.getValue() == 0)
            liftMotor.setPower(gamepad2.right_trigger);
        else
            liftMotor.setPower(0);
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
        if (abs(gamepad2.right_stick_y) > TOLERANCE) {
            if (/*collectorExtenderPosition < MAX_CE &&*/ gamepad2.right_stick_y > 0) {
                collectorExtender.setPower(gamepad2.right_stick_y*.2);
            } else if (/*MIN_CE < collectorExtenderPosition &&*/ gamepad2.right_stick_y < 0) {
                collectorExtender.setPower(gamepad2.right_stick_y*.2);
            }
        } else {
            collectorExtender.setPower(0);
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
}