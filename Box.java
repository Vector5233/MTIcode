package org.firstinspires.ftc.teamcode;

//hardware
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//webcam
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetectionWebcam;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import static java.lang.Math.abs;
import static java.lang.Math.floor;

@Autonomous(name="Box", group="TeamCode")

/* TODO
 * Fix sampling -- goes forward correctly, goes wrong way when gold on left, check convertion value
 * tes* reduce total time for opmod
 */
public class Box extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    //final int LEFT = 100 ;
    //final int CENTER = 450;
    //final int POS_TOLERANCE = 100;
    final int RIGHT = 1;
    final int LEFT = -1;
    final int CENTER = 0;
    final double ROOT2 = 1.414;

    final double FACE_FRONT = 0.425 ;
    final double FACE_FRONT_CORRECT = 0.5;

    final int DISTANCE_MINERAL_MARKING = 15;
    final int DISTANCE_MARKING_PARKING_RIGHT = 46;
    final int DISTANCE_MARKING_PARKING_CENTER = 45;
    final int DISTANCE_MARKING_PARKING_LEFT = 37;

    int goldMineralX = -1;
    int silverMineral1X = -1;
    int silverMineral2X = -1;
    int goldMineralPosition = 0;

    double timeMark = 0;
    public ElapsedTime runTime = new ElapsedTime();

    Servo markerServo, dumperServo, cameraServo;
    CRServo collectorServo;
    DcMotor backLeft,backRight, frontLeft, frontRight, liftMotor, collectorMove, collectorExtender;
    ModernRoboticsI2cGyro gyro;
    TouchSensor liftUpSensor;
    RoverRuckersDrive drive;

    VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    WebcamName webcamName;

    void initialize() {
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
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        liftUpSensor = hardwareMap.touchSensor.get("liftUpSensor");

        collectorServo.setDirection(CRServo.Direction.FORWARD);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        collectorMove.setDirection(DcMotor.Direction.FORWARD);
        collectorExtender.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorMove.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMove.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive = new RoverRuckersDrive(frontLeft, frontRight, backLeft, backRight, liftMotor, collectorMove, collectorExtender, dumperServo, cameraServo, markerServo, gyro, liftUpSensor, this);

        cameraServo.setPosition(0);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(/*cameraMonitorViewId*/ );
        parameters.cameraName = webcamName;
        parameters.vuforiaLicenseKey = "AbLVQDn/////AAABma+zV9cCqU+7pLBUXgQ3J6II1u1B8Vg4mrnGfVawPjc1l7C6GWoddOaL6Wqj5kXPBVUh3U3WND38234Tm0h3+LKmmTzzaVPRwOk3J+zBwKlOvv93+u7chctULk8ZYEyf0NuuEfsGwpgJx7xL9hIFBoaB2G1SpbJIt+n94wz6EvfRYSusBEiST/lUqgDISIlaeOLPWEipHh46axomcrGVRRl09pg6pCt2h7rU6us+guN5nKhupTXvM+BTUYW3kCO9YsUjz16jLr7GyFh8wVQbRS3dikSX7kzVsdkLjZnJdyinYaB5oDXfmmXtaC6ZXeD6vKs62vpaydAq9VGAlCtnSyq2J4NLI+LOIOvdtsCwarfS";
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        initTfod();
        telemetry.addLine("TFOD created");
        telemetry.update();

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorMove.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timeMark = runTime.time();

        gyro.calibrate();

        while (gyro.getIntegratedZValue() != 0 && (runTime.time() - timeMark) < 1.5)
            idle();
    }

    public void runOpMode() {
        initialize();
        waitForStart();
        idle();
        drive.cameraMove(FACE_FRONT);
        activateTfod();
        //drive.prepareToLand();
        drive.land();
        sleep(500);
        drive.unhook();
        drive.setLiftMotorNoBlock(1, 0);
        drive.sampleBox (goldMineralPosition);
        //drive.setCollectorMoveNoBlock(1, 0);
        getToMark();
        mark();
        readyToPark();
        park();
    }

    private void activateTfod(){
        List<Recognition> updatedRecognitions = null;

        if (tfod != null)
            tfod.activate();
        sleep(2500);

        if (tfod != null) {
            updatedRecognitions = tfod.getUpdatedRecognitions();
            telemetry.addLine("recognition variable created");
            telemetry.update();
            detectMineral(updatedRecognitions);
            if(updatedRecognitions == null)
                ;
            else if(updatedRecognitions.size() == 2 || updatedRecognitions.size() == 3)
                updatedRecognitions = sortList(updatedRecognitions);
        }
        tfod.shutdown();

        telemetry.addData("gold mineral position", goldMineralPosition);

        if(updatedRecognitions != null) {
            int i = 0;
            for (Recognition r : updatedRecognitions) {
                telemetry.addData("recognition List" + Integer.toString(i), r.getLeft());
                i ++;
            }
        }
        telemetry.update();
        sleep(500);
    }

    private void initTfod(){
        //int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(/*tfodMonitorViewId*/);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void detectMineral(List<Recognition> updatedRecognitions) {

        if (updatedRecognitions == null || updatedRecognitions.size() == 1) {
            drive.cameraMove(FACE_FRONT_CORRECT);
            updatedRecognitions = tfod.getUpdatedRecognitions();
            //<divided by zero> error 18.12.17
        }

        if (updatedRecognitions == null || updatedRecognitions.size() == 1) {
            //go middle
            telemetry.addLine("Failed to find any or only one mineral");
            telemetry.update();
            goldMineralPosition = -1;
        }
        if(updatedRecognitions != null) {
            if (updatedRecognitions.size() == 2 || updatedRecognitions.size() == 3) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineralX = (int) recognition.getLeft();
                    } else if (silverMineral1X == -1) {
                        silverMineral1X = (int) recognition.getLeft();
                    } else {
                        silverMineral2X = (int) recognition.getLeft();
                    }
                }

                if (goldMineralX == -1) {
                    goldMineralX = findVirtualMineralX(silverMineral1X, silverMineral2X);
                    identifyPosition();
                } else if (silverMineral2X == -1) {
                    silverMineral2X = findVirtualMineralX(goldMineralX, silverMineral1X);
                    identifyPosition();
                } else {
                    identifyPosition();
                }
            }
        }
        else
            goldMineralPosition = -1;
    }

    private int findVirtualMineralX(int mineral1, int mineral2){
        //find left mineral or right mineral
        if(mineral1 > mineral2) {
            return mineral1 + (mineral1 - mineral2);
        }
        else{
            return mineral2 + (mineral2 - mineral1);
        }
    }

    private void identifyPosition(){
        if(goldMineralX > silverMineral1X && goldMineralX > silverMineral2X)
            goldMineralPosition = RIGHT;
        else if(goldMineralX < silverMineral1X && goldMineralX < silverMineral2X)
            goldMineralPosition = LEFT;
        else
            goldMineralPosition = CENTER;
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

    private void getToMark(){
        if(goldMineralPosition == 1) {
            drive.driveDistance(1, (double) DISTANCE_MINERAL_MARKING, 2000);
            sleep(50);
            drive.diagonalDistance(0, 1, 10, 1500);
            sleep(50);
            drive.turnDegree(1, 45, 1000);
        }
        else if(goldMineralPosition == -1) {
            drive.driveDistance(1, (double) DISTANCE_MINERAL_MARKING, 2000);
            sleep(50);
            drive.diagonalDistance(1, 1, 10, 1500);
        }
        else{
            drive.driveDistance(1, (double) DISTANCE_MINERAL_MARKING + 3, 2000);
        }
        sleep(50);
    }

    //center sample
    //left sample and short parking

    private void mark(){
        drive.turnDegree(1, 45, 2000);
        markerServo.setPosition(1);
        sleep(500);
        markerServo.setPosition(0);
        sleep(50);
    }

    private void readyToPark(){
        if(goldMineralPosition == 1){
            drive.turnCorrect(90);
            sleep(50);
            drive.driveDistance(1, 22., 2000);
            drive.turnDegree(1, 38, 1000);
            drive.turnCorrect(128);
        }
        else if(goldMineralPosition == -1){
            drive.turnDegree(1, 85, 700);
            drive.turnCorrect(130);
            sleep(50);
        }
        else{
            drive.turnDegree(1, 45, 2000);
            drive.turnCorrect(90);
            sleep(50);
            drive.driveDistance(1, 15, 2000);
            drive.turnDegree(1, 40, 2000);
            drive.turnCorrect(130);
        }
    }

    private void park(){
        if(goldMineralPosition == 0)
            drive.driveDistance(1, DISTANCE_MARKING_PARKING_CENTER, 2000);
        else if(goldMineralPosition == -1)
            drive.driveDistance(1, DISTANCE_MARKING_PARKING_LEFT + 5, 2000);
        else
            drive.driveDistance(1, DISTANCE_MARKING_PARKING_RIGHT, 2000);
        //dumperServo.setPosition(0.5);
        sleep(50);
        drive.setCollectorMove(1, 935);
    }
}