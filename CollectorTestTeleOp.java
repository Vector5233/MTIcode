package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import static java.lang.Math.abs;

@TeleOp(name="CollectorTestTeleOp", group="TeamCode")

public class CollectorTestTeleOp extends OpMode {
    DcMotor collectorExtenderDc;
    CRServo collectorExtenderServo;

    boolean canWrite = false;

    int currentCEVal = 0;

    final double TOLERANCE = 0.5;
    final double collectorMin = -10;
    final double collectorMax = 370;

    public void init(){
        collectorExtenderDc = hardwareMap.dcMotor.get("collectorExtenderDc");
        collectorExtenderServo = hardwareMap.crservo.get("collectorExtenderServo");
        collectorExtenderDc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorExtenderDc.setDirection(DcMotor.Direction.FORWARD);
        collectorExtenderDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorExtenderDc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop(){
        canWrite = !canWrite;
        setCollectorExtender();

        if(!canWrite) {
            currentCEVal = collectorExtenderDc.getCurrentPosition();
            telemetry.addData("CE value", currentCEVal);
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
}
