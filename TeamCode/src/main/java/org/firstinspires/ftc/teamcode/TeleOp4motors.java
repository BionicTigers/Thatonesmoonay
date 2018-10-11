package org.firstinspires.ftc.teamcode;
//EXIST
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.BigDecimal;
import java.math.RoundingMode;


@TeleOp(name="4Motors", group="BTBT")
public class TeleOp4motors extends OpMode {

    //Drivetrain Motors//
    private DcMotor backLeft; //left motor back
    private DcMotor frontLeft; //left motor front (Ladies first)ot
    private DcMotor backRight; //right motor back
    private DcMotor frontRight; //right motor front (Ladies first)
//    private DcMotor collector;
//    private DcMotor evangelino;
//    private Servo teamMarker;

    //Variables//
    private double xValue, yValue, leftPower, rightPower, coarseDiff, fineDiff, stickDiff;
    public double calibToggle;
    public int driveMode;
//    public int target;
//    private double speed;
//    private Servo liftrawrh;

    //Objects//
    public ElapsedTime runtime = new ElapsedTime();

    //HardwareCatBot robot;

    public void init() {
        //Motors//
        backLeft = hardwareMap.dcMotor.get("backLeft"); //Left Back
        frontLeft = hardwareMap.dcMotor.get("frontLeft"); //Left Front
        backRight = hardwareMap.dcMotor.get("backRight"); //Right Back
        frontRight = hardwareMap.dcMotor.get("frontRight"); //Right Front
//        evangelino = hardwareMap.dcMotor.get("lift");
//        collector = hardwareMap.dcMotor.get("collector");
//        teamMarker = hardwareMap.servo.get("teamMarker");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        liftrawrh = hardwareMap.servo.get("liftrawrh");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        //Variables//
        calibToggle = 0;
        driveMode = 0;
//        target = 0;
//        speed = 0;
    }

    public void loop() {

        //robot = new Hardware_Pushbot();
        if (gamepad1.a && (runtime.seconds() > calibToggle)) {
            calibToggle = runtime.seconds() + 1;
            ++driveMode;
        }

        coarseDiff = .75; //Different offsets
        fineDiff = .45;
        stickDiff = .9;

        //=IF(ABS(B4) > 0.5, IF(ABS(A4) > 0.5, A4/2+B4*0.25, A4-B4), A4) // Left
        //=IF(ABS(B4) > 0.5, IF(ABS(A4) > 0.5, A4/2+-B4*0.25, A4+B4), A4) // Right

        yValue = (gamepad1.left_stick_y);
        xValue = (-gamepad1.right_stick_x * stickDiff); //Multiplied by .92 to eliminate over-turning

        if (driveMode % 2 == 0) { //Coarse Drive Mode
            if (Math.abs(xValue) > 0.5) {
                if (Math.abs(yValue) > 0.5) {
                    leftPower = yValue / 2 + xValue * 0.5;
                } else {
                    leftPower = yValue - xValue;
                }
            } else {
                leftPower = yValue;
            }

            if (Math.abs(xValue) > 0.5) {
                if (Math.abs(yValue) > 0.5) {
                    rightPower = yValue / 2 - xValue * 0.5;
                } else {
                    rightPower = yValue + xValue;
                }
            } else {
                rightPower = yValue;
            }

            leftPower = Math.pow(leftPower, 3) * coarseDiff;
            rightPower = Math.pow(rightPower, 3) * coarseDiff;

            telemetry.addData("Mode: ", "COARSE");
            telemetry.addData("Stick: ", "Y = " + round(yValue, 3) + ", X = " + round(xValue, 3));
            telemetry.addData("Power: ", "L = " + round(leftPower / coarseDiff, 3) + ", R = " + round(rightPower / coarseDiff, 3));
            telemetry.update();
        } else { //Fine Drive Mode
            if (Math.abs(xValue) > 0.5) {
                if (Math.abs(yValue) > 0.5) {
                    leftPower = yValue / 2 + xValue * 0.5;
                } else {
                    leftPower = yValue - xValue;
                }
            } else {
                leftPower = yValue;
            }

            if (Math.abs(xValue) > 0.5) {
                if (Math.abs(yValue) > 0.5) {
                    rightPower = yValue / 2 - xValue * 0.5;
                } else {
                    rightPower = yValue + xValue;
                }
            } else {
                rightPower = yValue;
            }

            leftPower = Math.pow(leftPower, 3) * fineDiff;
            rightPower = Math.pow(rightPower, 3) * fineDiff;

            telemetry.addData("Mode: ", "FINE");
            telemetry.addData("Stick: ", "Y = " + round(yValue, 3) + ", X = " + round(xValue, 3));
            telemetry.addData("Power: ", "L = " + round(leftPower / fineDiff, 3) + ", R = " + round(rightPower / fineDiff, 3));
            telemetry.update();
        }

        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);
        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
    }

    public static double round(double value, int places) { //Allows telemetry to display nicely
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}

