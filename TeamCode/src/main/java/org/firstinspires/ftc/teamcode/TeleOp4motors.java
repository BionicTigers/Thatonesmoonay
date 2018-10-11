package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.BigDecimal;
import java.math.RoundingMode;


@TeleOp(name="4Motors", group="BTBT")
public class TeleOp4motors extends OpMode {

    //Drivetrain Motors//
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //Variables//
    private double xValue, yValue, leftPower, rightPower, coarseDiff, fineDiff, stickDiff;
    public double calibToggle;
    public int driveMode;

    //Objects//
    public ElapsedTime runtime = new ElapsedTime();


    public void init() {
        //Motors//
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        //Variables//
        calibToggle = 0;
        driveMode = 0;
    }


    public void loop() {
        //Toggle Button
        if (gamepad1.a && (runtime.seconds() > calibToggle)) {
            calibToggle = runtime.seconds() + 1;
            ++driveMode;
        }

        //Different Offsets
        coarseDiff = .75;
        fineDiff = .45;
        stickDiff = .9;

        yValue = (gamepad1.left_stick_y);
        xValue = (-gamepad1.right_stick_x * stickDiff);

        //Left Side
        if (Math.abs(xValue) > 0.5) {
            if (Math.abs(yValue) > 0.5) {
                leftPower = yValue / 2 + xValue * 0.5;
            } else {
                leftPower = yValue + xValue;
            }
        } else {
            leftPower = yValue;
        }

        //Right Side
        if (Math.abs(xValue) > 0.5) {
            if (Math.abs(yValue) > 0.5) {
                rightPower = yValue / 2 - xValue * 0.5;
            } else {
                rightPower = yValue - xValue;
            }
        } else {
            rightPower = yValue;
        }

        if (driveMode % 2 == 0) { //Coarse Drive Mode
            leftPower = Math.pow(leftPower, 3) * coarseDiff;
            rightPower = Math.pow(rightPower, 3) * coarseDiff;

            telemetry.addData("Mode: ", "COARSE");
            telemetry.addData("Stick: ", "Y = " + round(yValue, 3) + ", X = " + round(xValue, 3));
            telemetry.addData("Power: ", "L = " + round(leftPower / coarseDiff, 3) + ", R = " + round(rightPower / coarseDiff, 3));
            telemetry.update();
        } else { //Fine Drive Mode
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