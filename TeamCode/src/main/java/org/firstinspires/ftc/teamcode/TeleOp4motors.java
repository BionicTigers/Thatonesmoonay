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
    private double leftStick, rightStick;
    private double leftPower, rightPower;
    private double calibToggle, coarseDiff, fineDiff, stickDiff;
    private int driveSpeed, driveMode;

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
        driveSpeed = 0;
        driveMode = 0;
    }


    public void loop() {
        // TOGGLE BUETONS //
        if (gamepad1.a && (runtime.seconds() > calibToggle)) {
            calibToggle = runtime.seconds() + 1;
            ++driveSpeed;
        }
        if (gamepad1.x && (runtime.seconds() > calibToggle)) {
            driveMode = 0;
        }
        if (gamepad1.y && (runtime.seconds() > calibToggle)) {
            driveMode = 1;
        }
        if (gamepad1.b && (runtime.seconds() > calibToggle)) {
            driveMode = 2;
        }

        if (driveMode == 0) {
            //////////////////////////////////// ARCADE DRIVE //////////////////////////////////////

            //Speed Offsets
            coarseDiff = .75;
            fineDiff = .45;
            stickDiff = 1;

            leftStick = (gamepad1.left_stick_y);
            rightStick = (-gamepad1.right_stick_x * stickDiff);

            //Left Side
            if (Math.abs(rightStick) > 0.45) {
                if (Math.abs(leftStick) > 0.45) {
                    leftPower = leftStick / 2 + rightStick / 2;
                } else {
                    leftPower = leftStick + rightStick;
                }
            } else {
                leftPower = leftStick;
            }

            //Right Side
            if (Math.abs(rightStick) > 0.45) {
                if (Math.abs(leftStick) > 0.45) {
                    rightPower = leftStick / 2 - rightStick / 2;
                } else {
                    rightPower = leftStick - rightStick;
                }
            } else {
                rightPower = leftStick;
            }
        } else if (driveMode == 1) {
            ///////////////////////////////////// TANK DRIVE ///////////////////////////////////////

            //Speed Offsets
            coarseDiff = .75;
            fineDiff = .45;

            leftPower = (gamepad1.left_stick_y * coarseDiff);
            rightPower = (gamepad1.right_stick_y * coarseDiff);
        } else if (driveMode == 2) {
            ////////////////////////////////// ACKERMAN DRIVE //////////////////////////////////////

            //Speed Offsets
            coarseDiff = .75;
            fineDiff = .45;

            leftStick = (gamepad1.left_stick_y);
            rightStick = 1;

            if (gamepad1.right_trigger > 0.15) {
                if (Math.abs(rightStick) > 0.45) {
                    leftPower = leftStick / 2 + gamepad1.right_trigger;
                } else {
                    leftPower = leftStick;
                }
                if (Math.abs(rightStick) > 0.45) {
                    rightPower = leftStick / 2 - gamepad1.right_trigger;
                } else {
                    rightPower = leftStick;
                }
            } else if (gamepad1.left_trigger > 0.15) {
                if (Math.abs(rightStick) > 0.45) {
                    leftPower = leftStick / 2 + gamepad1.left_trigger;
                } else {
                    leftPower = leftStick;
                }
                if (Math.abs(rightStick) > 0.45) {
                    rightPower = leftStick / 2 - gamepad1.left_trigger;
                } else {
                    rightPower = leftStick;
                }
            } else {
                leftPower = 0;
                rightPower = 0;
            }
        }

        if (driveSpeed % 2 == 0) { //Coarse Drive Mode
            leftPower = Math.pow(leftPower, 3) * coarseDiff;
            rightPower = Math.pow(rightPower, 3) * coarseDiff;

            telemetry.addData("Mode: ", "COARSE");
            telemetry.addData("Stick: ", "Y = " + round(leftStick, 3) + ", X = " + round(rightStick, 3));
            telemetry.addData("Power: ", "L = " + round(leftPower / coarseDiff, 3) + ", R = " + round(rightPower / coarseDiff, 3));
            telemetry.update();
        } else { //Fine Drive Mode
            leftPower = Math.pow(leftPower, 3) * fineDiff;
            rightPower = Math.pow(rightPower, 3) * fineDiff;

            telemetry.addData("Mode: ", "FINE");
            telemetry.addData("Stick: ", "Y = " + round(leftStick, 3) + ", X = " + round(rightStick, 3));
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