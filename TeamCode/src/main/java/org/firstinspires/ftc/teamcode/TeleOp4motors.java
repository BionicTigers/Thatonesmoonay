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
    private double gasPedal;
    private double coarseDiff, fineDiff, calibToggle;
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

            leftStick = gamepad1.left_stick_y;
            rightStick = -gamepad1.right_stick_x;

            //Left Side
            if (Math.abs(rightStick) > 0.5) {
                if (Math.abs(leftStick) > 0.5) {
                    leftPower = leftStick / 2 + rightStick / 2;
                } else {
                    leftPower = leftStick + rightStick;
                }
            } else {
                leftPower = leftStick;
            }

            //Right Side
            if (Math.abs(rightStick) > 0.5) {
                if (Math.abs(leftStick) > 0.5) {
                    rightPower = leftStick / 2 - rightStick / 2;
                } else {
                    rightPower = leftStick - rightStick;
                }
            } else {
                rightPower = leftStick;
            }

            if (driveSpeed % 2 == 0) {
                telemetry.addData("Mode: ", "ARCADE");
                telemetry.addData("Speed: ", "NORMAL");
                telemetry.addData("Stick: ", "L = " + round(leftStick, 3) + "R = " + round(rightStick, 3));
                telemetry.addData("Power: ", "L = " + round(leftPower, 3) + ", R = " + round(rightPower, 3));
                telemetry.update();

                backLeft.setPower(leftPower * coarseDiff);
                backRight.setPower(rightPower * coarseDiff);
                frontLeft.setPower(leftPower * coarseDiff);
                frontRight.setPower(rightPower * coarseDiff);
            } else {
                telemetry.addData("Mode: ", "ARCADE");
                telemetry.addData("Speed: ", "SLOW");
                telemetry.addData("Stick: ", "L = " + round(leftStick, 3) + "R = " + round(rightStick, 3));
                telemetry.addData("Power: ", "L = " + round(leftPower, 3) + ", R = " + round(rightPower, 3));
                telemetry.update();

                backLeft.setPower(leftPower * fineDiff);
                backRight.setPower(rightPower * fineDiff);
                frontLeft.setPower(leftPower * fineDiff);
                frontRight.setPower(rightPower * fineDiff);
            }
        } else if (driveMode == 1) {
            ///////////////////////////////////// TANK DRIVE ///////////////////////////////////////

            //Speed Offsets
            coarseDiff = .75;
            fineDiff = .45;

            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;

            if (driveSpeed % 2 == 0) {
                telemetry.addData("Mode: ", "TANK");
                telemetry.addData("Speed: ", "NORMAL");
                telemetry.addData("Stick: ", "L = " + round(leftStick, 3) + "R = " + round(rightStick, 3));
                telemetry.addData("Power: ", "L = " + round(leftPower, 3) + ", R = " + round(rightPower, 3));
                telemetry.update();

                backLeft.setPower(leftPower * coarseDiff);
                backRight.setPower(rightPower * coarseDiff);
                frontLeft.setPower(leftPower * coarseDiff);
                frontRight.setPower(rightPower * coarseDiff);
            } else {
                telemetry.addData("Mode: ", "TANK");
                telemetry.addData("Speed: ", "SLOW");
                telemetry.addData("Stick: ", "L = " + round(leftStick, 3) + "R = " + round(rightStick, 3));
                telemetry.addData("Power: ", "L = " + round(leftPower, 3) + ", R = " + round(rightPower, 3));
                telemetry.update();

                backLeft.setPower(leftPower * fineDiff);
                backRight.setPower(rightPower * fineDiff);
                frontLeft.setPower(leftPower * fineDiff);
                frontRight.setPower(rightPower * fineDiff);
            }
        } else if (driveMode == 2) {
            ////////////////////////////////// ACKERMAN DRIVE //////////////////////////////////////

            //Speed Offsets
            coarseDiff = .75;
            fineDiff = .45;

            leftStick = (-gamepad1.left_stick_x);

            gasPedal = (gamepad1.right_trigger - gamepad1.left_trigger);

            //Left Side
            if (Math.abs(leftStick) > 0.5) {
                if (Math.abs(gasPedal) > 0.5) {
                    leftPower = (gasPedal / 2 + leftStick / 2);
                } else {
                    leftPower = (gasPedal + leftStick);
                }
            } else {
                leftPower = gasPedal;
            }

            //Right Side
            if (Math.abs(leftStick) > 0.5) {
                if (Math.abs(gasPedal) > 0.5) {
                    leftPower = (gasPedal / 2 - leftStick / 2);
                } else {
                    leftPower = (gasPedal - leftStick);
                }
            } else {
                leftPower = gasPedal;
            }

            if (driveSpeed % 2 == 0) {
                telemetry.addData("Mode: ", "ACKERMAN");
                telemetry.addData("Speed: ", "NORMAL");
                telemetry.addData("Stick: ", "L = " + round(leftStick, 3) + "G: " + round(gasPedal, 3));
                telemetry.addData("Power: ", "L = " + round(leftPower, 3) + ", R = " + round(rightPower, 3));
                telemetry.update();

                backLeft.setPower(leftPower * coarseDiff);
                backRight.setPower(rightPower * coarseDiff);
                frontLeft.setPower(leftPower * coarseDiff);
                frontRight.setPower(rightPower * coarseDiff);
            } else {
                telemetry.addData("Mode: ", "ACKERMAN");
                telemetry.addData("Speed: ", "SLOW");
                telemetry.addData("Stick: ", "L = " + round(leftStick, 3) + "G: " + round(gasPedal, 3));
                telemetry.addData("Power: ", "L = " + round(leftPower / fineDiff, 3) + ", R = " + round(rightPower / fineDiff, 3));
                telemetry.update();

                backLeft.setPower(leftPower * fineDiff);
                backRight.setPower(rightPower * fineDiff);
                frontLeft.setPower(leftPower * fineDiff);
                frontRight.setPower(rightPower * fineDiff);
            }
        }
    }


    public static double round(double value, int places) { //Allows telemetry to display nicely
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}