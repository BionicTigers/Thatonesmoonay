package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="4Motors", group="BTBT")
public class TeleOp4motors extends OpMode {

        //Drivetrain Motors//
        private DcMotor backLeft; //left motor back
        private DcMotor frontLeft; //left motor front (Ladies first)ot
        private DcMotor backRight; //right motor back
        private DcMotor frontRight; //right motor front (Ladies first)
        private DcMotor collector;
        private DcMotor evangelino;
        private Servo teamMarker;

        //Variables//
        private double xValue, yValue, lValue, rValue, leftPower, rightPower;
        public int calibToggle;
        public int target;
        private double speed;
        private Servo liftrawrh;

        //HardwareCatBot robot;


        public void init() {
            //Motors//
            backLeft = hardwareMap.dcMotor.get("backLeft"); //Left Back
            frontLeft = hardwareMap.dcMotor.get("frontLeft"); //Left Front
            backRight = hardwareMap.dcMotor.get("backRight"); //Right Back
            frontRight = hardwareMap.dcMotor.get("frontRight"); //Right Front
//            evangelino = hardwareMap.dcMotor.get("lift");
//            //collector = hardwareMap.dcMotor.get("collector");
//            teamMarker = hardwareMap.servo.get("teamMarker");
            backLeft.setDirection(DcMotor.Direction.REVERSE);
//            liftrawrh = hardwareMap.servo.get("liftrawrh");
            frontLeft.setDirection(DcMotor.Direction.REVERSE);

            //Variables//
            calibToggle = 0;
            int target = 0;
            double speed = 0;}

        public void loop() {

            //robot = new Hardware_Pushbot();
            if (gamepad1.a) {
                calibToggle = 1;
            } else if (gamepad1.b) {
                calibToggle = 0;
            }

            if (calibToggle == 0) { //A
                yValue = (gamepad1.left_stick_y);
                xValue = (gamepad1.left_stick_x * .75);

                leftPower = Math.pow(yValue - xValue, 3);
                rightPower = Math.pow(yValue + xValue, 3);

                backLeft.setPower(Range.clip(leftPower, -0.6, 0.6));
                backRight.setPower(Range.clip(rightPower, -0.6, 0.6));
                frontLeft.setPower(Range.clip(leftPower, -0.6, 0.6));
                frontRight.setPower(Range.clip(rightPower, -0.6, 0.6));

                telemetry.addData("Mode: ", "Coarse");
                telemetry.addData("Stick: ", "Y = " + yValue + ", X = " + xValue / .75);
                telemetry.addData("Power: ", "L = " + leftPower + ", R = " + rightPower);
                telemetry.update();

            } else if (calibToggle == 1) { //B
                yValue = (gamepad1.left_stick_y);
                xValue = (gamepad1.left_stick_x * .75);

                leftPower = Math.pow(yValue - xValue, 3) * .75;
                rightPower = Math.pow(yValue + xValue, 3) * .75;

                backLeft.setPower(Range.clip(leftPower, -0.3, 0.3));
                backRight.setPower(Range.clip(rightPower, -0.3, 0.3));
                frontLeft.setPower(Range.clip(leftPower, -0.3, 0.3));
                frontRight.setPower(Range.clip(rightPower, -0.3, 0.3));

                telemetry.addData("Mode: ", "Fine");
                telemetry.addData("Stick: ", "Y = " + yValue + ", X = " + xValue / .75);
                telemetry.addData("Power: ", "L = " + leftPower / .75 + ", R = " + rightPower / .75);
                telemetry.update();
//                lValue = Math.pow((gamepad1.left_stick_y * .6), 3);
//                rValue = Math.pow((gamepad1.right_stick_y * .6), 3);
//
//                backLeft.setPower(lValue);    //Tank Drive
//                frontLeft.setPower(lValue);
//                backRight.setPower(rValue);
//                frontRight.setPower(rValue);
//
//                telemetry.addData("Mode: ", "Fine");
//                telemetry.addData("Power: ", "L = " + leftPower + ", R = " + rightPower);
//                telemetry.update();
            }

        }}

