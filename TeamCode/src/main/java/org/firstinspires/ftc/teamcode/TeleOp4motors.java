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
        private double yValue;
        private double xValue;
        private double leftPower;
        private double rightPower;
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
                yValue = gamepad1.left_stick_y;
                xValue = gamepad1.right_stick_x;

                leftPower = yValue - xValue;
                rightPower = yValue + xValue;

                backLeft.setPower(Range.clip(leftPower, -0.5, 0.5));
                backRight.setPower(Range.clip(rightPower, -0.5, 0.5));
                frontLeft.setPower(Range.clip(leftPower, -0.5, 0.5));
                frontRight.setPower(Range.clip(rightPower, -0.5, 0.5));

                telemetry.addData("Mode", "running");
                telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
                telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
                telemetry.update();

            } else if (calibToggle == 1) { //B
            backLeft.setPower(gamepad1.left_stick_y / 2);    //Tank Drive
            frontLeft.setPower(gamepad1.left_stick_y / 2);
            backRight.setPower(gamepad1.right_stick_y / 2);
            frontRight.setPower(gamepad1.right_stick_y / 2);
            }

        }}

