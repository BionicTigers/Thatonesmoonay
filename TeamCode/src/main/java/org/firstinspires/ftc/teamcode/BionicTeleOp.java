package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp", group="BTBT")

public class BionicTeleOp extends OpMode {
    //Drivetrain Motors//
    private DcMotor motorLeone; //left motor back
    private DcMotor motorLannay; //left motor front (Ladies first)ot
    private DcMotor motorRigetony; //right motor back
    private DcMotor motorRachella; //right motor front (Ladies first)
    private DcMotor evangelino;

    //Variables//
    private double yValue;
    private double xValue;
    private double leftPower;
    private double rightPower;
    public int calibToggle;

    public void init() {
        //Motors//
        motorLeone = hardwareMap.dcMotor.get("motorLeone"); //Left Back
        motorLannay = hardwareMap.dcMotor.get("motorLannay"); //Left Front
        motorRigetony = hardwareMap.dcMotor.get("motorRigetony"); //Right Back
        motorRachella = hardwareMap.dcMotor.get("motorRachella"); //Right Front
        evangelino = hardwareMap.dcMotor.get("evangelino");

        motorRigetony.setDirection(DcMotor.Direction.REVERSE);
        motorRachella.setDirection(DcMotor.Direction.REVERSE);

        //Variables//
        calibToggle = 0; }

    public void loop() {

        if (gamepad1.a) {
            calibToggle = 1; //Tank
        } else if (gamepad1.b){
            calibToggle = 0; } //JoyStick

        if (calibToggle == 1)  { //A
            motorLeone.setPower(gamepad1.left_stick_y/2);    //Tank Drive
            motorLannay.setPower(gamepad1.left_stick_y/2);
            motorRigetony.setPower(gamepad1.right_stick_y/2);
            motorRachella.setPower(gamepad1.right_stick_y/2);
        } else if(calibToggle == 0) { //B
                yValue = gamepad1.left_stick_y;
                xValue = gamepad1.right_stick_x;

                leftPower =  yValue - xValue;
                rightPower = yValue + xValue;

                motorLeone.setPower(Range.clip(leftPower, -0.5, 0.5));
                motorRigetony.setPower(Range.clip(rightPower, -0.5, 0.5));
                motorLannay.setPower(Range.clip(leftPower, -0.5, 0.5));
                motorRachella.setPower(Range.clip(rightPower, -0.5, 0.5));

                telemetry.addData("Mode", "running");
                telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
                telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
                telemetry.update();
        }

        if (gamepad2.x) {
            evangelino.setPower(0.50);
        } else if (gamepad2.y) {
            evangelino.setPower(-0.50);
        } else {
            evangelino.setPower(0.00); }

    }
}