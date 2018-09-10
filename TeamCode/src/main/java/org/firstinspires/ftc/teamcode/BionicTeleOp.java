package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp", group="BTBT")

public class BionicTeleOp extends OpMode {
    //Motors//
    private DcMotor motorLeone; //left motor back (Ladies first)
    private DcMotor motorLannay; //left motor front
    private DcMotor motorRigetony; //right motor back (Ladies first
    private DcMotor motorRachella; //right motor front

    //Variables//
    private double yValue;
    private double xValue;
    private double leftPower;
    private double rightPower;
    public int calibToggle;
    public void init() {

        motorLeone = hardwareMap.dcMotor.get("motorLeone");
        //motorLannay = hardwareMap.dcMotor.get("motorLannay");
        motorRigetony = hardwareMap.dcMotor.get("motorRigetony");
        //motorRachella = hardwareMap.dcMotor.get("motorRachella");
        motorRigetony.setDirection(DcMotor.Direction.REVERSE);
        calibToggle = 0; }

    public void loop() {

        if (gamepad1.a) {
            calibToggle = 1;
        } else if (gamepad1.b){
            calibToggle = 0; }

        if (calibToggle == 1)  {
            motorLeone.setPower(gamepad1.left_stick_y / 2);    //Tank Drive
            //motorLannay.setPower(gamepad1.left_stick_y);
            motorRigetony.setPower(gamepad1.right_stick_y / 2);
            //motorRachella.setPower(gamepad1.right_stick_y);
        } else if(calibToggle == 0) {
                yValue = gamepad1.left_stick_y;
                xValue = gamepad1.right_stick_x;

                leftPower =  yValue - xValue;
                rightPower = yValue + xValue;

                motorLeone.setPower(Range.clip(leftPower, -0.5, 0.5));
                motorRigetony.setPower(Range.clip(rightPower, -0.5, 0.5));

                telemetry.addData("Mode", "running");
                telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
                telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
                telemetry.update();
        }
    }
}