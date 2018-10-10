package org.firstinspires.ftc.teamcode;
//EXIST
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp", group="BTBT")

public class BionicTeleOp extends OpMode {
    //Drivetrain Motors//
    private DcMotor backLeft; //left motor back
    //private DcMotor frontLeft; //left motor front (Ladies first)ot
    private DcMotor backRight; //right motor back
    //private DcMotor frontRight; //right motor front (Ladies first)
    private DcMotor collector;
    private DcMotor evangelino;
    private Servo teamMarker;
    private Servo flickyWrist;
///////
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
        //frontLeft = hardwareMap.dcMotor.get("frontLeft"); //Left Front
        backRight = hardwareMap.dcMotor.get("backRight"); //Right Back
        //frontRight = hardwareMap.dcMotor.get("frontRight"); //Right Front
        evangelino = hardwareMap.dcMotor.get("lift");
        //collector = hardwareMap.dcMotor.get("collector");
        teamMarker = hardwareMap.servo.get("teamMarker");
        backRight.setDirection(DcMotor.Direction.REVERSE);
        liftrawrh = hardwareMap.servo.get("liftrawrh");
        //frontRight.setDirection(DcMotor.Direction.REVERSE);
        flickyWrist = hardwareMap.servo.get("flicky");
        collector = hardwareMap.dcMotor.get("collector");

        //Variables//
        calibToggle = 0;
        int target = 0;
        double speed = 0;}

    public void loop() {

        //robot = new Hardware_Pushbot();
        if (gamepad1.a) {
            calibToggle = 1; //One stick
        } else if (gamepad1.b) {
            calibToggle = 0;
        } //JoyStick

        if (calibToggle == 0) { //A
            yValue = gamepad1.left_stick_y;
            xValue = gamepad1.right_stick_x;

            leftPower = yValue - xValue;
            rightPower = yValue + xValue;

            backLeft.setPower(Range.clip(leftPower, -0.5, 0.5));
            backRight.setPower(Range.clip(rightPower, -0.5, 0.5));
            //frontLeft.setPower(Range.clip(leftPower, -0.6, 0.6));
            //frontRight.setPower(Range.clip(rightPower, -0.6, 0.6));

            telemetry.addData("Mode", "running");
            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
            telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
            telemetry.update();
//            backLeft.setPower(gamepad1.left_stick_y / 2);    //Tank Drive
//            frontLeft.setPower(gamepad1.left_stick_y / 2);
//            backRight.setPower(gamepad1.right_stick_y / 2);
//            frontRight.setPower(gamepad1.right_stick_y / 2);
        } else if (calibToggle == 1) { //B
            yValue = gamepad1.left_stick_y;
            xValue = gamepad1.right_stick_x;

            leftPower = yValue - xValue;
            rightPower = yValue + xValue;

            backLeft.setPower(Range.clip(leftPower, -0.6, 0.6));
            backRight.setPower(Range.clip(rightPower, -0.6, 0.6));
            //frontLeft.setPower(Range.clip(leftPower, -0.6, 0.6));
            //frontRight.setPower(Range.clip(rightPower, -0.6, 0.6));

            telemetry.addData("Mode", "running");
            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
            telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
            telemetry.update();
        }
        //
        evangelino.setPower(-gamepad2.right_stick_y/2);

        if (gamepad2.dpad_up) {
            collector.setPower(0.60);
        } else if (gamepad2.dpad_down) {
            collector.setPower(-0.60);
        } else {
            collector.setPower(0.00);
        }


        if (gamepad2.right_bumper) { //rightb - up righttrigger down
            liftrawrh.setPosition(0.3);
        } else if (gamepad2.right_trigger > 0.7) {
            liftrawrh.setPosition(1.0);
        }

        //y-up b- a-
        if (gamepad2.y) { //rightb - up righttrigger down
            flickyWrist.setPosition(0.3);
        } else if (gamepad2.b) {
            flickyWrist.setPosition(0.62);
        }
            else if (gamepad2.a) {
            flickyWrist.setPosition(0.65);

        }
        }

    }