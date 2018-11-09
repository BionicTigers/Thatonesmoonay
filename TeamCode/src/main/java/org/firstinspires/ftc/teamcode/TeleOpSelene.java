package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import java.math.BigDecimal;
import java.math.RoundingMode;

import static java.lang.Math.round;


@TeleOp(name="TeleOp Selene", group="TeleOp")
public class TeleOpSelene extends OpMode {

    //Drivetrain Motors//
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //Other Motors//
    private DcMotor extendy;
    private DcMotor lifty;
    private DcMotor liftyJr;

    //Servos//
    private Servo liftyLock;
    private CRServo collecty;
    private CRServo trappy;
    private Servo droppy;
    private Servo droppyJr;
    private Servo teamMarker;

    //Variables//
    private double leftPower, rightPower;
    private double leftStick, rightStick, gasPedal;
    private double coarseDiff, fineDiff;
    private double calibToggle;
    private int driveSpeed, driveMode;
    private double trappyJo;
    private int bvariable;
    private int trappyJoJo;
    //Objects//
    public ElapsedTime runtime = new ElapsedTime();


    public void init() {
        //Drivetrain Motors//
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        //Other Motors//
        extendy = hardwareMap.dcMotor.get("extendy");
        lifty = hardwareMap.dcMotor.get("lifty");
        liftyJr = hardwareMap.dcMotor.get("liftyJr");
        lifty.setDirection(DcMotor.Direction.REVERSE);

        trappyJo = 0;
        trappyJoJo = 0;
        //bvariable = 0;

        //Servos//
        liftyLock = hardwareMap.servo.get("liftyLock");

        trappy = hardwareMap.crservo.get("trappy");
        collecty = hardwareMap.crservo.get("collecty");
        droppy = hardwareMap.servo.get("droppy");
        droppyJr = hardwareMap.servo.get("droppyJr");
        teamMarker = hardwareMap.servo.get("teamMarker");

        droppyJr.setDirection(Servo.Direction.REVERSE);

        //Variables//
        calibToggle = 0;
        driveSpeed = 0;
        driveMode = 0;

        //Speed Offsets//
        coarseDiff = .6;
        fineDiff = .3;
    }


    public void loop() {
        //////////////////////////////////////// GAMEPAD 1 /////////////////////////////////////////
        // TOGGLE BUTTONS //
        if (gamepad1.a && (runtime.seconds() > calibToggle)) {
            calibToggle = runtime.seconds() + 1;
            ++driveSpeed;
        }
        if (gamepad1.x) {
            driveMode = 0;
        }
        if (gamepad1.y) {
            driveMode = 1;
        }
        if (gamepad1.b) {
            driveMode = 2;
        }

        bvariable = 1;
        // DIFFERENT DRIVE MODES //
        if (driveMode == 0) {
            // ARCADE DRIVE //
            leftStick = gamepad1.left_stick_y;
            rightStick = -gamepad1.right_stick_x;

            //Left Side
            if (Math.abs(rightStick) > 0.5) {
                leftPower = leftStick / 2 + rightStick / 2;
            } else {
                leftPower = leftStick + rightStick / 2;
            }

            //Right Side
            if (Math.abs(rightStick) > 0.5) {
                rightPower = leftStick / 2 - rightStick / 2;
            } else {
                rightPower = leftStick - rightStick / 2;
            }

            if (driveSpeed % 2 == 0) {
                telemetry.addData("Mode: ", "ARCADE");
                telemetry.addData("Speed: ", "NORMAL");
                telemetry.addData("Stick: ", "X = " + round(rightStick) + ", Y = " + round(leftStick));
                telemetry.addData("Power: ", "L = " + round(leftPower) + ", R = " + round(rightPower));

                backLeft.setPower(leftPower * coarseDiff);
                backRight.setPower(rightPower * coarseDiff);
                frontLeft.setPower(leftPower * coarseDiff);
                frontRight.setPower(rightPower * coarseDiff);
            } else {
                telemetry.addData("Mode: ", "ARCADE");
                telemetry.addData("Speed: ", "SLOW");
                telemetry.addData("Stick: ", "X = " + round(rightStick) + ", Y = " + round(leftStick));
                telemetry.addData("Power: ", "L = " + round(leftPower) + ", R = " + round(rightPower));

                backLeft.setPower(leftPower * fineDiff);
                backRight.setPower(rightPower * fineDiff);
                frontLeft.setPower(leftPower * fineDiff);
                frontRight.setPower(rightPower * fineDiff);
            }
        } else if (driveMode == 1) {
            /// TANK DRIVE ///
            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;

            if (driveSpeed % 2 == 0) {
                telemetry.addData("Mode: ", "TANK");
                telemetry.addData("Speed: ", "NORMAL");
                telemetry.addData("Stick: ", "X = " + round(rightStick) + ", Y = " + round(leftStick));
                telemetry.addData("Power: ", "L = " + round(leftPower) + ", R = " + round(rightPower));

                backLeft.setPower(leftPower * coarseDiff);
                backRight.setPower(rightPower * coarseDiff);
                frontLeft.setPower(leftPower * coarseDiff);
                frontRight.setPower(rightPower * coarseDiff);
            } else {
                telemetry.addData("Mode: ", "TANK");
                telemetry.addData("Speed: ", "SLOW");
                telemetry.addData("Stick: ", "X = " + round(rightStick) + ", Y = " + round(leftStick));
                telemetry.addData("Power: ", "L = " + round(leftPower) + ", R = " + round(rightPower));

                backLeft.setPower(leftPower * fineDiff);
                backRight.setPower(rightPower * fineDiff);
                frontLeft.setPower(leftPower * fineDiff);
                frontRight.setPower(rightPower * fineDiff);
            }
        } else if (driveMode == 2) {
            /// ACKERMAN DRIVE ///
            leftStick = (-gamepad1.left_stick_x);

            gasPedal = (gamepad1.left_trigger - gamepad1.right_trigger);

            //Left Side
            if (Math.abs(rightStick) > 0.5) {
                leftPower = gasPedal / 2 + rightStick / 2;
            } else {
                leftPower = gasPedal + rightStick / 2;
            }

            //Right Side
            if (Math.abs(rightStick) > 0.5) {
                rightPower = gasPedal / 2 - rightStick / 2;
            } else {
                rightPower = gasPedal - rightStick / 2;
            }

            if (driveSpeed % 2 == 0) {
                telemetry.addData("Mode: ", "ACKERMAN");
                telemetry.addData("Speed: ", "NORMAL");
                telemetry.addData("Stick: ", "X = " + round(leftStick) + ", G: " + round(gasPedal));
                telemetry.addData("Power: ", "L = " + round(leftPower) + ", R = " + round(rightPower));

                backLeft.setPower(leftPower * coarseDiff);
                backRight.setPower(rightPower * coarseDiff);
                frontLeft.setPower(leftPower * coarseDiff);
                frontRight.setPower(rightPower * coarseDiff);
            } else {
                telemetry.addData("Mode: ", "ACKERMAN");
                telemetry.addData("Speed: ", "SLOW");
                telemetry.addData("Stick: ", "L = " + round(leftStick) + ", G: " + round(gasPedal));
                telemetry.addData("Power: ", "L = " + round(leftPower) + ", R = " + round(rightPower));

                backLeft.setPower(leftPower * fineDiff);
                backRight.setPower(rightPower * fineDiff);
                frontLeft.setPower(leftPower * fineDiff);
                frontRight.setPower(rightPower * fineDiff);
            }
        }

        //////////////////////////////////////// GAMEPAD 2 /////////////////////////////////////////
        //Lift// - LeftStickUp= Lift Up | LeftStickDown= Lift Down
        lifty.setPower(gamepad2.left_stick_y / 2);
        liftyJr.setPower(gamepad2.left_stick_y / 2);
        telemetry.addData("Lift", lifty.getCurrentPosition() + "/" + liftyJr.getCurrentPosition());

        //Lift Lock// - DPadUp= Lock Lift | DPadDown= Unlock Lift
        if (gamepad2.dpad_up) {
            liftyLock.setPosition(0.7);
        } else if (gamepad2.dpad_down) {
            liftyLock.setPosition(0.2);
        }

//        //Team Marker Deployer// - DPadRight= Deploy | DPadLeft= Retract
//        if (gamepad2.dpad_right) {
//            flicky.setPosition(0.3);
//        } else if (gamepad2.dpad_left) {
//            flicky.setPosition(0.65);
//        }
        if (gamepad1.dpad_left) { //
            teamMarker.setPosition(0.3);

            //collecty.setPosition(collecty.getPosition() + 0.1);
        } else if (gamepad1.right_trigger > 0.5) {
            teamMarker.setPosition(0.7);

            //Collector// - A= Intake | B= Outtake //vexmotor
            if (gamepad2.right_bumper) { //
                collecty.setPower(0.5);

                //collecty.setPosition(collecty.getPosition() + 0.1);
            } else if (gamepad2.right_trigger > 0.5) {
                collecty.setPower(-0.5);
                //collecty.setPosition(collecty.getPosition() - 0.1);
            } else {
                collecty.setPower(0);
            }
            telemetry.addData("Collector power", collecty.getPower());

            //Hopper Storage Gate// - X= Open | x= Close //vexmotor
            if (gamepad2.x && (runtime.seconds() > trappyJo)) {
                trappy.setPower(-0.5);
                trappyJo = runtime.seconds();
                trappyJoJo = 0;
            } else if (gamepad2.x && (runtime.seconds() > trappyJo)) {
                trappy.setPower(0.5);
                trappyJo = runtime.seconds() + 1;
                trappyJoJo = 1;
            } else {
                trappy.setPower(0);
            }
            telemetry.addData("Trapdoor", trappy.getPower());

            //Collection Extension motor// - LeftBumper= Deploy | LeftTrigger= Retract
            if (gamepad2.left_bumper) {
                extendy.setPower(-1);
            } else if (gamepad2.left_trigger > 0.05) {
                extendy.setPower(1);
            } else {
                extendy.setPower(0);
            }
            telemetry.addData("Extension", extendy.getCurrentPosition());

            //Collector Dropper// - RightBumper= Drop Dropper | LeftBumper= Lift Dropper
//        if (gamepad2.y) {
//            droppy.setPosition(droppy.getPosition() + 0.05);
//            droppyJr.setPosition(droppyJr.getPosition() + 0.05);
//        } else if (gamepad2.a) {
//            droppy.setPosition(droppy.getPosition() - 0.05);
//            droppyJr.setPosition(droppyJr.getPosition() - 0.05);
//        } else if (gamepad2.b){
//            droppy.setPosition(droppy.getPosition() - 0.05);
//            droppyJr.setPosition(droppyJr.getPosition() - 0.05);
//        }
            if (gamepad2.y) {
                droppy.setPosition(0.0);
                droppyJr.setPosition(0.0);
            } else if (gamepad2.b) {
                droppy.setPosition(0.8);
                droppyJr.setPosition(0.55);
            } else if (gamepad2.a) {
                droppy.setPosition(0.9);
                droppyJr.setPosition(0.9);
            }
            telemetry.addData("Collector Drop", droppy.getPosition() + "/" + droppyJr.getPosition());

            telemetry.update();
        }


//    private static double Math.round(double value) { //Allows telemetry to display nicely
//        BigDecimal bd = new BigDecimal(value);
//        bd = bd.setScale(3, RoundingMode.HALF_UP);
//        return bd.doubleValue();
//    }
    }}