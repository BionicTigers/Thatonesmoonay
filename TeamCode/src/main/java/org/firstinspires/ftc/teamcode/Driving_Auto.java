
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="Drive_auto", group="Pushbot")
public class Driving_Auto extends LinearOpMode {

    //Drivetrain Motors//
    private DcMotor backLeft; //left motor back
    private DcMotor frontLeft; //left motor front (Ladies first)ot
    private DcMotor backRight; //right motor back
    private DcMotor frontRight; //right motor front (Ladies first)
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    public Servo teamMarker; //Glyph Flipper
    //public Servo burr; //Glyph Flipper 2
    public ElapsedTime runtime = new ElapsedTime();
    int i;
    int ralph;
    boolean blue;
//    private Servo clark; //drop down servo (for color sensor)
//    private Servo eddie; //swing servo (for color sensor)
//    private ColorSensor roger; //right color sensor
//    private ColorSensor leo; //left color sensor
    private double waitTime;
    private int gameState;

    @Override
    public void runOpMode() throws InterruptedException{
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        backLeft = hardwareMap.dcMotor.get("backLeft"); //Left Back
        frontLeft = hardwareMap.dcMotor.get("frontLeft"); //Left Front
        backRight = hardwareMap.dcMotor.get("backRight"); //Right Back
        frontRight = hardwareMap.dcMotor.get("frontRight"); //Right Front
        teamMarker = hardwareMap.servo.get("teamMarker");

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);


                //blue = false;
                gameState = 0;
                waitTime = 0;


                backLeft.setDirection(DcMotor.Direction.REVERSE);
                frontLeft.setDirection(DcMotor.Direction.REVERSE);

                // Send telemetry message to signify robot waiting;
                telemetry.addData("Status", "Resetting Encoders");    //
                telemetry.update();

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                time = getRuntime();

                // Send telemetry message to indicate successful Encoder reset
                telemetry.addData("Path0", "Starting at %7d :%7d",
                        frontLeft.getCurrentPosition(),
                        backRight.getCurrentPosition(),
                        frontRight.getCurrentPosition(),
                        backLeft.getCurrentPosition());

                telemetry.update();

                waitForStart();

                while (opModeIsActive()) {
                    //MOVE

                    driveBackward(.5, 1000);
                    sleep(500);


                    pointTurnLeft(.9, 1000);


                    driveBackward(.5, 1000);
                    sleep(500);



                    driveBackward(.5, -1000);
                    sleep(250);

                    driveBackward(.5, 1000);
                    sleep(500);

                    driveBackward(.5, -1000);
                    sleep(500);

                    teamMarker.setPosition(.7);
                    sleep(500);

                    frontLeft.setPower(0);
                    backRight.setPower(0);
                    backLeft.setPower(0);
                    frontRight.setPower(0);
                    sleep(10000);

                    break;
                }
            }

//    String format(OpenGLMatrix transformationMatrix) {
//        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";


            public void driveForward(double power, int distance) {

                backRight.setTargetPosition(distance);
                frontRight.setTargetPosition(distance);
                backLeft.setTargetPosition(distance);
                frontLeft.setTargetPosition(distance);

                frontLeft.setPower(power);
                backRight.setPower(power);
                backLeft.setPower(power);
                frontRight.setPower(power);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (backRight.isBusy() && frontRight.isBusy() && backLeft.isBusy() && frontLeft.isBusy()) {
                    telemetry.addData("moving", " forward");
                }
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);

                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(500);
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(500);
            }

            public void driveBackward(double power, int distance) {

                backRight.setTargetPosition(-distance);
                frontRight.setTargetPosition(-distance);
                backLeft.setTargetPosition(-distance);
                frontLeft.setTargetPosition(-distance);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeft.setPower(-power);
                backRight.setPower(-power);
                backLeft.setPower(-power);
                frontRight.setPower(-power);

                while (frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy() && frontRight.isBusy()) {

                }
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);

                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(500);
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(500);
            }

            public void pointTurnRight(double power, int distance) {

                backRight.setTargetPosition(-distance);
                frontRight.setTargetPosition(-distance);
                frontLeft.setTargetPosition(distance);
                backLeft.setTargetPosition(distance);

                frontLeft.setPower(power);
                backRight.setPower(-power);
                backLeft.setPower(power);
                frontRight.setPower(-power);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy() && frontRight.isBusy()) {

                }
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);

                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(500);
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(500);
            }

            public void pointTurnLeft(double power, int distance) {

                backRight.setTargetPosition(distance);
                frontRight.setTargetPosition(distance);
                frontLeft.setTargetPosition(distance);
                backLeft.setTargetPosition(distance);

                frontLeft.setPower(-power);
                backRight.setPower(power);
                backLeft.setPower(-power);
                frontRight.setPower(power);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy() && frontRight.isBusy()) {

                }
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);

                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(500);
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(500);
            }


        }

