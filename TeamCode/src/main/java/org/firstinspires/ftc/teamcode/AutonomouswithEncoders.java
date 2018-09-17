package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="EncodersAuto", group ="red")

public class AutonomouswithEncoders extends LinearOpMode {

    public DcMotor motorLannay;
    public DcMotor motorRigetony;
    public DcMotor motorRachella;
    public DcMotor motorLeone;
    public ElapsedTime runtime = new ElapsedTime();
    int i;
    int ralph; // distance variable for vuforia
    private double waitTime;
    private int gameState;
    BNO055IMU imu;
    public Orientation angles;

    @Override
    public void runOpMode() {
        gameState = 0;
        waitTime = 0;

        motorLannay = hardwareMap.dcMotor.get("frontLeft");
        motorRigetony = hardwareMap.dcMotor.get("backRight");
        motorRachella = hardwareMap.dcMotor.get("frontRight");
        motorLeone = hardwareMap.dcMotor.get("backLeft");

        motorRachella.setDirection(DcMotor.Direction.REVERSE);
        motorRigetony.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        motorLannay.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRigetony.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRachella.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeone.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLannay.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRigetony.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRachella.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeone.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        time = getRuntime();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                motorLannay.getCurrentPosition(),
                motorRigetony.getCurrentPosition(),
                motorRachella.getCurrentPosition(),
                motorLeone.getCurrentPosition());

        telemetry.update();

        while (opModeIsActive()) {

                //MOVE

                driveBackward(.5, 250);
                pointTurnRight(.5, 150);
                //driveForward(.5, 300);

                sleep(500);

                stop();
                break;
            }
        }

    public void driveForward(double power, int distance) {

        motorRigetony.setTargetPosition(distance);
        motorRachella.setTargetPosition(distance);
        motorLeone.setTargetPosition(distance);
        motorLannay.setTargetPosition(distance);

        motorLannay.setPower(power);
        motorRigetony.setPower(power);
        motorLeone.setPower(power);
        motorRachella.setPower(power);

        motorLannay.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRigetony.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLannay.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRachella.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motorRigetony.isBusy() && motorRachella.isBusy() && motorLeone.isBusy() && motorLannay.isBusy()) {
        }
        motorLannay.setPower(0);
        motorRigetony.setPower(0);
        motorLeone.setPower(0);
        motorRachella.setPower(0);

        motorLannay.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRigetony.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRachella.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeone.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
        motorLannay.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRigetony.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRachella.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeone.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
    }

    public void driveBackward(double power, int distance) {

        motorRigetony.setTargetPosition(-distance);
        motorRachella.setTargetPosition(-distance);
        motorLeone.setTargetPosition(-distance);
        motorLannay.setTargetPosition(-distance);

        motorLannay.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRigetony.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeone.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRachella.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLannay.setPower(-power*.65);
        motorRigetony.setPower(-power *1.35);
        motorLeone.setPower(-power *.65);
        motorRachella.setPower(-power *1.35);

        while (motorLannay.isBusy() && motorRigetony.isBusy() && motorLeone.isBusy() && motorRachella.isBusy()) {

        }
        motorLannay.setPower(0);
        motorRigetony.setPower(0);
        motorLeone.setPower(0);
        motorRachella.setPower(0);

        motorLannay.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRigetony.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRachella.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeone.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
        motorLannay.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRigetony.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRachella.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeone.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
    }

    public void pointTurnRight(double power, int distance) {

        motorRigetony.setTargetPosition(-distance);
        motorRachella.setTargetPosition(-distance);
        motorLannay.setTargetPosition(distance);
        motorLeone.setTargetPosition(distance);

        motorLannay.setPower(power);
        motorRigetony.setPower(-power);
        motorLeone.setPower(power);
        motorRachella.setPower(-power);

        motorLannay.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRigetony.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeone.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRachella.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motorLannay.isBusy() && motorRigetony.isBusy() && motorLeone.isBusy() && motorRachella.isBusy()) {

        }
        motorLannay.setPower(0);
        motorRigetony.setPower(0);
        motorLeone.setPower(0);
        motorRachella.setPower(0);

        motorLannay.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRigetony.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRachella.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeone.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
        motorLannay.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRigetony.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRachella.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeone.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
    }

    public void pointTurnLeft(double power, int distance) {

        motorRigetony.setTargetPosition(distance);
        motorRachella.setTargetPosition(distance);
        motorLannay.setTargetPosition(distance);
        motorLeone.setTargetPosition(distance);

        motorLannay.setPower(-power);
        motorRigetony.setPower(power);
        motorLeone.setPower(-power);
        motorRachella.setPower(power);

        motorLannay.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRigetony.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeone.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRachella.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motorLannay.isBusy() && motorRigetony.isBusy() && motorLeone.isBusy() && motorRachella.isBusy()) {

        }
        motorLannay.setPower(0);
        motorRigetony.setPower(0);
        motorLeone.setPower(0);
        motorRachella.setPower(0);

        motorLannay.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRigetony.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRachella.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeone.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
        motorLannay.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRigetony.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRachella.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeone.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
    }


    public void strafeLeft(double power, int distance) {

        motorRigetony.setTargetPosition(distance);
        motorRachella.setTargetPosition(distance);
        motorLannay.setTargetPosition(distance);
        motorLeone.setTargetPosition(distance);

        motorLannay.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRigetony.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeone.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRachella.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLannay.setPower(-power);
        motorRigetony.setPower(power);
        motorLeone.setPower(-power);
        motorRachella.setPower(power);

        while (motorLannay.isBusy() && motorRigetony.isBusy() && motorLeone.isBusy() && motorRachella.isBusy()) {

        }
        motorLannay.setPower(0);
        motorRigetony.setPower(0);
        motorLeone.setPower(0);
        motorRachella.setPower(0);

        motorLannay.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRigetony.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRachella.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeone.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
        motorLannay.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRigetony.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRachella.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeone.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
    }


    public void strafeRight(double power, int distance) {
        motorRigetony.setTargetPosition(distance);
        motorRachella.setTargetPosition(distance);
        motorLannay.setTargetPosition(-distance);
        motorLeone.setTargetPosition(-distance);

        motorLannay.setPower(power);
        motorRigetony.setPower(power);
        motorLeone.setPower(-power);
        motorRachella.setPower(-power);

        motorLannay.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRigetony.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeone.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRachella.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motorLannay.isBusy() && motorRigetony.isBusy() && motorLeone.isBusy() && motorRachella.isBusy()) {

        }
        motorLannay.setPower(0);
        motorRigetony.setPower(0);
        motorLeone.setPower(0);
        motorRachella.setPower(0);

        motorLannay.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRigetony.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRachella.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeone.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
        motorLannay.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRigetony.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRachella.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeone.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
    }

}


