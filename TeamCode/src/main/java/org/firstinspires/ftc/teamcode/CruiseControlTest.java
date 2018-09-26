package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// PID - Proportional Integral Derivative


// Setpoint - value you are wishing to achieve - Ex. 300 RPM
// Process Value - value motor is running at - Ex. 298/307 RPM
// Error - difference of setpoint and process value - Ex. -2/7

@TeleOp(name="CruiseTest", group ="summerProjects")

public abstract class CruiseControlTest extends OpMode {

    public DcMotor motorFrontLeft;
//    public DcMotor motorBackRight;
//    public DcMotor motorFrontRight;
//    public DcMotor motorBackLeft;

    public ElapsedTime runtime = new ElapsedTime();
    int i;

    public static double currentDesiredDistance = 0;

    public void init() {

        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
//        motorBackRight = hardwareMap.dcMotor.get("backRight");
//        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
//        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
//
//        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
//        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

//    public void loop() {
//        if(gamepad1.a) {
//            setCruiseControl(50, (int)(-gamepad1.left_stick_y * 100));
//            i =  (int)runtime.seconds()+1;
//        }
//    }

    public void setCruiseControl(int speed, int distance) { //Distance should be in full wheel rotations
        distance = distance * 1120; //1120 encoder ticks per revolution of Classic NeveRest 40s

        final double desiredDistancePerSecond = (((125 / 60) / 100) * speed); //125 is the reasonable RPM of a loaded NeveRest 40

        motorFrontLeft.setTargetPosition(distance);
//        motorBackRight.setTargetPosition(distance);
//        motorFrontRight.setTargetPosition(distance);
//        motorBackLeft.setTargetPosition(distance);

        motorFrontLeft.setPower(speed);
//        motorBackRight.setPower(speed);
//        motorFrontRight.setPower(speed);
//        motorBackLeft.setPower(speed);


        Runnable cruiseCheck = new Runnable() {
            public void run() {



            }
        };

        while ( (double)runtime.seconds()!=i) {}
            currentDesiredDistance = currentDesiredDistance + desiredDistancePerSecond;

            if(motorFrontLeft.getCurrentPosition() < currentDesiredDistance) {
                motorFrontLeft.setPower(motorFrontLeft.getPower() + ((currentDesiredDistance - motorFrontLeft.getCurrentPosition()) / 50));
            } else if(motorFrontLeft.getCurrentPosition() > currentDesiredDistance) {
                motorFrontLeft.setPower(motorFrontLeft.getPower() - ((motorFrontLeft.getCurrentPosition() - currentDesiredDistance) / 50));
            }
//            if(motorBackRight.getCurrentPosition() < currentDesiredDistance) {
//                motorBackRight.setPower(motorBackRight.getPower() + ((currentDesiredDistance - motorBackRight.getCurrentPosition()) / 50));
//            } else if(motorBackRight.getCurrentPosition() > currentDesiredDistance) {
//                motorBackRight.setPower(motorBackRight.getPower() - ((motorBackRight.getCurrentPosition() - currentDesiredDistance) / 50));
//            }
//
//            if(motorFrontRight.getCurrentPosition() < currentDesiredDistance) {
//                motorFrontRight.setPower(motorFrontRight.getPower() + ((currentDesiredDistance - motorFrontRight.getCurrentPosition()) / 50));
//            } else if(motorFrontRight.getCurrentPosition() > currentDesiredDistance) {
//                motorFrontRight.setPower(motorFrontRight.getPower() - ((motorFrontRight.getCurrentPosition() - currentDesiredDistance) / 50));
//            }
//
//            if(motorBackLeft.getCurrentPosition() < currentDesiredDistance) {
//                motorBackLeft.setPower(motorBackLeft.getPower() + ((currentDesiredDistance - motorBackLeft.getCurrentPosition()) / 50));
//            } else if(motorBackLeft.getCurrentPosition() > currentDesiredDistance) {
//                motorBackLeft.setPower(motorBackLeft.getPower() - ((motorBackLeft.getCurrentPosition() - currentDesiredDistance) / 50));
//            }



        motorFrontLeft.setPower(0);
//        motorBackRight.setPower(0);
//        motorBackLeft.setPower(0);
//        motorFrontRight.setPower(0);
//
       motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 }
}