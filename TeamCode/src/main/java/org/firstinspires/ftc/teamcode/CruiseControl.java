package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// PID - Proportional Integral Derivative

// Setpoint - value you are wishing to achieve - Ex. 300 RPM
// Process Value - value motor is running at - Ex. 298/307 RPM
// Error - difference of setpoint and process value - Ex. -2/7

// Search every hundredth of a second
// Use addition, if the process value is too low, add power and vice versa
// What should the tolerance be? 100? maybe it should depend on the motor.
// Make 4 methods, 1 for each number of motors to run, such as cruiseOneMotor, cruiseTwoMotor, etc.
// How do we accomodate for all the different motors there are?

@Autonomous(name="CruiseControl", group="Cruisin")

public class CruiseControl extends OpMode{

    public DcMotor frontLeft; //left motor back

    public ElapsedTime runtime = new ElapsedTime();
    double i;
    double w;

    public double currentDesiredDistance;

    public void init() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft"); //Left Back

        i = 0;

        currentDesiredDistance = 0;
    }

    public void loop(){
        cruiseOneMotor(75, 100, frontLeft);
        w = runtime.seconds() + 100;
        while(w != runtime.seconds());
    }

    public void cruiseOneMotor(int speed, int distance, DcMotor motor) { //Distance should be in full wheel rotations
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int futureDistance = distance * 1120; //1120 encoder ticks per revolution of Classic NeveRest 40s

        final double DESIRED_SPEED_PER_CSECOND = (((125 / 60) / 100) * speed); //125 is the reasonable RPM of a loaded NeveRest 40

        motor.setTargetPosition(distance);

        motor.setPower(speed);

        while (motor.getCurrentPosition() != futureDistance) {
            i = runtime.seconds() + 0.01;

            while (runtime.seconds() != i);

            currentDesiredDistance = currentDesiredDistance + DESIRED_SPEED_PER_CSECOND; // CSECOND = CentiSecond

            if (Math.abs(motor.getCurrentPosition() - currentDesiredDistance) > 100) { // Encoder tolerance is set at 100 ticks
                if (motor.getCurrentPosition() < currentDesiredDistance) {
                    motor.setPower(motor.getPower() + 1);
                } else if (motor.getCurrentPosition() > currentDesiredDistance) {
                    motor.setPower(motor.getPower() - 1);
                }
            }

            telemetry.addData(motor + "Power", motor.getPower());
            telemetry.update();
        }

        motor.setPower(0);

       motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}