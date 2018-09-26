package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// PID - Proportional Integral Derivative

// Setpoint - value you are wishing to achieve - Ex. 300 RPM
// Process Value - value motor is running at - Ex. 298/307 RPM
// Error - difference of setpoint and process value - Ex. -2/7

// Search every hundredth of a second
// Use addition, if the process value is too low, add power and vice versa
// What should the tolerance be?
// Make 4 methods, 1 for each number of motors to run, such as cruiseOneMotor, cruiseTwoMotor, etc.

public abstract class CruiseControl {

    public ElapsedTime runtime = new ElapsedTime();
    int i;

    public double currentDesiredDistance = 0;

    public void cruiseOneMotor(int speed, int distance, DcMotor motor) { //Distance should be in full wheel rotations
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        distance = distance * 1120; //1120 encoder ticks per revolution of Classic NeveRest 40s

        final double DESIRED_SPEED_PER_MSECOND = (((125 / 60) / 1000) * speed); //125 is the reasonable RPM of a loaded NeveRest 40

        motor.setTargetPosition(distance);

        motor.setPower(speed);

        while (runtime.seconds() != i) {}

        currentDesiredDistance = currentDesiredDistance + DESIRED_SPEED_PER_MSECOND;

        if(motor.getCurrentPosition() < currentDesiredDistance) {
            motor.setPower(motor.getPower() + 1);
        } else if(motor.getCurrentPosition() > currentDesiredDistance) {
            motor.setPower(motor.getPower() - 1);
        }

        motor.setPower(0);

       motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}