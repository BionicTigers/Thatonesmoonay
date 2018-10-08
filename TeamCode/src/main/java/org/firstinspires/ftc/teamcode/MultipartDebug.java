package org.firstinspires.ftc.teamcode;
//EXIST
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Debugger TeleOp", group="BTBT")
public class MultipartDebug extends OpMode   {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    //Variables//
    boolean wheelEncoders = true;

    public void init() {
        //Motors//
        frontLeft = hardwareMap.dcMotor.get("frontLeft"); //Left Back
        backLeft = hardwareMap.dcMotor.get("backLeft"); //Left Front
        frontRight = hardwareMap.dcMotor.get("frontRight"); //Right Back
        backRight = hardwareMap.dcMotor.get("backRight"); //Right Front

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop() {
        if(gamepad1.a) wheelEncoders = !wheelEncoders;


        String enabled = "";
        if(wheelEncoders) enabled += "Wheel Encoders,  ";

        telemetry.addData("Current: ",enabled);

        if(wheelEncoders) {
            telemetry.addData("Left Front", frontLeft.getCurrentPosition());
            telemetry.addData("Left Back", backLeft.getCurrentPosition());
            telemetry.addData("Right Front", frontRight.getCurrentPosition());
            telemetry.addData("Right Back", backRight.getCurrentPosition());
        }

    }
}
