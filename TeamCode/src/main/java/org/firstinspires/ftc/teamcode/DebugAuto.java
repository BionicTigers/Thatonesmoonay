package org.firstinspires.ftc.teamcode;
//EXIST
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Debug Autonomous", group ="test")
public class DebugAuto extends LinearOpMode {

    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    @Override public void runOpMode() {
        //initialization
        Navigation nav = new Navigation(this,telemetry, true,true, true,true);

        waitForStart();

        while(!nav.updateCubePos()) {
        }

        telemetry.addData("pos",nav.getCubePos());

        sleep(5000);

    }


}
