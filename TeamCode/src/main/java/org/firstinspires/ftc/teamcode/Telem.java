package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "telem")
public class Telem extends OpMode {
    private Navigation nav;
    @Override
    public void init() {
        nav = new Navigation(this, telemetry,true);

    }

    @Override
    public void loop() {
        nav.telemetryMethod();
        nav.updateCubePos();

    }
}
//RAWRHHHH