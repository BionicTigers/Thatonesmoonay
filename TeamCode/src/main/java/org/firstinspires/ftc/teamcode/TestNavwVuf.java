package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "thing")
public class TestNavwVuf extends OpMode {
    private Navigation nav;
    @Override
    public void init() {
        nav = new Navigation(this, telemetry, true, false);


    }

    @Override
    public void loop() {
    nav.updatePos();
    telemetry.addData("Position", nav.getPos());
    }
}

