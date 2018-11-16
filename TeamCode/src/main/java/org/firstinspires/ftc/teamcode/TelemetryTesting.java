package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.webserver.RobotControllerWebHandlers;
@TeleOp(name= "CHRIS IS DUMB")

public class TelemetryTesting extends OpMode {
    private Navigation nav;
    private OpMode opMode;
    @Override
    public void init() {
        nav = new Navigation(this,telemetry,true);

    }

    @Override
    public void loop() {
        nav.updateCubePos();
        nav.getCubePos();
        nav.telemetryMethod();

    }
}
