package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.webserver.RobotControllerWebHandlers;

public class AutoGeneric{

    public static enum StartPos {DEPOT, CRATER};
    private StartPos startZone;
    private OpMode opMode;
    private Telemetry telemetry;
    private Navigation nav;

    public AutoGeneric(StartPos startZone, com.qualcomm.robotcore.eventloop.opmode.OpMode opMode, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        this.startZone = startZone;
        this.opMode = opMode;
        this.telemetry = telemetry;
        nav = new Navigation(opMode, telemetry,true);
    }

    public void runOpMode() {


        //extendy - in
        //lift - down
        //collector height

        nav.setLiftLock(Navigation.LiftLock.LOCK);
        nav.setCollectorHeight(Navigation.CollectorHeight.DUMP);

        //Sampling on intit
        nav.updateCubePos();
        nav.setLiftLock(Navigation.LiftLock.LOCK);



        //detaching from hook w evan method
        //nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
        //nav.setCollectorHeight(Navigation.CollectorHeight.LOWER);
        //nav.setLiftHeight(Navigation.LiftHeight.HOOK);

        //detaching from hook w nick method
        nav.setLiftHeight(Navigation.LiftHeight.LOWER);
        nav.setLiftLock(Navigation.LiftLock.UNLOCK);
        nav.setCollectorHeight(Navigation.CollectorHeight.COLLECT);
        nav.holdForLift();
        nav.setLiftHeight(Navigation.LiftHeight.HOOK);


        nav.pointTurnRelative(45f,45f,2f);
        nav.holdForDrive();
        nav.setLiftHeight(Navigation.LiftHeight.LOWER);
        nav.pointTurnRelative(-45f,45f,2f);
        nav.holdForDrive();





        nav.goDistance(20f,30f);
        nav.setCollectionSweeper(Navigation.CollectorSweeper.INTAKE);
        nav.holdForDrive();

        switch(nav.getCubePos()) {
            case LEFT:
                nav.pointTurnRelative(45f,45f,2f);
                nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
                nav.holdForExtension();
                nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
                nav.holdForExtension();
                nav.pointTurnRelative(30f,30f,2f);
                break;
            case MIDDLE:
                nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
                nav.holdForExtension();
                nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
                nav.holdForExtension();
                nav.pointTurnRelative(75f,30f,2f);
                break;
            default:
                nav.pointTurnRelative(-45f,45f,2f);
                nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
                nav.holdForExtension();
                nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
                nav.holdForExtension();
                nav.pointTurnRelative(120f,30f,2f);
                break;
        }

        nav.holdForDrive();
        nav.goDistance(45f,20f);
        nav.holdForDrive();

        if(startZone == StartPos.CRATER) {
            nav.pointTurnRelative(60f,30f,2f);
        }
        else {
            nav.pointTurnRelative(-120f,30f,2f);
        }

        nav.holdForDrive();
        nav.goDistance(-55f,20f);
        nav.holdForDrive();
        nav.goDistance(90f,10f);
        nav.holdForDrive();
        nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
        nav.holdForExtension();

    }
}
