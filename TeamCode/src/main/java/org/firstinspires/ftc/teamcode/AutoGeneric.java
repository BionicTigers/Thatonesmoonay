package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.webserver.RobotControllerWebHandlers;

public class AutoGeneric extends LinearOpMode{

    public static enum StartPos {DEPOT, CRATER};
    private StartPos startZone;

    public AutoGeneric(StartPos startZone) {
        this.startZone = startZone;
    }

    public void runOpMode() {
        Navigation nav = new Navigation(this, telemetry,true);

        //extendy - in
        //lift - down
        //collector height

        //nav.setLiftLock(Navigation.LiftLock.LOCK);
        nav.setCollectorHeight(Navigation.CollectorHeight.PARK);

        waitForStart();

        //detaching from hook
        nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
        nav.setCollectorHeight(Navigation.CollectorHeight.LOWER);
        nav.setLiftHeight(Navigation.LiftHeight.HOOK);

        nav.updateCubePos();

        nav.goDistance(20f,5f);
        nav.setCollectionSweeper(Navigation.CollectorSweeper.INTAKE);

        switch(nav.getCubePos()) {
            case LEFT:
                nav.pointTurnRelative(45f,45f,5f);
                nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
                nav.setLiftHeight(Navigation.LiftHeight.LOWER);
                nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
                nav.pointTurnRelative(30f,30f,5f);
                break;
            case MIDDLE:
                nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
                nav.setLiftHeight(Navigation.LiftHeight.LOWER);
                nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
                nav.pointTurnRelative(75f,30f,5f);
                break;
            default:
                nav.pointTurnRelative(-45f,45f,5f);
                nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
                nav.setLiftHeight(Navigation.LiftHeight.LOWER);
                nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
                nav.pointTurnRelative(120f,30f,5f);
                break;
        }

        nav.goDistance(45f,20f);

        if(startZone == StartPos.CRATER) {
            nav.pointTurnRelative(60f,30f,5f);
        }
        else {
            nav.pointTurnRelative(-120f,30f,5f);
        }


        nav.goDistance(-55f,20f);
        nav.goDistance(90f,10f);
        nav.setCollectorExtension(Navigation.CollectorExtension.OUT);

    }
}
