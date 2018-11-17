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
    private Navigation.CubePosition position = Navigation.CubePosition.UNKNOWN;

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


//        nav.setLiftLock(Navigation.LiftLock.LOCK);
        nav.setCollectorHeight(Navigation.CollectorHeight.DUMP);
//        nav.setLiftLock(Navigation.LiftLock.LOCK);
        nav.updateCubePos();


        //detaching from hook w evan method
//        nav.setLiftHeight(Navigation.LiftHeight.LOWER);
//        nav.setLiftLock(Navigation.LiftLock.UNLOCK);
//        nav.holdForLift();
//        nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
//        nav.setCollectorHeight(Navigation.CollectorHeight.COLLECT);
//        nav.setLiftHeight(Navigation.LiftHeight.HOOK);
//        nav.holdForLift();

        //detaching from hook w nick method
//        nav.setLiftHeight(Navigation.LiftHeight.LOWER);
//        nav.setLiftLock(Navigation.LiftLock.UNLOCK);
//        nav.setCollectorHeight(Navigation.CollectorHeight.COLLECT);
//        nav.holdForLift();
//        nav.setLiftHeight(Navigation.LiftHeight.HOOK);
//        nav.pointTurnRelative(45f,45f,2f);
//        nav.holdForDrive();
//        nav.setLiftHeight(Navigation.LiftHeight.LOWER);
//        nav.pointTurnRelative(-45f,45f,2f);
//        nav.holdForDrive();


        //THIS IS WHERE SAMPLING TAKES PLACE MAKE SURE IT IS POINTING AT THE CUBES FOR SAMPLING AND NOthiNG ELSE
        nav.updateCubePos();
        position = nav.getCubePos();
        nav.goDistance(20f);
        nav.setCollectionSweeper(Navigation.CollectorSweeper.INTAKE);
        nav.holdForDrive();

        switch(position) {
            case LEFT:
                nav.pointTurnRelative(45f);
                nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
                nav.holdForExtension();
                nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
                nav.holdForExtension();
                nav.pointTurnRelative(45f);
                break;
            case MIDDLE:
                nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
                nav.holdForExtension();
                nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
                nav.holdForExtension();
                nav.pointTurnRelative(90f);
                break;
            default:
                nav.pointTurnRelative(-45f);
                nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
                nav.holdForExtension();
                nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
                nav.holdForExtension();
                nav.pointTurnRelative(135f);
                break;
        }

        nav.setCollectionSweeper(Navigation.CollectorSweeper.OFF);
        nav.setCollectorHeight(Navigation.CollectorHeight.HOLD);
        nav.holdForDrive();
        nav.goDistance(40f);
        nav.holdForDrive();

        if(startZone == StartPos.CRATER) {
            nav.pointTurnRelative(-135f);
            nav.holdForDrive();
            nav.goDistance(-30f);
        }
        else {
            nav.pointTurnRelative(45f);
            nav.holdForDrive();
            nav.goDistance(-65f);
        }
        nav.holdForDrive();
        nav.goDistance(90f);
        nav.holdForDrive();
        nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
        nav.holdForExtension();

    }
    public boolean sampling()
    {
    Boolean holder = nav.updateCubePos();
    position = nav.getCubePos();
    return holder;

    }
}
