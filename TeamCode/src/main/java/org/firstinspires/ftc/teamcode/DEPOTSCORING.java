package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DEPOTSCORING{

    public static enum StartPos {DOUBLESAMPLING};
    private StartPos startZone;
    private OpMode opMode;
    private Telemetry telemetry;
    private Navigation nav;
    private Navigation.CubePosition position = Navigation.CubePosition.UNKNOWN;

    public DEPOTSCORING(StartPos startZone, com.qualcomm.robotcore.eventloop.opmode.OpMode opMode, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
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
        //       nav.setCollectorHeight(Navigation.CollectorHeight.DUMP);
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

//RAWRHHHH
        //THIS IS WHERE SAMPLING TAKES PLACE MAKE SURE IT IS POINTING AT THE CUBES FOR SAMPLING AND NOthiNG ELSE
        nav.updateCubePos();
        position = nav.getCubePos();
        nav.goDistance(15f);
        nav.holdForDrive();
        nav.pointTurnRelative(180f);
        nav.hold(1);

        switch(position) {
            case MIDDLE:
                nav.goDistance(-30f);
                nav.holdForDrive();
                nav.pointTurnRelative(90);
                nav.hold(1);
                nav.goDistance(-25f);
                nav.holdForDrive();



                break;
            case RIGHT:
                nav.pointTurnRelative(40f);
                nav.hold(2);
                nav.goDistance(-30f);
                nav.holdForDrive();
                nav.setTeamMarker(0.8);
                nav.hold(1);

                break;
            default: //left
                nav.pointTurnRelative(-45f);
                nav.hold(2);
                nav.goDistance(-30f);
                nav.holdForDrive();
                nav.setTeamMarker(0.8);
                nav.hold(1);

                break;
        }

        // nav.setCollectionSweeper(Navigation.CollectorSweeper.OFF);
        //  nav.setCollectorHeight(Navigation.CollectorHeight.HOLD);
        nav.holdForDrive();

        if(startZone == StartPos.DOUBLESAMPLING) {
            nav.goDistance(-40f);
            nav.holdForDrive();
            nav.pointTurnRelative(90f);
            nav.hold(2);
            nav.setTeamMarker(0.8);
            nav.hold(1);
            nav.pointTurnRelative(-90f);
            nav.hold(1);
            nav.goDistance(70f);
            nav.holdForDrive();
        }

    }
}
