package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.webserver.RobotControllerWebHandlers;

/**
 * A class to run Autonomous given a strategy.
 */
public class AutoGeneric{

    public enum StartPos {DEPOT, CRATER, DOUBLESAMPLING};
    private StartPos startZone;
    private OpMode opMode;
    private Telemetry telemetry;
    private Navigation nav;

    /**
     * The constructor method that contains everything to run in initialization.
     * @param startZone - StartPos enumerator. Tells which strategy to run. Options are DEPOT, CRATER, or DOUBLESAMPLING.
     * @param opMode - The OpMode required to access motors. Often, 'this' will suffice.
     * @param telemetry - Telemetry of the current OpMode, used to output data to the screen.
     */
    public AutoGeneric(StartPos startZone, com.qualcomm.robotcore.eventloop.opmode.OpMode opMode, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        this.startZone = startZone;
        this.opMode = opMode;
        this.telemetry = telemetry;
        nav = new Navigation(opMode, telemetry,true);
    }

    /**
     * Run this to run Autonomous.
     */
    public void runOpMode() {


        //-----sampling-----//
        nav.updateCubePos();
        //-----unhooking-----//
        nav.setCollectorHeight(Navigation.CollectorHeight.DUMP);
        nav.pointTurnRelative(-90f);
        nav.holdForDrive();
        nav.goDistance(13f);
        nav.holdForDrive();
        switch(nav.getCubePos()) {
            case LEFT:
                nav.pointTurnRelative(55f);
                nav.holdForDrive();
                nav.goDistance(20f);
                nav.holdForDrive();
                nav.goDistance(-20f);
                nav.holdForDrive();
                nav.pointTurnRelative(37f);
                nav.holdForDrive();
                nav.goDistance(50f);
                nav.holdForDrive();
                break;
            case RIGHT:
                nav.pointTurnRelative(-50f);
                nav.holdForDrive();
              //  nav.hold(0.5f);
                nav.goDistance(20f);
                nav.holdForDrive();
                nav.goDistance(-20f);
                nav.holdForDrive();
                nav.pointTurnRelative(135f);
                nav.holdForDrive();
              //  nav.hold(0.6f);
                nav.goDistance(48f);
                nav.holdForDrive();
                break;
            default:
                nav.goDistance(15f);
                nav.holdForDrive();
                nav.goDistance(-15f);
                nav.holdForDrive();
                nav.pointTurnRelative(90f);
                nav.holdForDrive();
                nav.goDistance(49f);
                nav.holdForDrive();
                break;
        }

//        //-----driving to wall-----//
//        nav.holdForDrive();
//        nav.goDistance(49f);
//        nav.holdForDrive();

        //-----crater depot run-----//
        if(startZone == StartPos.CRATER) {
            nav.pointTurnRelative(-135f);
            nav.holdForDrive();
            nav.goDistance(-40f);
            nav.holdForDrive();
        }

        //-----crater doublesampling and depot run-----//
        else if(startZone == StartPos.DOUBLESAMPLING) {
            nav.pointTurnRelative(-135f);
            nav.holdForDrive();
            nav.goDistance(-40f);
            nav.holdForDrive();
            switch (nav.getCubePos()) {
                case MIDDLE:
                    nav.pointTurnRelative(-70f);
                    nav.holdForDrive();
                    nav.goDistance(25f);
                    nav.holdForDrive();
                    nav.goDistance(-25f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(69);
                    break;
                case RIGHT:
                    nav.pointTurnRelative(-37f);
                    nav.holdForDrive();
                    nav.goDistance(20f);
                    nav.holdForDrive();
                    nav.goDistance(-20f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(30f);
                    break;
                default: //left
                    nav.pointTurnRelative(-90f);
                    nav.holdForDrive();
                    nav.goDistance(25f);
                    nav.holdForDrive();
                    nav.goDistance(-25f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(87f);
                    break;
            }
            nav.holdForDrive();

        }

        //-----depot depot run-----//
        else {
            nav.pointTurnRelative(45f);
            nav.holdForDrive();
            nav.goDistance(-55f);
            nav.holdForDrive();
        }

        //-----marker deploy and driving to crater-----//
        nav.setTeamMarker(0.8f);
        nav.hold(1);
        nav.goDistance(63f);
        nav.holdForDrive();
        nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
        nav.hold(5);
        nav.setCollectorHeight(Navigation.CollectorHeight.COLLECT); //breaking crater plane
        nav.hold(2);
    }
}