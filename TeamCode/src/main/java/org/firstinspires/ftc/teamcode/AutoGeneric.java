package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    public AutoGeneric(StartPos startZone, com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        this.startZone = startZone;
        this.opMode = opMode;
        this.telemetry = telemetry;
        nav = new Navigation(opMode, telemetry,false,true);
        nav.hold(0.1f);


    }

    /**
     * Run this to run Autonomous.
     */
    public void runOpMode() {
        nav.updateCubePos();
        nav.setCollectorHeight(Navigation.CollectorHeight.DUMP);
        nav.setLiftHeight(Navigation.LiftHeight.HOOK);
        nav.holdForLift();
        nav.goDistance(3f);
        nav.holdForLift();
        nav.pointTurnRelative(-95f);
        nav.holdForDrive();
        nav.setLiftHeight(Navigation.LiftHeight.LOWER); //Lowering lift back to starting position
        nav.goDistance(13f);
        nav.holdForDrive();
        //-----unhooking-----//
        switch (nav.getCubePos()) {
            case LEFT:
                nav.pointTurnRelative(50f);
                nav.holdForDrive();
                nav.goDistance(20f);
                nav.holdForDrive();
                nav.goDistance(-20f);
                nav.holdForDrive();
                nav.pointTurnRelative(42f);
                nav.holdForDrive();
                nav.goDistance(52f);
                nav.holdForDrive();
                break;
            case RIGHT:
                nav.pointTurnRelative(-45f);
                nav.holdForDrive();
                nav.goDistance(20f);
                nav.holdForDrive();
                nav.goDistance(-20f);
                nav.holdForDrive();
                nav.pointTurnRelative(131f);
                nav.holdForDrive();
                nav.goDistance(49f);
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
        if (startZone == StartPos.CRATER) {
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
                    nav.goDistance(20f);
                    nav.holdForDrive();
                    nav.goDistance(-20f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(70);
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
        nav.goDistance(70f);
        nav.holdForDrive();
        nav.setCollectorHeight(Navigation.CollectorHeight.COLLECT); //breaking crater plane
        nav.hold(2);
    }
}