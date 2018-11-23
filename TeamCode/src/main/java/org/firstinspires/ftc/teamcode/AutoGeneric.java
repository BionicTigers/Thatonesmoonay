package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.webserver.RobotControllerWebHandlers;

public class AutoGeneric{

    public static enum StartPos {DEPOT, CRATER, DOUBLESAMPLING};
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

        nav.updateCubePos();

        //THIS IS WHERE SAMPLING TAKES PLACE MAKE SURE IT IS POINTING AT THE CUBES FOR SAMPLING AND NOthiNG ELSE
        nav.updateCubePos();
        position = nav.getCubePos();
        nav.pointTurnRelative(-90f);
        nav.hold(1);
        nav.goDistance(15f);
        nav.holdForDrive();

        switch(position) {
            case MIDDLE:
                nav.goDistance(15f);
                nav.holdForDrive();
                nav.goDistance(-15f);
                nav.holdForDrive();
                nav.pointTurnRelative(90f);
                nav.hold(1);
                nav.goDistance(45f);
                break;
            case RIGHT:
                nav.pointTurnRelative(-50f);
                nav.hold(1);
                nav.goDistance(20f);
                nav.holdForDrive();
                nav.goDistance(-19f);
                nav.holdForDrive();
                nav.pointTurnRelative(140f);
                nav.hold(1);
                nav.goDistance(45f);
                break;
            default: //left
                nav.pointTurnRelative(45f);
                nav.hold(2);
                nav.goDistance(20f);
                nav.holdForDrive();
                nav.goDistance(-17f);
                nav.holdForDrive();
                nav.pointTurnRelative(45f);
                nav.hold(1);
                nav.goDistance(48f);
                break;
        }

        nav.holdForDrive();

        if(startZone == StartPos.CRATER) {
            nav.pointTurnRelative(-135f);
            nav.hold(1);
            nav.goDistance(-40f);
            nav.holdForDrive();
            nav.setTeamMarker(0.8);
            nav.hold(1);
            nav.goDistance(70f);
            nav.hold(3);
        } else if(startZone == StartPos.DOUBLESAMPLING) {
            nav.pointTurnRelative(-135f);
            nav.hold(1);
            nav.goDistance(-40f);
            nav.holdForDrive();
            nav.setTeamMarker(0.8); //assuming marker is on the back @Brayden
            nav.hold(1);
            switch (position) {
                case MIDDLE:
                    nav.pointTurnRelative(-70f);
                    nav.hold(1);
                    nav.goDistance(20f);
                    nav.hold(1);
                    nav.goDistance(-20f);
                    nav.hold(1);
                    nav.pointTurnRelative(70);
                    break;
                case RIGHT:
                    nav.pointTurnRelative(-37f);
                    nav.hold(1);
                    nav.goDistance(20f);
                    nav.hold(1);
                    nav.goDistance(-20f);
                    nav.hold(1);
                    nav.pointTurnRelative(30f);
                    break;
                default: //left
                    nav.pointTurnRelative(-90f);
                    nav.hold(1);
                    nav.goDistance(25f);
                    nav.hold(1);
                    nav.goDistance(-25f);
                    nav.hold(1);
                    nav.pointTurnRelative(85f);
                    break;
            }
            nav.hold(1);
            nav.goDistance(70f);
            nav.holdForDrive();
        }
        else { //Depot
            nav.pointTurnRelative(45f);
            nav.hold(1);
            nav.goDistance(-55f);
            nav.holdForDrive();
            nav.setTeamMarker(0.8);
            nav.hold(1);
            nav.goDistance(70f);
            nav.holdForDrive();
        }


    }
    public boolean sampling()
    {
        Boolean holder = nav.updateCubePos();
        position = nav.getCubePos();
        return holder;

    }
}

