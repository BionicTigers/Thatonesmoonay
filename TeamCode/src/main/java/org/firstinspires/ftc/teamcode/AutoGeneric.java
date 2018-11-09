package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoGeneric extends LinearOpMode {

    public void runOpMode() {
        Navigation nav = new Navigation(this, telemetry,true);

        nav.setLiftLock(Navigation.LiftLock.UNLOCK);
        nav.setLiftHeight(Navigation.LiftHeight.LOWER);
        nav.setLiftLock(Navigation.LiftLock.LOCK);

        nav.setCollectorHeight(Navigation.CollectorHeight.PARK);
        nav.setCollectorExtension(Navigation.CollectorExtension.PARK);

        waitForStart();

        //detaching from hook
        nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
        nav.setLiftHeight(Navigation.LiftHeight.HOOK);
        nav.setCollectorHeight(Navigation.CollectorHeight.LOWER);

        nav.updateCubePos();

        switch(nav.getCubePos()) {
            case LEFT:
                nav.pointTurnRelative(-70,90,5);
                break;
            case MIDDLE:
                nav.pointTurnRelative(-90,90,5);
                break;
            default:
                nav.pointTurnRelative(-110,90,5);
                break;
        }


    }
}
