package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "launcher velocity")
public class LauncherVelocity extends LinearOpMode {
    DcMotorEx launcherL, launcherR;

    @Override
    public void runOpMode() {
        waitForStart();
        launcherL = hardwareMap.get(DcMotorEx.class, "launcherL");
        launcherR = hardwareMap.get(DcMotorEx.class, "launcherR");
        if(opModeIsActive()){
            launcherL.setVelocity(-3000);
            launcherR.setVelocity(2500);
            sleep(5000);
            launcherL.setVelocity(0);
            launcherR.setVelocity(0);
        }

    }
}
