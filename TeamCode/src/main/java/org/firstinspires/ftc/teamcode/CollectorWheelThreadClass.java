package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class CollectorWheelThreadClass implements Runnable{
    private boolean isRunning = true;
    final double CPRCollectorWheel = 288;
    final double CollectorWheelDiameter = 5;
    double CPICollectorWheel = CPRCollectorWheel/(CollectorWheelDiameter*3.1415);
    private int sleepTime;

    DcMotor collectorWheel;

    public CollectorWheelThreadClass(DcMotor collectorWheel){
        this.collectorWheel = collectorWheel;
    }

    @Override
    public void run() {
    }

    public void stop(){ isRunning = false; }

    public void moveCollectorWheel(int inches)
    { //place after go to position statements to shoot at power shot
        collectorWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        collectorWheel.setTargetPosition(collectorWheel.getCurrentPosition()- (int)(inches*CPICollectorWheel));// enter encoder counts or inches you want to move times counts per inch FOR THIS WHEEL AND MOTORS
        collectorWheel.setPower(1);
    }
}
