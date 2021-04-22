package org.firstinspires.ftc.OriginalExamples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;


@TeleOp(name="LED example")
@Disabled
public class LEDexample extends OpMode
{
    boolean toggle = false;
    LED LED;

    @Override
    public void init() {
    
        LED = hardwareMap.get(LED.class, "LED");
     //   LED.setMode(LED.Mode.OUTPUT);
    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }


    @Override
    public void loop() {
        if(gamepad1.a){
            toggle = !toggle;
            if(toggle){
                LED.enableLight(false);
            }
            else{
                LED.enableLight(true);
            }
        }
        telemetry.addData("LED Status: ", LED.isLightOn());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

}
