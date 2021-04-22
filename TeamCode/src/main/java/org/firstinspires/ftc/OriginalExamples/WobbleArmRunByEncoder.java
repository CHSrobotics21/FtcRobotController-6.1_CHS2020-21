/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.OriginalExamples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "EncoderWobble", group = "TFOdometry")
@Disabled
public class WobbleArmRunByEncoder extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    DcMotor brMotor;
    CRServo wobbleArmHingeL, wobbleArmHingeR;

    @Override
    public void runOpMode() {
        initDriveHardwareMap();
        timer.reset();
        if (opModeIsActive()) { // Linear OpMode
            hinge(true);
        }
    }
    private void initDriveHardwareMap(){
        wobbleArmHingeL = hardwareMap.crservo.get("HingeL");// hardware map initialize cr servo to hardware name
        wobbleArmHingeR = hardwareMap.crservo.get("HingeR");
        brMotor = hardwareMap.dcMotor.get("backright");//encoder calling from hardware motor
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reset encoder motor
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//don't run using the motor's encoder
    }

    public void hinge(boolean p)
    {
        if (p)// bring down arm
        {
            while(opModeIsActive()&&!(brMotor.getCurrentPosition()>-2900&&brMotor.getCurrentPosition()<-2500)) /*find threshold for when wobble goal arm is not down; don't want arm to be all the way down*/ {
                wobbleArmHingeL.setPower(-1);
                wobbleArmHingeR.setPower(1);
            }
            wobbleArmHingeL.setPower(0) ;
            wobbleArmHingeR.setPower(0);
        }
        else {//bring arm back up
            while(opModeIsActive()&&!(brMotor.getCurrentPosition()<-500&&brMotor.getCurrentPosition()>-900)) /*find threshold for when wobble goal arm is not up*/ {
                wobbleArmHingeL.setPower(1);
                wobbleArmHingeR.setPower(-1);
            }
            wobbleArmHingeL.setPower(0);
            wobbleArmHingeR.setPower(0);
        }
    }

}
