/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOpCode", group="Linear Opmode")
//@Disabled
public class TeleOpCode extends UsefulFunctions {
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Initialise();
        InitialiseArm();

        waitForStart();
        runtime.reset();

        boolean a2lock = false, bLock2 = false, yLock = false, aLock = false, dupLock = false, ddownLock = false,
                dleft = false, dright = false, rbumper2 = false, yLock2 = false, aLock2 = false, lbumper2 = false, xLock2 = false, ltrigger = false, rtrigger = false;



        while (opModeIsActive()) {
            TeleOpDrive();

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            if(gamepad2.a) {
                if(!aLock2)
                {
                    motorBratMicOnOff(1);
                    sleep(500);
                    motorBratMicOnOff(0);

                    sleep(100);

                    motorBratMareOnOff(1);
                    sleep(250);
                    toggleServo(servoSusJos);
                    sleep(300);
                    motorBratMareOnOff(0);

                    sleep(100);

                    motorBratMicOnOff(1);
                    sleep(350);
                    motorBratMicOnOff(0);
                    aLock2 = true;
                }
            } else if(aLock2) aLock2 = false;

            if(gamepad2.y) {
                if(!yLock2)
                {
                    motorBratMicOnOff(-1);
                    sleep(500);
                    motorBratMicOnOff(0);

                    sleep(100);

                    toggleServo(servoSusJos);

                    motorBratMareOnOff(-1);
                    sleep(550);
                    motorBratMareOnOff(0);

                    sleep(100);

                    motorBratMicOnOff(-1);
                    sleep(350);
                    motorBratMicOnOff(0);
                    yLock2 = true;
                }
            } else if(yLock2) yLock2 = false;

            if(gamepad2.b) {
                if(!bLock2)
                {
                    toggleServo(servoCleste);

//                    sleep(200);
//
//                    toggleServo(servoSusJos);
//
//                    sleep(200);
//
//                    toggleServo(servoSusJos);

                    bLock2 = true;
                }
            } else if(bLock2) bLock2 = false;

            if(gamepad2.x) {
                if(!xLock2)
                {
                    motorBratMicOnOff(1);
                    sleep(500);
                    motorBratMicOnOff(0);

//                    sleep(200);
//
//                    toggleServo(servoSusJos);

                    xLock2 = true;
                }
            } else if(xLock2) xLock2 = false;

            if(gamepad2.dpad_left){
                if(!dleft){
                    double pos = servoCleste.getPosition();
                    servoCleste.setPosition(pos + 0.1);

                    dleft = true;
                }
            } else if(dleft) dleft = false;

            UpdateTicks();
            UpdateOrientation();

            telemetry.addData("pozitie servo", servoSusJos.getPosition());

            telemetry.addData("Current ticks bl br fl fr", crticksbl + " " + crticksbr + " " + crticksfl + " " + crticksfr);
            telemetry.update();
        }

    }
}
