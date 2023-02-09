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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpCodeLift", group="Linear Opmode")
//@Disabled
public class TeleOpCodeLift extends UsefulFunctions {
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Initialise();
        InitialiseLift();

        waitForStart();
        runtime.reset();

        boolean a2lock = false, bLock2 = false, yLock = false, aLock = false, dupLock2 = false, ddownLock2 = false,
                dleft = false, dright = false, rbumper2 = false, yLock2 = false, aLock2 = false, lbumper2 = false, xLock2 = false, ltrigger = false, rtrigger = false;

        int sign = 1;

        servoAx.setPosition(0);

        while (opModeIsActive()) {

            TeleOpDrive();

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            if(gamepad2.left_bumper) {
                if(!lbumper2)
                {
                    servoBratStanga.setPosition(0.175);
                }
            } else if(lbumper2) lbumper2 = false;

            if(gamepad2.a) {
                if(!aLock2)
                {
                    double servoClestePosition = servoCleste.getPosition();
                    servoCleste.setPosition(servoClestePosition <= .25 ? 1 : .25);
                }
            } else if(aLock2) aLock2 = false;

            if(gamepad2.b) {
                if(!bLock2)
                {
                    servoBratDreapta.setPosition(0.04);
                    servoBratStanga.setPosition(0.04);
                }
            } else if(bLock2) bLock2 = false;

            if(gamepad2.y) {
                if(!yLock2)
                {
                    servoBratDreapta.setPosition(0.175);
                    servoBratStanga.setPosition(0.175);
                }
            } else if(yLock2) yLock2 = false;

            if(gamepad2.x) {
                if(!xLock2)
                {
                    servoBratDreapta.setPosition(0.31);
                    servoBratStanga.setPosition(0.31);
                }
            } else if(xLock2) xLock2 = false;

            if(gamepad2.right_bumper) {
                if(!rbumper2)
                {
                    double servoAxPosition = servoAx.getPosition();
                    servoAx.setPosition(servoAxPosition != 0 ? 0 : 0.65);
                }
            } else if(rbumper2) rbumper2 = false;

            if(gamepad2.dpad_up) {
                if(!dupLock2)
                {
                    Elevator(0.5, "high_junction");
                }
            } else if(dupLock2) dupLock2 = false;

            if(gamepad2.dpad_down) {
                if(!ddownLock2)
                {
                    Elevator(0.5, "ground");
                }
            } else if(ddownLock2) ddownLock2 = false;

            UpdateTicks();
            UpdateOrientation();

            telemetry.addData("pozitie servo stanga:", servoBratStanga.getPosition());
            telemetry.addData("pozitie servo dreapta:", servoBratDreapta.getPosition());
            telemetry.addData("pozitie servo cleste:", servoCleste.getPosition());
            telemetry.addData("pozitie servo ax:", servoAx.getPosition());

            telemetry.addData("Current ticks bl br fl fr", crticksbl + " " + crticksbr + " " + crticksfl + " " + crticksfr);
            telemetry.update();
        }

    }
}