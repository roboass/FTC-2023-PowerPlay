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
        telemetry.addData("left", rampaServoStanga.getPosition());
        telemetry.addData("right", rampaServoDreapta.getPosition());
        telemetry.update();
        waitForStart();
        runtime.reset();

        addToRampaAngle(-rampaAngle + unghiNivelSus);
        boolean a2lock = false, bLock = false, yLock = false, aLock = false, dupLock = false, ddownLock = false,
                dleft = false, dright = false, rbumper2 = false, yLock2 = false, aLock2 = false, lbumper2 = false, xLock2 = false, ltrigger = false, rtrigger = false;
        double prevAngle = rampaAngle;


        while (opModeIsActive()) {
            TeleOpDrive();

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            /*
            if(gamepad2.right_trigger > 0.5) {
                trafaletMotor.setPower(10);
            } else {
                trafaletMotor.setPower(0);
            }
            */

            if(gamepad2.left_trigger > 0.5) {
                //rampaMotorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);
                ltrigger = true;
                rampaMotorDreapta.setPower(-1);
                rampaMotorStanga.setPower(1);

            } else if(ltrigger) {
                ltrigger = false;
                rampaMotorDreapta.setPower(0);
                rampaMotorStanga.setPower(0);
            }

            if(gamepad2.left_bumper) {
                lbumper2 = true;
                rampaMotorStanga.setPower(-1);
                rampaMotorDreapta.setPower(-1);
                mergeRampa = true;
            } else if(lbumper2 && mergeRampa) {
                motorRampaOnOff(1);
            }

            if(gamepad2.right_bumper) {
                rbumper2 = true;
                rampaMotorStanga.setPower(1);
                rampaMotorDreapta.setPower(1);
                mergeRampa = true;
            } else if(rbumper2 && mergeRampa) {
                motorRampaOnOff(1);
            }

            if(gamepad2.y) {
                if(!yLock2)
                {
                    rampaState = 1;
                    addToRampaAngle(-rampaAngle + unghiCarousel);
                    yLock2 = true;
                }
            } else if(yLock2) yLock2 = false;

            if(gamepad2.a) {
                if(!aLock2)
                {
                    rampaState = 0;
                    addToRampaAngle(-rampaAngle + unghiNivelIntake);
                    aLock2 = true;
                }
            } else if(aLock2) aLock2 = false;

            /*

            if(gamepad2.y) {
                if(!yLock) {
                    addToTrafaletAngle(-5);
                    yLock = true;
                }
            } else if(yLock) {
                yLock = false;
            }

            if(gamepad2.x) {
                if(!xLock2) {
                    addToTrafaletAngle(5);
                    xLock2 = true;
                }
            } else if(xLock2) xLock2 = false;

            if(gamepad2.b) {
                if(!bLock) {
                    addToTrafaletAngle(-trafaletAngle + trafaletPozJos);
                    bLock = true;
                }
            } else if(bLock) bLock = false;
 */
            if(gamepad2.dpad_up) {
                if(!dupLock)
                {
                    changeRampaState(1);
                    dupLock = true;
                }
            } else if(dupLock) dupLock = false;

            if(gamepad2.dpad_down) {
                if(!ddownLock)
                {
                    changeRampaState(-1);
                    ddownLock = true;
                }
            } else if(ddownLock) ddownLock = false;

            if(gamepad2.dpad_right) {
                if(!dright)
                {
                    addToRampaAngle(-5);
                    dright = true;
                }
            } else if(dright) dright = false;

            if(gamepad2.dpad_left) {
                if(!dleft)
                {
                    addToRampaAngle(5);
                    dleft = true;
                }
            } else if(dleft) dleft = false;

            /* if(gamepad2.a) {
            if(gamepad2.a) {
                if(!a2lock)
                {
                    addToRampaAngle(-rampaAngle + unghiCarousel);
                    a2lock = true;
                }
            } else if(a2lock) a2lock = false; */

            UpdateTicks();
            UpdateOrientation();
            telemetry.addData("rampa angle", rampaAngle);
            telemetry.addData("rampa stanga", rampaServoStanga.getPosition());
            telemetry.addData("rampa dreapta", rampaServoDreapta.getPosition());

            telemetry.addData("Current ticks bl br fl fr", crticksbl + " " + crticksbr + " " + crticksfl + " " + crticksfr);
            telemetry.update();
        }

    }
}
