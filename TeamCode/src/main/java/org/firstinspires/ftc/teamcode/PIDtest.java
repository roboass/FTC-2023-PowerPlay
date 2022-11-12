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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.math.BigDecimal;
import java.math.RoundingMode;

@TeleOp(name="PIDtest", group="Linear Opmode")
//@Disabled
public class PIDtest extends UsefulFunctions {
    private ElapsedTime runtime = new ElapsedTime();


    public static double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = BigDecimal.valueOf(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Initialise();
        waitForStart();
        runtime.reset();

        boolean a2lock = false, bLock = false, xLock = false, yLock = false, aLock = false, dupLock = false, ddownLock = false,
                dleftLock = false, drightLock = false, rbumper2 = false;


        double precision = 0.05, val = 0.5;
        while (opModeIsActive()) {
            TeleOpDrive();


            if(gamepad1.dpad_up) {
                if(!dupLock) {
                    val += precision;
                    dupLock = true;
                }
            } else if(dupLock) {
                dupLock = false;
            }

            if(gamepad1.dpad_down) {
                if(!ddownLock) {
                    val -= precision;
                    ddownLock = true;
                }
            } else if(ddownLock) {
                ddownLock = false;
            }

            if(gamepad1.dpad_left) {
                if(!dleftLock) {
                    val /= 2.;
                    dleftLock = true;
                }
            } else if(dleftLock) {
                dleftLock = false;
            }

            if(gamepad1.dpad_right) {
                if(!drightLock) {
                    val *= 2.;
                    drightLock = true;
                }
            } else if(drightLock) {
                drightLock = false;
            }

            if(gamepad1.x) {
                if(!xLock) {
                    MotorValues.P += val;
                    xLock = true;
                }
            } else if(xLock) {
                xLock = false;
            }

            if(gamepad1.y) {
                if(!yLock) {
                    MotorValues.I += val;
                    yLock = true;
                }
            } else if(yLock) {
                yLock = false;
            }

            if(gamepad1.b) {
                if(!bLock) {
                    MotorValues.D += val;
                    bLock = true;
                }
            } else if(bLock) {
                bLock = false;
            }

            if(gamepad1.right_bumper) {
                if(!rbumper2) {
                    AutonomousMove(in_to_mm(24), 0);
                    sleep(200);
                    AutonomousMove(0, in_to_mm(24));
                    sleep(200);
                    AutonomousMove(-in_to_mm(24), 0);
                    sleep(200);
                    AutonomousMove(0, -in_to_mm(24));
                    rbumper2 = true;
                }
            } else if(rbumper2) {
                rbumper2 = false;
            }

            UpdateTicks();
            UpdateOrientation();
            round(MotorValues.I, 2);
            round(MotorValues.D, 2);
            round(val, 2);
            telemetry.addData("P", MotorValues.P);
            telemetry.addData("I", MotorValues.I);
            telemetry.addData("D", MotorValues.D);
            telemetry.addData("precision", precision);
            telemetry.addData("cal", val);

            telemetry.addData("Current ticks bl br fl fr", crticksbl + " " + crticksbr + " " + crticksfl + " " + crticksfr);
            telemetry.update();
        }

    }
}
