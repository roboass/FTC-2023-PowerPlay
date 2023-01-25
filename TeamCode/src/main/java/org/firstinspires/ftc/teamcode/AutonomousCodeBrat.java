package org.firstinspires.ftc.teamcode;

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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousCodeBrat", group="Linear Opmode")
//@Disabled
public class AutonomousCodeBrat extends UsefulFunctions {
    private ElapsedTime runtime = new ElapsedTime();
    String position;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Initialise();
        InitialiseArm();
//        InitialiseVision();

        waitForStart();
        runtime.reset();

        toggleServo(servoCleste);

        sleep(100);

//        position = pipeline.getColor();
//        telemetry.addData("color", position);
//        telemetry.update();
//        StopVision();

//      toggleServo(servoCleste);
//
//      sleep(1000);
//
//      toggleServo(servoSusJos);
//
//      sleep(1000);
//
//      AutonomousMove(0, in_to_mm(0.1 * 24));
//
//      AutonomousMove(in_to_mm(1.5 * 24), 0);
//
//      sleep(2000);
//
//      AutonomousMove(0, -in_to_mm(0.4 * 24));
//
//      sleep(100);
//
//      motorBratMicOnOff(1);
//      sleep(500);
//      motorBratMicOnOff(0);
//
//      sleep(100);
//
//      motorBratMareOnOff(1);
//      sleep(550);
//      motorBratMareOnOff(0);
//
//      sleep(100);
//
//      motorBratMicOnOff(1);
//      toggleServo(servoSusJos);
//      sleep(450);
//      motorBratMicOnOff(0);

    //        motorBratMicOnOff(1);
    //        motorBratMareOnOff(1);
    //        sleep(500);
    //        motorBratMicOnOff(0);
    //        motorBratMareOnOff(0);

    //        motorBratMicOnOff(1);;
    //        sleep(100);
    //        motorBratMicOnOff(0);

//        sleep(10);
//
//        toggleServo(servoCleste);
//
//        sleep(2000);
//
//        AutonomousMove(in_to_mm(0.1 * 24), 0);

        AutonomousMove(0, -in_to_mm(0.1 * 24));

        AutonomousMove(in_to_mm(0.8 * 24), 0);

        AutonomousMove(0, -in_to_mm(1.95 * 24));

        AutonomousMove(-in_to_mm(0.8 * 24), 0);

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("pozitie cleste", servoSusJos.getPosition());
            telemetry.addData("pozitie strangere", servoCleste.getPosition());
            telemetry.update();
        }

    }
}