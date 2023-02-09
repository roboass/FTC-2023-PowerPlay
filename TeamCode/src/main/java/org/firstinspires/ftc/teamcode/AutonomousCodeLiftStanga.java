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

@Autonomous(name="AutonomousCodeLiftStanga", group="Linear Opmode")
//@Disabled
public class AutonomousCodeLiftStanga extends UsefulFunctions {
    private ElapsedTime runtime = new ElapsedTime();
    String color;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Initialise();
        InitialiseLift();
        InitialiseVision();

        waitForStart();
        runtime.reset();

        toggleServo(servoCleste);

        color = pipeline.getColor();
        telemetry.addData("color", color);
        telemetry.update();
        StopVision();

       AutonomousMoveRiseLift(0, -in_to_mm(2 * 24), "high_junction", 0);
       AutonomousRotate(-45);

       servoBratStanga.setPosition(0.04);
       servoBratDreapta.setPosition(0.04);

       toggleServo(servoCleste);

       //Autonomous Loop
        for (int i = 1; i <= 3; i++) {
            AutonomousRotateReverseClaw(-45);
            AutonomousMoveRiseLift(0, in_to_mm(1 * 24), "cone_stack", i);

            toggleServo(servoCleste);

            AutonomousMoveRiseLift(0, -in_to_mm(1 * 24), "high_junction", 0);
            AutonomousRotateReverseClaw(45);

            toggleServo(servoCleste);
        }

        AutonomousRotateReverseClaw(-45);
        AutonomousMoveRiseLift(0, in_to_mm(1 * 24), "cone_stack", 4);

        toggleServo(servoCleste);

        servoBratDreapta.setPosition(0.175);
        servoBratStanga.setPosition(0.175);

        if(color.equals("GREEN")){
            AutonomousMove(0, -in_to_mm(0.1 * 24));
        } else if(color.equals("WHITE")){
            AutonomousMove(0, in_to_mm(0.9 * 24));
        } else if(color.equals("BLACK")){
            AutonomousMove(0, in_to_mm(1.9 * 24));
        }

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

    }
}