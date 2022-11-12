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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousCode", group="Linear Opmode")
//@Disabled
public class AutonomousCode extends UsefulFunctions {
    private ElapsedTime runtime = new ElapsedTime();
    String position;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Initialise();
        InitialiseVision();

        waitForStart();
        runtime.reset();

        sleep(300);
        position = pipeline.getPosition();
        telemetry.addData("pos", position);
        telemetry.update();
        StopVision();

        if(position == "FIRST") {
            addToRampaAngle(-rampaAngle + unghiNivelJos);
        } else if(position == "SECOND") {
            addToRampaAngle(-rampaAngle + unghiNivelMij);

        } else if(position == "THIRD") {
            addToRampaAngle(-rampaAngle + unghiNivelSus);
        }
        AutonomousMove(in_to_mm(1.5 * 24), 0);
        sleep(200);
        AutonomousRotate(-90);
        sleep(200);

        motorRampaOnOff(1);
        sleep(7500);
        motorRampaOnOff(1);

        telemetry.addData("angle", gyro.getAngularOrientation().firstAngle);
        telemetry.update();
        sleep(200);

        addToRampaAngle(-rampaAngle + unghiCarousel);

        AutonomousRotate(175);

        telemetry.addData("angle", gyro.getAngularOrientation().firstAngle);
        telemetry.update();

        sleep(200);
        AutonomousMove(in_to_mm(2.5 * 24), 0);

        sleep(200);

        rampaMotorDreapta.setPower(-1);
        rampaMotorStanga.setPower(1);
        mergeRampa = true;
        sleep(7500);
        motorRampaOnOff(1);

        AutonomousRotate(-90);
        sleep(200);
        AutonomousMove(in_to_mm(3.5 * 24), 0);
        AutonomousMove(0, in_to_mm(23));
        AutonomousMove(in_to_mm(24 * 1.5), 0);
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Position", "Position: " + position);
            telemetry.update();
        }

    }
}