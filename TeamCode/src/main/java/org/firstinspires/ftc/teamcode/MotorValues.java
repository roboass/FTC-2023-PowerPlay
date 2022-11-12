package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;

public class MotorValues {
    public double fl, fr, bl, br, globalMultiplier = 0.5;

    public static double P = 2.5, I = 1.5, D = 23;
    public double lastActual, actual, lastOutput, output, errorSum = 0, ImaxOutput;
    public double minOutput = 0, maxOutput = 0, outputRampRate = 5, maxError = 250, outputFilter = 5;
    public boolean reversed = false, firstrun = true;


    public double slowModeMultiplier = 0.75;

    public MotorValues(double cfl, double cfr, double cbl, double cbr) {
        fl = cfl * globalMultiplier;
        fr = cfr * globalMultiplier;
        bl = cbl * globalMultiplier;
        br = cbr * globalMultiplier;
    }

    public MotorValues(double powerAll) {
        powerAll *= globalMultiplier;
        fl = powerAll;
        fr = powerAll;
        bl = powerAll;
        br = powerAll;
    }

    public void SlowMode() {
        fl *= slowModeMultiplier;
        fr *= slowModeMultiplier;
        bl *= slowModeMultiplier;
        br *= slowModeMultiplier;
    }

    public void NormaliseValues() {
        double maxSpeed = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));

        if (maxSpeed < 1) return;

        fl /= maxSpeed;
        fr /= maxSpeed;
        bl /= maxSpeed;
        br /= maxSpeed;
    }

    /**
     *          _        _        _
     *         /\ \     /\ \     /\ \
     *        /  \ \    \ \ \   /  \ \____
     *       / /\ \ \   /\ \_\ / /\ \_____\
     *      / / /\ \_\ / /\/_// / /\/___  /
     *     / / /_/ / // / /  / / /   / / /
     *    / / /__\/ // / /  / / /   / / /
     *   / / /_____// / /  / / /   / / /
     *  / / /   ___/ / /__ \ \ \__/ / /
     * / / /   /\__\/_/___\ \ \___\/ /
     * \/_/    \/_________/  \/_____/
     * (PID)
     */
    public void applyPID(DcMotor motor, int actualTicks, int setpoint)
    {
        int sign = (setpoint < 0 ? -1 : 1);
        maxOutput = setpoint;
        int error = setpoint - actualTicks;
        double Poutput, Ioutput, Doutput;

        Poutput = P*error;

        if(firstrun)
        {
            firstrun = false;
            lastActual = actualTicks;
            lastOutput = Poutput;
        }

        Doutput= -D*(actual-lastActual);
        lastActual = actual;

        Ioutput = I*errorSum;
        if(ImaxOutput != 0) Ioutput = constrain(Ioutput, ImaxOutput, -ImaxOutput);

        output = Poutput + Ioutput + Doutput;
        output *= sign;

        if(minOutput!=maxOutput && !bounded(output, minOutput,maxOutput)){
            errorSum=error;
        }
        else if(outputRampRate!=0 && !bounded(output, lastOutput-outputRampRate,lastOutput+outputRampRate) ){
            errorSum=error;
        }
        else if(ImaxOutput!=0){
            errorSum=constrain(errorSum+error,-maxError,maxError);
        }
        else{
            errorSum+=error;
        }

        if(outputRampRate!=0){
            output=constrain(output, lastOutput-outputRampRate,lastOutput+outputRampRate);
        }
        if(minOutput!=maxOutput){
            output=constrain(output, minOutput,maxOutput);
        }
        if(outputFilter!=0){
            output=lastOutput*outputFilter+output*(1-outputFilter);
        }

        lastOutput = output;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition((int)output);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    private double constrain(double value, double mx, double mn)
    {
        if(value > mx) return mx;
        if(value < mn) return mn;
        return value;
    }

    private boolean bounded(double value, double mx, double mn) {
        return (mn < value && value < mx);
    }
}