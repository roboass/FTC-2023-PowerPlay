package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="SiaClesteTriaaaall")
public class SiaClesteTest extends OpMode {

   Servo grip_arm;
   int i=1;

    @Override
    public void init() {

        grip_arm = hardwareMap.get(Servo.class, "grip_arm");
    }

    @Override
    public void loop() {
     /*if(gamepad1.x  && grip_arm.getPosition()==1){
      grip_arm.setPosition(0);
     }
     if(grip_arm.getPosition()==0) {
      grip_arm.setPosition(0);
     }
      else if(gamepad1.x && grip_arm.getPosition()==0) {
       grip_arm.setPosition(1);
     }
     if(grip_arm.getPosition()==1) {
      grip_arm.setPosition(1);
     }*/



     /*if (gamepad1.x && i%2==1) {
      grip_arm.setPosition(1);
      i++;

      if(gamepad1.x==false){
       grip_arm.setPosition(1);
      }

     }

     else if(gamepad1.x && i%2==0) {
      grip_arm.setPosition(0);
      i++;

      if(gamepad1.x==false){
       grip_arm.setPosition(0);}
     } */


      while (gamepad1.x  && i==1) {
      grip_arm.setPosition(0);
      }
      i=0;
      while (!gamepad1.x && i==0)
      {
       grip_arm.setPosition(0);
      }
      while (gamepad1.x && i==0)
      {
       grip_arm.setPosition(1);
      }
      i=1;
      while(!gamepad1.x && i==1)
      {
       grip_arm.setPosition(1);
      }


    }
}
