package org.firstinspires.ftc.teamcode.TestOpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;

@TeleOp(name = "TestOpArm", group = "TestOp")
public class ArmTestOpMode extends LinearOpMode {

    public Arm arm;

    public void runOpMode() throws InterruptedException{

        arm = new Arm(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            arm.turnArmBrakeModeOn();

            //Extend the arm based on the right joystick
            if (gamepad2.left_stick_x > 0) {
                arm.extendArm();
                arm.runArmToLevel(gamepad2.left_stick_x);
                arm.runArmToLevelState = false;
            }

            //retract the arm based on the right joystick
            if (gamepad2.left_stick_x < 0) {
                arm.retractArm();
                arm.runArmToLevel(gamepad2.left_stick_x);
                arm.runArmToLevelState = false;
            }

            //Move arm to preset positions
            if (gamepad2.x){
                arm.moveToArmLowJunction();
                arm.runArmToLevel(arm.MED_POWER);
                arm.runArmToLevelState = false;
            }

            if (gamepad2.b){
                arm.moveToArmHighJunction();
                arm.runArmToLevel(arm.MED_POWER);
                arm.runArmToLevelState = false;
            }

            if (gamepad2.a){
                arm.moveToArmGroundJunction();
                arm.runArmToLevel(arm.MED_POWER);
                arm.runArmToLevelState = false;
            }
            if (gamepad2.y){
                arm.moveToArmMidJunction();
                arm.runArmToLevel(arm.MED_POWER);
                arm.runArmToLevelState = false;
            }

        }

    }

}