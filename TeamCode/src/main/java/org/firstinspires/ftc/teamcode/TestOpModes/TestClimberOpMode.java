package org.firstinspires.ftc.teamcode.TestOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;

@TeleOp(name = "Climber Test OpMode", group = "02-Test OpModes")
public class TestClimberOpMode extends LinearOpMode {

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public Climber climber;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);


    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.HAZMAT_TELEOP;

        initSubsytems();
        printDebugMessages();

        waitForStart();
        gameTimer.reset();

        telemetry.addLine("Start Pressed");
        telemetry.update();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                runClimber();

                if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                    printDebugMessages();
                    telemetry.update();
                }
            }
        }
        GameField.poseSetInAutonomous = false;
    }

    public void runClimber(){
        if (gamepadController.gp1GetY()){
            climber.moveClimberMotor(Climber.CLIMBER_MOTOR_STATE.MAX_EXTENDED);
        } else {
            climber.modifyClimberSlidesLength(0);
        }
    }

    public void initSubsytems(){
        telemetry.setAutoClear(false);

        //Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        /* Create Subsystem Objects*/
        climber = new Climber(hardwareMap);
        telemetry.addLine("DriveTrain Initialized");
        telemetry.update();




        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //GameField.debugLevel = GameField.DEBUG_LEVEL.NONE;
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;

        telemetry.addLine("+++++++++++++++++++++++");
        telemetry.addLine("Init Completed, All systems Go! Let countdown begin. Waiting for Start");
        telemetry.update();
        printDebugMessages();
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages() {
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_LEVEL is : ", GameField.debugLevel);
        telemetry.addData("Robot ready to start", "");

        if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {

            telemetry.addData("Game Timer : ", gameTimer.time());
            //telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
            //telemetry.addData("GameField.currentPose : ", GameField.currentPose);
            //telemetry.addData("startPose : ", startPose);

            if (GameField.debugLevel == GameField.DEBUG_LEVEL.MAXIMUM){
                telemetry.addData("Climber Motor Position", "%.2f", climber.climberMotor.getCurrentPosition());
                telemetry.addData("Climber Power", climber.climberMotor.getPower());

                telemetry.addData("Climber is busy", climber.climberMotor.isBusy());
                telemetry.addData("CLIMBER BRAKE MODE", climber.climberMotor.getZeroPowerBehavior());
            }

            telemetry.addLine("=============");



            telemetry.addLine("=============");

        }
        telemetry.update();
    }



}
