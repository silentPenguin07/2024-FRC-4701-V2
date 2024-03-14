package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
/* 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
*/
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.robot_subsystems.ArmSubsystem;
import frc.robot.robot_subsystems.DriveSubsystem;

public class RobotContainer {
    
    public static final double GAMEPAD_AXIS_THRESHOLD = 0.2;
    private final SendableChooser<Command> autoChooser;
    //private final SendableChooser<Command> sideChooser;
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    /*
    private List<PathPlannerPath> pathGroup;
    private Pose2d startingPose;
    private Pose2d endingPose;
    private PathPlannerPath path;
    */
    
    
    Joystick driverGamepad = new Joystick(RobotConstants.Ports.CONTROLLER.DRIVER_JOYSTICK);
    XboxController armGamepad = new XboxController(RobotConstants.Ports.CONTROLLER.ARM_JOYSTICK);

    // the container for the robot. contains subsystems, OI devices, commands
    public RobotContainer() {

        
        // build an auto chooser This will use Commands.none() as the default option
        autoChooser = AutoBuilder.buildAutoChooser();

        driveSubsystem.initialize();

        // configure the trigger bindings
        configureBindings();

        SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

    }
    
    /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
    private void configureBindings()
    {
        // adds a button to run Example Path (CLICK THE BUTTON)
        SmartDashboard.putData("Straight Path", new PathPlannerAuto("Straight Path"));
    }

    // passes the autonomous command to the main Robot class
    public Command getAutonomousCommand()
    {
        //return autoChooser.getSelected();
        return  driveSubsystem.auton();
    }
    /*
    public Trajectory createExampleTrajectory(TrajectoryConfig config) {
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        return exampleTrajectory;
    }*/

    public ArmSubsystem getArmSubsystem()
    {
        return armSubsystem;
    }
    
    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }
}
