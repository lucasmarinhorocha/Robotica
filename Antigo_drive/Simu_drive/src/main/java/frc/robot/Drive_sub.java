package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive_sub extends SubsystemBase {

  // ======================
  // CONSTANTES
  // ======================
  private static final double TRACK_WIDTH = 0.7112; // DEVE bater com settings.json
  private static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
  private static final double GEAR_RATIO = 10.71;
  private static final double METERS_PER_ROTATION =
      (Math.PI * WHEEL_DIAMETER) / GEAR_RATIO;

  // ======================
  // HARDWARE
  // ======================
  private final SparkMax leftMaster  = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax rightMaster = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax leftFollow  = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax rightFollow = new SparkMax(4, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder  = leftMaster.getEncoder();
  private final RelativeEncoder rightEncoder = rightMaster.getEncoder();

  private final Pigeon2 gyro = new Pigeon2(0);

  DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  // ======================
  // SIMULAÇÃO
  // ======================
  private double simLeftPos = 0;
  private double simRightPos = 0;
  private double simLeftVel = 0;
  private double simRightVel = 0;
  private Rotation2d simGyroRotation = new Rotation2d();

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(TRACK_WIDTH);

  private final DifferentialDriveOdometry odometry;
  private final Field2d field = new Field2d();

  public Drive_sub() {

    SmartDashboard.putBoolean("EventoAtivo", false);

        SmartDashboard.putBoolean("Atirando", false);

    drive.setSafetyEnabled(false);
    SparkMaxConfig masterConfig = new SparkMaxConfig();
    masterConfig.encoder
        .positionConversionFactor(METERS_PER_ROTATION)
        .velocityConversionFactor(METERS_PER_ROTATION / 60.0);

    SparkMaxConfig followLeftConfig = new SparkMaxConfig();
    SparkMaxConfig followRightConfig = new SparkMaxConfig();

    followLeftConfig.follow(1);
    followRightConfig.follow(2);

    leftMaster.configure(
        masterConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
    );

    rightMaster.configure(
        masterConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
    );

    leftFollow.configure(
        followLeftConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
    );

    rightFollow.configure(
        followRightConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
    );

    odometry = new DifferentialDriveOdometry(
        gyro.getRotation2d(),
        0,
        0
    );

    configurePathPlanner();
    SmartDashboard.putData("Field", field);
  }

  // ======================
  // PATHPLANNER
  // ======================
  private void configurePathPlanner() {

    

    try {
      RobotConfig robotConfig = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getRobotRelativeSpeeds,
          this::driveRobotRelative,
          new PPLTVController(0.02),
          robotConfig,
          () -> DriverStation.getAlliance()
              .map(a -> a == DriverStation.Alliance.Red)
              .orElse(false),
          this
      );
    } catch (Exception e) {
      DriverStation.reportError("PathPlanner Error", e.getStackTrace());
    }

    PathPlannerLogging.setLogActivePathCallback(
        poses -> field.getObject("path").setPoses(poses)
    );

  }

  // ======================
  // LOOP
  // ======================
  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      odometry.update(simGyroRotation, simLeftPos, simRightPos);
    } else {
      odometry.update(
          gyro.getRotation2d(),
          leftEncoder.getPosition(),
          rightEncoder.getPosition()
      );
    }

    field.setRobotPose(getPose());
  }

  // ======================
  // DRIVE
  // ======================
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {

    ChassisSpeeds targetSpeeds =
        ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    DifferentialDriveWheelSpeeds wheelSpeeds =
        kinematics.toWheelSpeeds(targetSpeeds);

    if (RobotBase.isSimulation()) {

      simLeftVel = wheelSpeeds.leftMetersPerSecond;
      simRightVel = wheelSpeeds.rightMetersPerSecond;

      simLeftPos += simLeftVel * 0.02;
      simRightPos += simRightVel * 0.02;

      simGyroRotation = simGyroRotation.plus(
          new Rotation2d(targetSpeeds.omegaRadiansPerSecond * 0.02)
      );

    } else {
      // Controle simples e estável (igual ao código que funciona)
      leftMaster.set(wheelSpeeds.leftMetersPerSecond / 3.0);
      rightMaster.set(wheelSpeeds.rightMetersPerSecond / 3.0);
    }
  }

  // ======================
  // ODOMETRIA
  // ======================
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    if (RobotBase.isSimulation()) {
      simLeftPos = 0;
      simRightPos = 0;
      simGyroRotation = pose.getRotation();
    }

    odometry.resetPosition(
        gyro.getRotation2d(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition(),
        pose
    );
  }
  // MOVIMENTAÇÃO

  public ChassisSpeeds getRobotRelativeSpeeds() {

    if (RobotBase.isSimulation()) {
      return kinematics.toChassisSpeeds(
          new DifferentialDriveWheelSpeeds(simLeftVel, simRightVel)
      );
    }

    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity(),
            rightEncoder.getVelocity()
        )
    );
  }

 public void setArcade(double reto, double rotacao) {
   // Multiplicamos por uma velocidade máxima (ex: 3 m/s e 4 rad/s)
   // para converter o sinal do joystick (-1 a 1) em unidades reais
   double linearVel = reto * 3.0; 
   double angularVel = -rotacao * 4.0; // Invertido para o padrão da WPILib

   // Encaminha para o método que já lida com Real vs Simulação
   driveRobotRelative(new ChassisSpeeds(linearVel, 0, angularVel));
   
   // Alimenta o MotorSafety se ele estiver ativado
   drive.feed();
 }
}
