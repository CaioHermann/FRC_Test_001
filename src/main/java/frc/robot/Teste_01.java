package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class Teste_01 extends TimedRobot {

private final TalonFX m_shooter = new TalonFX(3);
private final FlywheelSim m_shooterSim = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.004, 1.0),
    DCMotor.getKrakenX60(1));
private final Mechanism2d m_shooterMech = new Mechanism2d(4, 4);
private final MechanismLigament2d m_pa1;
private final MechanismLigament2d m_pa2;
private double m_shooterAngle = 0;
private final TalonFX m_dirigirEsquerda = new TalonFX(1);
private final TalonFX m_dirigirDireita = new TalonFX(2);

    private final DifferentialDrive m_robotDirigir =
        new DifferentialDrive(m_dirigirEsquerda::set, m_dirigirDireita::set);
    private final XboxController m_Controller = new XboxController(0);
    private final Timer m_timer = new Timer();

    private final DifferentialDrivetrainSim m_driveSim =
    DifferentialDrivetrainSim.createKitbotSim(
        DifferentialDrivetrainSim.KitbotMotor.kDualCIMPerSide,
        DifferentialDrivetrainSim.KitbotGearing.k7p31,
        DifferentialDrivetrainSim.KitbotWheelSize.kEightInch,
        null);

    private final Field2d m_field = new Field2d();
    public Teste_01() {
        MechanismRoot2d root = m_shooterMech.getRoot("Shooter", 2, 2);
        m_pa1 = root.append(new MechanismLigament2d("Pa1", 1.5, 0, 6, new Color8Bit(255, 140, 0)));
        m_pa2 = root.append(new MechanismLigament2d("Pa2", 1.5, 180, 6, new Color8Bit(255, 140, 0)));
        SmartDashboard.putData("Shooter Mech", m_shooterMech);
        SmartDashboard.putData("Field", m_field);
        m_driveSim.setPose(new Pose2d(2, 2, new Rotation2d()));
        m_robotDirigir.setDeadband(0.1);
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_dirigirDireita.getConfigurator().apply(cfg);
}
    @Override
    public void simulationPeriodic() {
        double volts = RobotController.getBatteryVoltage();
        m_shooterAngle += m_shooterSim.getAngularVelocityRPM() / 60.0 * 360.0 * 0.02 * 0.05;
        m_pa1.setAngle(m_shooterAngle);
        m_pa2.setAngle(m_shooterAngle + 180);
        var shooterSimState = m_shooter.getSimState();
        shooterSimState.setSupplyVoltage(volts);
        m_shooterSim.setInputVoltage(shooterSimState.getMotorVoltage());
        m_shooterSim.update(0.02);
        shooterSimState.setRotorVelocity(m_shooterSim.getAngularVelocityRadPerSec() / (2 * Math.PI));
        SmartDashboard.putNumber("Shooter RPM", m_shooterSim.getAngularVelocityRPM());
        m_driveSim.setInputs(
            m_dirigirEsquerda.get() * volts,
            m_dirigirDireita.get() * volts);
        m_driveSim.update(0.02);
        m_field.setRobotPose(m_driveSim.getPose());
}
    @Override
    public void autonomousInit() {
        m_timer.restart();
    }
    @Override
public void robotPeriodic() {
    SmartDashboard.putNumber("Esquerda", m_dirigirEsquerda.get());
    SmartDashboard.putNumber("Direita", m_dirigirDireita.get());
    SmartDashboard.putNumber("Heading", m_driveSim.getPose().getRotation().getDegrees());
    for (int i = 0; i < 6; i++) {
        SmartDashboard.putNumber("Eixo " + i, m_Controller.getRawAxis(i));
    }
}
    @Override
    public void autonomousPeriodic() {
        if (m_timer.get() < 2.0) {
            m_robotDirigir.arcadeDrive(0.0, 0.5, false);
        }
        else {
            m_robotDirigir.stopMotor();
        }

    }
    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        m_robotDirigir.arcadeDrive(-m_Controller.getRawAxis(1), -m_Controller.getRawAxis(4), false);
        m_shooter.set(m_Controller.getRightTriggerAxis());
    }   
}
