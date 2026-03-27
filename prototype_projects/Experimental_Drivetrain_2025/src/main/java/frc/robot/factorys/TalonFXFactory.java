package frc.robot.factorys;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXFactory {
    public TalonFXFactory() {}

    public Optional<TalonFX> construct(int CANID) {
        Optional<TalonFX> newMotor;
        try {
            newMotor = Optional.of(new TalonFX(CANID));
        } catch (Exception e) {
            newMotor = Optional.empty();
        }

        return newMotor;
    }
}
