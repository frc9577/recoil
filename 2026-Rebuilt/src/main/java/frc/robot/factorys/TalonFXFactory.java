package frc.robot.factorys;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXFactory {
    public TalonFXFactory() {}

    public Optional<TalonFX> construct(int CANID) {
        TalonFX talon = new TalonFX(CANID);

        if (talon.isConnected()) {
            return Optional.of(talon);
        } else {
            return Optional.empty();
        }
    }
}
