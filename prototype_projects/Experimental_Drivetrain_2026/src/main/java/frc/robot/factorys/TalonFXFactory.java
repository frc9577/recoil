package frc.robot.factorys;

import com.ctre.phoenix6.hardware.TalonFX;
import java.util.Optional;

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
