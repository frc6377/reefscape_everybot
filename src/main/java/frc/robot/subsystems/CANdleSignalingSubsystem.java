package frc.robot.subsystems;

import frc.robot.Constants.SignalingConstants;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.led.CANdle;
import java.util.function.IntSupplier;
import frc.utils.RGB;



public class CANdleSignalingSubsystem {

    private final CANdle candle = new CANdle(SignalingConstants.kCANdle);

    private void setFullStrip(final RGB rgb) {
        Logger.recordOutput("Signaling/LED Color", rgb.toHex());
        setSection(rgb, 8, SignalingConstants.NUMBER_OF_LEDS);
    }

    private void setSection(final RGB rgb, final int startID, final int count) {
        candle.setLEDs(rgb.red, rgb.green, rgb.blue, 0, startID, count);
    }

    private void setSectionStrip(final RGB rgb, final int startID, final int count) {
        if (startID == 10) {
        Logger.recordOutput("Signaling/LED Color", rgb.toHex());
        }
        setSection(rgb, startID + 8, count);
    }
}
