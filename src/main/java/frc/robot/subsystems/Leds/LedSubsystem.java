package frc.robot.subsystems.Leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private final long numberOfLEDs = 2;
    private final double halfWidth = ((double) numberOfLEDs - 1.0) / 2.0;
    private final int brightness = 255;


public LedSubsystem(){

    led = new AddressableLED(0);
    
    
    ledBuffer = new AddressableLEDBuffer(2);
    led.setLength(ledBuffer.getLength());

    led.start();
}
@Override
public void periodic() {}

public void 

public void setLedSubsystem(double x, double size, int color){
    long howMany = Math.round(size * 22);
    long center = Math.round((x + 1) * halfWidth);
    long start = center - (howMany / 2);


int blue;
int red;
int green;

if (color == 0) { // If we are purple
    red = brightness;
    green = 0;
    blue = brightness;
    
} else if (color == 1) { // If we are yellow
    red = brightness;
    green = brightness;
    blue = 0;

} else {
    red = 0;
    green = 0;
    blue = 0;

  for (var i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 0, 0, 0);
}
for (int i = (int) start; i < start + howMany; i++) {
    if (i >= 0 && i < numberOfLEDs) {
        ledBuffer.setRGB(i, red, green, blue);
    }
}
led.setData(ledBuffer);
}
}
public void clear() {
    for (int i = 0; i < numberOfLEDs; i++) {
        ledBuffer.setRGB(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
}
}
