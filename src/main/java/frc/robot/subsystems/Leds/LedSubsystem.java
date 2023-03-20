package frc.robot.subsystems.Leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public LedSubsystem() {

        led = new AddressableLED(0);

        ledBuffer = new AddressableLEDBuffer(2);

        led.setLength(ledBuffer.getLength());

        led.start();
    }

    public CommandBase coneSlide() {
        return Commands.runOnce(
                () -> {
                    ledBuffer.setRGB(0, 255, 255, 0);
                    ledBuffer.setRGB(1, 255, 255, 0);

                    led.setData(ledBuffer);
                });
    }

    public CommandBase cubeSlide() {
        return Commands.runOnce(
                () -> {
                    ledBuffer.setRGB(0, 255, 0, 255);
                    ledBuffer.setRGB(1, 255, 0, 255);

                    led.setData(ledBuffer);
                });
    }

    public CommandBase coneSlideDrop() {
        return Commands.repeatingSequence(
                Commands.runOnce(
                                () -> {
                                    ledBuffer.setRGB(0, 255, 255, 0);
                                    ledBuffer.setRGB(1, 255, 255, 0);

                                    led.setData(ledBuffer);
                                },
                                this)
                        .andThen(Commands.waitSeconds(.25))
                        .andThen(
                                Commands.runOnce(
                                        () -> {
                                            ledBuffer.setRGB(0, 0, 0, 0);
                                            ledBuffer.setRGB(1, 0, 0, 0);

                                            led.setData(ledBuffer);
                                        },
                                        this))
                        .andThen(Commands.waitSeconds(.25)));
    }

    public CommandBase cubeSlideDrop() {
        return Commands.repeatingSequence(
                Commands.runOnce(
                                () -> {
                                    ledBuffer.setRGB(0, 255, 0, 255);
                                    ledBuffer.setRGB(1, 255, 0, 255);

                                    led.setData(ledBuffer);
                                },
                                this)
                        .andThen(Commands.waitSeconds(.25))
                        .andThen(
                                Commands.runOnce(
                                        () -> {
                                            ledBuffer.setRGB(0, 0, 0, 0);
                                            ledBuffer.setRGB(1, 0, 0, 0);

                                            led.setData(ledBuffer);
                                        },
                                        this))
                        .andThen(Commands.waitSeconds(.25)));
    }

    public CommandBase coneLeftShelf() {
        return Commands.runOnce(
                () -> {
                    ledBuffer.setRGB(1, 255, 255, 0);
                    ledBuffer.setRGB(1, 0, 0, 0);

                    led.setData(ledBuffer);
                });
    }

    public CommandBase coneRightShelf() {
        return Commands.runOnce(
                () -> {
                    ledBuffer.setRGB(0, 255, 255, 0);
                    ledBuffer.setRGB(1, 0, 0, 0);

                    led.setData(ledBuffer);
                });
    }

    public CommandBase cubeLeftShelf() {
        return Commands.runOnce(
                () -> {
                    ledBuffer.setRGB(1, 255, 0, 255);
                    ledBuffer.setRGB(1, 0, 0, 0);

                    led.setData(ledBuffer);
                });
    }

    public CommandBase cubeRightShelf() {
        return Commands.runOnce(
                () -> {
                    ledBuffer.setRGB(0, 255, 0, 255);
                    ledBuffer.setRGB(1, 0, 0, 0);

                    led.setData(ledBuffer);
                });
    }

    public CommandBase coneLeftShielfDrop() {
        return Commands.repeatingSequence(
                Commands.runOnce(
                                () -> {
                                    ledBuffer.setRGB(1, 255, 0, 255);

                                    led.setData(ledBuffer);
                                },
                                this)
                        .andThen(Commands.waitSeconds(.25))
                        .andThen(
                                Commands.runOnce(
                                        () -> {
                                            ledBuffer.setRGB(1, 0, 0, 0);

                                            led.setData(ledBuffer);
                                        },
                                        this))
                        .andThen(Commands.waitSeconds(.25)));
    }

    public CommandBase coneRightShielfDrop() {
        return Commands.repeatingSequence(
                Commands.runOnce(
                                () -> {
                                    ledBuffer.setRGB(0, 255, 0, 255);

                                    led.setData(ledBuffer);
                                },
                                this)
                        .andThen(Commands.waitSeconds(.25))
                        .andThen(
                                Commands.runOnce(
                                        () -> {
                                            ledBuffer.setRGB(0, 0, 0, 0);

                                            led.setData(ledBuffer);
                                        },
                                        this))
                        .andThen(Commands.waitSeconds(.25)));
    }

    public CommandBase cubeLeftShielfDrop() {
        return Commands.repeatingSequence(
                Commands.runOnce(
                                () -> {
                                    ledBuffer.setRGB(1, 255, 0, 255);

                                    led.setData(ledBuffer);
                                },
                                this)
                        .andThen(Commands.waitSeconds(.25))
                        .andThen(
                                Commands.runOnce(
                                        () -> {
                                            ledBuffer.setRGB(1, 0, 0, 0);

                                            led.setData(ledBuffer);
                                        },
                                        this))
                        .andThen(Commands.waitSeconds(.25)));
    }

    public CommandBase cubeRightShielfDrop() {
        return Commands.repeatingSequence(
                Commands.runOnce(
                                () -> {
                                    ledBuffer.setRGB(0, 255, 0, 255);

                                    led.setData(ledBuffer);
                                },
                                this)
                        .andThen(Commands.waitSeconds(.25))
                        .andThen(
                                Commands.runOnce(
                                        () -> {
                                            ledBuffer.setRGB(0, 0, 0, 0);

                                            led.setData(ledBuffer);
                                        },
                                        this))
                        .andThen(Commands.waitSeconds(.25)));
    }
}
