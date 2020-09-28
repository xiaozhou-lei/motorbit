// tests go here; this will not be compiled when this package is used as a library
basic.forever(() => {
    motorbit.RUS_04(1, RgbColors.Red, ColorEffect.Breathing, DigitalPin.P2);
})
