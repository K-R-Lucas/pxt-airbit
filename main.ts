input.onButtonPressed(Button.AB, function() {
    if (AirBit.isInitialised()) {
        AirBit.emergencyStop();
    }
});