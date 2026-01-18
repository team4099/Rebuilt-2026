class Shooter(private val io: ShooterIO) : SubsytemBase() {
    val inputs = ShooterIO.ShooterInputs()

    var currentState: ShooterState.UNINITIALIZED
    var currentRequest: ShooterRequest = ShooterRequest.OpenLoop(0.0.volts)
        set(value) {
            when (value) {
                is ShooterRequest.OpenLoop -> shooterVoltageTarget = value.voltage
                is ShooterRequest.ClosedLoop -> {}
                else -> {}
            }
            field = value
        }
}