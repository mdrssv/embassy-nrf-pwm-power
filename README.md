# nrf9160 PWM idle power consumption

I've noticed that after using the PWM peripheral, the idle power consumption won't go as low as it would if I never had used the peripheral
which raises the suspicion, that the peripheral might not get disabled properly

### With Debbugger attached
3.8mA after PWM

5.25mA after PWM without modem setup

### No Debugger + Power Cycle
`cargo run --release -F no-pwm`
0.03mA after modem setup without PWM

`cargo run --release`
0.37mA after modem setup & PWM

`cargo run --release -F no-pwm -F no-modem`
2.94mA no modem setup no PWM
