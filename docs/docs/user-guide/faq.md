# Frequently Asked Questions

## Does Astra-Rocket include Astra?

Yes. PlatformIO installs Astra through Astra-Rocket's declared dependency. Use
the Astra documentation for inherited sensors, logging, filters, and math APIs.

## Does a detected recovery stage fire a deployment output?

No. `EXPECTING_DROGUE`, `UNDER_DROGUE`, `EXPECTING_MAIN`, and `UNDER_MAIN` are
estimated states. Astra-Rocket does not energize a pyro channel or actuator.

## Can I construct `AstraRocket` without a config?

The default constructor exists, but mission sensors still have to be
configured. Prefer an explicit, long-lived `AstraRocketConfig` so the hardware
and thresholds are visible in application code.

## Can the config be a temporary expression?

No. `AstraRocket` stores a reference to its config. Construct the config in
static or global storage and keep it alive for the lifetime of the rocket.

## Where is the current integrated example?

`Side_Projects/STM32FC` in SRAD-Avionics is the current compile-validated
Astra-Rocket → Astra integration target. A successful build is not a hardware
or flight qualification.
