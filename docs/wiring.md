# Wiring and power evidence

[README](../README.md)

## Established configuration

| Signal / device | Connection | Evidence |
|---|---|---|
| Platform sensor | MPU6050 at `0x68` | `MPU_ADDR[PLATFORM]` |
| Base sensor | MPU6050 at `0x69` | `MPU_ADDR[BASE]` |
| Roll signal | Uno D10 | `ROLL_PIN` |
| Pitch signal | Uno D9 | `PITCH_PIN` |
| I²C SDA / SCL | Uno A4 / A5 (duplicated SDA/SCL header is equivalent) | Uno hardware mapping used by `Wire.begin()` |
| Platform AD0 | Logic low | Address selection needed for `0x68` |
| Base AD0 | Logic high | Address selection needed for `0x69` |
| Ground | Common signal ground | Required electrical reference; not continuity-tested |

The MPU6050 INT pins are not used by this sketch. Both sensors share the I²C bus; distinct AD0 states avoid an address collision. The source uses Wire's default clock and does not call `setClock()`.

## Power is incompletely documented

The builder identifies two 18650 lithium-ion cells and a buck-converter module. The images do not establish series versus parallel battery wiring, protection/charging hardware, converter output setting, current rating or which rail feeds the Uno. A definitive battery wiring diagram would therefore be invented.

For reproduction, use a regulated servo supply within the actual servo's specified voltage range and rated for both motors' load and stall currents. Supply current for the MG996R pair should not pass through the Uno's 5 V pin or USB power path. Establish a common ground. Verify converter polarity and output voltage with a meter before connection, and use appropriate cell protection and charging for the actual battery arrangement.

MPU6050 breakout boards differ in regulators and pull-ups. Verify the module schematic before choosing VCC or connecting pull-ups to a 5 V Uno. A bare MPU6050 is not a 5 V device; AD0 high means a compatible sensor logic level, not an unconditional 5 V connection. Inspect the combined pull-ups of both modules and the need for level translation.

These are reproduction requirements, not claims that the supplied prototype has a measured electrical defect. Unverified items to record next: battery topology, protection/charger model, converter model and output voltage, idle/load voltage, peak current, grounding, and a pin-by-pin continuity table.
