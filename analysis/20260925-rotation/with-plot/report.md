# Offline rotation calibration

Quality checks passed: True
Body translation conditioned on supplied Z (or Z=0): [-0.10792124614804675, -0.02335029346145572, 0.0] m
Z externally measured: False
Left/right difference: 0.004319327476709376 m

## Limitations
- Z is externally supplied or fixed to zero, never estimated.
- Rotation RPY is not calibrated by this tool.
- A fixed effective rotation centre is assumed; low residual does not prove ground truth.
- With unknown Z, X/Y are conditional on Z=0; tilted mounting couples all components.

## Quality issues
