### External Triggering of acquisition




### Master-slave camera synchronization

This setup uses one camera as the master camera, using the flash outputs to trigger the acquisition of one or more slave cameras. The hardware setup is described in the IDS [documentation](https://en.ids-imaging.com/techtipp-details/items/app-note-synchronizing-image-acquisition.html):
 - The `F-` pin of the master camera is connected to `GND` of the power supply
 - The `F+` pin of the master camera is connected to all `T-` pins of the slave camera(s)
 - The `T+` pin of all slave camera(s) are connected to `Vcc`, the positive power supply.

In the parameter file of the master camera, activate the flash ouput, and set the desired framerate:

```yaml
/**:
  ros__parameters:

    FlashActive: true
    FlashReference: "ExposureStart"
    FlashDuration: 15000.0
    FlashStartDelay: 1.0
    FlashInvertSignal: false

    TriggerMode: "Off"
    AcquisitionFrameRate: 10
```

In the parameter file of the slave camera(s), activate external triggering through `Line0`:

```yaml
/**:
  ros__parameters:

    TriggerMode: "On"
    TriggerSource: "Line0"
    TriggerActivation: "RisingEdge"
    TriggerDivider: 1
```

