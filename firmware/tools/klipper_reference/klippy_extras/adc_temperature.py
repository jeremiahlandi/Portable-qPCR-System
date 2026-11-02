# Stub standing in for klippy/extras/adc_temperature.py so thermistor.py imports without a printer.
class PrinterADCtoTemperature:  # pragma: no cover - never constructed by the harness
    def __init__(self, config, adc_convert):
        raise RuntimeError("ADC objects are not available in the reference harness")
