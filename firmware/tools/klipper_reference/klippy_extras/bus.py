# Stub standing in for klippy/extras/bus.py so spi_temperature.py imports without a printer.
def MCU_SPI_from_config(*args, **kwargs):  # pragma: no cover - never called by the harness
    raise RuntimeError("SPI is not available in the reference harness")
