# Portable qPCR System (ESP32-P4)

A high-performance, RISC-V based qPCR thermal cycler controller featuring a Hosyond TFT touchscreen and MAX6675 thermocouple integration.

## 🛠 Development Environment (Ubuntu 24.04 Fix)

Ubuntu 24.04 enforces PEP 668, which prevents global `pip` installations. To develop locally without environment conflicts, we use **uv** for high-speed, isolated package management.

### 1. Install uv
```bash
curl -LsSf [https://astral.sh/uv/install.sh](https://astral.sh/uv/install.sh) | sh
# Restart your terminal after installation