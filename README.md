# RPiZA — Raspberry Pi Pedestrian Traffic Light Controller

## Installation on the Raspberry Pi

### 1. Install dependencies

```bash
sudo apt update && sudo apt install python3-tk python3-gpiozero python3-rpi.gpio
```

### 2. Download the program

```bash
wget https://raw.githubusercontent.com/ahs20/rasberry_pi_traffic/main/rpiza.py
```

### 3. Run

```bash
python3 rpiza.py
```

---

## Auto-start on boot (optional)

```bash
mkdir -p ~/.config/autostart
nano ~/.config/autostart/rpiza.desktop
```

Paste the following, save with `Ctrl+O`, exit with `Ctrl+X`:

```
[Desktop Entry]
Type=Application
Name=RPiZA
Exec=python3 /home/pi/rpiza.py
```

---

## Pin wiring (BCM numbering)

| Component | Signal | BCM Pin |
|-----------|--------|---------|
| FzA       | Red    | 5       |
| FzA       | Yellow | 6       |
| FzA       | Green  | 13      |
| FgA1      | Red    | 27      |
| FgA1      | Green  | 22      |
| FgA2      | Red    | 21      |
| FgA2      | Green  | 16      |
| HC-SR04   | TRIG   | 15      |
| HC-SR04   | ECHO   | 17      |

> **Important:** The HC-SR04 ECHO pin outputs 5 V but the Pi GPIO is 3.3 V.
> Wire a voltage divider on the ECHO line: 330 Ω from ECHO to GPIO 17, and 470 Ω from GPIO 17 to GND.
