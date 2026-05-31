# Base System — How it works (FRA263 / FRA264)

This document describes the **Base System** application that runs on the PC: what it connects to, **how data moves**, and **what each Modbus register address means**.

---

## 1. What the Base System does (big picture)

1. You use the **web user interface (UI)** in the browser.
2. The UI talks to the Base System over a **local WebSocket** (JSON messages on your PC).
3. The Base System talks to the **robot controller (STM32)** over **USB serial** using **Modbus RTU**.
4. Each side only sees its own link: the UI never speaks Modbus directly; the robot never sees the WebSocket.

**Serial link (Modbus RTU)** — must match the robot firmware:

| Setting    | Value  |
|-----------|--------|
| Baud      | 230400 |
| Data bits | 8      |
| Parity    | Even   |
| Stop bits | 1      |

**Modbus slave address:** default **21**
**Base System version:** V1.2 (baud 19200 → **230400**) | poll ~50 Hz, HI ~5 Hz

---

## 2. How data is sent and received

### 2.1 From PC to robot (WRITE)

The Base System sends **Write Single Register** commands (FC06). Each write targets **one 16-bit holding register** at a time.

- Some registers carry **bit patterns** (mode flags, gripper command codes).
- Others carry **signed numbers** (−32768 … +32767) as 16-bit two's complement.

### 2.2 From robot to PC (READ)

The Base System periodically **reads a block** of holding registers starting at **0x00** through **0x31** (50 registers in one FC03 read).

### 2.3 Heartbeat (register 0x00)

| Who          | Action                                         |
|--------------|------------------------------------------------|
| Robot        | Writes **22881** ("YA") into register **0x00** |
| Base System  | Writes **18537** ("HI") back into **0x00**     |

If the Base System does not see the expected heartbeat pattern in time → UI shows link as **not alive**.

### 2.4 Numbers with a decimal place (0x28, 0x29, 0x30)

| Register | Meaning      | Decode                      |
|----------|--------------|-----------------------------|
| 0x28     | Position     | `real = (signed raw) / 10` |
| 0x29     | Velocity     | same                        |
| 0x30     | Acceleration | same                        |

---

## 3. WRITE register map (PC → STM32)

### 3.1 Summary

| Addr         | Topic              | Short description                                           |
|-------------:|--------------------|-------------------------------------------------------------|
| **0x00**     | Heartbeat          | Reply HI (18537) when robot sends YA (22881)               |
| **0x01**     | Operating mode     | Home / Jog / Auto / Set home / Test (one flag at a time)   |
| **0x02**     | Manual gripper     | Up, Down, Open, Close                                       |
| **0x03**     | Gripper sequence   | Pick or Place                                               |
| **0x04**     | Gripper in AUTO    | Enable or disable gripper during automatic motion           |
| **0x05**     | Jog                | Signed step size in degrees (+ = CCW, − = CW)              |
| **0x06**     | Test type          | Performance vs Precision test                               |
| **0x07**     | Performance test   | Desired velocity                                            |
| **0x08**     | Performance test   | Desired acceleration                                        |
| **0x09**     | Precision test     | Initial position                                            |
| **0x10**     | Precision test     | Final (target) position                                     |
| **0x11**     | Precision test     | Repeat count; sign selects unit (degree vs index)           |
| **0x12–0x21**| Pick & place       | Sequence slots: hole index and direction per slot (int16)   |
| **0x22**     | Pick & place       | Number of pairs (pick+place) in the sequence                |
| **0x23**     | Point-to-point     | Unit: degree or saved index                                 |
| **0x24**     | Point-to-point     | Target value (signed)                                       |
| **0x25**     | Safety             | Soft stop: 0=run, 1=stop                                    |

### 3.2 0x01 — Operating mode

| Bit | Mask (dec) | Mask (hex) | Mode / command |
|:---:|:----------:|:----------:|----------------|
| 0   | **1**      | 0x0001     | Go home        |
| 1   | **2**      | 0x0002     | Manual / Jog   |
| 2   | **4**      | 0x0004     | Auto           |
| 3   | **8**      | 0x0008     | Set home       |
| 4   | **16**     | 0x0010     | Test           |

### 3.3 0x02 — Gripper manual

| Command | Dec | Hex    |
|---------|----:|--------|
| Up      | 0   | 0x0000 |
| Down    | 1   | 0x0001 |
| Open    | 2   | 0x0002 |
| Close   | 4   | 0x0004 |

### 3.4 0x03 — Gripper sequence

| Command | Dec | Hex    |
|---------|----:|--------|
| Pick    | 1   | 0x0001 |
| Place   | 2   | 0x0002 |

### 3.5 0x04 — Gripper enable (AUTO)

| Dec | Meaning                       |
|----:|-------------------------------|
| 0   | Gripper disabled during auto  |
| 1   | Gripper enabled during auto   |

### 3.6 0x06 — Test type

| Dec | Meaning          |
|----:|------------------|
| 0   | Precision test   |
| 1   | Performance test |

### 3.7 0x23 — P2P unit

| Dec | Unit   |
|----:|--------|
| 0   | Degree |
| 1   | Index  |

### 3.8 0x25 — Soft stop

| Dec | Meaning        |
|----:|----------------|
| 0   | Normal running |
| 1   | Request stop   |

---

## 4. READ register map (STM32 → PC)

| Addr     | Name             | What you use it for                               |
|---------:|------------------|---------------------------------------------------|
| **0x00** | Heartbeat        | Robot sends YA (22881); PC answers HI on write    |
| **0x26** | Reed sensors     | Physical gripper limits and jaw state (bits)      |
| **0x27** | Current task     | What the motion sequencer is doing                |
| **0x28** | Position         | θ vs current home (÷10 for display)               |
| **0x29** | Velocity         | ÷10 for display                                   |
| **0x30** | Acceleration     | ÷10 for display                                   |
| **0x31** | Emergency        | E-stop / safety state (bit 0)                     |

### 4.1 0x26 — Reed sensors (bit map)

| Bit | Hex    | 1 = meaning        |
|:---:|--------|--------------------|
| 0   | 0x0001 | Reed 1 ON (Up)     |
| 1   | 0x0002 | Reed 2 ON (Down)   |
| 2   | 0x0004 | Reed 3 ON (Closed) |

### 4.2 0x27 — Current task (bit map, first match wins)

| Bit | Hex    | Dec | Task shown |
|:---:|--------|----:|------------|
| 0   | 0x0001 | 1   | Homing     |
| 1   | 0x0002 | 2   | Go Pick    |
| 2   | 0x0004 | 4   | Go Place   |
| 3   | 0x0008 | 8   | Go Point   |
| —   | —      | 0   | Idle       |

### 4.3 0x31 — Emergency

| Dec | Meaning                    |
|----:|----------------------------|
| 0   | Normal                     |
| 1   | Emergency active (E-stop)  |

---

## 5. Two's complement for signed registers

| Raw value  | Signed value      |
|------------|-------------------|
| ≥ 32768    | raw − 65536       |
| < 32768    | raw (as-is)       |

**Example:** raw 65413 → signed −123 → after ÷10 = −12.3 display units.

---

## 6. Windows: finding the COM port

1. Open **Device Manager** → **Ports (COM & LPT)**
2. Find **STMicroelectronics STLink Virtual COM Port (COMx)**
3. Enter that **COM** number in the Base System connect dialog

---

## 7. Installation

### Prerequisites
- Docker Desktop installed

### Steps
1. Put `frontend-image.tar`, `docker-compose.yml`, and `main.exe` in the same directory
2. Load image: `docker load -i <image-name.tar>`
3. Start stack: `docker-compose up -d`
4. Start Python server: `main.exe`
5. Open browser: http://localhost:3000

### Useful commands
```bash
docker-compose down       # stop and remove containers
docker-compose ps         # check status
docker-compose logs backend  # view backend logs
docker-compose restart    # restart without deleting
```

> **Important:** Port 3000 (Web UI) and port 8765 (WebSocket) must be free.
