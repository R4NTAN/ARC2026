# 📖 ESP32 + Ra-01H Transmitter README

## 🛠️ Hardware Setup

* **Board:** ESP32 DevKit v1
* **Module:** Ra-01H (SX1276)
* **Pin Mapping (Board Label → Code GPIO):**
* `D18` (SCK) → GPIO 18
* `D19` (MISO) → GPIO 19
* `D23` (MOSI) → GPIO 23
* `D27` (NSS / CS) → GPIO 27
* `D14` (RST) → GPIO 14
* `D26` (DIO0) → GPIO 26


* **Baud Rate:** 115200 (Serial Monitor)
* **Line Ending:** Newline or Both NL & CR

---

## ⚙️ Configuration Parameters (`config` command)

**Syntax:** `config RUN POINT PRESET MODE COUNT INTERVAL_MS POWER_DBM`

Example: `config 1001 1 1 0 100 200000 10`

| Parameter | Type | Description |
| --- | --- | --- |
| **RUN** | `uint32_t` | Unique ID for the experiment (e.g., `1001`). The ESP32 remembers the last run ID and will **refuse to start** if you reuse it. You must increment this for every new experiment. |
| **POINT** | `uint16_t` | Measurement point ID (e.g., `1`, `2`, `3`). Useful if you are moving the transmitter to different physical locations. |
| **PRESET** | `1` to `5` | LoRa modem preset (see table below). |
| **MODE** | `0` or `1` | `0` = One-way (TX only, no ACK).<br>

<br>`1` = Two-way (TX waits for ACK from receiver to calculate PDR). |
| **COUNT** | `1` to `10000` | Total number of packets to transmit in this run. |
| **INTERVAL_MS** | `uint32_t` | Time to wait between transmissions in milliseconds. Must be greater than the calculated minimum interval to respect the 1% duty cycle. |
| **POWER_DBM** | `2` to `17` | Transmit power in dBm (PA_BOOST). |

### 📊 Preset Table (`table` command)

| Preset | Spreading Factor (SF) | Bandwidth (BW) | Coding Rate (CR) | Min Interval (ms) |
| --- | --- | --- | --- | --- |
| 1 | 7 | 125 kHz | 4/5 | ~16,000 |
| 2 | 9 | 125 kHz | 4/5 | ~41,000 |
| 3 | 12 | 125 kHz | 4/5 | ~148,000 |
| 4 | 7 | 250 kHz | 4/5 | ~8,500 |
| 5 | 7 | 125 kHz | 4/8 | ~28,000 |

---

## 💻 Serial Commands

| Command | Description |
| --- | --- |
| `help` or `?` | Prints the command list to the Serial Monitor. |
| `status` | Prints current radio status, active state, fault state, and configuration. |
| `table` | Prints the 5 available LoRa presets and their minimum intervals. |
| `config ...` | Updates and saves the experiment configuration to NVS memory. |
| `start` | Begins the experiment. Waits 5 seconds for the receiver to arm, then starts transmitting. |
| `stop` | Aborts the current experiment immediately. |
| `reset` | Manually clears `seq`, `ackCount`, and `fault` without deleting the CSV file. |
| `dump` | Prints the entire `/measurements.csv` file to the Serial Monitor. |
| `erase YES` | Deletes the `/measurements.csv` file (starts a fresh CSV on next run). |
| `format YES` | Formats the LittleFS flash partition (use only on first boot or if the filesystem crashes). |

---

## 📈 CSV Output Format

The data is saved to `/measurements.csv` and can be exported using `dump` or the Python script.

**Columns:**

1. `schema`: Always `1`
2. `node`: Always `A` (Transmitter)
3. `boot_id`: Random ID generated on ESP32 boot.
4. `event`: The action that occurred.
5. `run_id`: Your configured RUN ID.
6. `point_id`: Your configured POINT ID.
7. `preset`: Your configured PRESET.
8. `mode`: Your configured MODE.
9. `seq`: The current packet sequence number.
10. `t_ms`: Milliseconds since ESP32 boot.
11. `freq_hz`: Frequency (868100000).
12. `sf`: Spreading Factor.
13. `bw_hz`: Bandwidth.
14. `cr_den`: Coding Rate denominator.
15. `tx_power_dbm`: Configured power.
16. `packet_bytes`: Always `32`.
17. `planned_count`: Total packets planned.
18. `interval_ms`: Configured interval.
19. `toa_ms`: Time on Air (calculated).
20. `rssi_dbm`: Received Signal Strength (`ACK_RX`).
21. `snr_db`: Signal to Noise Ratio (`ACK_RX`).
22. `pdr_pct`: Cumulative Packet Delivery Ratio (0.00 to 100.00).
23. `ack_count`: Total successful ACKs received.

**Event Types:** `START`, `TX_ATTEMPT`, `TX_DONE`, `ACK_RX`, `ACK_TIMEOUT`, `TX_UNCERTAIN`, `TX_FAIL`, `END`, `STOP`.

---

## 💾 Flash Memory & Storage Limits

* **Capacity:** ~30 to 40 experiments (~8,000–10,000 rows) before the 1.5 MB LittleFS partition fills up.
* *Tip:* Always use `dump` to save data and `erase YES` to clear space between testing sessions.

---

## 🔄 Recommended Workflow for a New Experiment

1. **Change Configuration:** `config 1002 1 1 1 100 200000 14`
2. **Clear Old Data (Optional):** `erase YES`
3. **Start Run:** `start`
4. **Export Data:** `dump` (or use `serial_monitor.py` to auto-capture).

---
