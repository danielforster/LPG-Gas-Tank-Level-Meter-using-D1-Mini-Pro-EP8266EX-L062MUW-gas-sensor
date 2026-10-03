# LPG Level Monitor – D1 mini Pro + DYP L062MUW

Reads the L062MUW (UART **auto** output model) and publishes the liquid level,
gas temperature, echo signal strength and tilt angle to MQTT. A built-in web
interface handles all configuration.

## 1. Wiring

| Sensor wire | Function            | D1 mini Pro pin |
|-------------|---------------------|-----------------|
| Red         | VCC                 | **3V3** (see note) |
| Black       | GND                 | **G**           |
| Green       | TX (data out)       | **D7** (GPIO13) |
| Yellow      | RX (mode select)    | leave **unconnected** |

**Yellow wire:** unconnected (or high) = filtered "processed" value, about
one reading every 800 ms – recommended. Tie it to GND for faster, noisier
real-time values. It is only checked in the first 100 ms after power-on.

**Power / logic level:** the ESP8266 pins are 3.3 V. Powering the sensor from
3V3 keeps its output at 3.3 V, so the green wire can go straight to D7.
The datasheet's minimum supply is 3.3 V, so if readings are unreliable,
power it from **5V** instead – but first measure the green wire's idle
voltage with a multimeter. If it is around 5 V, add a divider:
green → 2.2 kΩ → D7, and D7 → 3.3 kΩ → GND.

**Don't connect anything to D8** – it must stay low at boot.

## 2. Build and upload

1. Install VS Code and the **PlatformIO IDE** extension.
2. *File → Open Folder…* and choose this `lpg-level-monitor` folder.
3. Plug in the D1 mini Pro by USB and click the **→ Upload** arrow in the
   blue status bar (or *PlatformIO → d1_mini_pro → Upload*).
   PlatformIO downloads the ESP8266 platform and libraries automatically
   the first time.

The sensor uses the hardware UART, so after the first boot message the USB
Serial Monitor goes quiet – that's expected. The live log is shown on the
web status page instead (it is also output on D4 if you have a USB-TTL adapter).
Uploading over USB still works normally.

## 3. First-time setup

1. After uploading, the board starts a WiFi network called
   **`LPG-Setup-xxxxxx`**, password **`lpgsetup`**.
2. Join it from your phone or laptop; the settings page should pop up
   automatically (otherwise browse to **http://192.168.4.1/config**).
3. Enter your WiFi, MQTT broker details and save. The device restarts and
   joins your network.
4. Open **http://lpg-sensor.local** (or the IP from your router).

If WiFi is ever unreachable for 60 s, the setup network reappears so you can
fix the settings; it switches itself off again 5 minutes after WiFi is back.

## 4. Web interface

* **Status** – the tank picture, gas level, temperature, low level warning,
  a plain-English sensor status and an **Order gas** card with call / email /
  website buttons for each supplier. Kept simple for everyday users.
* **Info** – sensor details (echo signal, tilt, frames), WiFi/MQTT state,
  log, "Publish now" and "Restart" buttons.
* **Settings** – tank name and calibration, gas suppliers (up to three),
  WiFi (with network scan), MQTT and admin password. Password fields are left blank to keep the saved value; enter
  `-` to clear one.
* **Firmware** – upload a new `firmware.bin` over WiFi
  (`.pio/build/d1_mini_pro/firmware.bin` after a build).

Set an **admin password** if the device is on a shared network – it protects
Settings, Firmware and the action buttons (username `admin`).

## 5. MQTT

With base topic `lpg/tank1`:

| Topic             | Payload |
|-------------------|---------|
| `lpg/tank1/state` | JSON, every publish interval (retained by default) |
| `lpg/tank1/status`| `online` / `offline` (retained, last-will) |
| `lpg/tank1/alert` | `low` / `ok` (retained) – low level warning, see below |
| `lpg/tank1/info`  | JSON with tank name and supplier contacts (retained, sent on connect) |
| `lpg/tank1/cmd`   | send `publish` to force a reading, `restart` to reboot |

Example state message:

```json
{"status":"ok","tank":"House tank","level_mm":754,"level_cm":75.4,"fill_pct":62.8,"temp_c":18.3,
 "signal_mv":1320,"angle_deg":0.6,"samples":74,"wifi_rssi":-61,"uptime_s":3600}
```

* `level_mm` is the **average** of all good readings since the last publish.
* `status` is `ok`, `no_liquid` (sensor reports no level) or
  `sensor_offline` (no valid frames for 5 s – check wiring).
* `fill_pct` is `null` until you set **Full level** under Settings → Tank.

**Credentials:** enter the broker username and password in Settings. For
cloud brokers (HiveMQ Cloud, EMQX Cloud etc.) tick **Use TLS** and use port
**8883**. Without a fingerprint the connection is encrypted but the broker's
certificate isn't verified; paste the broker's SHA1 fingerprint to verify it.

**Home Assistant:** tick *Home Assistant auto-discovery* and the sensors appear
automatically under a device named after the tank.

## 6. Low level alert

Set **Settings → Tank → Low level alert (%)** (needs *Full level* set; 0 = off).

* Below the threshold the Status page shows a red banner, the tank outline
  turns red, and `low` is published (retained) to `<base>/alert`.
* It goes back to `ok` once the level is **5 % above** the threshold, so a
  level hovering on the line doesn't keep toggling.
* A change only takes effect after it has held for **30 seconds**.
* "No level detected" (an empty tank, below the 3 cm blind zone) counts as 0 %;
  if the sensor stops sending data the alert keeps its last state.
* The state JSON also carries `"alert":"low"|"ok"`. With Home Assistant
  discovery on, a *Low gas level* problem sensor appears too.

**Email via Node-RED:** import `node-red-low-level-alert.json`
(menu → Import). It listens to `lpg/+/alert` and `lpg/+/info`, so one flow
covers every tank. The email names the tank and lists each supplier's phone,
email, website, account number and address, plus a link to the tank's page.
Alerts replayed when Node-RED or the device reconnects don't send a second
email. Install `node-red-node-email` from *Manage palette* if you don't have
it, then enter your own server, address and password in the e-mail and
broker nodes.

## 7. More than one tank

Give each device its own:

* **Tank name** (Settings → Tank), e.g. *House tank*, *Barn tank* – shown at the
  top of every page, in Home Assistant and in alert emails.
* **Device name** (Settings → WiFi), e.g. `lpg-house`, `lpg-barn` – so each has
  its own address (`http://lpg-house.local`).
* **Base topic** (Settings → MQTT), e.g. `lpg/house`, `lpg/barn`. Keep the
  `lpg/<something>` pattern and the Node-RED flow picks them all up.

Suppliers are set per device, so each tank can have its own.

## 8. Mounting tips

* The sensor measures **through the bottom of the tank** (metal or fibreglass,
  1–5 mm wall). Mount it on the base, flat and level, with ultrasonic
  couplant/gel between sensor and tank.
* Watch **Echo signal** on the status page while positioning: higher is
  better (max ~1500 mV). Very low values mean poor coupling.
* The datasheet quotes best accuracy at 0–35 °C and a 30 mm blind zone; levels
  below ~3 cm can't be measured.
* Calibrate with **Level offset** against a known level, and set **Full level**
  to the liquid height at your normal fill (cylinders are filled to ~80 %).
