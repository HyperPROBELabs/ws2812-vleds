# ws2812-vleds changelog

Date: 2026-10-07
Target: (Test) Luckfox Pico Mini A/B (rv1103g), Linux 5.10.x, gnu89/C90-strict build
File: `ws2812-vleds.c`

Status: Everything works on target hardware.

---

## Added

### netdev LED trigger support
- No driver code is needed for the trigger itself. `netdev` is a generic LED
  trigger and every LED is already a plain `led_classdev`.
- Kernel requirements: `CONFIG_LEDS_TRIGGERS=y` and `CONFIG_LEDS_TRIGGER_NETDEV=y`.
- Usage:
  ```
  echo netdev > /sys/class/leds/ws-led{n}/trigger
  echo eth0   > /sys/class/leds/ws-led{n}/device_name
  echo 1      > /sys/class/leds/ws-led{n}/link
  echo 1      > /sys/class/leds/ws-led{n}/rx
  echo 1      > /sys/class/leds/ws-led{n}/tx
  ```
  On 5.10 the attributes are `device_name`, `link`, `tx`, `rx`, `interval`.
  Newer kernels changed them (`mode`, `link_10`, ...).
- New optional DT property `linux,default-trigger = "netdev";` per LED node
  (read into `led_classdev.default_trigger`). `device_name` is still set via sysfs.

### SPI update rate limit
- New module parameter `max_update_hz` (default `50`, `0` = unlimited),
  writable at runtime (`/sys/module/<module>/parameters/max_update_hz`).
- Brightness callbacks no longer call `spi_write()`. They update the frame
  buffer under the mutex and call `ws2812_vleds_request_update()`.
- `ws2812_vleds_request_update()` queues one `delayed_work`:
  - strip idle for at least one interval: flush immediately (leading edge)
  - otherwise: one flush at the end of the window (trailing edge)
  - requests while a flush is pending are no-ops (`queue_delayed_work()` returns false)
- `ws2812_vleds_flush()` is the worker; SPI errors are logged with
  `dev_err_ratelimited()`.
- `ws2812_vleds_update()` is kept as the synchronous write, used only in
  probe/remove. It also records `last_flush`.
- Effective ceiling is the jiffy rate (100 Hz at `HZ=100`).
- Guards against `jiffies` wraparound: a `last_flush` older than about 2^31
  jiffies could make `time_before()` return true and stall the update, so the
  delay is clamped to the interval. `last_flush` is initialised in probe, because
  `jiffies` starts near `INITIAL_JIFFIES` and a zero `last_flush` would delay the
  first update by about 5 minutes.

### New struct members / includes
- `driver_data`: `struct delayed_work flush_work`, `unsigned long last_flush`.
- Includes: `<linux/workqueue.h>`, `<linux/jiffies.h>`, `<linux/moduleparam.h>`.

---

## Fixed

| # | Bug | Fix |
|---|-----|-----|
| 1 | **Wrong pixel index.** In the `filter_main` path `_index` was never incremented, so every `ws-led{n}` wrote pixel 0. Invisible with a single LED. | New `__find_wsled()` helper returns the node and its real index for all filter types. |
| 2 | **Invalid node on lookup miss.** If no LED matched, `_node` pointed at the container of the list head and was dereferenced. | `__find_wsled()` returns `NULL`; callback returns `-ENODEV`. |
| 3 | **Unlocked frame buffer.** `ws2812_set_pixel()` modified the buffer without any lock while `spi_write()` could be reading it. Concurrent per-LED trigger/blink work items make this reachable. | Colour state and pixel writes now happen under `drv->mutex`, the same mutex that covers the SPI write. |
| 4 | **Registration race.** LEDs were registered before being added to `drv->leds`, so a trigger could call the brightness callback before the lookup list contained the node. | Node is added to the list before `led_classdev_register()`. |
| 5 | **Remove ordering.** The strip was cleared before LEDs were unregistered, so a trigger could re-light it afterwards. | Unregister LEDs first (stops triggers and flushes their brightness work), then `cancel_delayed_work_sync()`, then clear the strip with a synchronous write. |
| 6 | **Trigger showed the DTS colour, not the current colour.** The netdev trigger drives the main `ws-led{n}`, and each main-LED brightness change recomputes `color` from `origin_color` (the DTS preset). Writes to `ws-led{n}:red/green/blue` only changed the live `color`, so the next trigger event overwrote them. | Channel writes now also update `origin_color`. Direct channel writes still show immediately (behaviour unchanged). The comment on `origin_color` was corrected ("origin color is readonly" no longer true). |

### Behaviour notes
- Channel colour changes are not persistent. After a reload or reboot the
  colour returns to the DTS `color-value`.
- The trigger drives the main LED at `max_brightness` (255 unless set lower in
  the DTS), so the colour appears at that level.

### Known, left alone (out of scope)
- Return values of `led_classdev_register()` in probe are not checked; an
  error path there would leak earlier registrations.

---

## Touched code (summary)

- `struct driver_data`: + `flush_work`, `last_flush`
- `static unsigned int max_update_hz` + `module_param`
- `__find_wsled()` (new), `__compare_set_brightness()` (rewritten)
- `ws2812_vleds_update()` (now records `last_flush`), `ws2812_vleds_flush()` (new),
  `ws2812_vleds_request_update()` (new)
- `ws2812_vleds_probe()`: `INIT_DELAYED_WORK`, `last_flush` init,
  `linux,default-trigger`, list-add-before-register
- `ws2812_vleds_remove()`: reordered teardown
- Header comment documents the trigger usage and rate limiting

