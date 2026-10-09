# freebsd-ssam

A FreeBSD kernel module for the Microsoft Surface System Aggregator
Module (SSAM).

On newer Surface devices, the battery and the AC adapter are not
available through the ACPI Embedded Controller. An embedded controller
(the SAM) gives this data through a serial link to an Intel LPSS UART.
FreeBSD has no driver for this link, so the battery status is not
available. This module adds the driver.

## Features

- Operates the Intel LPSS UART that the ACPI device `MSHW0084` uses.
- Uses the Surface Serial Hub (SSH) protocol to send requests to the SAM.
- Adds a `battery` device that registers with the ACPI battery
  framework. Then `acpiconf -i 0`, `apm`, and the `hw.acpi.battery.*`
  sysctls work.
- Adds `hw.acpi.acline` if the system has no ACPI AC adapter.
- Sends the `ACAD` and `CMBAT` notifications to devd, the same as
  `acpi_acad` and `acpi_cmbat`. Then `powerd` and the `power_profile`
  service see the AC changes.
- Shows statistics and debug controls in `dev.ssam.0.*`.

## Supported hardware

| Device | UART | Status |
|--------|------|--------|
| Surface Pro 7 | Intel Ice Lake LPSS UART (`8086:34a8`) | Tested |

Other Surface devices with the SAM can work, but they need their PCI ID
in `ssam_probe()`.

## Requirements

- FreeBSD 15 (amd64). The module was tested on 15.1-STABLE.
- The kernel source tree that matches the running kernel.

## Build

1. Get the kernel source that matches the running kernel, for example
   in `/usr/src`.
2. Build the module:

   ```sh
   make SYSDIR=/usr/src/sys
   ```

## Install

1. Load the module for a test:

   ```sh
   sudo kldload ./ssam.ko
   ```

2. Make sure that the module found the SAM and the battery:

   ```sh
   dmesg | grep -E 'ssam|battery'
   acpiconf -i 0
   ```

3. Install the module:

   ```sh
   sudo install -o root -g wheel -m 444 ssam.ko /boot/modules/
   sudo kldxref /boot/modules
   ```

4. Load the module at boot:

   ```sh
   sudo sysrc -f /boot/loader.conf ssam_load=YES
   ```

CAUTION: Build the module again after each kernel upgrade, before you
restart. A module that does not match the kernel does not load.

## Known problems

- `apm` always shows `AC Line status: on-line`. The kernel reads this
  value only from the ACPI AC adapter driver (`acpi_acad`).
- The driver does not enable the SAM events. It reads the AC status and
  the battery status every `dev.battery.0.poll_interval` seconds
  (default 5). Thus a notification can come up to 5 seconds late.
- MSI does not work on the Surface Pro 7. The driver uses INTx.

## References

- The SSH protocol documentation in the Linux kernel:
  `Documentation/driver-api/surface_aggregator/`.
- The linux-surface project: <https://github.com/linux-surface>.

## License

BSD 2-Clause. See the `LICENSE` file.
