# Modem: wifi fallback then cellular

> Archived from the eel README. **Not** what `scripts/modem/install_modem_service.sh` does today (that always starts `quectel-CM`). Preserved as a design idea.

## Idea

1. Prefer wifi when it has an expected IPv4 address.
2. If that address never appears (e.g. within ~20s), start the cellular modem with the APN, e.g.:

```bash
quectel-CM -s 4g.tele2.se
```

## Current install path

For production install, use:

```bash
make install-modem
```

That points at `scripts/modem/install_modem_software.sh` and `scripts/modem/install_modem_service.sh` (Sixfab QMI + systemd modem service). OpenVPN/WireGuard paths from the old README are unused and were removed.
