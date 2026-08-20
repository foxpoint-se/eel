"""HTTP reachability check used by modem logic (not chip I/O)."""

import requests
from requests.exceptions import ConnectionError, ConnectTimeout, ReadTimeout


def http_ping(url: str = "https://8.8.8.8", timeout_s: float = 1.0) -> bool:
    try:
        response = requests.get(url, timeout=timeout_s)
        return response.status_code == 200
    except (ConnectionError, ConnectTimeout, ReadTimeout):
        return False
