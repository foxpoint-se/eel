"""HTTP reachability check used by modem logic (not chip I/O)."""

import requests
from requests.exceptions import RequestException

DEFAULT_PING_URL = "http://connectivitycheck.gstatic.com/generate_204"


def http_ping(url: str = DEFAULT_PING_URL, timeout_s: float = 1.0) -> bool:
    try:
        response = requests.get(url, timeout=timeout_s)
        return 200 <= response.status_code < 400
    except RequestException:
        return False
