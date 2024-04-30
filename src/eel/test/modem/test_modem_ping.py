import requests


def ping_server():
    url = "https://8.8.8.8"
    timeout_value = 2.0
    
    try:
        response = requests.get(url, timeout=timeout_value)
        status_code = response.status_code
        response_time_s = response.elapsed.total_seconds()
    except (requests.exceptions.ConnectionError, requests.exceptions.ConnectTimeout) as e:
        status_code = 408
        response_time_s = -1

    print(type(status_code))
    print(type(response_time_s))
    print(response_time_s)

    assert status_code == 200
    assert response_time_s < timeout_value


if __name__ == "__main__":
    ping_server()
