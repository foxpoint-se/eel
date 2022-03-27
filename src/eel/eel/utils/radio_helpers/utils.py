import json


def from_json(data):
    return json.loads(data)


def to_json(obj):
    return json.dumps(obj, default=lambda o: o.__dict__)


def to_json_filtered(obj):
    only_set_keys = {k: v for k, v in obj.__dict__.items() if v is not None}
    return to_json(only_set_keys)


def instance_to_str(obj):
    return "{}\n{}".format(obj.__repr__(), to_json(obj))
