# Local certs and config

To be able to simplify mapping to config and certs for the MQTT bridge node, move your files here.

Then, edit your `iot_config.json` and replace your paths with the one that is passed to the MQTT node's `path_for_config` param.

For example:
- Replace: `/home/my-user/somewhere/local_certs_and_config/`
- with: `/eel/local_certs_and_config/`
