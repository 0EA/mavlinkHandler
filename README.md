# mavlinkHandler

[![PyPI](https://img.shields.io/pypi/v/mavlinkhandler.svg)](https://pypi.org/project/mavlinkhandler/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/your-username/mavlinkHandler/blob/main/LICENSE)

mavlinkHandler is a controller library for UAVs, compatible with both ArduPilot and DroneKit.

## Installation
You can install mavlinkHandler using pip:
```
pip install mavlinkHandler
```

## Usage
```
from mavlinkHandler import MAVLinkHandlerDronekit as MAVLinkHandler

mavlink_handler = MAVLinkHandler('127.0.0.1:14591')
```
