"""
Copyright (c) 2020 Imagimob AB. Distributed under MIT license.
â€‹"""

import time
from typing import List

import numpy as np

class SensorConnection():
    """
    Abstract base class that defines a connection to a sensor. All functions
    must be overridden by subclass for the CaptureServer to be able to work.
    """

    def __init__(self, *args, **kwargs):
        raise NotImplementedError

    def connect(self) -> None:
        """
        Setup the connection to the sensor, e.g. connecting over serial port
        and sending configuring commands.
        """
        raise NotImplementedError

    def read_data(self) -> np.ndarray:
        """
        Read the data buffer from the sensor. Should return a n-dimensional
        numpy array.
        """
        raise NotImplementedError

    def disconnect(self) -> None:
        """
        Disconnect from the sensor
        """
        raise NotImplementedError

    def get_json_packet(self) -> List[dict]:
        """
        A list of dictionaries data packet (aka json) that describes the data format that
        is to be sent to the app. E.g.:
        [
            {
                "type": "int", # or "float"
                "count": 3,
                "tag": "accelerometer"
            }
        ]
        """
        raise NotImplementedError


class RandomNumberSensorConnection(SensorConnection):
    """
    Simulates a sensor returning a N-dimensional signal of random values.
    Can generate both int or float values depending on the value of type.
    """

    def __init__(self, *args, **kwargs):
        self._min_value = kwargs.get("min_value", 0)
        self._max_value = kwargs.get("max_value", 100)
        self._dimensions = kwargs.get("dimensions", 3)
        self._sample_time = kwargs.get("sample_time", 1.0 / 10.0)
        self._type = kwargs.get("type", "int")
        if self._type == "int":
            self._random_func = np.random.randint
        else:
            # Assume float
            self._random_func = np.random.uniform
        self._connected = False
        self._last_sensor_reading = time.time()

    def connect(self) -> None:
        self._connected = True
        print("Random sensor connected.")

    def read_data(self) -> np.ndarray:
        if not self._connected:
            raise RuntimeError("Random sensor not connected.")

        elapsed_time = time.time() - self._last_sensor_reading
        if elapsed_time > self._sample_time:
            sensor_data = self._random_func(self._min_value, self._max_value, size=self._dimensions)
            self._last_sensor_reading = time.time()
        else:
            sensor_data = None

        return sensor_data

    def disconnect(self):
        self._connected = False
        print("Random sensor disconnected.")

    def get_json_packet(self):
        return [{"type": self._type, "count": self._dimensions, "tag": "random"}]
