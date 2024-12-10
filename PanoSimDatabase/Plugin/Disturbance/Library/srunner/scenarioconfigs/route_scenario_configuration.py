#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the key configuration parameters for a route-based scenario
"""

from srunner.scenariomanager.data_provider import PanoSimLocation, PanoSimRoadOption
from srunner.scenarioconfigs.scenario_configuration import ScenarioConfiguration


class RouteConfiguration(object):

    """
    This class provides the basic  configuration for a route
    """

    def __init__(self, route=None):
        self.data = route

    def parse_xml(self, node):
        """
        Parse route config XML
        """
        self.data = []

        for waypoint in node.iter("waypoint"):
            x = float(waypoint.attrib.get('x', 0))
            y = float(waypoint.attrib.get('y', 0))
            z = float(waypoint.attrib.get('z', 0))
            c = waypoint.attrib.get('connection', '')
            connection = PanoSimRoadOption[c.split('.')[1]]

            self.data.append((PanoSimLocation(x, y, z), connection))


class RouteScenarioConfiguration(ScenarioConfiguration):

    """
    Basic configuration of a RouteScenario
    """

    def __init__(self):
        super(RouteScenarioConfiguration, self).__init__()
        self.keypoints = None
        self.scenario_configs = []
