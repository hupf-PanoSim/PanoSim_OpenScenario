#!/usr/bin/env python

# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides some basic unit tests for the OpenSCENARIO feature of ScenarioRunner
"""

from unittest import TestCase
import glob

from srunner.scenarioconfigs.openscenario_configuration import OpenScenarioConfiguration
from srunner.scenariomanager.data_provider import PanoSimDataProvider, PanoSimClient
from srunner.scenarios.open_scenario import OpenScenario


class TestLoadingXOSC(TestCase):
    """
    Test class to load OpenSCENARIO files and test for exceptions
    """

    def test_all_xosc(self):
        """
        Load all examples OpenSCENARIO files
        """
        all_test_files = glob.glob('**/srunner/examples/*.xosc', recursive=True)

        for filename in all_test_files:
            client = PanoSimClient()
            config = OpenScenarioConfiguration(filename, client, {})
            self.assertTrue(config is not None)
            PanoSimDataProvider.set_client(client)
            ego_vehicles = []
            for vehicle in config.ego_vehicles:
                ego_vehicles.append(PanoSimDataProvider.request_new_actor(vehicle.model, vehicle.transform, vehicle.rolename, color=vehicle.color, actor_category=vehicle.category))

            scenario = OpenScenario(world=client.get_world(), ego_vehicles=ego_vehicles, config=config, config_file=filename, timeout=100000)
            self.assertTrue(scenario is not None)

            PanoSimDataProvider.cleanup()
