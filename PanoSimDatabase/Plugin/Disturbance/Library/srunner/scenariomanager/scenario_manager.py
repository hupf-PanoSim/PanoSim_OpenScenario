#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the ScenarioManager implementation.
It must not be modified and is for reference only!
"""

from __future__ import print_function
import sys
import time

import py_trees

from srunner.autoagents.agent_wrapper import AgentWrapper
from srunner.scenariomanager.data_provider import PanoSimDataProvider, Timestamp
from srunner.scenariomanager.result_writer import ResultOutputProvider
from srunner.scenariomanager.timer import GameTime


class ScenarioManager(object):

    def __init__(self, debug_mode=False, sync_mode=False, timeout=2.0):
        self.scenario = None
        self.scenario_tree = None
        self.ego_vehicles = None
        self.other_actors = None

        self._debug_mode = debug_mode
        self._agent = None
        self._sync_mode = sync_mode
        self._timeout = timeout

        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None

        self.start_game_time = GameTime.get_time()
        self.timestamp = Timestamp()
        self.start_time = time.time()

    def _reset(self):
        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        GameTime.restart()

    def cleanup(self):
        if self.scenario is not None:
            self.scenario.terminate()

        if self._agent is not None:
            self._agent.cleanup()
            self._agent = None

        PanoSimDataProvider.cleanup()

    def load_scenario(self, scenario, agent=None):
        self._reset()
        self._agent = AgentWrapper(agent) if agent else None
        if self._agent is not None:
            self._sync_mode = True
        self.scenario = scenario
        self.scenario_tree = self.scenario.scenario_tree
        self.ego_vehicles = scenario.ego_vehicles
        self.other_actors = scenario.other_actors

        # To print the scenario tree uncomment the next line
        # py_trees.display.render_dot_tree(self.scenario_tree)

        if self._agent is not None:
            self._agent.setup_sensors(self.ego_vehicles[0], self._debug_mode)

    def scenario_start(self):
        print("ScenarioManager: Running scenario {}".format(self.scenario_tree.name))
        self._running = True
        self.start_system_time = time.time()
        self.start_game_time = GameTime.get_time()
        self.timestamp = Timestamp()
        self.start_time = 0

    def ModelOutput(self, userData):
        self.timestamp.elapsed_seconds = (userData['time'] - self.start_time) / 1000
        self.timestamp.frame += 1
        if self.timestamp:
            self._tick_scenario(self.timestamp, userData)

    def ModelTerminate(self):
        self.cleanup()
        self.end_system_time = time.time()
        end_game_time = GameTime.get_time()
        self.scenario_duration_system = self.end_system_time - self.start_system_time
        self.scenario_duration_game = end_game_time - self.start_game_time
        if self.scenario_tree.status == py_trees.common.Status.FAILURE:
            print("ScenarioManager: Terminated due to failure")

    def _tick_scenario(self, timestamp, userData):
        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds

            GameTime.on_carla_tick(timestamp)

            bus_trafffic = userData["bus_traffic"].getReader(userData["time"])
            _, width = bus_trafffic.readHeader()
            for index in range(width):
                id, _, _, x, y, z, yaw, pitch, roll, speed = bus_trafffic.readBody(index)
                if id in PanoSimDataProvider._actor_pool.keys():
                    actor = PanoSimDataProvider._actor_pool[id]
                    if actor.actor_category == 'bicycle':
                        actor.transform.location.x = x
                        actor.transform.location.y = y
                        actor.transform.location.z = z
                        actor.transform.rotation.yaw = yaw
                        actor.transform.rotation.pitch = pitch
                        actor.transform.rotation.roll = roll
                        actor.speed = speed
                    break

            PanoSimDataProvider.on_carla_tick()

            if self._agent is not None:
                ego_action = self._agent()

            if self._agent is not None:
                self.ego_vehicles[0].apply_control(ego_action)

            self.scenario_tree.tick_once()

            if self._debug_mode:
                print("\n")
                py_trees.display.print_ascii_tree(self.scenario_tree, show_status=True)
                sys.stdout.flush()

            if self.scenario_tree.status != py_trees.common.Status.RUNNING:
                self._running = False

        if self._sync_mode and self._running:
            PanoSimDataProvider.get_world().tick()

    def stop_scenario(self):
        self._running = False

    def analyze_scenario(self, stdout, filename, junit, json):
        failure = False
        timeout = False
        result = "SUCCESS"

        criteria = self.scenario.get_criteria()
        if len(criteria) == 0:
            print("Nothing to analyze, this scenario has no criteria")
            return True

        for criterion in criteria:
            if (not criterion.optional and criterion.test_status != "SUCCESS" and criterion.test_status != "ACCEPTABLE"):
                failure = True
                result = "FAILURE"
            elif criterion.test_status == "ACCEPTABLE":
                result = "ACCEPTABLE"

        if self.scenario.timeout_node.timeout and not failure:
            timeout = True
            result = "TIMEOUT"

        output = ResultOutputProvider(self, result, stdout, filename, junit, json)
        output.write()

        return failure or timeout
