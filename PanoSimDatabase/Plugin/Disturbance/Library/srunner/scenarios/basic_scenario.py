#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provide BasicScenario, the basic class of all the scenarios.
"""

from __future__ import print_function

import operator
import py_trees

from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (WaitForBlackboardVariable, InTimeToArrivalToLocation)
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import WaitForever
from srunner.scenariomanager.data_provider import PanoSimDataProvider, PanoSimLocation, PanoSimTransform
from srunner.scenariomanager.timer import TimeOut
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import UpdateAllActorControls
from srunner.scenariomanager.scenarioatomics.atomic_criteria import Criterion


class BasicScenario(object):
    def __init__(self, name, ego_vehicles, config, world, debug_mode=False, terminate_on_failure=False, criteria_enable=False):
        self.name = name
        self.ego_vehicles = ego_vehicles
        self.other_actors = []
        self.parking_slots = []
        self.config = config
        self.world = world
        self.debug_mode = debug_mode
        self.terminate_on_failure = terminate_on_failure
        self.criteria_enable = criteria_enable

        self.route_mode = bool(config.route)
        self.behavior_tree = None
        self.criteria_tree = None

        if not hasattr(self, 'timeout'):
            self.timeout = 60
        if debug_mode:
            py_trees.logging.level = py_trees.logging.Level.DEBUG

        if not self.route_mode:
            self._initialize_environment(world)

        self._initialize_actors(config)

        if PanoSimDataProvider.is_runtime_init_mode():
            world.wait_for_tick()
        elif PanoSimDataProvider.is_sync_mode():
            world.tick()
        else:
            world.wait_for_tick()

        self.scenario_tree = py_trees.composites.Parallel(name, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        self.behavior_tree = py_trees.composites.Sequence()

        trigger_behavior = self._setup_scenario_trigger(config)
        if trigger_behavior:
            self.behavior_tree.add_child(trigger_behavior)

        scenario_behavior = self._create_behavior()
        if scenario_behavior is not None:
            self.behavior_tree.add_child(scenario_behavior)
            self.behavior_tree.name = scenario_behavior.name

        end_behavior = self._setup_scenario_end(config)
        if end_behavior:
            self.behavior_tree.add_child(end_behavior)

        lights = self._create_lights_behavior()
        if lights:
            self.scenario_tree.add_child(lights)

        weather = self._create_weather_behavior()
        if weather:
            self.scenario_tree.add_child(weather)

        self.scenario_tree.add_child(self.behavior_tree)

        if self.criteria_enable:
            criteria = self._create_test_criteria()

            if isinstance(criteria, py_trees.composites.Composite):
                self.criteria_tree = criteria

            elif isinstance(criteria, list):
                for criterion in criteria:
                    criterion.terminate_on_failure = terminate_on_failure

                self.criteria_tree = py_trees.composites.Parallel(name="Test Criteria", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
                self.criteria_tree.add_children(criteria)
                self.criteria_tree.setup(timeout=1)

            else:
                raise ValueError("WARNING: Scenario {} couldn't be setup, make sure the criteria is either a list or a py_trees.composites.Composite".format(self.name))

            self.scenario_tree.add_child(self.criteria_tree)

        self.timeout_node = self._create_timeout_behavior()
        if self.timeout_node:
            self.scenario_tree.add_child(self.timeout_node)

        self.scenario_tree.add_child(UpdateAllActorControls())

        self.scenario_tree.setup(timeout=1)

    def _initialize_environment(self, world):
        world.set_weather(self.config.weather)

        if self.config.friction is not None:
            friction_bp = world.get_blueprint_library().find('static.trigger.friction')
            extent = PanoSimLocation(1000000.0, 1000000.0, 1000000.0)
            friction_bp.set_attribute('friction', str(self.config.friction))
            friction_bp.set_attribute('extent_x', str(extent.x))
            friction_bp.set_attribute('extent_y', str(extent.y))
            friction_bp.set_attribute('extent_z', str(extent.z))

            transform = PanoSimTransform()
            transform.location = PanoSimLocation(-10000.0, -10000.0, 0.0)
            world.spawn_actor(friction_bp, transform)

    def _initialize_actors(self, config):
        if config.other_actors:
            new_actors = PanoSimDataProvider.request_new_actors(config.other_actors)
            if not new_actors:
                raise Exception("Error: Unable to add actors")
            for new_actor in new_actors:
                self.other_actors.append(new_actor)

    def _setup_scenario_trigger(self, config):
        if config.trigger_points and config.trigger_points[0]:
            start_location = config.trigger_points[0].location
        else:
            return None

        if not self.route_mode or config.route_var_name is None:
            return InTimeToArrivalToLocation(self.ego_vehicles[0], 2.0, start_location)

        check_name = "WaitForBlackboardVariable: {}".format(config.route_var_name)
        return WaitForBlackboardVariable(config.route_var_name, True, False, name=check_name)

    def _setup_scenario_end(self, config):
        if not self.route_mode or config.route_var_name is None:
            return None

        end_sequence = py_trees.composites.Sequence()
        name = "Reset Blackboard Variable: {} ".format(config.route_var_name)
        end_sequence.add_child(py_trees.blackboard.SetBlackboardVariable(name, config.route_var_name, False))
        end_sequence.add_child(WaitForever())

        return end_sequence

    def _create_behavior(self):
        raise NotImplementedError(
            "This function is re-implemented by all scenarios"
            "If this error becomes visible the class hierarchy is somehow broken")

    def _create_test_criteria(self):
        raise NotImplementedError(
            "This function is re-implemented by all scenarios"
            "If this error becomes visible the class hierarchy is somehow broken")

    def _create_weather_behavior(self):
        pass

    def _create_lights_behavior(self):
        pass

    def _create_timeout_behavior(self):
        return TimeOut(self.timeout, name="TimeOut")

    def change_control(self, control):
        return control

    def get_criteria(self):
        criteria = []
        if not self.criteria_tree:
            return criteria

        criteria_nodes = self._extract_nodes_from_tree(self.criteria_tree)
        for criterion in criteria_nodes:
            if isinstance(criterion, Criterion):
                criteria.append(criterion)

        return criteria

    def _extract_nodes_from_tree(self, tree):
        node_list = [tree]
        more_nodes_exist = True
        while more_nodes_exist:
            more_nodes_exist = False
            for node in list(node_list):
                if node.children:
                    node_list.remove(node)
                    more_nodes_exist = True
                    for child in node.children:
                        node_list.append(child)

        if len(node_list) == 1 and isinstance(node_list[0], py_trees.composites.Parallel):
            return []

        return node_list

    def terminate(self):
        node_list = self._extract_nodes_from_tree(self.scenario_tree)
        for node in node_list:
            node.terminate(py_trees.common.Status.INVALID)

        actor_dict = {}
        try:
            check_actors = operator.attrgetter("ActorsWithController")
            actor_dict = check_actors(py_trees.blackboard.Blackboard())
        except AttributeError:
            pass
        for actor_id in actor_dict:
            actor_dict[actor_id].reset()
        py_trees.blackboard.Blackboard().set("ActorsWithController", {}, overwrite=True)

    def remove_all_actors(self):
        if not hasattr(self, 'other_actors'):
            return
        for i, _ in enumerate(self.other_actors):
            if self.other_actors[i] is not None:
                if PanoSimDataProvider.actor_id_exists(self.other_actors[i].id):
                    PanoSimDataProvider.remove_actor_by_id(self.other_actors[i].id)
                self.other_actors[i] = None
        self.other_actors = []

    def get_parking_slots(self):
        return self.parking_slots
