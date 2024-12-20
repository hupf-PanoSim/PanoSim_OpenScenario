#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


from __future__ import print_function

import math
import re
import threading
from numpy import random
from six import iteritems

# from agents.navigation.global_route_planner import GlobalRoutePlanner

import copy
from enum import Enum, IntEnum
import numpy as np

from TrafficModelInterface import *


class PanoSimCommand:
    blueprint = None

    def SpawnActor(blueprint, point):
        new_command = PanoSimCommand()
        new_command.blueprint = copy.deepcopy(blueprint)
        return new_command

    def SetSimulatePhysics(blueprint, physics):
        return None

    def FutureActor():
        return None

    def ApplyTransform():
        return None

    def SetAutopilot(actor, autopilot, port):
        return None

    def SetVehicleLightState():
        return None

    def DestroyActor(actor):
        return None

    def then(self, other_command):
        return self


class PanoSimActorAttributeType(IntEnum):
    Bool    = 0
    Int     = 1
    Float   = 2
    String  = 3
    RGBColor= 4
    SIZE    = 5
    INVALID = 6


class PanoSimRoadOption(IntEnum):
    VOID            = -1
    LEFT            = 1
    RIGHT           = 2
    STRAIGHT        = 3
    LANEFOLLOW      = 4
    CHANGELANELEFT  = 5
    CHANGELANERIGHT = 6
    ROADEND         = 7


class PanoSimLaneType(IntEnum):
    NONE          = 0x1
    Driving       = 0x1 << 1
    Stop          = 0x1 << 2
    Shoulder      = 0x1 << 3
    Biking        = 0x1 << 4
    Sidewalk      = 0x1 << 5
    Border        = 0x1 << 6
    Restricted    = 0x1 << 7
    Parking       = 0x1 << 8
    Bidirectional = 0x1 << 9
    Median        = 0x1 << 10
    Special1      = 0x1 << 11
    Special2      = 0x1 << 12
    Special3      = 0x1 << 13
    RoadWorks     = 0x1 << 14
    Tram          = 0x1 << 15
    Rail          = 0x1 << 16
    Entry         = 0x1 << 17
    Exit          = 0x1 << 18
    OffRamp       = 0x1 << 19
    OnRamp        = 0x1 << 20
    Any           = -2 // 0xFFFFFFFE


class PanoSimVehicleLightState(IntEnum):
    Position     = 0x1
    LowBeam      = 0x1 << 1
    HighBeam     = 0x1 << 2
    Brake        = 0x1 << 3
    RightBlinker = 0x1 << 4
    LeftBlinker  = 0x1 << 5
    Reverse      = 0x1 << 6
    Fog          = 0x1 << 7
    Interior     = 0x1 << 8
    Special1     = 0x1 << 9
    Special2     = 0x1 << 10
    All          = 0xFFFFFFFF


class PanoSimLightGroup(IntEnum):
    NONE    = 0
    Vehicle = 1
    Street  = 2
    Building= 3
    Other   = 4


class PanoSimTrafficLightState(Enum):
    Red     = 0
    Yellow  = 1
    Green   = 2
    Off     = 3
    Unknown = 4
    SIZE    = 5


class PanoSimLaneChange(Enum):
    NONE    = 0
    Right   = 1
    Left    = 2
    Both    = 3


class PanoSimVehicleDoor(Enum):
    FL  = 0
    FR  = 1
    RL  = 2
    RR  = 3
    All = 6


class PanoSimColor:
    def __init__(self, r=0, g=0, b=0, a=255):
        self.r = r
        self.g = g
        self.b = b
        self.a = a


class PanoSimLightGroup(Enum):
    NONE    = 0
    Vehicle = 1
    Street  = 2
    Building= 3
    Other   = 4


class PanoSimGearPhysicsControl:
    def __init__(self, ratio=1.0, down_ratio=0.5, up_ratio=0.65):
        self.ratio = ratio
        self.down_ratio = down_ratio
        self.up_ratio = up_ratio


class PanoSimBluePrint(object):
    def __init__(self):
        self.id = -1
        self.attributes = {'role_name': ''}

    def has_attribute(self, attribute_string=''):
        return attribute_string in self.attributes

    def set_attribute(self, key, value):
        self.attributes[key] = value

    def has_tag(self, tag_string=''):
        return False


class PanoSimBluePrintLibrary:
    def filter(self, filterstring):
        return [PanoSimBluePrint()]

    def __len__(self):
        return 1

    def find(self, filterstring):
        return PanoSimBluePrint()


class PanoSimWeather:
    def __init__(self, cloudiness = -1.0, precipitation = -1.0, precipitation_deposits = -1.0,
                 wind_intensity = -1.0, sun_azimuth_angle = -1.0, sun_altitude_angle = -1.0,
                 fog_density = -1.0, fog_distance = -1.0, fog_falloff = -1.0, wetness = -1.0,
                 scattering_intensity = 1.0, mie_scattering_scale = 0.03, rayleigh_scattering_scale = 0.0331, dust_storm = 0.0):
        self.cloudiness = cloudiness
        self.precipitation = precipitation
        self.precipitation_deposits = precipitation_deposits
        self.wind_intensity = wind_intensity
        self.sun_azimuth_angle = sun_azimuth_angle
        self.sun_altitude_angle = sun_altitude_angle
        self.fog_density = fog_density
        self.fog_distance = fog_distance
        self.fog_falloff = fog_falloff
        self.wetness = wetness
        self.scattering_intensity = scattering_intensity
        self.mie_scattering_scale = mie_scattering_scale
        self.rayleigh_scattering_scale = rayleigh_scattering_scale
        self.dust_storm = dust_storm


class PanoSimGeoLocation:
    longitude = 0
    latitude = 0


class PanoSimVector2D:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y


class PanoSimVector3D:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


class PanoSimLocation():
    def __init__(self, x=0, y=0, z=0):
        if isinstance(x, PanoSimVector3D):
            self.x = x.x
            self.y = x.y
            self.z = x.z
        else:
            self.x = x
            self.y = y
            self.z = z

    def __add__(self, other):
        return PanoSimLocation(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return PanoSimLocation(self.x - other.x, self.y - other.y, self.z - other.z)

    def distance(self, other):
        return math.sqrt(pow((self.x - other.x), 2) + pow((self.y - other.y), 2) + pow((self.z - other.z), 2))


class PanoSimRotation():
    def __init__(self, yaw=0, pitch=0, roll=0):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw

    def get_forward_vector(self):
        return PanoSimVector3D()


class PanoSimTransform:
    def __init__(self, location=PanoSimLocation(0, 0, 0), rotation=PanoSimRotation(0, 0, 0)):
        self.location = PanoSimLocation(location.x, location.y, location.z)
        self.rotation = PanoSimRotation(rotation.yaw, rotation.pitch, rotation.roll)
        self.type = ''
        self.data = {}
        self._get_actor_transform = True

    def get_forward_vector(self):
        return PanoSimLocation()

    def get_right_vector(self):
        return PanoSimLocation()

    def get_up_vector(self):
        return PanoSimLocation()

    def get_matrix(self):
        return None

    def get_inverse_matrix(self):
        return None


class PanoSimBoundingBox:
    def __init__(self, location=PanoSimLocation(0, 0, 0), extent=PanoSimVector3D(), rotation=PanoSimRotation(0, 0, 0)):
        self.location = PanoSimLocation(location.x, location.y, location.z)
        self.extent = PanoSimVector3D(extent.x, extent.y, extent.z)
        self.rotation = PanoSimRotation(rotation.yaw, rotation.pitch, rotation.roll)


class PanoSimWaypoint():
    transform = PanoSimTransform(PanoSimLocation(), PanoSimRotation())
    road_id = 0
    lane_id = 0
    s = 0
    lane_width = 0
    lane_type = PanoSimLaneType.NONE

    def next(self, step):
        return None


class PanoSimWheelPhysicsControl:
    # tire_friction = 2.0
    # damping_rate = 0.25
    # max_steer_angle = 70.0
    # radius = 30.0
    # max_brake_torque = 1500.0
    # max_handbrake_torque = 3000.0
    # lat_stiff_max_load = 2.0
    # lat_stiff_value = 17.0
    # long_stiff_value = 1000.0
    # position = PanoSimVector3D()

    def __init__(self, tire_friction, damping_rate, max_steer_angle, radius, max_brake_torque, max_handbrake_torque, lat_stiff_max_load, lat_stiff_value, long_stiff_value, position):
        self.tire_friction = tire_friction
        self.damping_rate = damping_rate
        self.max_steer_angle = max_steer_angle
        self.radius = radius
        self.max_brake_torque = max_brake_torque
        self.max_handbrake_torque = max_handbrake_torque
        self.lat_stiff_max_load = lat_stiff_max_load
        self.lat_stiff_value = lat_stiff_value
        self.long_stiff_value = long_stiff_value
        self.position = PanoSimVector3D(position.x, position.y, position.z)


class PanoSimVehiclePhysicsControl:
    torque_curve = [PanoSimVector2D(0.0, 500.0), PanoSimVector2D(5000.0, 500.0)]
    max_rpm = 5000.0
    moi = 1.0
    damping_rate_full_throttle = 0.15
    damping_rate_zero_throttle_clutch_engaged = 2.0
    damping_rate_zero_throttle_clutch_disengaged = 0.35
    use_gear_autobox = True;
    gear_switch_time = 0.5
    clutch_strength = 10.0
    final_ratio = 4.0
    forward_gears = []
    mass = 1000.0
    drag_coefficient = 0.3
    center_of_mass = PanoSimLocation()
    steering_curve = [PanoSimVector2D(0.0, 1.0), PanoSimVector2D(10.0, 0.5)]
    wheels = []
    use_sweep_wheel_collision = False


class PanoSimControl:
    def __init__(self):
        self.steer = 0
        self.throttle = 0
        self.brake = 0


class PanoSimVehicleControl(PanoSimControl):
    def __init__(self):
        self.hand_brake = False
        self.reverse = False
        self.manual_gear_shift = False
        self.gear = 0


class PanoSimWorldSettings:
    def __init__(self):
        self.synchronous_mode = False
        self.no_rendering_mode = False
        self.fixed_delta_seconds = 0
        self.substepping = True
        self.max_substep_delta_time = 0.01
        self.max_substeps = 10
        self.max_culling_distance = 0
        self.deterministic_ragdolls = False


class PanoSimActorList:
    def __init__(self, actor_list):
        self.actor_list = actor_list

    def filter(self, filterstring):
        return []

    def __len__(self):
        return len(self.actor_list)

    def __getitem__(self, i):
        return self.actor_list[i]


class PanoSimMap:
    name = ""

    def get_spawn_points(self):
        return []

    def transform_to_geolocation(self, transform):
        return PanoSimGeoLocation()

    def get_waypoint(self, location, project_to_road=True, lane_type= PanoSimLaneType.Driving):
        return PanoSimWaypoint()

    def get_waypoint_xodr(self, a, b, c):
        return PanoSimWaypoint()

    def get_topology(self):
        return []


class PanoSimActor:
    def __init__(self) -> None:
        self.attributes = {'role_name': ''}

        self.id = -1
        self.type_id = None
        self.transform = PanoSimTransform(PanoSimLocation(), PanoSimRotation())
        self.world = PanoSimWorld()
        self.ctrl = PanoSimControl()
        self.speed = 0
        self.model = None
        self.actor_category = None
        self.is_alive = True
        self.bounding_box = PanoSimBoundingBox()

    def get_transform(self):
        if self.actor_category == 'bicycle' or self.actor_category == 'pedestrian':
            return self.transform
        else:
            if PanoSimDataProvider._timestamp > 0:
                self.transform.location.x = getVehicleX(self.id)
                self.transform.location.y = getVehicleY(self.id)
                self.transform.location.z = getVehicleZ(self.id)
                self.transform.rotation.yaw = getVehicleYaw(self.id)
            return self.transform

    def get_location(self):
        if self.actor_category == 'bicycle' or self.actor_category == 'pedestrian':
            return self.transform.location
        else:
            if PanoSimDataProvider._timestamp > 0:
                self.transform.location.x = getVehicleX(self.id)
                self.transform.location.y = getVehicleY(self.id)
                self.transform.location.z = getVehicleZ(self.id)
            return self.transform.location

    def get_world(self):
        return self.world

    def get_control(self):
        return self.ctrl

    def listen(self, callback):
        pass

    def set_autopilot(self, autopilot=True, port=8000):
        pass

    def stop(self):
        print('stop:', self.id)

    def destroy(self):
        del self

    def get_velocity(self):
        if self.actor_category == 'bicycle' or self.actor_category == 'pedestrian':
            return self.speed
        else:
            return getVehicleSpeed(self.id)


class PanoSimWalker(PanoSimActor):
    is_walker = True


class PanoSimVehicle(PanoSimActor):
    is_vehicle = True


class PanoSimWorld:
    actors = []

    def get_settings(self):
        return PanoSimWorldSettings()

    def get_map(self):
        return PanoSimMap()

    def get_blueprint_library(self):
        return PanoSimBluePrintLibrary()

    def wait_for_tick(self):
        pass

    def get_actors(self, ids=[]):
        actor_list = []
        for actor in self.actors:
            if actor.id in ids:
                actor_list.append(actor)

        return PanoSimActorList(actor_list)

    def try_spawn_actor(self, blueprint, spawn_point):
        new_actor = PanoSimVehicle()
        new_actor.attributes['role_name'] = blueprint.attributes['role_name']
        new_actor.id = len(self.actors) + 10000
        self.actors.append(new_actor)
        return new_actor

    def spawn_actor(self, blueprint, spawn_point, attach_to):
        new_actor = self.try_spawn_actor(blueprint, spawn_point)
        return new_actor

    def set_weather(self, weather):
        self.weather = weather
        # print('set_weather:', weather.cloudiness, weather.precipitation, weather.precipitation_deposits, 
        #       weather.wind_intensity, weather.sun_azimuth_angle, weather.sun_altitude_angle, weather.fog_density, 
        #       weather.fog_distance, weather.fog_falloff, weather.wetness, weather.scattering_intensity,
        #       weather.mie_scattering_scale, weather.rayleigh_scattering_scale, weather.dust_storm)
        # todo: data mapping from carla to PanoSim
        if weather.cloudiness > 90:
            temperature = 14.66
            pressure = 1
            humidity = 60
            precipitation_type = 0
            particle_size = 0.5
            particle_capacity = 0
            falling_alpha = 0
            falling_beta = 0
            falling_speed = 0.8
            fog_visibility = 499
            lighting_alpha = 0
            lighting_beta = 0
            lighting_intensity = 100
            lighting_ambient = 1
            street_light = 1
            vehicle_light = 1
            skybox = 1
            friction = 0.8
            wetness = 0
            snow = 0
            PanoSimDataProvider._bus_weather.writeHeader(*(
                PanoSimDataProvider._timestamp, temperature, pressure, humidity, precipitation_type, 
                particle_size, particle_capacity, falling_alpha, falling_beta, falling_speed, fog_visibility, lighting_alpha, 
                lighting_beta, lighting_intensity, lighting_ambient, street_light, vehicle_light, skybox, friction, wetness, snow))


class PanoSimClient:
    world = PanoSimWorld()

    def load_world(self, name):
        return None

    def get_world(self):
        return self.world

    def get_trafficmanager(self, port):
        return None

    def apply_batch_sync(self, batch, sync_mode=False):
        class Response:
            def __init__(self, id):
                self.actor_id = id
                self.error = None

        reponse_list = []
        for batch_cmd in batch:
            if batch_cmd is not None:
                new_actor = PanoSimVehicle()
                new_actor.attributes['role_name'] = batch_cmd.blueprint.attributes['role_name']
                new_actor.id = len(self.world.actors)
                self.world.actors.append(new_actor)
                reponse_list.append(Response(new_actor.id))

        return reponse_list


class PanoSimLightState:
    intensity = 0.0
    group = PanoSimLightGroup.NONE
    color = PanoSimColor()
    active = False
    def __init__(self, intensity, group, color, active):
        self.location = PanoSimVector3D()
        self.intensity = intensity
        self.group = group
        self.color = color
        self.active = active


class Timestamp:
    def __init__(self) -> None:
        self.elapsed_seconds = 0
        self.frame = 0
        self.frame_count = 0
        self.delta_seconds = 0.01
        self.platform_timestamp = 0


def PanoSim_vector(location_1, location_2):
    """
    Returns the unit vector from location_1 to location_2
        :param location_1, location_2: carla.Location objects
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps
    return [x / norm, y / norm, z / norm]


def calculate_velocity(actor):
    velocity_squared = actor.get_velocity().x**2
    velocity_squared += actor.get_velocity().y**2
    return math.sqrt(velocity_squared)


def get_speed(vehicle):
    vel = vehicle.get_velocity()
    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
    return speed


class PanoSimDataProvider(object):

    _actor_velocity_map = {}
    _actor_location_map = {}
    _actor_transform_map = {}
    _traffic_light_map = {}
    _actor_pool = {}
    _global_osc_parameters = {}
    _net_offset = (0, 0)
    _client = None
    _world = None
    _map = None
    _sync_flag = False
    _spawn_points = None
    _spawn_index = 0
    _blueprint_library = None
    _all_actors = None
    _ego_vehicle_route = None
    _traffic_manager_port = 8000
    _random_seed = 2000
    _rng = random.RandomState(_random_seed)
    _local_planner = None
    _grp = None
    _runtime_init_flag = False
    _lock = threading.Lock()
    _bus_weather = None
    _timestamp = 0

    @staticmethod
    def set_local_planner(plan):
        PanoSimDataProvider._local_planner = plan

    @staticmethod
    def get_local_planner():
        return PanoSimDataProvider._local_planner

    @staticmethod
    def register_actor(actor, transform=None):
        with PanoSimDataProvider._lock:
            if actor in PanoSimDataProvider._actor_velocity_map:
                raise KeyError("Vehicle '{}' already registered. Cannot register twice!".format(actor.id))
            else:
                PanoSimDataProvider._actor_velocity_map[actor] = 0.0
            if actor in PanoSimDataProvider._actor_location_map:
                raise KeyError("Vehicle '{}' already registered. Cannot register twice!".format(actor.id))
            elif transform:
                PanoSimDataProvider._actor_location_map[actor] = transform.location
            else:
                PanoSimDataProvider._actor_location_map[actor] = None

            if actor in PanoSimDataProvider._actor_transform_map:
                raise KeyError("Vehicle '{}' already registered. Cannot register twice!".format(actor.id))
            else:
                PanoSimDataProvider._actor_transform_map[actor] = transform

    @staticmethod
    def update_osc_global_params(parameters):
        PanoSimDataProvider._global_osc_parameters.update(parameters)

    @staticmethod
    def get_osc_global_param_value(ref):
        return PanoSimDataProvider._global_osc_parameters.get(ref.replace("$", ""))

    @staticmethod
    def register_actors(actors, transforms=None):
        if transforms is None:
            transforms = [None] * len(actors)
        for actor, transform in zip(actors, transforms):
            PanoSimDataProvider.register_actor(actor, transform)

    @staticmethod
    def keep_route():
        direction2route = {
            next_junction_direction.straight: route_type.straight,
            next_junction_direction.left: route_type.left,
            next_junction_direction.right: route_type.right,
            next_junction_direction.u_turn: route_type.turn_round
        }
        for id in getVehicleList():
            if id > 100 and getRoute(id) == next_junction_direction.unknown:
                current_lane = getVehicleLane(id)
                if len(current_lane) > 0:
                    directions = getValidDirections(current_lane)
                    if len(directions) > 0:
                        changeRoute(id, direction2route[directions[0]])
                    else:
                        junction = getToJunction(current_lane)
                        if len(junction) > 0:
                            for lane in getIncomingLanes(junction):
                                directions = getValidDirections(lane)
                                if len(directions) > 0:
                                    changeRoute(id, direction2route[directions[0]])
                                    break

    @staticmethod
    def on_carla_tick():
        with PanoSimDataProvider._lock:
            for actor in PanoSimDataProvider._actor_velocity_map:
                if actor is not None and actor.is_alive:
                    if actor.id > 100:
                        speed = actor.speed
                        changeSpeed(actor.id, actor.speed, 0)
                    else:
                        speed = getVehicleSpeed(actor.id)
                    PanoSimDataProvider._actor_velocity_map[actor] = speed
                    # PanoSimDataProvider._actor_velocity_map[actor] = calculate_velocity(actor)

            for actor in PanoSimDataProvider._actor_location_map:
                if actor is not None and actor.is_alive:
                    PanoSimDataProvider._actor_location_map[actor] = actor.get_location()

            for actor in PanoSimDataProvider._actor_transform_map:
                if actor is not None and actor.is_alive:
                    PanoSimDataProvider._actor_transform_map[actor] = actor.get_transform()

            PanoSimDataProvider.keep_route()

            world = PanoSimDataProvider._world
            if world is None:
                print("WARNING: PanoSimDataProvider couldn't find the world")

            PanoSimDataProvider._all_actors = None

    @staticmethod
    def get_velocity(actor):
        for key in PanoSimDataProvider._actor_velocity_map:
            if key.id == actor.id:
                if actor.actor_category == 'bicycle' or actor.actor_category == 'pedestrian':
                    return actor.speed
                else:
                    return getVehicleSpeed(actor.id)
                # return PanoSimDataProvider._actor_velocity_map[key]
        print('{}.get_velocity: {} not found!' .format(__name__, actor))
        return 0.0

    @staticmethod
    def get_location(actor):
        for key in PanoSimDataProvider._actor_location_map:
            if key.id == actor.id:
                location = PanoSimDataProvider._actor_location_map[key]
                return PanoSimLocation(location.x, location.y, location.z) 
        print('{}.get_location: {} not found!' .format(__name__, actor))
        return None

    @staticmethod
    def get_transform(actor):
        for key in PanoSimDataProvider._actor_transform_map:
            if key.id == actor.id:
                if PanoSimDataProvider._actor_transform_map[key] is None:
                    return actor.get_transform()
                return PanoSimDataProvider._actor_transform_map[key]
        print('{}.get_transform: {} not found!' .format(__name__, actor))
        return None

    @staticmethod
    def set_client(client):
        PanoSimDataProvider._client = client

    @staticmethod
    def get_client():
        return PanoSimDataProvider._client

    @staticmethod
    def set_world(world):
        PanoSimDataProvider._world = world
        PanoSimDataProvider._sync_flag = world.get_settings().synchronous_mode
        PanoSimDataProvider._map = world.get_map()
        PanoSimDataProvider._blueprint_library = world.get_blueprint_library()
        # PanoSimDataProvider._grp = GlobalRoutePlanner(PanoSimDataProvider._map, 2.0)
        PanoSimDataProvider.generate_spawn_points()
        PanoSimDataProvider.prepare_map()

    @staticmethod
    def get_world():
        return PanoSimDataProvider._world

    @staticmethod
    def get_map(world=None):
        if PanoSimDataProvider._map is None:
            if world is None:
                if PanoSimDataProvider._world is None:
                    raise ValueError("class member \'world'\' not initialized yet")
                else:
                    PanoSimDataProvider._map = PanoSimDataProvider._world.get_map()
            else:
                PanoSimDataProvider._map = world.get_map()

        return PanoSimDataProvider._map

    @staticmethod
    def get_random_seed():
        return PanoSimDataProvider._rng

    @staticmethod
    def get_global_route_planner():
        return PanoSimDataProvider._grp

    @staticmethod
    def get_all_actors():
        if PanoSimDataProvider._all_actors:
            return PanoSimDataProvider._all_actors
        PanoSimDataProvider._all_actors = PanoSimDataProvider._world.get_actors()
        return PanoSimDataProvider._all_actors

    @staticmethod
    def is_sync_mode():
        return PanoSimDataProvider._sync_flag

    @staticmethod
    def set_runtime_init_mode(flag):
        PanoSimDataProvider._runtime_init_flag = flag

    @staticmethod
    def is_runtime_init_mode():
        return PanoSimDataProvider._runtime_init_flag

    @staticmethod
    def find_weather_presets():
        rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
        name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
        presets = [x for x in dir(PanoSimWeather) if re.match('[A-Z].+', x)]
        return [(getattr(PanoSimWeather, x), name(x)) for x in presets]

    @staticmethod
    def prepare_map():
        if PanoSimDataProvider._map is None:
            PanoSimDataProvider._map = PanoSimDataProvider._world.get_map()
        PanoSimDataProvider._traffic_light_map.clear()
        for traffic_light in PanoSimDataProvider._world.get_actors().filter('*traffic_light*'):
            if traffic_light not in list(PanoSimDataProvider._traffic_light_map):
                PanoSimDataProvider._traffic_light_map[traffic_light] = traffic_light.get_transform()
            else:
                raise KeyError("Traffic light '{}' already registered. Cannot register twice!".format(traffic_light.id))

    @staticmethod
    def annotate_trafficlight_in_group(traffic_light):
        dict_annotations = {'ref': [], 'opposite': [], 'left': [], 'right': []}
        ref_location = PanoSimDataProvider.get_trafficlight_trigger_location(traffic_light)
        ref_waypoint = PanoSimDataProvider.get_map().get_waypoint(ref_location)
        ref_yaw = ref_waypoint.transform.rotation.yaw
        group_tl = traffic_light.get_group_traffic_lights()
        for target_tl in group_tl:
            if traffic_light.id == target_tl.id:
                dict_annotations['ref'].append(target_tl)
            else:
                target_location = PanoSimDataProvider.get_trafficlight_trigger_location(target_tl)
                target_waypoint = PanoSimDataProvider.get_map().get_waypoint(target_location)
                target_yaw = target_waypoint.transform.rotation.yaw
                diff = (target_yaw - ref_yaw) % 360
                if diff > 330:
                    continue
                elif diff > 225:
                    dict_annotations['right'].append(target_tl)
                elif diff > 135.0:
                    dict_annotations['opposite'].append(target_tl)
                elif diff > 30:
                    dict_annotations['left'].append(target_tl)
        return dict_annotations

    @staticmethod
    def get_trafficlight_trigger_location(traffic_light):
        def rotate_point(point, angle):
            x_ = math.cos(math.radians(angle)) * point.x - math.sin(math.radians(angle)) * point.y
            y_ = math.sin(math.radians(angle)) * point.x - math.cos(math.radians(angle)) * point.y
            return PanoSimVector3D(x_, y_, point.z)

        base_transform = traffic_light.get_transform()
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(traffic_light.trigger_volume.location)
        area_ext = traffic_light.trigger_volume.extent
        point = rotate_point(PanoSimVector3D(0, 0, area_ext.z), base_rot)
        point_location = area_loc + PanoSimLocation(x=point.x, y=point.y)
        return PanoSimLocation(point_location.x, point_location.y, point_location.z)

    @staticmethod
    def update_light_states(ego_light, annotations, states, freeze=False, timeout=1000000000):
        reset_params = []
        for state in states:
            relevant_lights = []
            if state == 'ego':
                relevant_lights = [ego_light]
            else:
                relevant_lights = annotations[state]
            for light in relevant_lights:
                prev_state = light.get_state()
                prev_green_time = light.get_green_time()
                prev_red_time = light.get_red_time()
                prev_yellow_time = light.get_yellow_time()
                reset_params.append({'light': light,
                                     'state': prev_state,
                                     'green_time': prev_green_time,
                                     'red_time': prev_red_time,
                                     'yellow_time': prev_yellow_time})

                light.set_state(states[state])
                if freeze:
                    light.set_green_time(timeout)
                    light.set_red_time(timeout)
                    light.set_yellow_time(timeout)

        return reset_params

    @staticmethod
    def reset_lights(reset_params):
        for param in reset_params:
            param['light'].set_state(param['state'])
            param['light'].set_green_time(param['green_time'])
            param['light'].set_red_time(param['red_time'])
            param['light'].set_yellow_time(param['yellow_time'])

    @staticmethod
    def get_next_traffic_light(actor, use_cached_location=True):
        if not use_cached_location:
            location = actor.get_transform().location
        else:
            location = PanoSimDataProvider.get_location(actor)

        waypoint = PanoSimDataProvider.get_map().get_waypoint(location)
        list_of_waypoints = []
        while waypoint and not waypoint.is_intersection:
            list_of_waypoints.append(waypoint)
            waypoint = waypoint.next(2.0)[0]

        if not list_of_waypoints:
            return None

        relevant_traffic_light = None
        distance_to_relevant_traffic_light = float("inf")
        for traffic_light in PanoSimDataProvider._traffic_light_map:
            if hasattr(traffic_light, 'trigger_volume'):
                tl_t = PanoSimDataProvider._traffic_light_map[traffic_light]
                transformed_tv = tl_t.transform(traffic_light.trigger_volume.location)
                distance = PanoSimLocation(transformed_tv).distance(list_of_waypoints[-1].transform.location)
                if distance < distance_to_relevant_traffic_light:
                    relevant_traffic_light = traffic_light
                    distance_to_relevant_traffic_light = distance

        return relevant_traffic_light

    @staticmethod
    def generate_spawn_points():
        spawn_points = list(PanoSimDataProvider.get_map(PanoSimDataProvider._world).get_spawn_points())
        PanoSimDataProvider._rng.shuffle(spawn_points)
        PanoSimDataProvider._spawn_points = spawn_points
        PanoSimDataProvider._spawn_index = 0

    @staticmethod
    def check_road_length(wp, length: float):
        waypoint_separation = 5
        cur_len = 0
        road_id, lane_id = wp.road_id, wp.lane_id
        while True:
            wps = wp.next(waypoint_separation)
            next_wp = None
            for p in wps:
                if p.road_id == road_id and p.lane_id == lane_id:
                    next_wp = p
                    break
            if next_wp is None:
                break
            cur_len += waypoint_separation
            if cur_len >= length:
                return True
            wp = next_wp
        return False

    @staticmethod
    def get_road_lanes(wp):
        if wp.is_junction:
            return []

        lane_id_set = set()
        pre_left = wp
        while wp and wp.lane_type == PanoSimLaneType.Driving:
            if wp.lane_id in lane_id_set:
                break
            lane_id_set.add(wp.lane_id)
            pre_left = wp
            wp = wp.get_left_lane()

        lane_list = []
        lane_id_set.clear()
        wp = pre_left
        while wp and wp.lane_type == PanoSimLaneType.Driving:
            if wp.lane_id in lane_id_set:
                break
            lane_id_set.add(wp.lane_id)
            lane_list.append(wp)
            wp = wp.get_right_lane()

        return lane_list

    @staticmethod
    def get_road_lane_cnt(wp):
        lanes = PanoSimDataProvider.get_road_lanes(wp)
        return len(lanes)

    @staticmethod
    def get_waypoint_by_laneid(lane_num: int):
        if PanoSimDataProvider._spawn_points is None:
            PanoSimDataProvider.generate_spawn_points()

        if PanoSimDataProvider._spawn_index >= len(PanoSimDataProvider._spawn_points):
            print("No more spawn points to use")
            return None
        else:
            pos = PanoSimDataProvider._spawn_points[PanoSimDataProvider._spawn_index]
            PanoSimDataProvider._spawn_index += 1
            wp = PanoSimDataProvider.get_map().get_waypoint(pos.location, project_to_road=True, lane_type=PanoSimLaneType.Driving)

            road_lanes = PanoSimDataProvider.get_road_lanes(wp)

            lane = int(float(lane_num))
            if lane > len(road_lanes):
                return None
            else:
                return road_lanes[lane - 1]

    @staticmethod
    def create_blueprint(model, rolename='scenario', color=None, actor_category="car", attribute_filter=None):
        def check_attribute_value(blueprint, name, value):
            if not blueprint.has_attribute(name):
                return False

            attribute_type = blueprint.get_attribute(key).type
            if attribute_type == PanoSimActorAttributeType.Bool:
                return blueprint.get_attribute(name).as_bool() == value
            elif attribute_type == PanoSimActorAttributeType.Int:
                return blueprint.get_attribute(name).as_int() == value
            elif attribute_type == PanoSimActorAttributeType.Float:
                return blueprint.get_attribute(name).as_float() == value
            elif attribute_type == PanoSimActorAttributeType.String:
                return blueprint.get_attribute(name).as_str() == value

            return False

        _actor_blueprint_categories = {
            'car': 'vehicle.tesla.model3',
            'van': 'vehicle.volkswagen.t2',
            'truck': 'vehicle.carlamotors.carlacola',
            'trailer': '',
            'semitrailer': '',
            'bus': 'vehicle.volkswagen.t2',
            'motorbike': 'vehicle.kawasaki.ninja',
            'bicycle': 'vehicle.diamondback.century',
            'train': '',
            'tram': '',
            'pedestrian': 'walker.pedestrian.0001',
        }

        try:
            blueprints = PanoSimDataProvider._blueprint_library.filter(model)
            if attribute_filter is not None:
                for key, value in attribute_filter.items():
                    blueprints = [x for x in blueprints if check_attribute_value(x, key, value)]

            blueprint = PanoSimDataProvider._rng.choice(blueprints)
        except ValueError:
            bp_filter = "vehicle.*"
            new_model = _actor_blueprint_categories[actor_category]
            if new_model != '':
                bp_filter = new_model
            print("WARNING: Actor model {} not available. Using instead {}".format(model, new_model))
            blueprint = PanoSimDataProvider._rng.choice(PanoSimDataProvider._blueprint_library.filter(bp_filter))

        if color:
            if not blueprint.has_attribute('color'):
                print("WARNING: Cannot set Color ({}) for actor {} due to missing blueprint attribute".format(color, blueprint.id))
            else:
                default_color_rgba = blueprint.get_attribute('color').as_color()
                default_color = '({}, {}, {})'.format(default_color_rgba.r, default_color_rgba.g, default_color_rgba.b)
                try:
                    blueprint.set_attribute('color', color)
                except ValueError:
                    # Color can't be set for this vehicle
                    print("WARNING: Color ({}) cannot be set for actor {}. Using instead: ({})".format(color, blueprint.id, default_color))
                    blueprint.set_attribute('color', default_color)
        else:
            if blueprint.has_attribute('color') and rolename != 'hero':
                color = PanoSimDataProvider._rng.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)

        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'false')

        if blueprint.has_attribute('role_name'):
            blueprint.set_attribute('role_name', rolename)

        return blueprint

    @staticmethod
    def handle_actor_batch(batch, tick=True):
        sync_mode = PanoSimDataProvider.is_sync_mode()
        actors = []

        if PanoSimDataProvider._client:
            responses = PanoSimDataProvider._client.apply_batch_sync(batch, sync_mode and tick)
        else:
            raise ValueError("class member \'client'\' not initialized yet")

        if not tick:
            pass
        elif PanoSimDataProvider.is_runtime_init_mode():
            PanoSimDataProvider._world.wait_for_tick()
        elif sync_mode:
            PanoSimDataProvider._world.tick()
        else:
            PanoSimDataProvider._world.wait_for_tick()

        actor_ids = [r.actor_id for r in responses if not r.error]
        for r in responses:
            if r.error:
                print("WARNING: Not all actors were spawned")
                break
        actors = list(PanoSimDataProvider._world.get_actors(actor_ids))
        return actors

    @staticmethod
    def request_new_actor(model, spawn_point, rolename='scenario', autopilot=False,
                          random_location=False, color=None, actor_category="car",
                          attribute_filter=None, tick=True):
        actor = PanoSimActor()
        actor.id = 0
        actor.model = model
        actor.attributes['role_name'] = rolename
        actor.actor_category = actor_category
        actor.transform = spawn_point
        PanoSimDataProvider._actor_pool[actor.id] = actor
        PanoSimDataProvider.register_actor(actor, spawn_point)
        return actor

    @staticmethod
    def request_new_actors(actor_list, attribute_filter=None, tick=True):
        SpawnActor = PanoSimCommand.SpawnActor
        PhysicsCommand = PanoSimCommand.SetSimulatePhysics
        FutureActor = PanoSimCommand.FutureActor
        ApplyTransform = PanoSimCommand.ApplyTransform
        SetAutopilot = PanoSimCommand.SetAutopilot
        SetVehicleLightState = PanoSimCommand.SetVehicleLightState

        batch = []
        actors = []

        id_offset = 0
        for actor in actor_list:
            if actor.category != 'misc':
                if not actor.transform._get_actor_transform:
                    continue
                new = PanoSimActor()
                new.id = -1 - id_offset
                new.model = actor.model
                new.attributes['role_name'] = actor.rolename
                new.actor_category = actor.category
                new.transform = actor.transform
                new.speed = 0
                PanoSimDataProvider._actor_pool[new.id] = new
                PanoSimDataProvider.register_actor(new, new.transform)
                actors.append(new)
                id_offset += 1

        return actors

    @staticmethod
    def AddActor(model, spawn_point, rolename='scenario', autopilot=False,
                          random_location=False, color=None, actor_category="car",
                          attribute_filter=None, tick=True):
        new = PanoSimActor()
        new.model = model
        new.attributes['role_name'] = rolename
        new.actor_category = actor_category
        new.transform = spawn_point
        new.speed = 0

        type = vehicle_type.Car
        if new.actor_category == 'bicycle':
            type = vehicle_type.NonMotorVehicle
        elif new.actor_category == 'pedestrian':
            type = vehicle_type.Pedestrian
        if new.transform.type == 'WorldPosition':
            x = new.transform.location.x
            y = new.transform.location.y
            new.id = addVehicle(x, y, 0, type)
            if new.id > 100 and (type == vehicle_type.NonMotorVehicle or type == vehicle_type.Pedestrian):
                moveTo(new.id, x, y, 90 - new.transform.rotation.yaw)
        elif new.transform.type == 'RelativeRoadPosition':
            ds = new.transform.data['ds']
            ref = new.transform.data['entityRef']
            if ref == 'hero':
                new.id = addVehicleRelated(0, float(ds), 0, 0, lane_type.current, type)
            else:
                for k, v in PanoSimDataProvider._actor_pool.items():
                    if v.attributes['role_name'] == ref and k > 100:
                        new.id = addVehicleRelated(k, float(ds), 0, 0, lane_type.current, type)
                        break
        elif new.transform.type == 'RoadPosition':
            print('create_actor:', new.id, new.transform.type, new.transform.data)
        elif new.transform.type == 'LanePostion':
            print('create_actor:', new.id, new.transform.type, new.transform.data)
        if new.id > 100:
            PanoSimDataProvider._actor_pool[new.id] = new
            PanoSimDataProvider._actor_location_map[new] = new.transform.location
            PanoSimDataProvider._actor_location_map[new] = new.transform
            return new
        return None


    @staticmethod
    def request_new_batch_actors(model, amount, spawn_points, autopilot=False,
                                 random_location=False, rolename='scenario',
                                 attribute_filter=None, tick=True):
        SpawnActor = PanoSimCommand.SpawnActor
        SetAutopilot = PanoSimCommand.SetAutopilot
        FutureActor = PanoSimCommand.FutureActor
        PanoSimDataProvider.generate_spawn_points()
        batch = []

        for i in range(amount):
            blueprint = PanoSimDataProvider.create_blueprint(model, rolename, attribute_filter=attribute_filter)
            if random_location:
                if PanoSimDataProvider._spawn_index >= len(PanoSimDataProvider._spawn_points):
                    print("No more spawn points to use. Spawned {} actors out of {}".format(i + 1, amount))
                    break
                else:
                    spawn_point = PanoSimDataProvider._spawn_points[PanoSimDataProvider._spawn_index]
                    PanoSimDataProvider._spawn_index += 1
            else:
                try:
                    spawn_point = spawn_points[i]
                except IndexError:
                    print("The amount of spawn points is lower than the amount of vehicles spawned")
                    break
            if spawn_point:
                batch.append(SpawnActor(blueprint, spawn_point).then(
                    SetAutopilot(FutureActor, autopilot, PanoSimDataProvider._traffic_manager_port)))

        actors = PanoSimDataProvider.handle_actor_batch(batch, tick)
        for actor in actors:
            if actor is None:
                continue
            PanoSimDataProvider._actor_pool[actor.id] = actor
            PanoSimDataProvider.register_actor(actor, spawn_point)

        return actors

    @staticmethod
    def get_actors():
        return iteritems(PanoSimDataProvider._actor_pool)

    @staticmethod
    def actor_id_exists(actor_id):
        return actor_id in PanoSimDataProvider._actor_pool

    @staticmethod
    def get_hero_actor():
        for actor_id in PanoSimDataProvider._actor_pool:
            if PanoSimDataProvider._actor_pool[actor_id].attributes['role_name'] == 'hero':
                return PanoSimDataProvider._actor_pool[actor_id]
        return None

    @staticmethod
    def get_actor_by_id(actor_id):
        if actor_id in PanoSimDataProvider._actor_pool:
            return PanoSimDataProvider._actor_pool[actor_id]

        print("Non-existing actor id {}".format(actor_id))
        return None

    @staticmethod
    def get_actor_by_name(role_name: str):
        for actor_id in PanoSimDataProvider._actor_pool:
            if PanoSimDataProvider._actor_pool[actor_id].attributes['role_name'] == role_name:
                return PanoSimDataProvider._actor_pool[actor_id]
        print(f"Non-existing actor name {role_name}")
        return None

    @staticmethod
    def remove_actor_by_id(actor_id):
        for actor, _ in PanoSimDataProvider._actor_transform_map.items():
            if actor.id == actor_id:
                del PanoSimDataProvider._actor_transform_map[actor]
                break

        for actor, _ in PanoSimDataProvider._actor_velocity_map.items():
            if actor.id == actor_id:
                del PanoSimDataProvider._actor_velocity_map[actor]
                break

        for actor, _ in PanoSimDataProvider._actor_location_map.items():
            if actor.id == actor_id:
                del PanoSimDataProvider._actor_location_map[actor]
                break

        if actor_id in PanoSimDataProvider._actor_pool:
            PanoSimDataProvider._actor_pool[actor_id].destroy()
            PanoSimDataProvider._actor_pool[actor_id] = None
            PanoSimDataProvider._actor_pool.pop(actor_id)
        else:
            print("Trying to remove a non-existing actor id {}".format(actor_id))
        deleteVehicle(actor_id)

    @staticmethod
    def remove_actors_in_surrounding(location, distance):
        for actor_id in PanoSimDataProvider._actor_pool.copy():
            if PanoSimDataProvider._actor_pool[actor_id].get_location().distance(location) < distance:
                PanoSimDataProvider._actor_pool[actor_id].destroy()
                PanoSimDataProvider._actor_pool.pop(actor_id)

        PanoSimDataProvider._actor_pool = dict({k: v for k, v in PanoSimDataProvider._actor_pool.items() if v})

    @staticmethod
    def get_traffic_manager_port():
        return PanoSimDataProvider._traffic_manager_port

    @staticmethod
    def set_traffic_manager_port(tm_port):
        PanoSimDataProvider._traffic_manager_port = tm_port

    @staticmethod
    def cleanup():
        DestroyActor = PanoSimCommand.DestroyActor
        batch = []

        for actor_id in PanoSimDataProvider._actor_pool.copy():
            actor = PanoSimDataProvider._actor_pool[actor_id]
            if actor is not None and actor.is_alive:
                batch.append(DestroyActor(actor))

        if PanoSimDataProvider._client:
            try:
                PanoSimDataProvider._client.apply_batch_sync(batch)
            except RuntimeError as e:
                if "time-out" in str(e):
                    pass
                else:
                    raise e

        PanoSimDataProvider._actor_velocity_map.clear()
        PanoSimDataProvider._actor_location_map.clear()
        PanoSimDataProvider._actor_transform_map.clear()
        PanoSimDataProvider._traffic_light_map.clear()
        PanoSimDataProvider._map = None
        PanoSimDataProvider._world = None
        PanoSimDataProvider._sync_flag = False
        PanoSimDataProvider._ego_vehicle_route = None
        PanoSimDataProvider._all_actors = None
        PanoSimDataProvider._actor_pool = {}
        PanoSimDataProvider._client = None
        PanoSimDataProvider._spawn_points = None
        PanoSimDataProvider._spawn_index = 0
        PanoSimDataProvider._rng = random.RandomState(PanoSimDataProvider._random_seed)
        PanoSimDataProvider._grp = None
        PanoSimDataProvider._runtime_init_flag = False

    @property
    def world(self):
        return self._world
