
[PanoSim](https://www.panosim.com/) V33 began to provide support for [ASAM](https://www.asam.net/) - [OpenScenario 1.0](https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/index.html). Here are some examples to show how to run scenario files (xosc) in PanoSim.


## Installation

Follow these steps to set up the run environment and install the package:

### 1. downloand [files](./PanoSimDatabase)

### 2. find local folder
![image](docs/images/folder.jpg)

### 3. copy files to local folder

### 4. restart PanoExp

### 5. instal python package

1) start command prompt(cmd.exe)

2) enter Scripts folde
    ```
    cd {PanoSim install folder}\Bin\py36\Scripts
    ```

3) run follow command
    ```
    pip install -v xmlschema==1.0.18
    pip install -v py_trees==0.8.3
    pip install -v ephem==4.1.5
    pip install -v networkx==2.2
    pip install tabulate
    ```

## Run

![image](docs/images/open.jpg)

![image](docs/images/result.jpg)


| Press Key | Signal          |
|-----------|-----------------|
| Page Up   | forward gear    |
| Page Down | reverse gear    |
| Up        | speed up        |
| Down      | brspeed down    |
| Left      | turn left       |
| Right     | turn right      |


### control ego by keyboard

## Special Thanks

- **CARLA Simulator**: [CARLA Simulator GitHub](https://github.com/carla-simulator/carla)

- **ScenarioRunner for CARLA**: [ScenarioRunner for CARLA GitHub](https://github.com/carla-simulator/scenario_runner)

## License

The software is licensed under the MIT License - see [`LICENSE`](https://github.com/liyanlee/PanoSim_OpenScenario/LICENSE) for more details.
