# Score-P VEDA Plugin

This code provides a Score-P plugin to access the metrics provided by NEC SX Aurora Tsubasa Accelerators.

## Prerequisites

- Score-P 
- NEC VEDA library (The CMake script assumes a fixed location for the VEDA library, which corresponds to the NEC installation at DKRZ)

## Installation


```
git clone --recursive-submodules git@gitlab.dkrz.de:eeclips/scorep_plugin_nec.git
cd scorep_plugin_nec
mkdir build & cd build
cmake ../
make

#copy the resulting libnec_plugin.so into your LD_LIBRARY_PATH
```

## Usage

```
export SCOREP_METRIC_PLUGINS=nec
export SCOREP_METRIC_NEC_PLUGIN="power,..."
```

## Available Metrics

- power
- more TBD
