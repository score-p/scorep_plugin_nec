# Score-P NEC Plugin

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
export SCOREP_METRIC_NEC_PLUGIN="ve0::power,ve*::temp::core*"
```

Metrics are given in the form of:

```
ve[DEVICE_ID]::[METRIC_NAME]
```

Where `DEVICE_ID` is the number of the device you want to measure (or *, if you want to measure the metric on all available devices) and [METRIC_NAME] is the name of the metric.

Temperature measurements take an additional core parameter

```
ve[DEVICE_ID]::temp::core[CORE_ID]
```

Where `CORE_ID` is the ID of the core you want to measure, or * if you want to measure the temperature of all cores.

## Available Metrics

- power
- temp
- current
- current_edge
- voltage
- voltage_edge
