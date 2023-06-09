/*
 * Copyright (c) 2016, Technische Universität Dresden, Germany
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 *    and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <cassert>
#include <mutex>
#include <thread>
#include <regex>
#include <vector>

#include <scorep_plugin_nec/nec_sensors.hpp>

#include <scorep/plugin/plugin.hpp>

#include <veda/api.h>

using TVPair = std::pair<scorep::chrono::ticks, double>;

using scorep::plugin::logging;
class NecMeasurementThread
{
public:
    void add_sensor(NecSensor sensor)
    {
        sensors_.emplace(sensor, std::vector<TVPair>());
    }

    void measurement()
    {
        while (!stop_)
        {
            for (auto& sensor : sensors_)
            {
                std::lock_guard<std::mutex> lock(read_mutex_);
                sensor.second.emplace_back(scorep::chrono::measurement_clock::now(),
                                           sensor.first.value());
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void stop_measurement()
    {
        std::lock_guard<std::mutex> lock(read_mutex_);
        stop_ = true;
    }

    std::vector<TVPair> get_values_for_sensor(NecSensor sensor)
    {
        std::lock_guard<std::mutex> lock(read_mutex_);
        auto ret = sensors_[sensor];
        sensors_[sensor].clear();
        return ret;
    }

private:
    bool stop_ = false;
    std::mutex read_mutex_;
    std::map<NecSensor, std::vector<TVPair>> sensors_;
};

using namespace scorep::plugin::policy;

template <typename T, typename Policies>
using nec_object_id = object_id<NecSensor, T, Policies>;

class nec_plugin : public scorep::plugin::base<nec_plugin, async, once, scorep_clock, nec_object_id>
{
public:
    nec_plugin()
    {
        vedaInit(0);
    }

    ~nec_plugin()
    {
        vedaExit();
    }

    std::vector<scorep::plugin::metric_property>
    get_metric_properties(const std::string& metric_name)
    {
	std::vector<scorep::plugin::metric_property> sensors;

	std::regex sensor_regex("ve([0-9*]+)::(\\w+)(::core([0-9*]+))?");
	std::smatch sensor_match;

	std::vector<int> devices;
	std::vector<int> cores; 
	
	if(std::regex_match(metric_name, sensor_match, sensor_regex))
	{
		int count;
		vedaDeviceGetCount(&count);

		if(sensor_match[1] == "*")
		{
			for(int id = 0; id < count; id++)
			{
				devices.emplace_back(id);
			}
		}
		else
		{
			try
			{
				int id = std::stoi(sensor_match[1]);
			
				if(id < 0 || id >= count)
				{
					logging::warn() << "Invalid device id " << id << " given, ignoring event";
					return {};
				}
				devices.emplace_back(id);
			}
			catch (std::exception &e)
			{
					logging::warn() << "Invalid device id " << sensor_match[1] << " given, ignoring event";
					return {};
			}
		}

		if(!sensor_match[4].str().empty())
		{
			int num_cores;
			//FIXME: Nobody uses irregular configurations of NEC Cards, right?
			vedaDeviceGetAttribute(&num_cores, VEDA_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, 0);

			if (sensor_match[4] == "*")
			{
				for(int core = 0; core < num_cores; core++)
				{
					cores.emplace_back(core);
				}
			}
			else
			{
			try
			{
				int id = std::stoi(sensor_match[4]);
			
				if(id < 0 || id >= num_cores)
				{
					logging::warn() << "Invalid core id " << id << " given, ignoring event";
					return {};
				}
				cores.emplace_back(id);
			}
			catch (std::exception &e)
			{
					logging::warn() << "Invalid core id " << sensor_match[4] << " given, ignoring event";
					return {};
			}

			}
		}
		else if(sensor_match[2] == "temp")
		{
			logging::warn() << "Temperature requieres you to specify the measurement core as ve*::temp::core* or ve*::temp::core4";
			return {};
		}
		else
		{
			cores.emplace_back(0);
		}

		for(auto device_id : devices)
		{
			for(auto core_id : cores)
			{
				auto sensor = get_sensor_by_name(device_id, sensor_match[2], core_id);

				if(sensor.invalid())
				{
					logging::warn() << "Invalid sensor: " << sensor_match[2] << ", ignored";
				}
				
				sensors.push_back(scorep::plugin::metric_property(sensor.name(), sensor.description(),
                                                     sensor.unit())
                         .absolute_point()
                         .value_double());
				
				make_handle(sensor.name(), sensor.type(), device_id, core_id);
				nec.add_sensor(sensor);
	
			}
		}
	}
	else
	{
		logging::warn() << "Invalid sensor: " << sensor_match[2] << ", ignored";
	}

	return sensors;

    }

    void add_metric(NecSensor& f)
    {
    }

    template <typename C>
    void get_all_values(NecSensor& f, C& cursor)
    {
        std::vector<TVPair> values = nec.get_values_for_sensor(f);

        for (auto& value : values)
        {
            cursor.write(value.first, value.second);
        }
    }

    void start()
    {
        measurement_thread_ = std::thread([this]() { this->nec.measurement(); });
    }

    void stop()
    {
        nec.stop_measurement();

        if (measurement_thread_.joinable())

        {
            measurement_thread_.join();
        }
    }

private:
    NecSensor get_sensor_by_name(int device_id,  std::string name, int core_id = 0)
    {
	    if(name == "power")
	    {
		    return NecSensor(SensorType::POWER, device_id);
	    }
	    else if(name == "voltage")
	    {
		    return NecSensor(SensorType::VOLTAGE, device_id);
	    }
	    else if(name == "voltage_edge")
	    {
		    return NecSensor(SensorType::VOLTAGE_EDGE, device_id);
	    }
	    else if(name == "current")
	    {
		    return NecSensor(SensorType::CURRENT, device_id);
	    }
	    else if(name == "current_edge")
	    {
		    return NecSensor(SensorType::CURRENT_EDGE, device_id);
	    }
	    else if(name == "temp")
	    {
		    return NecSensor(SensorType::TEMPERATURE, device_id, core_id);
	    }

	    return NecSensor();
    }

    NecMeasurementThread nec;
    std::thread measurement_thread_;
};

SCOREP_METRIC_PLUGIN_CLASS(nec_plugin, "nec")
