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
		    sensor.second.emplace_back(scorep::chrono::measurement_clock::now(), sensor.first.value());
            }
	    std::this_thread::sleep_for(std::chrono::seconds(1));
        }
	logging::warn() << "Stopped!";
    }

    void stop_measurement()
    {
	logging::warn() << "stop_measurement called!";
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
    std::map<NecSensor,std::vector<TVPair>> sensors_;
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
	logging::warn() << "Adding counter \"" << metric_name << "\"";

	try
	{
		NecSensor sensor = get_sensor_by_name(metric_name);
	logging::warn() << "NAME" << sensor.name();
	make_handle(metric_name, 0);
	nec.add_sensor(sensor);
	
	return { scorep::plugin::metric_property(sensor.name(), sensor.description(), sensor.unit())
                     .absolute_point()
                     .value_double() };
	}
	catch (std::runtime_error &e)
	{
		return {};
	}
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
    NecSensor get_sensor_by_name(const std::string& name)
    {
        if (name == "power")
        {
            return NecPowerSensor(0);
        }
	else
	{
		logging::warn() << "Ignoring Invalid metric: " << name;
		throw std::runtime_error("Invalid sensor");
	}
    }

    NecMeasurementThread nec;
    std::thread measurement_thread_;

};

SCOREP_METRIC_PLUGIN_CLASS(nec_plugin, "nec")
