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

extern "C"
{
#include <veda/api.h>
}

using TVPair = std::pair<scorep::chrono::ticks, double>;

class NecMeasurementThread
{
public:
    void add_sensors(std::vector<NecSensor> sensors)
    {
        assert(sensors_.empty());

        sensors_ = sensors;

        for (const auto& sensor : sensors_)
        {
            values_.emplace_back(std::vector<TVPair>());
        }
    }

    void measurement()
    {
        while (!stop_)
        {
            std::lock_guard<std::mutex> lock(read_mutex_);
            int i = 0;
            for (auto& sensor : sensors_)
            {
                values_[i].emplace_back(scorep::chrono::measurement_clock::now(), sensor.value());
                i++;
            }
        }
    }

    void stop_measurement()
    {
        std::lock_guard<std::mutex> lock(read_mutex_);
        stop_ = true;
    }

    std::vector<TVPair> get_values_for_id(int32_t id)
    {
        std::lock_guard<std::mutex> lock(read_mutex_);
        auto ret = values_[id];
        values_[id].clear();
        return ret;
    }

private:
    bool stop_ = false;
    std::vector<NecSensor> sensors_;
    std::mutex read_mutex_;
    std::vector<std::vector<TVPair>> values_;
};

using namespace scorep::plugin::policy;

class nec_plugin : public scorep::plugin::base<nec_plugin, async, once, scorep_clock>
{
public:
    nec_plugin()
    {
        vedaInit(0);
        vedaDeviceGet(&device_, 0);
    }
    ~nec_plugin()
    {
        vedaExit();
    }

    std::vector<scorep::plugin::metric_property>
    get_metric_properties(const std::string& metric_name)
    {
        NecSensor sensor = get_sensor_by_name(metric_name);
        return { scorep::plugin::metric_property(sensor.name(), sensor.description(), sensor.unit())
                     .absolute_point()
                     .value_double() };
    }

    int32_t add_metric(const std::string& event)
    {
        int32_t id = sensors_.size();
        sensors_.emplace_back(get_sensor_by_name(event));
        return id;
    }

    template <typename C>
    void get_all_values(int32_t id, C& cursor)
    {
        std::vector<TVPair> values = nec.get_values_for_id(id);

        for (auto& value : values)
        {
            cursor.write(value.first, value.second);
        }
    }

    void start()
    {
        nec.add_sensors(sensors_);

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
            return NecPowerSensor(device_, 0);
        }
    }

    NecMeasurementThread nec;
    VEDADevice device_;
    std::thread measurement_thread_;

    std::vector<NecSensor> sensors_;
};

SCOREP_METRIC_PLUGIN_CLASS(nec_plugin, "nec")
