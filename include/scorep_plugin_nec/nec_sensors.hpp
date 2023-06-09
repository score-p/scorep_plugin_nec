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

#include <fmt/core.h>
#include <scorep/plugin/plugin.hpp>
#include <veda/api.h>

enum class SensorType
{
    INVALID,
    POWER,
    CURRENT,
    CURRENT_EDGE,
    VOLTAGE,
    VOLTAGE_EDGE,
    TEMPERATURE
};

class NecSensor
{
public:
    NecSensor() : type_(SensorType::INVALID), id_(0)
    {
    }
    NecSensor(SensorType type, int id, int core = 0) : type_(type), id_(id), core_(core)
    {
        vedaDeviceGet(&dev_, id);
    }
    std::string name() const
    {
        if (type_ == SensorType::POWER)
        {
            return fmt::format("ve{}::power", id_);
        }
	else if(type_ == SensorType::CURRENT)
	{
	    return fmt::format("ve{}::current", id_);
	}
	else if(type_ == SensorType::CURRENT_EDGE)
	{
		return fmt::format("ve{}::current_edge", id_);
	}
	else if(type_ == SensorType::VOLTAGE)
	{
		return fmt::format("ve{}::voltage", id_);
	}
	else if(type_ == SensorType::VOLTAGE_EDGE)
	{
		return fmt::format("ve{}::voltage_edge", id_);
	}
	else if(type_ == SensorType::TEMPERATURE)
	{
		return fmt::format("ve{}::temp::core{}", id_, core_);
	}
        else
        {
            throw std::runtime_error("Invalid sensor!");
        }
    }
    std::string unit() const
    {
        if (type_ == SensorType::POWER)
        {
            return "Watt";
        }
	else if(type_ == SensorType::CURRENT)
	{
		return "Ampere";
	}
	else if(type_ == SensorType::CURRENT_EDGE)
	{
		return "Ampere";
	}
	else if(type_ == SensorType::VOLTAGE)
	{
		return "Volt";
	}
	else if(type_ == SensorType::VOLTAGE_EDGE)
	{
		return "Volt";
	}
	else if(type_ == SensorType::TEMPERATURE)
	{
		return "Celsius";
	}
        else
        {
            throw std::runtime_error("Invalid sensor!");
        }
    }
    std::string description() const
    {
        if (type_ == SensorType::POWER)
        {
            return fmt::format("Vector Engine {} Power Consumption", id_);
        }
	else if(type_ == SensorType::CURRENT)
	{
		return fmt::format("Vector Engine {} Current Draw", id_);
	}
	else if(type_ == SensorType::CURRENT_EDGE)
	{
		return fmt::format("Vector Engine {} Current Edge", id_);
	}
	else if(type_ == SensorType::VOLTAGE)
	{
		return fmt::format("Vector Engine {} Voltage", id_);
	}
	else if(type_ == SensorType::VOLTAGE_EDGE)
	{
		return fmt::format("Vector Engine {} Voltage Edge", id_);
	}
	else if(type_ == SensorType::TEMPERATURE)
	{
		return fmt::format("Vector Engine {} Core {} Temperature", id_, core_);
	}
	else
        {
            throw std::runtime_error("Invalid sensor!");
        }
    }
    double value() const
    {
        float value;

        if (type_ == SensorType::POWER)
        {
            vedaDeviceGetPower(&value, dev_);
            return value;
        }
	else if (type_ == SensorType::CURRENT)
	{
		vedaDeviceGetCurrent(&value, dev_);
		return value;
	}
	else if(type_ == SensorType::CURRENT_EDGE)
	{
		vedaDeviceGetCurrentEdge(&value, dev_);
		return value;
	}
	else if(type_ == SensorType::VOLTAGE)
	{
		vedaDeviceGetVoltage(&value, dev_);
		return value;
	}
	else if(type_ == SensorType::VOLTAGE_EDGE)
	{
		vedaDeviceGetVoltageEdge(&value, dev_);
		return value;
	}
	else if(type_ == SensorType::TEMPERATURE)
	{
		vedaDeviceGetTemp(&value, core_, dev_);
		return value;
	}
	else
        {
            throw std::runtime_error("Invalid sensor!");
        }
    }

    friend bool operator<(const NecSensor& lhs, const NecSensor& rhs)
    {
        if (lhs.type_ == rhs.type_)
        {
		if(lhs.id_ == rhs.id_)
		{
			return lhs.core_ < rhs.core_;
		}
            return lhs.id_ < rhs.id_;
        }
        return lhs.type_ < rhs.type_;
   }

    friend bool operator==(const NecSensor& lhs, const NecSensor& rhs)
    {
	return lhs.type_ == rhs.type_ && lhs.id_ == rhs.id_ && lhs.core_ == rhs.core_;
    }

    bool invalid() const
    {
	    return type_ == SensorType::INVALID;
    }

    SensorType type() const
    {
	    return type_;
    }
protected:
    SensorType type_;
    int id_;
    int core_;
    VEDAdevice dev_;
};
