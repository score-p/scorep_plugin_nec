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

// #include <veda/api.h>
#include <fmt/core.h>

#include <veda/api.h>

enum class SensorType
{
    INVALID,
    POWER
};

class NecSensor
{
public:
    NecSensor() : type_(SensorType::INVALID), id_(0)
    {
    }
    NecSensor(SensorType type, int id) : type_(type), id_(id)
    {
        vedaDeviceGet(&dev_, id);
    }
    virtual std::string name() const
    {
        if (type_ == SensorType::POWER)
        {
            return fmt::format("VE{}::power", id_);
        }
        else
        {
            throw std::runtime_error("Invalid sensor!");
        }
    }
    virtual std::string unit() const
    {
        if (type_ == SensorType::POWER)
        {
            return "Watt";
        }
        else
        {
            throw std::runtime_error("Invalid sensor!");
        }
    }
    virtual std::string description() const
    {
        if (type_ == SensorType::POWER)
        {
            return "Vector Engine Power Consumption";
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
        else
        {
            throw std::runtime_error("Invalid sensor!");
        }
    }

    friend bool operator<(const NecSensor& lhs, const NecSensor& rhs)
    {
        if (lhs.type_ == rhs.type_)
        {
            return lhs.id_ < rhs.id_;
        }
        return lhs.type_ < rhs.type_;
    }

protected:
    SensorType type_;
    int id_;
    VEDAdevice dev_;
};
