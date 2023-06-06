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

class NecSensor
{
public:
    NecSensor() : dev_(0), id_(0)
    {
    }
    NecSensor(VEDAdevice dev, int id) : dev_(dev), id_(id)
    {
    }
    virtual std::string name()
    {
	    return "";
    }
    virtual std::string unit()
    {
	    return "";
    }
    virtual std::string description()
    {
	 return "";
    }
    virtual double value()
    {
	    return 0;
    }

protected:
    VEDAdevice dev_;
    int id_;
};

class NecPowerSensor : public NecSensor
{
public:
    NecPowerSensor() : NecSensor()
	{
	}
    NecPowerSensor(VEDAdevice dev, int id) : NecSensor(dev, id)
    {
    }

    std::string name() override
    {
        return fmt::format("VE{}:power", id_);
    }

    std::string description() override
    {
        return fmt::format("Power consumption of vector engine {}", id_);
    }

    double value() override
    {
        float power;
        vedaDeviceGetPower(&power, dev_);

        return power;
    }

    std::string unit() override
    {
	     return "Watt";
    }
};
