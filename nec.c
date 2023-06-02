#include <veda/api.h>
#include <stdio.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>

#include <scorep/SCOREP_MetricPlugins.h>

#define MAX_SENSOR_COUNTER 40

static uint64_t (*wtime)(void) = NULL;
static pthread_mutex_t read_mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_t thread;

static int nr_results[MAX_SENSOR_COUNTER];
static int max_results[MAX_SENSOR_COUNTER];

static int added_sensor_counters;

static SCOREP_MetricTimeValuePair *results[MAX_SENSOR_COUNTER];
SCOREP_MetricTimeValuePair *return_values;
int return_values_length;
VEDAdevice device;

bool counter_enabled = false;
bool thread_finished = false;


class Nec_Metric 
{
	protected:
		std::string name;
		std::string desc;
		std::string unit;
}

template<typename T, typename Policies>
using nec_object_id = object_id<nec_metric_handle, T, Policies>;

class NecMeasurementThread
{
public:
	void measurement()
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		for (auto& metric : nec_metrics)
		{
			std::uint64_t value = metric.first.get_value(nec_device);
	
		}
	}
private:
	VEDADevice nec_device;
}


class nec_plugin : public scorep:plugin::base<async_plugin, async, once, scorep_clock>
{
	public:
	nec_plugin() : measurement_thread_(&nec_plugin::thread_report, this)
	{
	}
	std::vector<scorep::plugin::metric_property>
		get_metric_properties(const std::string& metric_name)
		{
			return {scorep::plugin::metric_property(metric_name, "Power Consumption", "Watt")
				.absolute_point()
				.value_double()};
		}
	int32_t add_metric(const std::string& event)
	{

		return 0;
	}

	void start()
	{
		measurement_thread_ = std::thread([this]() {this->thread_report});
	}

	void stop()
	{

	}

	template <typename C>
		void get_all_values(nvm
	void thread_report()
	{
	}
private:
	NecMeasurementThread nec;
	std::thread measurement_thread_;
}

static int perid = 1000000;
void * thread_report(void *ignored)
{
	int retVal =0;
	union{
		double dbl;
		uint64_t uint64;
	} value;

	while(counter_enabled){
		int i=0;
		if(wtime==NULL) continue;
		pthread_mutex_lock(&read_mutex);

		if(nr_results ==max_results)
		{
			SCOREP_MetricTimeValuePair * reallocated;
			reallocated = (SCOREP_MetricTimeValuePair *)realloc(results, nr_results*2*sizeof(SCOREP_MetricTimeValuePair));

			results = reallocated;

			max_results=nr_results*2;
		}

		results[nr_results].timestamp=wtime();
		float power;
		vedaDeviceGetPower(&power, device);

		value.dbl = power;
		results[nr_results].value = value.uint64;

		nr_results++;

		pthread_mutex_unlock(&read_mutex);
		usleep(period);
	}

	thread_finished = true;

	return NULL;
}
int32_t init()
{
	vedaInit(0);
	vedaDeviceGet(&device, 0);
	nr_results = 0;
	max_results = 1000000;

	return_values = (SCOREP_MetricTimeValuePair*) malloc(sizeof(SCOREP_MetricTimeValuePair) * 1000000);

	return_values_length = 1000000;

	char *from_env = getenv("SCOREP_METRIC_NEC_PLUGIN_PERIOD");

	if(from_env != NULL)
	{
		period = atoi(from_env);
		if (period == 0)
		{
			period = 1000000;
		}
	}
	counter_enabled = 1;
	thread = 0;
	thread_finished = 1;
	
	return 0;
}

int32_t add_counter(char *event_name)
{
	if(added_sensor_counters>=MAX_SENSOR_COUNTER)
	results = (SCOREP_MetricTimeValuePair *)malloc(max_results*sizeof(SCOREP_MetricTimeValuePair));

	pthread_create(&thread, NULL, &thread_report, NULL);

	return 0;
}

SCOREP_Metric_Plugin_MetricProperties * get_event_info(char *event_name)
{
}

void fini()
{
	counter_enabled = 0;
	pthread_join(thread, NULL);
	pthread_mutex_destroy(&read_mutex);
	vedaExit();

}

void set_timer( uint64_t (*timer)(void))
{
	wtime=timer;
}


uint64_t get_all_values(int32_t id, SCOREP_MetricTimeValuePair** time_value_list)
{
	int saved_nr_results;

	pthread_mutex_lock(&read_mutex);
	saved_nr_results = nr_results;
	SCOREP_MetricTimeValuePair * return_values= (SCOREP_MetricTimeValuePair*)malloc(saved_nr_results*sizeof(SCOREP_MetricTimeValuePair));

	memcpy(return_values, results, saved_nr_results*sizeof(SCOREP_MetricTimeValuePair));

	time_value_list[0] = return_values;
	nr_results=0;
	pthread_mutex_unlock(&read_mutex);

	return saved_nr_results;
}
SCOREP_METRIC_PLUGIN_ENTRY( nec_plugin )
{
	SCOREP_Metric_Plugin_Info info;
	memset(&info, 0, sizeof( SCOREP_Metric_Plugin_Info));
	info.plugin_version = SCOREP_METRIC_PLUGIN_VERSION;
	info.run_per = SCOREP_METRIC_PER_HOST;
	info.sync = SCOREP_METRIC_ASYNC;
	info.delta_t = UINT64_MAX;
	info.initialize = init;
	info.finalize = fini;
	info.get_event_info = get_event_info;
	info.add_counter = add_counter;
	info.get_all_values = get_all_values;
	info.set_clock_function = set_timer;

	return info;
}
