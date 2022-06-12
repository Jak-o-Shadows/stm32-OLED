#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#ifndef TASKS_RTOSTASK_H_
#define TASKS_RTOSTASK_H_

// From https://github.com/ppykalo/ps2keyboard


class RtosTask
{
public:
	//virtual ~RtosTask() {}

	inline void start(const char * const pcName, const uint16_t usStackDepth, UBaseType_t uxPriority) {
		this->stopCalled = false;
		xTaskCreate(RtosTask::bootstrap, pcName, usStackDepth, this, uxPriority, &this->xHandle);
	}
	inline void stop() {
		this->stopCalled = true;
	}

protected:
	inline void sleep(int time_ms) {
		vTaskDelay(time_ms/portTICK_PERIOD_MS);
	}

private:
	virtual void setup() {}
	virtual void loop() {}
	virtual void cleanup() {}

	static void bootstrap(void* pvParameters) {
		RtosTask* taskObject = reinterpret_cast<RtosTask*>(pvParameters);

	 // setup in new thread
		taskObject->setup();

		while (!taskObject->stopCalled) {
			taskObject->loop();
		}

	 // task clean up
		taskObject->cleanup();
		vTaskDelete(taskObject->xHandle);
	}
	bool stopCalled = false;
	TaskHandle_t xHandle = 0;
};

#endif /* TASKS_RTOSTASK_H_ */