#ifndef _HELLO_TEST_H_
#define _HELLO_ANDROID_H_

#include <linux/cdev.h>
#include <linux/semaphore.h>

#define HELLO_DEVICE_NODE_NAME  "hello"
#define HELLO_DEVICE_FILE_NAME  "hello"
#define HELLO_DEVICE_CLASS_NAME "hello"

struct hello_test_dev {
	char * val;
	struct semaphore sem;
	struct cdev dev;
};

#endif


