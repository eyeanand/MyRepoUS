#ifndef WIFI_COMMON_H

	#define WIFI_COMMON_H





	#define PORT1 1234

	#define SPEED B9600



	#define QUERY_ACTUAL_LEVEL 0xA0

	#define RECALL_MAX_LEVEL (0x05)

	#define MAX_WIFI 4

	#define WIFI_ID_START 9

	#define BROADCAST_LEVEL (0xFE)

	#define CONVERSION_FACTOR (2.54f)

	#define MAX_LOG_MSG_LEN	100

	#define MAX_LOG_FILE_LEN 5000

	#define NUM_OF_LIGHTS 13



	#define RST 24 /* P1-18 */

	#define OE 23  /* P1-16 */

	#define IN   0

	#define OUT  1

	#define LOW  0

	#define HIGH 1



	typedef struct Light_Info 

	{

			unsigned int address;

			unsigned int level;

	}L_INFO;



	struct dali_payload

	{

		   unsigned int id;

		   unsigned int cmd;

		   unsigned int active;

		   unsigned int level;

		   unsigned int group;

	};



	//GPIO Functions

	int GPIOExport(int pin);

	int GPIOUnexport(int pin);

	int GPIODirection(int pin, int dir);

	int GPIORead(int pin);

	int GPIOWrite(int pin, int value);



	//Misc Functions

	void StoreLightInfo(void);

	int uart_init(void);

	int reset(void);

	void sigintHandler(int sig_num);

	int log_func(char* msg);



	//Threads per light

	void *Light0(void*);

	void *Light1(void*);

	void *Light2(void*);

	void *Light3(void*);

	void *Light4(void*);



#endif
