#ifndef ROBOCARE_TYPES_H
#define ROBOCARE_TYPES_H
#ifndef LORA_SENSOR_DATA_T_DEFINED
#define LORA_SENSOR_DATA_T_DEFINED
typedef struct {
    int   node_id;
    float temperature;
    float humidity;
    float ec;
    float ph;
    float nitrogen;
    float phosphorus;
    float potassium;
    char  date[12];
    char  time_str[10];
    int   rssi;
    float snr;
} lora_sensor_data_t;
#endif
#endif
