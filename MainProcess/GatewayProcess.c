#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <sys/un.h>
#include <linux/can/raw.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include <sys/wait.h>
#include <mosquitto.h>
#include <json-c/json.h>


#define SOCKET_PATH "/tmp/gui_socket"
int unix_fd;
struct sockaddr_un addr;

int sockfd; // socket CAN

// Định nghĩa struct VehicleStatus giống như STM32
typedef struct {
    uint8_t engine_status;
    uint8_t light_status;
    uint8_t tire_pressure;
    uint8_t door_status;
    uint8_t seat_belt_status;
    uint8_t battery_level;
    uint8_t speed;
    uint8_t arrived_distance;
    uint8_t remain_distance;
    uint8_t drived_time; 
    uint8_t transmission_gear;
    uint8_t reserved1;
    uint8_t reserved2;
    float gps_lat;
    float gps_lon;
} __attribute__((packed)) VehicleStatus;

typedef struct  {
    uint8_t air_condition_temperature;
    uint8_t speed_limit;
}controlData_t;

typedef struct {
    VehicleStatus vehicle_status;
    controlData_t control_data;
}DataTransfer_t;

DataTransfer_t data_transfer;
uint8_t thingsboard_fixed_setting;

void reset_packet_receiver();
void process_header_packet(struct can_frame *frame);
int process_data_packet(struct can_frame *frame);
void print_vehicle_status(const VehicleStatus status);

void publish_telemetry();
void on_connect(struct mosquitto *mosq, void *userdata, int rc);
void on_message(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *msg);


//-------------------------------------------------------------CAN HANDLER-------------------------------------------------------------
// Định nghĩa CAN ID
#define PACKET_HEADER_ID 0x024
#define PACKET_DATA_ID 0x025
#define AIRCONDITION_SPEEDLIMIT_CAN_ID 0x030

// Struct để lưu trữ dữ liệu nhận được
typedef struct {
    uint8_t frame_count;
    uint32_t data_size;
    uint8_t message_type;
    uint8_t frames_received;
    uint8_t data_buffer[256];
    uint8_t frame_status[32]; // Đánh dấu frame nào đã nhận
} PacketReceiver;

PacketReceiver packet_receiver;

// Hàm khởi tạo CAN socket
int setup_can_socket(const char* interface) {
    int sockfd;
    struct sockaddr_can addr;
    struct ifreq ifr;
    
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0) {
        perror("Socket creation failed");
        return -1;
    }
    
    strcpy(ifr.ifr_name, interface);
    ioctl(sockfd, SIOCGIFINDEX, &ifr);
    
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind failed");
        close(sockfd);
        return -1;
    }
    
    return sockfd;
}

// Hàm reset packet receiver
void reset_packet_receiver() {
    memset(&packet_receiver, 0, sizeof(PacketReceiver));
}

// Hàm xử lý header packet
void process_header_packet(struct can_frame *frame) {
    if (frame->can_dlc >= 4) {
        packet_receiver.frame_count = frame->data[0];
        packet_receiver.data_size = frame->data[1] | (frame->data[2] << 8);
        packet_receiver.message_type = frame->data[3];
        packet_receiver.frames_received = 0;
        
        printf("Header received: %d frames, %d bytes, type: %d\n",
               packet_receiver.frame_count, packet_receiver.data_size,
               packet_receiver.message_type);
        
        // Reset frame status
        memset(packet_receiver.frame_status, 0, sizeof(packet_receiver.frame_status));
    }
}

int process_data_packet(struct can_frame *frame) {
    if (frame->can_dlc < 1) return 0;
    
    uint8_t frame_number = frame->data[0];
    
    // Check valid frame number
    if (frame_number >= packet_receiver.frame_count) {
        printf("Invalid frame number: %d\n", frame_number);
        return 0;
    }
    
    // Check if frame already received
    if (packet_receiver.frame_status[frame_number]) {
        printf("Frame %d already received\n", frame_number);
        return 0;
    }
    
    // Save data
    uint8_t data_bytes = frame->can_dlc - 1;
    uint32_t offset = frame_number * 7;  // Each frame carries 7 bytes
    
    if (offset + data_bytes <= packet_receiver.data_size && 
        offset + data_bytes <= sizeof(packet_receiver.data_buffer)) {
        
        memcpy(&packet_receiver.data_buffer[offset], &frame->data[1], data_bytes);
        packet_receiver.frame_status[frame_number] = 1;
        packet_receiver.frames_received++;
        
        printf("Frame %d received (%d bytes) - offset: %d\n", frame_number, data_bytes, offset);
        
        // Debug: Print received bytes
        printf("Data: ");
        for (int i = 0; i < data_bytes; i++) {
            printf("0x%02X ", frame->data[1 + i]);
        }
        printf("\n");
        
        // Check if all frames received
        if (packet_receiver.frames_received == packet_receiver.frame_count) {
            return 1; // All frames received
        }
    }
    
    return 0;
}
//-------------------------------------------------------------END CAN HANDLER-------------------------------------------------------------



// Hàm in thông tin VehicleStatus
void print_vehicle_status(const VehicleStatus status) {
    printf("\n==== VEHICLE STATUS ====\n");
    printf("Engine Status: %s\n", status.engine_status ? "ON" : "OFF");
    printf("Light Status: %s\n", status.light_status ? "ON" : "OFF");
    printf("Tire Pressure: %s\n", status.tire_pressure ? "OK" : "ERROR");
    printf("Door Status: %s\n", status.door_status ? "OPEN" : "CLOSED");
    printf("Seat Belt: %s\n", status.seat_belt_status ? "FASTENED" : "UNFASTENED");
    printf("Battery Level: %d%%\n", status.battery_level);
    printf("Speed: %d km/h\n", status.speed);
    printf("Arrived Distance: %d km\n", status.arrived_distance);
    printf("Remain Distance: %d km\n", status.remain_distance);
    printf("Drived Time: %d km/h\n", status.drived_time);
    printf("Transmission Gear: %d\n", status.transmission_gear);
    printf("GPS Latitude: %.6f\n", status.gps_lat);
    printf("GPS Longitude: %.6f\n", status.gps_lon);
    printf("========================\n\n");
}


// --------------------------------------------------------------QT HANDLER--------------------------------------------------------------


void* air_condition_thread(void* arg) {
    while (1) {
        controlData_t controlData;
        ssize_t recv_bytes = recvfrom(unix_fd, &controlData, sizeof(controlData_t), 0, NULL, NULL);
        if (recv_bytes > 0) {

            printf("Received air condition temperature (thread): %d\n", controlData.air_condition_temperature);
            // Cập nhật biến toàn cục
            data_transfer.control_data.air_condition_temperature = controlData.air_condition_temperature;
            if(!thingsboard_fixed_setting)
            {
                data_transfer.control_data.speed_limit = controlData.speed_limit;
            }
            else
            {
                int sent = sendto(unix_fd, &data_transfer, sizeof(DataTransfer_t), 0, 
                                                (struct sockaddr *)&addr, sizeof(addr));
                                if (sent < 0) {
                                    perror("sendto");
                                } else {
                                    printf("Sent vehicle status to GUI\n");
                                }
            }
            
            // Gửi qua MQTT
            publish_telemetry();
            // Gửi qua CAN
            struct can_frame air_frame;
            memset(&air_frame, 0, sizeof(air_frame));
            air_frame.can_id = AIRCONDITION_SPEEDLIMIT_CAN_ID;
            air_frame.can_dlc = 2;
            air_frame.data[0] = (uint8_t)data_transfer.control_data.air_condition_temperature;
            air_frame.data[1] = (uint8_t)data_transfer.control_data.speed_limit;
            if (write(sockfd, &air_frame, sizeof(air_frame)) < 0) {
                perror("CAN write airConditionTemperature (thread)");
            } else {
                printf("Sent airConditionTemperature to CAN (thread): %d\n", controlData.air_condition_temperature);
            }
        }
    }
    return NULL;
}
// --------------------------------------------------------------END QT HANDLER--------------------------------------------------------------

//---------------------------------------------------------------MQTT HANDLER--------------------------------------------------------------
#define ACCESS_TOKEN    "eYtpAuJ4FRtVETCtobDW"
#define MQTT_HOST       "app.coreiot.io"
#define MQTT_PORT       1883

struct mosquitto *mosq;

void publish_telemetry() {
    struct json_object *telemetry = json_object_new_object();

    // Chuyển đổi sang chuỗi mô tả
    const char *engine_str = data_transfer.vehicle_status.engine_status ? "ON" : "OFF";

    const char *light_str;
    switch (data_transfer.vehicle_status.light_status) {
        case 1: light_str = "ON"; break;
        case 2: light_str = "ON"; break;
        case 3: light_str = "ON"; break;
        default: light_str = "OFF"; break;
    }

    const char *door_str = data_transfer.vehicle_status.door_status ? "Close" : "Open";

    // Thêm vào JSON
    json_object_object_add(telemetry, "engine_status", json_object_new_string(engine_str));
    json_object_object_add(telemetry, "light_status", json_object_new_string(light_str));
    json_object_object_add(telemetry, "door_status", json_object_new_string(door_str));

    // Các dữ liệu khác 
    json_object_object_add(telemetry, "battery_level", json_object_new_int(data_transfer.vehicle_status.battery_level));
    json_object_object_add(telemetry, "speed", json_object_new_int(data_transfer.vehicle_status.speed));
    json_object_object_add(telemetry, "air_condition_temp", json_object_new_int(data_transfer.control_data.air_condition_temperature));
    json_object_object_add(telemetry, "speed_limit", json_object_new_int(data_transfer.control_data.speed_limit));

    // Tọa độ GPS
    json_object_object_add(telemetry, "latitude", json_object_new_double(data_transfer.vehicle_status.gps_lat));
    json_object_object_add(telemetry, "longitude", json_object_new_double(data_transfer.vehicle_status.gps_lon));

    // Gửi MQTT
    const char *payload = json_object_to_json_string(telemetry);
    mosquitto_publish(mosq, NULL, "v1/devices/me/telemetry", strlen(payload), payload, 1, false);

    json_object_put(telemetry); // Giải phóng bộ nhớ
}

void on_connect(struct mosquitto *mosq, void *userdata, int rc) {
    if (rc == 0) {
        printf("Connected to CoreIotCloud Cloud\n");
        mosquitto_subscribe(mosq, NULL, "v1/devices/me/rpc/request/+", 1);
    } else {
        printf("Failed to connect, return code %d\n", rc);
    }
}

void on_message(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *msg) {
    printf("Received RPC: topic = %s, payload = %s\n", msg->topic, (char *)msg->payload);

    struct json_object *root = json_tokener_parse(msg->payload);
    if (!root) return;

    struct json_object *method_obj, *params_obj;
    if (json_object_object_get_ex(root, "method", &method_obj)) {
        const char *method = json_object_get_string(method_obj);
        printf(">> RPC Method: %s\n", method);

        // Lấy request ID từ topic
        const char *last_slash = strrchr(msg->topic, '/');
        const char *request_id = (last_slash != NULL) ? last_slash + 1 : "1";
        char response_topic[128];
        snprintf(response_topic, sizeof(response_topic), "v1/devices/me/rpc/response/%s", request_id);

        // Phản hồi các lệnh getXYZ
        if (strcmp(method, "getEngineStatus") == 0) {
            const char *resp = data_transfer.vehicle_status.engine_status ? "ON" : "OFF";
            mosquitto_publish(mosq, NULL, response_topic, strlen(resp), resp, 1, false);
            printf(">> Replied engine status: %s\n", resp);
        } else if (strcmp(method, "getLightStatus") == 0) {
            const char *resp = data_transfer.vehicle_status.light_status ? "ON" : "OFF";
            mosquitto_publish(mosq, NULL, response_topic, strlen(resp), resp, 1, false);
            printf(">> Replied light status: %s\n", resp);
        } else if (strcmp(method, "getACTemp") == 0) {
            char resp[32];
            snprintf(resp, sizeof(resp), "%d", data_transfer.control_data.air_condition_temperature);
            mosquitto_publish(mosq, NULL, response_topic, strlen(resp), resp, 1, false);
            printf(">> Replied AC temp: %s\n", resp);
        } else if (strcmp(method, "getSpeedLimit") == 0) {
            char resp[32];
            snprintf(resp, sizeof(resp), "%d",  data_transfer.control_data.speed_limit);
            mosquitto_publish(mosq, NULL, response_topic, strlen(resp), resp, 1, false);
            printf(">> Replied speed limit: %s\n", resp);
        }

        // Xử lý điều khiển từ widget
        else if (json_object_object_get_ex(root, "params", &params_obj)) {
            if (strcmp(method, "setEngineStatus") == 0) {
                int value = json_object_get_boolean(params_obj);
                data_transfer.vehicle_status.engine_status = value;
                publish_telemetry();
                printf(">> [ENGINE] Set to: %s\n", value ? "ON" : "OFF");
            } else if (strcmp(method, "setHeadLightStatus") == 0) {
                int value = json_object_get_boolean(params_obj);
                data_transfer.vehicle_status.light_status = value;
                publish_telemetry();
                printf(">> [LIGHT] Set to: %s\n", value ? "ON" : "OFF");
            } else if (strcmp(method, "setAirConditionTemp") == 0) {
                int value = json_object_get_int(params_obj);
                data_transfer.control_data.air_condition_temperature = value;
                publish_telemetry();
                printf(">> [A/C TEMP] Set to: %d °C\n", value);
            } else if (strcmp(method, "setSpeedLimit") == 0) {
                int value = json_object_get_int(params_obj);
                data_transfer.control_data.speed_limit = value;
                publish_telemetry();
                printf(">> [SPEED LIMIT] Set to: %d km/h\n", value);
            } else if (strcmp(method, "setDoorStatus") == 0) {
                int value = json_object_get_int(params_obj);
                data_transfer.vehicle_status.door_status = value;
                publish_telemetry();
                printf(">> [DOOR STATUS] Set to: %s \n", value ? "CLOSED" : "OPEN");
            }
            else if (strcmp(method, "setDoorStatus") == 0) {
                int value = json_object_get_int(params_obj);
                data_transfer.vehicle_status.door_status = value;
                publish_telemetry();
                printf(">> [DOOR STATUS] Set to: %s \n", value ? "CLOSED" : "OPEN");
            }
            else if (strcmp(method, "setFixedLimit") == 0) {
                int value = json_object_get_int(params_obj);
                thingsboard_fixed_setting = value;
                publish_telemetry();
                printf(">> [DOOR STATUS] Set to: %s \n", value ? "CLOSED" : "OPEN");
            }
        }
    }

    // Gửi dữ liệu điều khiển qua CAN
    struct can_frame air_frame;
    memset(&air_frame, 0, sizeof(air_frame));
    air_frame.can_id = AIRCONDITION_SPEEDLIMIT_CAN_ID;
    air_frame.can_dlc = 5;
    air_frame.data[0] = (uint8_t)data_transfer.control_data.air_condition_temperature;
    air_frame.data[1] = (uint8_t)data_transfer.control_data.speed_limit;
    air_frame.data[2] = (uint8_t)data_transfer.vehicle_status.engine_status;
    air_frame.data[3] = (uint8_t)data_transfer.vehicle_status.light_status;
    air_frame.data[4] = (uint8_t)data_transfer.vehicle_status.door_status;
    if (write(sockfd, &air_frame, sizeof(air_frame)) < 0) {
        perror("CAN write airConditionTemperature (thread)");
    } else {
        printf("Sent airConditionTemperature to CAN (thread): %d\n", data_transfer.control_data.air_condition_temperature);
    }

    // Gửi vehicle status qua Unix socket
    int sent = sendto(unix_fd, &data_transfer, sizeof(DataTransfer_t), 0, (struct sockaddr *)&addr, sizeof(addr));
    if (sent < 0) {
        perror("sendto");
    } else {
        printf("Sent vehicle status to GUI\n");
    }
    json_object_put(root);
}



//----------------------------------------------------------------END MQTT HANDLER--------------------------------------------------------------

// Hàm chính
int main(int argc, char *argv[]) {
    //--------------------------------Khoi tao CAN Socket--------------------------------
    pid_t pid;
    pid= fork();
    if (pid < 0) {
        perror("Fork failed");
        exit(EXIT_FAILURE);
    } else if (pid == 0) {
        // Child process
        system("dos2unix /root/MainProcess/script.sh");
        system("/root/MainProcess/script.sh");
    } else {
        sleep(2); // Đợi child process khởi động
        struct can_frame frame;
        const char *interface = "can1";
        
        if (argc > 1) {
            interface = argv[1];
        }
        
        printf("Setting up CAN interface: %s\n", interface);
        
        // Khởi tạo CAN socket
        sockfd = setup_can_socket(interface);
        if (sockfd < 0) {
            printf("Failed to setup CAN socket\n");
            return 1;
        }
        
        printf("CAN socket setup successful. Listening for messages...\n");
        
        // Reset packet receiver
        reset_packet_receiver();
        
        //---------------------------------Khoi tao Datagram Unix Domain Socket--------------------------------


        unix_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
        if (unix_fd < 0) {
            perror("socket");
            exit(1);
        }

        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);

        struct sockaddr_un test_addr;
        memset(&test_addr, 0, sizeof(test_addr));
        test_addr.sun_family = AF_UNIX;
        strncpy(test_addr.sun_path, "/tmp/test_socket", sizeof(test_addr.sun_path) - 1);
        unlink("/tmp/test_socket");
        bind(unix_fd, (struct sockaddr *)&test_addr, sizeof(test_addr));

        pthread_t tid;
        pthread_create(&tid, NULL, air_condition_thread, NULL);
        data_transfer.control_data.air_condition_temperature = 25; // Giá trị mặc định
        data_transfer.control_data.speed_limit = 80; // Giá trị mặc định

        //---------------------------------Kết nối MQTT--------------------------------
        mosquitto_lib_init();
        mosq = mosquitto_new(NULL, true, NULL);
        if (!mosq) {
            fprintf(stderr, "Failed to create mosquitto instance\n");
            return EXIT_FAILURE;
        }

        mosquitto_username_pw_set(mosq, ACCESS_TOKEN, NULL);
        mosquitto_connect_callback_set(mosq, on_connect);
        mosquitto_message_callback_set(mosq, on_message);

        if (mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, 60) != MOSQ_ERR_SUCCESS) {
            fprintf(stderr, "Unable to connect to broker\n");
            return EXIT_FAILURE;
        }

        // Start loop in a background thread
        mosquitto_loop_start(mosq);
        //-----------------------------------------Kết thúc khởi tạo--------------------------------


        //------------------------------------ Vòng lặp chính để nhận dữ liệu------------------------------------
        while (1) {
                ssize_t nbytes = read(sockfd, &frame, sizeof(struct can_frame));
                
                if (nbytes < 0) {
                    perror("CAN read error");
                    break;
                }
                
                if (nbytes < sizeof(struct can_frame)) {
                    printf("Incomplete CAN frame received\n");
                    continue;
                }
                
                // Xử lý frame theo CAN ID
                switch (frame.can_id) {
                    case PACKET_HEADER_ID:
                        reset_packet_receiver();
                        process_header_packet(&frame);
                        break;
                        
                    case PACKET_DATA_ID:
                        if (packet_receiver.frame_count > 0) {
                            if (process_data_packet(&frame)) {
                                // Đã nhận đủ dữ liệu
                                if (packet_receiver.message_type == 0x01 && 
                                    packet_receiver.data_size == sizeof(VehicleStatus)) {
                                    // Chuyển đổi dữ liệu nhận được thành VehicleStatus
                                    memcpy(&data_transfer.vehicle_status, packet_receiver.data_buffer, sizeof(VehicleStatus));
                                    print_vehicle_status(data_transfer.vehicle_status);
                                    // Gửi vehicle status qua Unix socket
                                    int sent = sendto(unix_fd, &data_transfer, sizeof(DataTransfer_t), 0, 
                                                    (struct sockaddr *)&addr, sizeof(addr));
                                    if (sent < 0) {
                                        perror("sendto");
                                    } else {
                                        printf("Sent vehicle status to GUI\n");
                                    }
                                    // Gửi qua MQTT
                                    publish_telemetry();
                                }
                                
                                // Reset để chuẩn bị cho message tiếp theo
                                reset_packet_receiver();
                            }
                        }
                        break;
                        
                    default:
                        printf("Unknown CAN ID: 0x%X\n", frame.can_id);
                        break;
                }
                
                
            }
            wait(NULL); // Chờ child process kết thúc
            mosquitto_destroy(mosq);
            mosquitto_lib_cleanup();
            close(sockfd);
            close(unix_fd);
            return 0;
        }
    
}