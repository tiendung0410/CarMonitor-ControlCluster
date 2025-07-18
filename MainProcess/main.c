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

#define SOCKET_PATH "/tmp/gui_socket"

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
    uint8_t avg_speed;
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
    printf("Average Speed: %d km/h\n", status.avg_speed);
    printf("Transmission Gear: %d\n", status.transmission_gear);
    printf("GPS Latitude: %.6f\n", status.gps_lat);
    printf("GPS Longitude: %.6f\n", status.gps_lon);
    printf("========================\n\n");
}

typedef struct {
    int unix_fd;
    int can_fd;
} ThreadArgs;

void* air_condition_thread(void* arg) {
    ThreadArgs *args = (ThreadArgs*)arg;
    int unix_fd = args->unix_fd;
    int can_fd = args->can_fd;
    while (1) {
        controlData_t controlData;
        ssize_t recv_bytes = recvfrom(unix_fd, &controlData, sizeof(controlData_t), 0, NULL, NULL);
        if (recv_bytes > 0) {

            printf("Received air condition temperature (thread): %d\n", controlData.air_condition_temperature);
            // Cập nhật biến toàn cục
            data_transfer.control_data.air_condition_temperature = controlData.air_condition_temperature;
            data_transfer.control_data.speed_limit = controlData.speed_limit;
            // Gửi qua CAN
            struct can_frame air_frame;
            memset(&air_frame, 0, sizeof(air_frame));
            air_frame.can_id = AIRCONDITION_SPEEDLIMIT_CAN_ID;
            air_frame.can_dlc = 2;
            air_frame.data[0] = (uint8_t)data_transfer.control_data.air_condition_temperature;
            air_frame.data[1] = (uint8_t)data_transfer.control_data.speed_limit;
            if (write(can_fd, &air_frame, sizeof(air_frame)) < 0) {
                perror("CAN write airConditionTemperature (thread)");
            } else {
                printf("Sent airConditionTemperature to CAN (thread): %d\n", controlData.air_condition_temperature);
            }
        }
    }
    return NULL;
}



// Hàm chính
int main(int argc, char *argv[]) {
    //--------------------------------Khoi tao CAN Socket--------------------------------
    int sockfd;
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

    int unix_fd;
    struct sockaddr_un addr;

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

    ThreadArgs thread_args;
    thread_args.unix_fd = unix_fd;
    thread_args.can_fd = sockfd;
    pthread_t tid;
    pthread_create(&tid, NULL, air_condition_thread, &thread_args);
    data_transfer.control_data.air_condition_temperature = 25; // Giá trị mặc định
    data_transfer.control_data.speed_limit = 80; // Giá trị mặc định

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
    
    close(sockfd);
    close(unix_fd);
    return 0;
}