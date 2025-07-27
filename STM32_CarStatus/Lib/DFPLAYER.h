/******************************************************************************************************************
 * @File:    DFPlayer Mini Module
 * @Author:  Khue Nguyen
 * @Website: khuenguyencreator.com
 * @Youtube: https://www.youtube.com/channel/UCt8cFnPOaHrQXWmVkk-lfvg
 * 
 * @Description:
 *   Đây là file header điều khiển module DFPlayer Mini sử dụng giao thức UART thông qua thư viện HAL.
 *   Người dùng có thể dễ dàng phát nhạc, điều chỉnh âm lượng, chuyển bài, dừng phát, và các chức năng khác
 *   của module DFPlayer Mini.
 * 
 * @Instructions:
 *   - Yêu cầu sử dụng thư viện HAL.
 *   - Khởi tạo UART với baud rate 9600.
 *   - Tạo một biến kiểu `DFPLAYER_Name` để đại diện cho DFPlayer:
 *       DFPLAYER_Name MP3;
 *   - Khởi tạo module DFPlayer bằng cách gọi hàm:
 *       DFPLAYER_Init(&MP3, &huart1);
 *   - Để sử dụng các chức năng, truyền địa chỉ của biến DFPlayer vào các hàm điều khiển, ví dụ:
 *       DFPLAYER_Play(&MP3);
 * 
 * @API:
 *   - DFPLAYER_Init: Khởi tạo DFPlayer Mini.
 *   - DFPLAYER_PlayTrack: Phát bài hát theo số thứ tự.
 *   - DFPLAYER_Next: Phát bài hát tiếp theo.
 *   - DFPLAYER_Prev: Phát bài hát trước đó.
 *   - DFPLAYER_SetVolume: Thiết lập âm lượng.
 *   - DFPLAYER_Play: Tiếp tục phát nhạc.
 *   - DFPLAYER_Pause: Tạm dừng phát nhạc.
 *   - DFPLAYER_Stop: Dừng phát nhạc.
 *   - DFPLAYER_RandomPlay: Phát nhạc ngẫu nhiên.
 *   - DFPLAYER_PlayFileInFolder: Phát nhạc theo file trong thư mục chỉ định.
 * 
 ******************************************************************************************************************/
#ifndef __DFPLAYER_H
#define __DFPLAYER_H

#include "stm32f1xx_hal.h"

// Định nghĩa cấu trúc DFPlayer
typedef struct
{
    UART_HandleTypeDef* DFP_UART;  // UART sử dụng để giao tiếp với DFPlayer
    uint8_t SendBuff[10];         // Bộ đệm dữ liệu gửi đi
    uint16_t Checksum;            // Giá trị checksum để kiểm tra dữ liệu
} DFPLAYER_Name;

// Các lệnh điều khiển DFPlayer
#define DFP_PLAYTRACK             0X03  // Phát bài hát theo số thứ tự
#define DFP_NEXT                  0X01  // Phát bài tiếp theo
#define DFP_PREV                  0X02  // Phát bài trước đó
#define DFP_SETVOLUME             0X06  // Thiết lập âm lượng
#define DFP_PLAY                  0X0D  // Tiếp tục phát
#define DFP_PAUSE                 0X0E  // Tạm dừng phát
#define DFP_STOP                  0X16  // Dừng phát
#define DFP_RANDOM                0X18  // Phát ngẫu nhiên
#define DFP_PLAYFILEINFOLDER      0X0F  // Phát nhạc theo file trong thư mục

// Khai báo các hàm điều khiển DFPlayer
void DFPLAYER_Init(DFPLAYER_Name* MP3, UART_HandleTypeDef* UART); // Khởi tạo DFPlayer Mini
void DFPLAYER_PlayTrack(DFPLAYER_Name* MP3, uint16_t num);       // Phát bài hát theo số thứ tự
void DFPLAYER_Next(DFPLAYER_Name* MP3);                         // Phát bài tiếp theo
void DFPLAYER_Prev(DFPLAYER_Name* MP3);                         // Phát bài trước đó
void DFPLAYER_SetVolume(DFPLAYER_Name* MP3, uint16_t volume);   // Thiết lập âm lượng
void DFPLAYER_Play(DFPLAYER_Name* MP3);                         // Tiếp tục phát nhạc
void DFPLAYER_Pause(DFPLAYER_Name* MP3);                        // Tạm dừng phát nhạc
void DFPLAYER_Stop(DFPLAYER_Name* MP3);                         // Dừng phát nhạc
void DFPLAYER_RandomPlay(DFPLAYER_Name* MP3);                   // Phát nhạc ngẫu nhiên
void DFPLAYER_PlayFileInFolder(DFPLAYER_Name* MP3, uint8_t folder, uint32_t num); // Phát nhạc trong thư mục

#endif /* __DFPLAYER_H */
