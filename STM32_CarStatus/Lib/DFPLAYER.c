/******************************************************************************************************************
 * @File:    DFPlayer Mini Module
 * @Author:  Khue Nguyen
 * @Website: khuenguyencreator.com
 * @Youtube: https://www.youtube.com/channel/UCt8cFnPOaHrQXWmVkk-lfvg
 * 
 * @Description:
 *   Đây là file nguồn triển khai các hàm điều khiển module DFPlayer Mini. Bao gồm cả hàm mức thấp (low-level)
 *   để giao tiếp UART và hàm mức cao (high-level) để điều khiển các chức năng của DFPlayer Mini.
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
 ******************************************************************************************************************/

#include "DFPLAYER.h"

// Khung dữ liệu mẫu để giao tiếp với DFPlayer Mini
uint8_t SendFrame[10] = {0x7E, 0xFF, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF};

//******************************** LOW Level Functions ****************************//

/**
 * @brief Truyền dữ liệu qua UART tới module DFPlayer.
 * @param MP3 Con trỏ tới cấu trúc DFPlayer.
 */
static void DFPLAYER_SendUART(DFPLAYER_Name* MP3)
{
    HAL_UART_Transmit(MP3->DFP_UART, (uint8_t *)&MP3->SendBuff, sizeof(MP3->SendBuff), 1000);
}

/**
 * @brief Tạo độ trễ.
 * @param Time Độ trễ (ms).
 */
static void DFPLAYER_Delay(uint16_t Time)
{
    HAL_Delay(Time);
}

/**
 * @brief Tính toán giá trị checksum cho khung dữ liệu.
 * @param thebuf Bộ đệm dữ liệu.
 * @return Giá trị checksum.
 */
static uint16_t DFPLAYER_GetChecksum(uint8_t *thebuf)
{
    uint16_t sum = 0;
    for (int i = 1; i < 7; i++) 
    {
        sum += thebuf[i];
    }
    return -sum;
}

/**
 * @brief Chuyển đổi số 16-bit sang định dạng big-endian.
 * @param thebuf Con trỏ tới vị trí cần ghi.
 * @param data Giá trị cần chuyển đổi.
 */
static void DFPLAYER_FillBigend(uint8_t *thebuf, uint16_t data)
{
    *thebuf = (uint8_t)(data >> 8);
    *(thebuf + 1) = (uint8_t)data;
}

/**
 * @brief Điền giá trị checksum vào khung dữ liệu.
 * @param MP3 Con trỏ tới cấu trúc DFPlayer.
 */
static void DFPLAYER_FillChecksum(DFPLAYER_Name* MP3)
{
    uint16_t checksum = DFPLAYER_GetChecksum(MP3->SendBuff);
    DFPLAYER_FillBigend(MP3->SendBuff + 7, checksum);
}

/**
 * @brief Gửi lệnh tới module DFPlayer Mini.
 * @param MP3 Con trỏ tới cấu trúc DFPlayer.
 * @param cmd Lệnh điều khiển.
 * @param high_arg Tham số cao.
 * @param low_arg Tham số thấp.
 */
static void DFPLAYER_SendCmd(DFPLAYER_Name* MP3, uint8_t cmd, uint16_t high_arg, uint16_t low_arg)
{
    DFPLAYER_Delay(100);
    MP3->SendBuff[3] = cmd;
    MP3->SendBuff[5] = high_arg;
    MP3->SendBuff[6] = low_arg;
    DFPLAYER_FillChecksum(MP3);
    DFPLAYER_SendUART(MP3);
}

//******************************** High Level Functions ****************************//

/**
 * @brief Khởi tạo module DFPlayer Mini.
 * @param MP3 Con trỏ tới cấu trúc DFPlayer.
 * @param UART Con trỏ tới cấu trúc UART_HandleTypeDef.
 */
void DFPLAYER_Init(DFPLAYER_Name* MP3, UART_HandleTypeDef* UART)
{
    for (int i = 0; i < 10; i++)
    {
        MP3->SendBuff[i] = SendFrame[i];
    }
    MP3->DFP_UART = UART;
}

/**
 * @brief Phát bài hát theo số thứ tự.
 * @param MP3 Con trỏ tới cấu trúc DFPlayer.
 * @param num Số thứ tự bài hát.
 */
void DFPLAYER_PlayTrack(DFPLAYER_Name* MP3, uint16_t num)
{
    uint8_t num1 = num >> 8;
    uint8_t num2 = num;
    DFPLAYER_SendCmd(MP3, DFP_PLAYTRACK, num1, num2);
}

/**
 * @brief Phát bài hát tiếp theo.
 * @param MP3 Con trỏ tới cấu trúc DFPlayer.
 */
void DFPLAYER_Next(DFPLAYER_Name* MP3)
{
    DFPLAYER_SendCmd(MP3, DFP_NEXT, 0, 0);
}

/**
 * @brief Phát bài hát trước đó.
 * @param MP3 Con trỏ tới cấu trúc DFPlayer.
 */
void DFPLAYER_Prev(DFPLAYER_Name* MP3)
{
    DFPLAYER_SendCmd(MP3, DFP_PREV, 0, 0);
}

/**
 * @brief Thiết lập âm lượng.
 * @param MP3 Con trỏ tới cấu trúc DFPlayer.
 * @param volume Giá trị âm lượng (0-30).
 */
void DFPLAYER_SetVolume(DFPLAYER_Name* MP3, uint16_t volume)
{
    uint8_t volume1 = volume >> 8;
    uint8_t volume2 = volume;
    DFPLAYER_SendCmd(MP3, DFP_SETVOLUME, volume1, volume2);
}

/**
 * @brief Tiếp tục phát nhạc.
 * @param MP3 Con trỏ tới cấu trúc DFPlayer.
 */
void DFPLAYER_Play(DFPLAYER_Name* MP3)
{
    DFPLAYER_SendCmd(MP3, DFP_PLAY, 0, 0);
}

/**
 * @brief Tạm dừng phát nhạc.
 * @param MP3 Con trỏ tới cấu trúc DFPlayer.
 */
void DFPLAYER_Pause(DFPLAYER_Name* MP3)
{
    DFPLAYER_SendCmd(MP3, DFP_PAUSE, 0, 0);
}

/**
 * @brief Dừng phát nhạc.
 * @param MP3 Con trỏ tới cấu trúc DFPlayer.
 */
void DFPLAYER_Stop(DFPLAYER_Name* MP3)
{
    DFPLAYER_SendCmd(MP3, DFP_STOP, 0, 0);
}

/**
 * @brief Phát nhạc ngẫu nhiên.
 * @param MP3 Con trỏ tới cấu trúc DFPlayer.
 */
void DFPLAYER_RandomPlay(DFPLAYER_Name* MP3)
{
    DFPLAYER_SendCmd(MP3, DFP_RANDOM, 0, 0);
}

/**
 * @brief Phát nhạc theo file trong thư mục chỉ định.
 * @param MP3 Con trỏ tới cấu trúc DFPlayer.
 * @param folder Thư mục chứa file nhạc.
 * @param num Số thứ tự file nhạc.
 */
void DFPLAYER_PlayFileInFolder(DFPLAYER_Name* MP3, uint8_t folder, uint32_t num)
{
    DFPLAYER_SendCmd(MP3, DFP_PLAYFILEINFOLDER, folder, num);
}
