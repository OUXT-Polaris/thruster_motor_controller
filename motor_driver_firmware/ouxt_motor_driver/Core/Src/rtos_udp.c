/*
 * rtos_udp.c
 *
 *  Created on: Jun 9, 2024
 *      Author: masaya
 */

//@sa https://qiita.com/hirekatsu0523/items/df651e7da8dee32e7891

#include <rtos_udp.h>
static struct receive_data ros_data = { };
static struct send_data f7_data = { };
struct timeval tv;
//IPとポート
#define F7_ADDR "192.168.0.200"
#define PC_ADDR "192.168.0.100"
#define F7_PORT 1000
#define PC_PORT 1000

osThreadId udpTaskHandle;
uint32_t udpTaskBuffer[512];
osStaticThreadDef_t udpTaskControlBlock;
void UDPSendReceive(void const *argument);

void UDPDefineTasks() {
	osThreadStaticDef(udpTask, UDPSendReceive, osPriorityNormal, 0, 512, udpTaskBuffer, &udpTaskControlBlock);
	udpTaskHandle = osThreadCreate(osThread(udpTask), NULL);
}

void UDPSendReceive(void const *argument) {
	osDelay(1000);
	fd_set reading;
	int rxsock;
    int maxfd = 0;
	char rxbuf[256]; //最大受信データサイズ
	char txbuf[256]; //最大送信データサイズ
	struct sockaddr_in rxAddr, txAddr;
    //ソケットを作成(IPv4, UDPプロトコル)
	rxsock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    //構造体の初期化
	memset((char*) &rxAddr, 0, sizeof(rxAddr));
	memset((char*) &txAddr, 0, sizeof(txAddr));
    //受信アドレスの設定
	rxAddr.sin_family = AF_INET;
	rxAddr.sin_len = sizeof(rxAddr);
	rxAddr.sin_addr.s_addr = INADDR_ANY; //全てのIPから受信
	rxAddr.sin_port = htons(F7_PORT); //マイコンのポート
    //送信アドレスの設定
	txAddr.sin_family = AF_INET;
	txAddr.sin_len = sizeof(txAddr);
	txAddr.sin_addr.s_addr = inet_addr(PC_ADDR); //PCのIP
	txAddr.sin_port = htons(PC_PORT); //PCのポート
    //ソケットにマイコンのIPとポートを紐づける
	(void)lwip_bind(rxsock, (struct sockaddr*)&rxAddr, sizeof(rxAddr));
    //ディスクリプタ集合の初期化
	FD_ZERO(&reading);
	while (1) {
		FD_SET(rxsock, &reading);
		maxfd = rxsock + 1;
		memset(&tv, 0, sizeof(tv));
        tv.tv_usec = 20000;
		(void)select(maxfd, &reading, NULL, NULL, &tv);
        //readingにrxsockが登録されているか調べる
		if (FD_ISSET(rxsock, &reading)) {
			socklen_t n;
			socklen_t len = sizeof(rxAddr);
            //rxbufに受信データを格納
			n = lwip_recvfrom(rxsock, (char*) rxbuf, sizeof(rxbuf), (int) NULL, (struct sockaddr*) &rxAddr, &len);
			if (n == sizeof(struct receive_data)) {
				//rxbufの位置にreceive_data構造体を作る
				struct receive_data *d = (struct receive_data*) &rxbuf;
				//受信データをコピーする
				memcpy(&ros_data, d, sizeof(struct receive_data));
			}
		}
        //txbufの位置にsdを宣言
    	struct send_data* sd = (struct send_data*)&txbuf;
        //送信データをコピーする
		memcpy(sd, &f7_data, sizeof(struct send_data));
		(void)lwip_sendto(rxsock, (char*)txbuf, sizeof(struct send_data), 0, (struct sockaddr*)&txAddr, sizeof(txAddr));
		osDelay(10);
	}
}

struct receive_data GetROSData(){
	//データを取得する関数
	return ros_data;
}

void SendF7Data(struct send_data *data){
	//データを送信する関数
	memcpy(&f7_data, data, sizeof(struct send_data));
}

