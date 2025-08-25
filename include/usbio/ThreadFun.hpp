/**********************************************************************
 *
 *  ThreadFun.h
 *
 *  Header of Thread Function
 *
 *  v 0.0.0 2013.4.9 by Golden Wang
 *
 *    create
 *
 **********************************************************************/

#include "Global.hpp"
#include "USBIO_Comm.hpp"

typedef struct receivethread {
	int DevIdx;
	int ReceiveHandle;
	pthread_t RxThread;  //tid
	//  PICPDAS_USBIO_DEV Interface;
	pthread_mutex_t mutex;
	WORD(*thread_func) (struct receivethread *, BOOL);
	pthread_cond_t cond;
}rxthread_t;

//typedef struct receivethread rxthread_t;
static rxthread_t *rx_thread[MAX_USB_DEVICES] = { 0 };
static int enable_rxthread[MAX_USB_DEVICES] = { 0 };

rxthread_t *new_rxthread(long index);
WORD rxthread_threadfunc(struct receivethread *thread, BOOL Flage);
void Rx_Thread(int index);
//void Rx_Thread(PICPDAS_USBIO_DEV USBDev);

rxthread_t *new_rxthread(long index)
{
	int ret;

	rxthread_t *thread = malloc(sizeof(rxthread_t));
	memset(thread, 0, sizeof(rxthread_t));
/*
	thread->Interface = malloc(sizeof(ICPDAS_USBIO_DEV));
	memset(thread->Interface, 0, sizeof(ICPDAS_USBIO_DEV));
*/
	//thread->USBIO->PortOpen = 1;
	//thread->USBIO->DevPort = DevPort;

	pthread_cond_init(&thread->cond, NULL);
	pthread_mutex_init(&thread->mutex, NULL);

//  ret = pthread_create(&thread->RxThread, NULL, (void *) Rx_Thread, (void *) thread->Interface);

	ret =
		pthread_create(&thread->RxThread, NULL, (void *) Rx_Thread,
					   (void *) index);

	if (ret) 
		thread->ReceiveHandle = 0;	//create Rx_Thread fail
	else 
	{
		enable_rxthread[index] = 1;
		thread->DevIdx = index;
		thread->ReceiveHandle = 1;	//create Rx_Thread OK
	}
	thread->thread_func = rxthread_threadfunc;
	return thread;
}

WORD rxthread_threadfunc(struct receivethread * thread, BOOL Flage)
{	
	DWORD ret;
	int index = thread->DevIdx;

	if (!thread->ReceiveHandle)
		return FAILURE;
	//printf("Flage is %d\n",Flage);
	if (Flage) 
	{
		pthread_mutex_lock(&thread->mutex);
		enable_rxthread[index] = 1;	//resume Rx_Thread
		ret = pthread_cond_signal(&thread->cond);
		pthread_mutex_unlock(&thread->mutex);

		if (ret)
			return FAILURE;
	} 
	else 
	{
		pthread_mutex_lock(&thread->mutex);
		enable_rxthread[index] = 0;	//suspend Rx_Thread
		pthread_mutex_unlock(&thread->mutex);
	}
	return SUCCESS;
}

//void Rx_Thread(PICPDAS_USBIO_DEV USBDev)
void Rx_Thread(int index)
{
	WORD Ret, wt;
	DWORD Sleep_Cnt = 0;
	DWORD i = 0;
	PICPDAS_USBIO_DEV USBDev = usbio[index];
	int fd = USBDev->FD;
	//  int index = USBDev->DevIdx;
	BYTE byPkt[65];

	pthread_detach(pthread_self());

	while (!rx_thread[index]) {  //addmyself not execute
		//printf("initialize rx_thread[%d] memory OK\n", index);
	}

	//printf("%s ... /dev/hidraw%d : PortOpen %d\n", __FUNCTION__, index, USBDev->m_bDeviceOpened);
	
	while (USBDev->m_bDeviceOpened) 
	{
		if (!enable_rxthread[index]) 
		{
			pthread_mutex_lock(&rx_thread[index]->mutex);
			//printf("in thread_wait()\n");
			pthread_cond_wait(&rx_thread[index]->cond,
							  &rx_thread[index]->mutex);
			//printf("in thread_wait(1)\n");
			pthread_mutex_unlock(&rx_thread[index]->mutex);
		}


		pthread_mutex_lock(&rx_thread[index]->mutex);

		//printf("%s ... ready read\n",__func__);
		memset(byPkt, 0x0, sizeof(byPkt));
		Ret = read(fd, byPkt, 54);
		//printf("%s ... read\n",__func__);
		if (Ret) 
		{
			switch (byPkt[0]) 
			{
			case 0x88:
				//printf("Emergency packet received\n");
				break;

			case 0x83:
				//printf("SYNC packet received\n");
				break;

			
			case 0x11:
			case 0x3case 0x81::
			case 0x1:
				// Normal or error packet received
				if ((byPkt[0] >= 0 && byPkt[0] <= 0x13)
					|| byPkt[0] == 0x81) 
				{
					//printf("Normal(error) packet received\n");

					if (USBDev->m_wCurrentAccessObj ==
						*(WORD *)&byPkt[NOR_PKT_OBJECT_INDEX]) {
						//printf("Object code is: [%04x]\n", *(WORD *) & byPkt[NOR_PKT_OBJECT_INDEX]);

						memcpy(USBDev->m_byNormalPkt, byPkt,
							   (*(WORD *)&byPkt[NOR_PKT_PACKET_LENGTH] +
								10)); //addmyself byPkt copy to m_bynormalPkt
						USBDev->m_byNormalPktArrive = 1;
					}
				}
				break;
/*
			case 0x41:

				// Data log packet received
				if(byPkt[0] >= 0x40 && byPkt[0] <= 0x43)
				{
					tempClass->SetActivePktByteArray(byPkt, (*(WORD *)&byPkt[ACT_PKT_PACKET_LENGTH] + 4));
				}

				break;
*/
			}  //end switch
		} 
		else 
		{
			//  perror("read");
		}

		pthread_mutex_unlock(&rx_thread[index]->mutex);

		if (((Sleep_Cnt++) & 0x1f) == 0) {
			Sleep_Cnt = 0;
			usleep(1);
		}
		
	}//end while
	pthread_exit(0);
}
