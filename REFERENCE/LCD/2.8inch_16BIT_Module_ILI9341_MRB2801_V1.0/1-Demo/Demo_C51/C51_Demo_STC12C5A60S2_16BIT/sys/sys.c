//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//����Ӳ������Ƭ��STC12C5A60S2,����30M  ��Ƭ��������ѹ3.3V
//QDtech-LCDҺ������ for C51
//xiao��@ShenZhen QDtech co.,LTD
//��˾��վ:www.qdtft.com
//�Ա���վ��http://qdtech.taobao.com
//wiki������վ��http://www.lcdwiki.com
//��˾�ṩ����֧�֣��κμ������⻶ӭ��ʱ����ѧϰ
//�̻�(����) :+86 0755-23594567 
//�ֻ�:15989313508���빤�� 
//����:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com
//����֧��QQ:3002773612  3002778157
//��������QQȺ:324828016
//��������:2018/7/7
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������ȫ�����Ӽ������޹�˾ 2018-2028
//All rights reserved
//********************************************************************************
//=========================================��Դ����================================================//
//VDD��DC 5V��3.3V��Դ
//GND�ӵ�
//=======================================Һ���������߽���==========================================//
//��ģ��Ĭ��������������Ϊ16λ����
//Һ����ģ��             C51��Ƭ��
// DB0~DB7       ��       P00~P07        //���ݵ�8λ
// DB8~DB15      ��       P20~P27        //���ݸ�8λ(���ʹ��8λģʽ����ʹ�ø�8λ������������)
//=======================================Һ���������߽���==========================================//
//Һ����ģ��             C51��Ƭ��
//   CS          ��        P13           //Ƭѡ�����ź�
//   RST         ��        P33           //��λ�ź�
//   RS          ��        P12           //����/����ѡ������ź�
//   WR          ��        P11           //д�����ź�
//   RD          ��        P10           //�������ź�
//   BL          ��        P32           //��������ź�
//=========================================����������=========================================//
//��ʹ�ô�������ģ�鱾��������������ɲ�����
//������ʹ�õ�������������ΪSPI
//������ģ��             C51��Ƭ��
//  CLK          ��        P36           //������SPIʱ���ź�
//  T_CS         ��        P37           //������Ƭѡ�����ź�
//  MISO         ��        P35           //������SPIд�ź�
//  MOSI         ��        P34           //������SPI���ź�
//  PEN          ��        P40           //��������Ӧ����źţ��絥Ƭ����P4�飬�����и�����������IO���޸Ĵ��붨��
//**************************************************************************************************/			

#include "sys.h"
void delay_ms(int count)  // /* X1ms */
{
        int i,j;
        for(i=0;i<count;i++)
                for(j=0;j<1000;j++);
}

void delay_us(int count)  // /* X1us */
{
        int i,j;
        for(i=0;i<count;i++)
                for(j=0;j<1;j++);
}