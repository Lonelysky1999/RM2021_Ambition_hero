#ifndef __UI_H
#define __UI_H
#include "main.h"
#include "sys.h"

#define  Cred1   0x0101     //ºì1Ó¢ÐÛ¿Í»§¶ËID
#define  Cred2   0x0102     //ºì2¹¤³Ì¿Í»§¶ËID
#define  Cred3   0x0103     //ºì3²½±ø¿Í»§¶ËID
#define  Cred4   0x0104     //ºì4²½±ø¿Í»§¶ËID
#define  Cred5   0x0105     //ºì5²½±ø¿Í»§¶ËID
#define  Cred6   0x0106     //ºì6ÎÞÈË»ú¿Í»§¶ËID
//computer red 
#define  Cblue1  0x0165     //À¶1Ó¢ÐÛ¿Í»§¶ËID
#define  Cblue2  0x0166     //À¶2¹¤³Ì¿Í»§¶ËID
#define  Cblue3  0x0167     //À¶3²½±ø¿Í»§¶ËID
#define  Cblue4  0x0168     //À¶4²½±ø¿Í»§¶ËID
#define  Cblue5  0x0169     //À¶5²½±ø¿Í»§¶ËID
#define  Cblue6  0x016A     //À¶6ÎÞÈË»ú¿Í»§¶ËID
//computer blue

#define  red1    1          //ºì1»úÆ÷ÈËID
#define  red2    2          //ºì2»úÆ÷ÈËID
#define  red3    3          //ºì3»úÆ÷ÈËID
#define  red4    4          //ºì4»úÆ÷ÈËID
#define  red5    5          //ºì5»úÆ÷ÈËID
#define  red6    6          //ºì6»úÆ÷ÈËID
#define  red7    7  
#define  red9    9
//robot red
#define  blue1   101        //ºì1»úÆ÷ÈËID
#define  blue2   102        //ºì2»úÆ÷ÈËID
#define  blue3   103        //ºì3»úÆ÷ÈËID
#define  blue4   104        //ºì4»úÆ÷ÈËID
#define  blue5   105        //ºì5»úÆ÷ÈËID
#define  blue6   106        //ºì6»úÆ÷ÈËID
#define  blue7   107        //ºì6»úÆ÷ÈËID
#define  blue9   109        //ºì6»úÆ÷ÈËID

//robot blue

#define  delP     0x0100     //É¾³ýÍ¼ÐÎÖ¸ÁîID         del picture    ID
#define  draw1P   0x0101     //»æÖÆÒ»¸öÍ¼ÐÎÖ¸ÁîID     draw 1 pixture ID
#define  draw2P   0x0102     //»æÖÆÁ½¸öÍ¼ÐÎÖ¸ÁîID     draw 2 pixture ID
#define  draw5P   0x0103     //»æÖÆÎå¸öÍ¼ÐÎÖ¸ÁîID     draw 5 pixture ID
#define  draw7P   0x0104     //»æÖÆÆß¸öÍ¼ÐÎÖ¸ÁîID     draw 7 pixture ID
#define  drawWord 0x0110     //»æÖÆ×Ö·û               draw word

#define  RtoR     0x0301     //»úÆ÷ÈË¶Ô»úÆ÷ÈËÍ¨ÐÅcmdid±êÊ¶
//robot to robot

#define  red      0   //ºìÉ«
#define  blue     0   //À¶É«
#define  yellow   1   //»ÆÉ«
#define  green    2   //ÂÌÉ«
#define  orange   3   //éÙÉ«
#define  amaranth 4   //×ÏºìÉ«
#define  pink     5   //·ÛÉ«
#define  cyan     6   //ÇàÉ«
#define  black    7   //ºÚÉ«
#define  white    8   //°×É«

#define  line     0   //Ö±Ïß
#define  rele     1   //¾ØÐÎ    rectangle
#define  round    2   //ÕýÔ²  
#define  elal     3   //ÍÖÔ²    elliptical
#define  ciar     4   //Ô²»¡    circular arc

typedef struct//·¢ËÍÍ¼ÐÎÊý¾Ý¶Î
{
	uint8_t  graphic_name[4]; 
  uint32_t operate_tpye:3; 
  uint32_t graphic_tpye:3; 
  uint32_t layer:4; 
  uint32_t color:4; 
  uint32_t start_angle:9;
  uint32_t end_angle:9;
  uint32_t width:10; 
  uint32_t start_x:11; 
  uint32_t start_y:11; 
  uint32_t radius:10; 
  uint32_t end_x:11; 
  uint32_t end_y:11;
}graphic_data_struct_t; 

typedef struct//ÕûÌåÊý¾ÝÖ¡
{
	 u8 SOF;                  //Ö¡Í·
	 u16 data_length;         //Êý¾Ý³¤¶È
   u8 seq;                  //°üÊý
   u8 CRC8;                 //CRC8Ð£Ñé
   u16 cmd_id;	            //ÃüÁîÂëID
   u16 ID;                  //Êý¾ÝÄÚÈÝID
	 u16 sendID;              //·¢ËÍÕßID
   u16 receptionID;         //½ÓÊÕÕßID
	 graphic_data_struct_t picture1;        //Í¼ÐÎ1
	 u16 CRC16;               //CRC16Ð£Ñé

}draw1picture;

typedef struct//ÕûÌåÊý¾ÝÖ¡
{
	 u8 SOF;                  //Ö¡Í·
	 u16 data_length;         //Êý¾Ý³¤¶È
   u8 seq;                  //°üÊý
   u8 CRC8;                 //CRC8Ð£Ñé
   u16 cmd_id;	            //ÃüÁîÂëID
   u16 ID;                  //Êý¾ÝÄÚÈÝID
	 u16 sendID;              //·¢ËÍÕßID
   u16 receptionID;         //½ÓÊÕÕßID
	u8 operate_tpye;
	u8 layer;
	 u16 CRC16;               //CRC16Ð£Ñé

}clean_ui_t;

void drawOnePicture(u16 ID,u16 sendID,u16 receptionID,graphic_data_struct_t PICTURE);
void clean_ui(u16 sendID,u16 receptionID);


#endif



