#ifndef _DISPLAY_MENU__H_
#define _DISPLAY_MENU__H_


#if (SELF_POWER_EN > 0u)
#define FORM_ID_MAIN_FORM       0
#define FORM_ID_KEEP_FORM       1
#define FORM_ID_SELF_FORM       2
#define FORM_ID_ABOUT_FORM       3
#define FORM_ID_SETTING_FORM      4
#else
#define FORM_ID_MAIN_FORM         0
#define FORM_ID_KEEP_FORM         1
#define FORM_ID_ABOUT_FORM        2
#define FORM_ID_SETTING_FORM      3
#endif

#if (SELF_POWER_EN > 0u)
#define MAX_FORM_NUM      5
#else
#define MAX_FORM_NUM      4
#endif

#define FIND_UP      0
#define FIND_DOWN    1

#define FORM_MSG_NON       0X0
#define FORM_MSG_RERRESH   0X10
#define FORM_MSG_KEY       0X11
#define FORM_MSG_DATA      0X12


#define MENU_SETTING_NUM   3

#define DROP_TIME_ADJUST_STEP 100
#define SELF_TIME_ADJUST_STEP 1

#define MAX_DROP_TIME 2000

#if 0
#define SELF_POWER_ON_TIME  100
#else //华兄
#define SELF_POWER_ON_TIME_HIGH  99
#define SELF_POWER_ON_TIME_LOW   9
#endif

#define SYS_VER_LEN              5

typedef enum //华兄
{
    enum_pwd_0,
    enum_pwd_1,
    enum_pwd_2,
    enum_pwd_3,
    enum_pwd_4,
    enum_pwd_5,
    enum_pwd_ok,
    enum_pwd_cancel,
    enum_pwd_button
}enum_pwd_item;

typedef enum //华兄
{
    enum_self_power_0,
    enum_self_power_1,
    enum_self_power_button
}enum_self_power_item;

typedef enum //华兄
{
    enum_mode_0 = 0,
    enum_mode_1 = 1,
    enum_mode_button = 2
}enum_mode_item;

typedef enum
{

    en_year_set,
    en_month_set,
    en_day_set,
    en_hour_set,
    en_min_set,
    en_sec_set,
    en_ok_set,
    //en_exit_set,
    en_time_butt,

}en_time_item;

typedef enum menu_set_item
{
    enum_menu_password, //华兄
    en_menu_set_droptimelength,
    
#if (SELF_POWER_EN > 0u)    
    en_menu_set_selfpowerdeadline, //华兄
    en_menu_set_selfpoweractiontime, //华兄
#endif

    en_menu_set_hdswitch,
    
#if (SELF_POWER_EN > 0u)    
    en_menu_set_selfswitch,
#endif

#if (MULTI_MODE_EN > 0u)
    en_menu_set_mode, //华兄
#endif

    en_menu_set_addr,
    en_menu_set_date,
    en_menu_set_reset,
    enum_menu_set_password, //华兄
    en_menu_set_butt,
}EN_SET_ITEM;

typedef int  (*FORM_PROC_PTR)(unsigned int key_event, unsigned int form_msg);

typedef struct _disp_menu_item_
{
    //unsigned char form_id;
    FORM_PROC_PTR event_proc;
} LCD_FORM, *P_LCD_FORM;

extern const char SYS_HARDWARE_VER[];
extern const char SYS_SOFTWARE_VER[];
extern BOOL g_pwd_flag; //华兄
extern enum_pwd_item g_new_pwd[7], g_pwd[6];

void LCD_Time_Refreash();
int LCD_disp_main_form(unsigned int key_event, unsigned int form_msg);
int LCD_disp_setting_form(unsigned int key_event, unsigned int form_msg);
int LCD_disp_self_form(unsigned int key_event, unsigned int form_msg);
int LCD_disp_about_form(unsigned int key_event, unsigned int form_msg);
void GUI_Sec_Refresh();    
#endif
