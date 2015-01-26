#ifndef		__REGISTER_GPM650_H
#define		__REGISTER_GPM650_H

int32 reg_gpm650_ctrl_operation(int32 reg_addr, int32 reg_val);

int32 reg_gpm650_fast_read(int32 reg_addr, int32 reg_num, uint8 resp[]);

int32 reg_gpm650_grp_read(int32 reg_addr, int32 reg_num, uint8 resp[]);
int32 reg_gpm650_grp_write(int32 reg_addr, int32 reg_num, const uint8 req[]);

int32 reg_gpm650_time_read(int32 reg_addr, int32 reg_num, uint8 resp[]);
int32 reg_gpm650_time_write(int32 reg_addr, int32 reg_num, const uint8 req[]);

int32 reg_gpm650_ext_grp_read(int32 reg_addr, int32 reg_num, uint8 req[]);
int32 reg_gpm650_ext_grp_write(int32 reg_addr, int32 reg_num, const uint8 req[]);

int32 reg_gpm650_conf_read(int32 reg_addr, int32 reg_num, uint8 resp[]);
int32 reg_gpm650_conf_write(int32 reg_addr, int32 reg_num, const uint8 req[]);


#endif	/* __REGISTER_651X_H */

