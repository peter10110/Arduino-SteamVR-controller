#ifndef MPU_H
#define MPU_H

struct s_quat { float w, x, y, z; }; 

struct s_mympu {
	float ypr[3];
	float gyro[3];
  s_quat quat;
};

extern struct s_mympu mympu;

int mympu_open(unsigned int rate);
int mympu_update();

#endif

