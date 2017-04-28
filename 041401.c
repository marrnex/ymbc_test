#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <ypspur.h>

#ifdef __WIN32
#	include <windows.h>
#endif

int main(int argc, char *argv[]){

	 Spur_init();//初期化

	//ロボットの制御パラメータの設定
	Spur_set_vel(0.2);//速度設定
	Spur_set_accel(1.0);//加速度設定
	Spur_set_angvel(M_PI/2.0);//角速度設定
	Spur_set_angaccel(M_PI/2.0);//角加速度設定

	Spur_set_pos_GL(0,0,0);//座標系の設定(x,y,th)

	Spur_stop_line_GL( 1.0, 0, 0 );
	while( !Spur_over_line_GL( 1.0 - 0.005, 0.0, 0.0 ) )
		usleep( 100000 );

	Spur_stop();

	return 0;
}