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

	Spur_stop_line_GL( 0.5, 0, 0 );//
	while( !Spur_over_line_GL( 0.5 - 0.005, 0.0, 0.0 ) )//0.5m進むまで
		usleep( 100000 );//待機


	Spur_spin_GL(M_PI / 2);//左90度回転
	while( !Spur_near_ang_GL(M_PI / 2, M_PI / 18.0 ) )
		usleep( 100000 );//待機


	Spur_stop_line_GL( 0.5, 0.5, M_PI / 2.0 );//直進
	while( !Spur_over_line_GL( 0.5, 0.5 - 0.005, M_PI / 2.0 ) )
		usleep( 100000 );

	Spur_spin_GL(0);//右90度回転
	while( !Spur_near_ang_GL(0, M_PI / 18.0 ) )
		usleep( 100000 );//待機

	Spur_stop_line_GL( 1.5, 0.5, 0 );//直進
	while( !Spur_over_line_GL( 1.5, 0.5 - 0.005, 0 ) )
		usleep( 100000 );

	Spur_spin_GL(-M_PI / 2);//右90度回転
	while( !Spur_near_ang_GL(-M_PI / 2, M_PI / 18.0 ) )
		usleep( 100000 );//待機

	Spur_stop_line_GL( 1.5, -0.5, -M_PI / 2 );//直進
	while( !Spur_over_line_GL( 1.5, -(0.5 - 0.005), -M_PI / 2 ) )
		usleep( 100000 );

	Spur_spin_GL(-M_PI);//右90度回転
	while( !Spur_near_ang_GL(-M_PI, M_PI / 18.0 ) )
		usleep( 100000 );//待機

	Spur_stop_line_GL( 0.5, -0.5, -M_PI );//直進
	while( !Spur_over_line_GL( 0.5, -(0.5 - 0.005), -M_PI ) )
		usleep( 100000 );

	Spur_spin_GL(M_PI/2);//右90度回転
	while( !Spur_near_ang_GL(M_PI/2, M_PI / 18.0 ) )
		usleep( 100000 );//待機

	Spur_stop_line_GL( 0.5, 0.5, M_PI / 2.0 );//直進
	while( !Spur_over_line_GL( 0.5, 0.5 - 0.005, M_PI / 2.0 ) )
		usleep( 100000 );

	Spur_spin_GL(0);//右90度回転
	while( !Spur_near_ang_GL(0, M_PI / 18.0 ) )
		usleep( 100000 );//待機

	Spur_stop_line_GL( 2.5, 0.5, 0 );//直進
	while( !Spur_over_line_GL( 2.5, 0.5 - 0.005, 0 ) )
		usleep( 100000 );

	Spur_spin_GL(-M_PI / 2);//右90度回転
	while( !Spur_near_ang_GL(-M_PI / 2, M_PI / 18.0 ) )
		usleep( 100000 );//待機

	Spur_stop_line_GL( 2.5, -0.5, -M_PI / 2 );//直進
	while( !Spur_over_line_GL( 2.5, -(0.5 - 0.005), -M_PI / 2 ) )
		usleep( 100000 );

	Spur_spin_GL(0);//右90度回転
	while( !Spur_near_ang_GL(0, M_PI / 18.0 ) )
		usleep( 100000 );//待機

	Spur_stop_line_GL( 3.8, -0.5, 0 );//直進
	while( !Spur_over_line_GL( 3.8, -(0.5 - 0.005), 0 ) )
		usleep( 100000 );

	Spur_spin_GL(M_PI/2);//右90度回転
	while( !Spur_near_ang_GL(M_PI/2, M_PI / 18.0 ) )
		usleep( 100000 );//待機

	Spur_stop_line_GL( 3.8, 1, M_PI/2 );//直進
	while( !Spur_over_line_GL( 3.8, (1 - 0.005), M_PI/2 ) )
		usleep( 100000 );

	Spur_spin_GL(-M_PI);//右90度回転
	while( !Spur_near_ang_GL(-M_PI, M_PI / 18.0 ) )
		usleep( 100000 );//待機

	Spur_stop_line_GL( 1.9, 1, -M_PI );//直進
	while( !Spur_over_line_GL( 1.9, (1 - 0.005), -M_PI ) )
		usleep( 100000 );
	

	Spur_spin_GL(M_PI/2);//右90度回転
	while( !Spur_near_ang_GL(M_PI/2, M_PI / 18.0 ) )
		usleep( 100000 );//待機

	Spur_stop_line_GL( 1.9, 2.7, M_PI/2 );//直進
	while( !Spur_over_line_GL( 1.9, (2.7 - 0.005), M_PI/2 ) )
		usleep( 100000 );

	Spur_spin_GL(M_PI);//右90度回転
	while( !Spur_near_ang_GL(M_PI, M_PI / 18.0 ) )
		usleep( 100000 );//待機

	Spur_stop_line_GL( -0.4, 2.7, M_PI );//直進
	while( !Spur_over_line_GL(-0.4, (2.5 - 0.005), M_PI ) )
		usleep( 100000 );//本来なら終わり？

	Spur_spin_GL( 0 );
	while( !Spur_near_ang_GL( 0, M_PI / 18.0 ) )
		usleep( 100000 );


	Spur_stop();

	return 0;
}