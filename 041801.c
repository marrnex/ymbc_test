#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <ypspur.h>

#ifdef __WIN32
#	include <windows.h>
#endif
double x=0,y=0,th=0;
void goahead(){
	if(th==0){
		x+=0.1;
	}else if(th==M_PI/2){
		y+=0.1;
	}else if(th==-M_PI/2){
		y-=0.1;
	}else if(th==M_PI){
		x+=0.1;
	}
	Spur_stop_line_GL( x, y,th );//
	while( !Spur_over_line_GL( x , y, th ) )//0.5m進むまで
		usleep( 100000 );//待機

}


void left(){
	if(th==0)
		th=M_PI/2;
	else if(th==M_PI/2)
		th=M_PI;
	else if(th==M_PI)
		th=-M_PI/2;
	else
		th=0;
	Spur_spin_GL(th);//左90度回転
	while( !Spur_near_ang_GL(th, M_PI / 18.0 ) )
		usleep( 100000 );//待機
}

void right(){
	if(th==0)
		th=-M_PI/2;
	else if(th==M_PI/2)
		th=0;
	else if(th==M_PI)
		th=M_PI/2;
	else
		th=M_PI;

	Spur_spin_GL(th);//左90度回転
	while( !Spur_near_ang_GL(th, M_PI / 18.0 ) )
		usleep( 100000 );//待機
}


int main(int argc, char *argv[]){

	 Spur_init();//初期化
	int i=0; 
	//ロボットの制御パラメータの設定
	Spur_set_vel(0.2);//速度設定
	Spur_set_accel(1.0);//加速度設定
	Spur_set_angvel(M_PI/2.0);//角速度設定
	Spur_set_angaccel(M_PI/2.0);//角加速度設定

	Spur_set_pos_GL(0,0,0);//座標系の設定(x,y,th)

	//Spur_stop_line_GL( 0.5, 0, 0 );//
	//while( !Spur_over_line_GL( 0.5 , 0.0, 0.0 ) )//0.5m進むまで
	//	usleep( 100000 );//待機
	
	puts("start");
	Spur_circle_GL(1,0,0.5);
	while( !Spur_over_line_GL( 1.5 , 0.0, M_PI/2 ) )//0.5m進むまで
		usleep( 100000 );//待機
	puts("1");
	while( !Spur_over_line_GL( 0.5 , 0.0, -M_PI/2 ) )//0.5m進むまで
		usleep( 100000 );//待機
	puts("2");
	while( !Spur_over_line_GL( 1.5 , 0.0, M_PI/2 ) )//0.5m進むまで
		usleep( 100000 );//待機
	puts("3");

	Spur_spin_GL(0);//左90度回転
	while( !Spur_near_ang_GL(0, M_PI / 18.0 ) )
		usleep( 100000 );//待機	

	Spur_stop_line_GL( 2.5, 0, 0 );//直進
	while( !Spur_over_line_GL( 2.5, 0, 0 ) )
		usleep( 100000 );

	Spur_circle_GL(3,0,-0.5);
	while( !Spur_over_line_GL( 3.5 , 0.0, -M_PI/2 ) )//0.5m進むまで
		usleep( 100000 );//待機
	puts("4");
	while( !Spur_over_line_GL( 2.5 , 0.0, M_PI/2 ) )//0.5m進むまで
		usleep( 100000 );//待機
	puts("5");

	Spur_spin_GL(M_PI);//左90度回転
	while( !Spur_near_ang_GL(M_PI, M_PI / 18.0 ) )
		usleep( 100000 );//待機	

	Spur_stop_line_GL( 2.0, 0, M_PI );//直進
	while( !Spur_over_line_GL( 2.0, 0, M_PI ) )
		usleep( 100000 );

	Spur_spin_GL(M_PI/2);//左90度回転
	while( !Spur_near_ang_GL(M_PI/2, M_PI / 18.0 ) )
		usleep( 100000 );//待機

	Spur_stop_line_GL( 2.0, 2.5, M_PI/2 );//直進
	while( !Spur_over_line_GL( 2.0, 2.5, M_PI/2 ) )
		usleep( 100000 );

	Spur_spin_GL(M_PI);//左90度回転
	while( !Spur_near_ang_GL(M_PI, M_PI / 18.0 ) )
		usleep( 100000 );//待機	

	Spur_stop_line_GL( -0.3, 2.5, M_PI );//直進
	while( !Spur_over_line_GL( -0.3, 2.5, M_PI ) )
		usleep( 100000 );

	Spur_spin_GL(0);//左90度回転
	while( !Spur_near_ang_GL(0, M_PI / 18.0 ) )
		usleep( 100000 );//待機	
/*
	x=2.5;y=0.0;th=M_PI/2;
	
	left();
	for(i=0;i<5;i++)
		goahead();
	right();

	for(i=0;i<25;i++)
		goahead();
*/
	Spur_stop();

	return 0;
}
