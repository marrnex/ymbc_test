#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <ypspur.h>

#ifdef __WIN32
#	include <windows.h>
#endif
double x=0,y=0,th=0;

void goahead(){
	/*
	switch(th){
	case 0:	x+=0.1;
			break;
	case M_PI/2: y+=0.1;
	case -M_PI/2: y-=0.1;
	case M_PI:x-=0.1; break;

	}
	*/
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
void back(){
	if(th==0){
		x-=0.1;
	}else if(th==M_PI/2){
		y-=0.1;
	}else if(th==-M_PI/2){
		y+=0.1;
	}else if(th==M_PI){
		x-=0.1;
	}
	Spur_stop_line_GL( x, y,th );//
	while( !Spur_over_line_GL( x , y, th ) )//0.5m進むまで
		usleep( 100000 );//待機
}

void left(){
	th=+M_PI/2;
	Spur_spin_GL(th);//左90度回転
	while( !Spur_near_ang_GL(th, M_PI / 18.0 ) )
		usleep( 100000 );//待機
}

void right(){
	th=-M_PI/2;
	Spur_spin_GL(th);//左90度回転
	while( !Spur_near_ang_GL(th, M_PI / 18.0 ) )
		usleep( 100000 );//待機
}

int main(int argc, char *argv[]){
	int i;
	 Spur_init();//初期化

	//ロボットの制御パラメータの設定
	Spur_set_vel(0.2);//速度設定
	Spur_set_accel(1.0);//加速度設定
	Spur_set_angvel(M_PI/2.0);//角速度設定
	Spur_set_angaccel(M_PI/2.0);//角加速度設定

	Spur_set_pos_GL(0,0,0);//座標系の設定(x,y,th)

	//for(i=0;i<5;i++)
	//	goahead();
	//left();
	//goahead();
	//right();
	//for(i=0;i<5;i++)
	//	goahead();
	//left();
	//for(i=0;i<5;i++)
	//goah
		back();

	Spur_stop_line_GL( -0.1, 0,0 );//
	//while( !Spur_over_line_GL( 0 , 0, 0 ) )//0.5m進むまで
	//	usleep( 100000 );//待機


	Spur_stop();

	return 0;
}