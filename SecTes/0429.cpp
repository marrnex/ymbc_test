//urg_param.step_resolution=1024
//this program output '0' if exitst obstacle in front otherwise '1' .
//g++ 0428.cpp -lscip2awd -lpthread -lypspur

#include <cstdio>
#include <cmath>

#include <unistd.h>
#include <signal.h>
#include <scip2awd.h>
#include <ypspur.h>

int gIsShuttingDown;

void ctrl_c(int aStatus) {
  /* exit loop */
  gIsShuttingDown = 1;
  signal(SIGINT, NULL);
}

int main(int argc, char **argv) {
  /* variables */
  int ret;
  int str=0;
  int obs=0,obsr=0,obsl=0,obslf=0,obslb=0,obsrf=0,obsrb=0, obsld=0;
  double *xp,*yp,*thp;
  S2Port   *urg_port;   /* port デバイスの識別子*/
  S2Sdd_t   urg_buff;   /* bufferデータを保存 */
  S2Scan_t *urg_data;   /* pointer to bufferバッファへのアクセス用 */
  S2Param_t urg_param;  /* parameter*/

   Spur_init();//初期化

  //ロボットの制御パラメータの設定
  Spur_set_vel(0.2);//速度設定
  Spur_set_accel(1.0);//加速度設定
  Spur_set_angvel(M_PI/2.0);//角速度設定
  Spur_set_angaccel(M_PI/2.0);//角加速度設定

  Spur_set_pos_GL(0,0,0);//座標系の設定(x,y,th)



  /* set Ctrl-c function */
  signal(SIGINT, ctrl_c);


  /* check argument */
/*
  if( argc < 2 ) {
    fprintf(stderr, "ERORR: missing device operand\n");
    fprintf(stderr, "USAGE: %s <device>\n", argv[0]);
    fprintf(stderr, " e.g.: %s /dev/ttyACM0\n", argv[0]);
    return 1;
  }
*/
  /* open port */
  urg_port = Scip2_Open(argv[1], B115200);
 /*
  if(urg_port == 0) {
    fprintf(stderr, "ERORR: cannot open %s\n", argv[1]);
    return 1;
  }
  fprintf(stdout, "port opened\n");
*/
  /* init buffer */
  S2Sdd_Init(&urg_buff);

  /* get paramerter */
  Scip2CMD_PP(urg_port, &urg_param);//URGのパラメータの取得

  /* scan start */
  // Scip2CMD_StartMS(urg_port, 44, 725, 1, 0, 0, &urg_buff, SCIP2_ENC_3BYTE);
  ret = Scip2CMD_StartMS(urg_port, urg_param.step_min, urg_param.step_max, 1, 0, 0, &urg_buff, SCIP2_ENC_3BYTE );
  /*
  if(ret == 0) {
    fprintf(stderr, "ERROR: Scip2CMD_StartMS\n");
    return 1;
  }
  fprintf(stdout, "scan started\n");
*/
  /* open pipe to gnuplot */
  /*
  FILE *fp = popen("gnuplot -noraise", "w");
  if(fp == NULL) {
    fprintf(stderr, "ERROR: popen\n");
    return 1;
  }
  fprintf(stdout, "pipe opened\n");
*/
  //fputs("set terminal x11\n", fp);   /* drawing destination */
  //fputs("set grid\n", fp);  /* draw grid */
  //fputs("set mouse\n", fp);  /* use mouse */
  //fputs("set xlabel \"y [m]\"\n", fp);  /* label of x-axis */
  //fputs("set ylabel \"x [m]\"\n", fp);  /* label of y-axis */
  //fputs("set xrange [-6:6] reverse\n", fp);  /* range of x-axis */
  //fputs("set yrange [-6:6]\n", fp);  /* range of y-axis */
  //fputs("set size ratio -1\n", fp);  /* aspect ratio */
  //fputs("unset key\n", fp);  /* hide graph legends */

  /* main loop */
  gIsShuttingDown = 0;
  int step=0;
  double x=0.0, y=0.0, rad,th=0.0;
  while(!gIsShuttingDown) {//多分ここで描画とかが起きてる
    int i;
    
    obs=0;obsr=0;obsl=0,obsrf=0,obsrb=0,obslf=0,obslb=0;
    obsld=0;
    
    /* lock buffer */
    ret = S2Sdd_Begin(&urg_buff, &urg_data);//データ拾ってくるretが0以上でデータ取得成功

    if(ret > 0) {//成功
    //  fputs("plot '-'\n", fp);
      for(i = 0; i < urg_data->size; ++i) {//取得できたデータの個数だけループ
        if(urg_data->data[i] < 20) { /* error code */
          continue;
        }
        if((i>281)&&(i<427)&&(urg_data->data[i]<600))//前方障害物
          obs=1;
        if((i>631)&&(i<649)&&(urg_data->data[i]<600))//左障害物
          obsl=1;
        if((i>119)&&(i<137)&&(urg_data->data[i]<600))//右障害物
          obsr=1;
        if((i>205)&&(i<223)&&(urg_data->data[i]<600))//右前障害物
          obsrf=1;
        if((i>0)&&(i<53)&&(urg_data->data[i]<600))//右後ろ障害物
          obsrb=1;
        if((i>545)&&(i<563)&&(urg_data->data[i]<600))//左前障害物
          obslf=1;
        if((i>710)&&(i<725)&&(urg_data->data[i]<600))//左後ろ障害物
          obslb=1;
        if((i>631)&&(i<649)&&(urg_data->data[i]<600))//左障害物
          obsld=1;
      	//rad = (2 * M_PI / urg_param.step_resolution) * (i + urg_param.step_min - urg_param.step_front);
        //x = urg_data->data[i] * cos(rad) / 1000.0;
      //  y = urg_data->data[i] * sin(rad) / 1000.0;
        //fprintf(fp, "%f %f\n", y, x);//ファイルに書いてgnuplotで描画？
      }/*
      if(obs<40)
        obs=0;
      else
        obs=1;
      
      if(obsl<9)
        obsl=0;
      else
        obsl=1;

      if(obsr<9)
        obsr=0;
      else
        obsr=1;

      if(obsrb<4)
        obsrb=0;
      else
        obsrb=1;
      
      if(obsrf<4)
        obsrf=0;
      else
        obsrf=1;

      if(obslb<4)
        obslb=0;
      else
        obslb=1;

      if(obslf<4)
        obslf=0;
      else
        obslf=1;      
      */
printf("step:%d front:%d left:%d right:%d  lf:%d lb:%d rf:%d rb:%d\n",step,obs,obsl,obsr,obslf,obslb,obsrf,obsrb);


      switch(step){
        case 0: Spur_stop_line_LC( x, y, 0 ); 
                if(obs==0){
                  x+=0.2;
                  //usleep( 100000 );//待機
                }
                else{
                  step++;x=0;y=0;
                //  Spur_stop();
                }
                break;//直進
        case 1: Spur_spin_LC(th);//右90度回転
                if(obsl==0){
                  th+=(-M_PI/50);
                  //usleep( 100000 );//待機
                }else{th=0;
                step++;}
                break;//方向転換

        case 2: if(obsrf!=1){
                  Spur_get_pos_LC(&x,&y,&th);
                  Spur_orient_LC(th);
        }else{step++;}break;
        case 3://Spur_stop_line_LC( x, y, 0 );
              /*
               if(obsld==1){
                  Spur_spin_LC(th);
                  th+=(-M_PI/50);
               }else if(obsl==1){
                  Spur_stop_line_LC( x, y, 0 );
                  x+=0.2;
               }else if(obsl==0){
                  Spur_spin_LC(th);
                  th+=(M_PI/50);
               }if(obsl==1&&obsr==0&&obsrb==1){
                step++;th=0;x=0;
               }
               */
              if(obsr==1)
                str=1;
              if(obs==1){
                if(obsl==1||obslf==1){
                  Spur_spin_LC(th);
                  th+=(-M_PI/50);
                  puts("left");
                }else if(obsr==1||obsrf==1){    
                  Spur_spin_LC(th);
                  th+=(M_PI/50);
                  puts("right");
              }
              }else if(obs==0){
                  //Spur_stop_line_LC( x, y, 0 );
                  Spur_get_pos_LC(&x,&y,&th);
                  Spur_orient_LC(th);
                  puts("go");
                  //x+=0.2;
                  th=0;
              }if(str==1&&(obsl==1)&&obsr==0&&obsrb==0&&obsrf==0){
                step++;th=0;x=0;
               }
         break;//
        case 4: Spur_stop();break;
      }
//      fputs("e\n", fp);

      /* unlock buffer */
      S2Sdd_End(&urg_buff);

      /* wait 90 ms (URG-04LX 1scan = 100 ms) */
      //usleep(90000);
      /*
      Spur_stop_line_GL( 1.5, 0, 0 );//
      if(obs==0){
        //while( !Spur_over_line_GL( 1.5 - 0.005, 0.0, 0.0 ) )//0.5m進むまで
        usleep( 100000 );//待機
      }else{
        Spur_stop();
      }
      */
      //printf("step:%d front:%d left:%d right:%d  lf:%d lb:%d rf:%d rb:%d\n",step,obs,obsl,obsr,obslf,obslb,obsrf,obsrb);





    } else if(ret == -1) {//データ取得失敗
  //    fprintf(stderr, "ERROR: S2Sdd_Begin\n");
      break;
    } else {
      usleep(100);
    }
  }

  /* close pipe */
  //pclose(fp);
  //fprintf(stdout, "pipe closed\n");

  /* scan stop */
  ret = Scip2CMD_StopMS(urg_port, &urg_buff);
  if(ret == 0) {
    //fprintf(stderr, "ERROR: Scip2CMD_StopMS\n");
    return 1;
  }
  //fprintf(stdout, "scan stopped\n");

  /* destruct buffer */
  S2Sdd_Dest(&urg_buff);

  /* close port */
  Scip2_Close(urg_port);
  //fprintf(stdout, "port closed\n");
}
