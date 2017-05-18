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
  int flag=0;
  int obs=0,obsr=0,obsl=0,obslf=0,obslb=0,obsrf=0,obsrb=0, obsld=0,lobslf=0,lobsrf=0;
  double maxobs=0;//前方に存在する中

  S2Port   *urg_port;   /* port デバイスの識別子*/
  S2Sdd_t   urg_buff;   /* bufferデータを保存 */
  S2Scan_t *urg_data;   /* pointer to bufferバッファへのアクセス用 */
  S2Param_t urg_param;  /* parameter*/

   Spur_init();//初期化

  //ロボットの制御パラメータの設定
  Spur_set_vel(0.3);//速度設定
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
  int leftob=0,rightob=0;
  double x=0.0, y=0.0,xp=0.0,yp=0.0,rad,th=0.0;
  double xob[500],yob[500];
  double a = 0, b = 0;
  int i;
  int count=0;
  int xmin,ymin,ymax,ysel;
  int flag1=0;
  double sum_x,sum_x2,sum_xy,sum_y;
  for(i=0;i<500;i++){
    xob[i]=0.0;
    yob[i]=0.0;
  }
  while(!gIsShuttingDown) {//多分ここで描画とかが起きてる
    
    for(i=0;i<500;i++){
    xob[i]=0.0;
    yob[i]=0.0;
  }
  sum_x=0;
  sum_x2=0;
  sum_xy=0;
  sum_y=0;
    
    obs=0;obsr=0;obsl=0,obsrf=0,obsrb=0,obslf=0,obslb=0;
    obsld=0;leftob=0;rightob=0;lobsrf=0;lobslf=0;
    count=0;
    /* lock buffer */
    ret = S2Sdd_Begin(&urg_buff, &urg_data);//データ拾ってくるretが0以上でデータ取得成功

    if(ret > 0) {//成功
    //  fputs("plot '-'\n", fp);
      maxobs=0.0;
      for(i = 0; i < urg_data->size; ++i) {//取得できたデータの個数だけループ
        if(urg_data->data[i] < 20) { /* error code */
          continue;
        }
        if((i>281)&&(i<427)&&(urg_data->data[i]<550))//前方障害物
          obs=1;
        if((i>631)&&(i<649)&&(urg_data->data[i]<600))//左障害物
          obsl=1;
        if((i>119)&&(i<137)&&(urg_data->data[i]<600))//右障害物
          obsr=1;
        if((i>205)&&(i<223)&&(urg_data->data[i]<600))//右前障害物
          obsrf=1;
        if((i>205)&&(i<223)&&(urg_data->data[i]<1000))//右前障害物
          lobsrf=1;
        if((i>0)&&(i<53)&&(urg_data->data[i]<600))//右後ろ障害物
          obsrb=1;
        if((i>545)&&(i<563)&&(urg_data->data[i]<600))//左前障害物
          obslf=1;
        if((i>545-50)&&(i<563-50)&&(urg_data->data[i]<1000))//左前障害物
          lobslf=1;
        if((i>710)&&(i<725)&&(urg_data->data[i]<600))//左後ろ障害物
          obslb=1;
        if((i>631)&&(i<649)&&(urg_data->data[i]<600))//左障害物
          obsld=1;
        //if(flag==0){
          if((i>281)&&(i<427)&&(urg_data->data[i]<3000)){//前方障害物
              if(maxobs<=urg_data->data[i]){//前方の中で最も距離のある物体を導出

         obslf=1;                maxobs=urg_data->data[i];
                rad = (2 * M_PI / urg_param.step_resolution) * (i + urg_param.step_min - urg_param.step_front);
                xp = urg_data->data[i] * cos(rad)/1000.0;
                yp = urg_data->data[i] * sin(rad) / 1000.0;
              }
          }
        //}
        if((i>417)&&(i<427)&&(urg_data->data[i]<1000))//左前障害物
          leftob+=1;
        if((i>281)&&(i<291)&&(urg_data->data[i]<1000))//右前障害物
          rightob+=1;
        if((i>281)&&(i<427)&&(urg_data->data[i]<1500)){//前方障害物
         rad = (2 * M_PI / urg_param.step_resolution) * (i + urg_param.step_min - urg_param.step_front);
          x = urg_data->data[i] * cos(rad) / 1000.0;
          y = urg_data->data[i] * sin(-rad) / 1000.0;
         xob[count] = urg_data->data[i] * cos(rad) / 1000.0;
          yob[count] = urg_data->data[i] * sin(-rad) / 1000.0;
         //fprintf(fp, "%f %f\n", xob[count], yob[count]);
          //fprintf(fp, "%f %f\n", y, x);
         count++;
}

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
      xmin=0;
      ymin=0;
      ymax=0;
      
      if(flag==0){
      for(i=1;i<count;i++){
        if(xob[xmin]>xob[i])
          xmin=i;
        if(yob[ymin]>yob[i])
          ymin=i;
        if(yob[ymax]<yob[i])
          ymax=i;
      }/*
      if(xob[ymin]>xob[ymax]){
        //fprintf(fp, "%f %f\n", yob[ymax], xob[ymax]);
        a=(yob[ymax]-yob[xmin])/(xob[ymax]-xob[xmin]);
        b=-xob[xmin]*(yob[ymax]-yob[xmin])+yob[xmin];
        //ysel=ymax;
        printf("y:%f x:%f\n",yob[ymax], xob[ymax]);
      }else{
        //fprintf(fp, "%f %f\n", yob[ymin], xob[ymin]);
        a=(yob[ymin]-yob[xmin])/(xob[ymin]-xob[xmin]);
        b=-xob[xmin]*(yob[ymin]-yob[xmin])+yob[xmin];
        //ysel=ymin;
        printf("y:%f x:%f\n",yob[ymin], xob[ymin]);
      }*/
        //fprintf(fp, "%f %f\n", yob[xmin], xob[xmin]);
        printf("y:%f x:%f\n",yob[xmin], xob[ymin]);
        puts("");
      }

 
      switch(step){
        case 0: /*
                Spur_stop_line_LC( x, y, 0 ); 
                if(obs==0){
                  x+=0.2;
                  //usleep( 100000 );//待機
                }
                else{
                  step++;x=0;y=0;
                //  Spur_stop();
                }
                break;//直進
*/


                Spur_get_pos_GL(&x,&y,&th);
                //printf("x:%f y:%f th:%f \n",x,y,th);
                Spur_stop_line_GL( xp-0.45, y, th );
                //usleep(80000000);
                
                Spur_orient_GL(th);
                while( !Spur_over_line_GL( xp-0.4, y, th ) )
                    usleep( 100000 );
                
                //Spur_stop();
                //printf("%f\n",x);
                  step++;
                break;

        case 1: if(flag1==0){
                for (i=0; i<count; i++) {
                  sum_xy += yob[i] * xob[i];
                  sum_x += yob[i];
                  sum_y += xob[i];
                  sum_x2 += pow(yob[i], 2);
                } 
      
                a = (count * sum_xy - sum_x * sum_y) / (count * sum_x2 - pow(sum_x, 2));
                b = (sum_x2 * sum_y - sum_xy * sum_x) / (count * sum_x2 - pow(sum_x, 2));
                printf("atan:%f[rad] %f[deg] \n",atan(b/(-b/a)),atan(b/(-b/a))*180/3.1415);
                flag1=1;
                }
                //Spur_stop();
        //Spur_spin_LC(th);//右90度回転
                Spur_get_pos_LC(&x,&y,&th);
                Spur_spin_LC(-atan(b/(-b/a))+(-M_PI/2));
                /*if(obsl==0){
                  th+=(-M_PI/50);
                  //usleep( 100000 );//待機
                }else{th=0;
                step++;}*/
                usleep(4000000);
                //Spur_stop();
                step++;
                flag1=0;
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
              }if(obsr==0&&obsrb==0&&obsrf==0){
                step++;th=0;x=0;
               }
         break;//
         case 4:
         Spur_stop_line_FS( 0.2, 0, 0 );
         usleep(2000000);
         Spur_get_pos_LC(&x,&y,&th);
          Spur_spin_LC(th-M_PI/2);
                  usleep(4000000);
                  step++;
         break;
        case 5: //flag=1;
                //printf("%f\n",xp);
                //Spur_get_pos_LC(&x,&y,&th);
                //printf("x:%f y:%f th:%f \n",x,y,th);
                Spur_stop_line_FS( xp-0.4, 0, 0 );
                printf("xp:%f\n",xp);
                //usleep(80000000);
                
                //Spur_orient_LC(th);
                //while( !Spur_over_line_LC( xp-0.4, y, th ) )
                    usleep( 8000000 );
                
                //Spur_stop();
                //printf("%f\n",x);
                  step++;
                break;
        case 6:
                //Spur_spin_LC(-M_PI/2);//右90度回転
                /*
                if(!(obslf==1&&obsrf==1)){
                  th+=(-M_PI/50);
                  usleep( 100000 );//待機
                }else{th=0;
                step++;}
                */Spur_get_pos_LC(&x,&y,&th);
          Spur_spin_LC(th-M_PI/2);
                  usleep(4000000);
                usleep(4000000);
                step++;
                break; 
        case 7:

                Spur_stop_line_FS( 0.8, 0, 0 );


                //Spur_get_pos_LC(&x,&y,&th);
                //printf("x:%f y:%f th:%f \n",x,y,th);
                //Spur_stop_line_GL( x+0.5, y, th );
                //usleep(80000000);
                
                //Spur_orient_LC(th);
                //while( !Spur_over_line_LC( x+0.5, y, th ) )
                    usleep( 2000000 );
                //  step++;

                  th=0;
                  //Spur_stop();
                  step++;
                  break;
        case 8:
        /*if(!((yob[xmin]==yob[ymax]&&xob[xmin]==xob[ymax])||(yob[xmin]==yob[ymin]&&xob[xmin]==xob[ymin]))){
                flag=1;
                Spur_get_pos_LC(&x,&y,&th);
                if(xob[ymin]>xob[ymax]){
                Spur_spin_LC(th+atan((yob[xmin]-yob[ymax])/(xob[xmin]-xob[ymax])));
              }else{
                Spur_spin_LC(th+atan((yob[xmin]-yob[ymin])/(xob[xmin]-xob[ymin])));
              }//while(Spur_near_ang_GL(th+atan((yob[xmin]-yob[ymax])/(xob[xmin]-xob[ymax])),M_PI/64)||Spur_near_ang_GL(th+atan((yob[xmin]-yob[ymin])/(xob[xmin]-xob[ymin])),M_PI/64)){
               // puts("spin");
                 // usleep(100000);}
                usleep(4000000);
                  step++;
                }
                */
        if(!((yob[xmin]==yob[ymax]&&xob[xmin]==xob[ymax])||(yob[xmin]==yob[ymin]&&xob[xmin]==xob[ymin]))){
                if(flag1==0){
                //printf("atan:%f[rad] %f[deg] \n",atan(b/(-b/a)),atan(b/(-b/a))*180/3.1415);
                flag1=1;
                     for (i=0; i<count; i++) {
        sum_xy += yob[i] * xob[i];
        sum_x += yob[i];
        sum_y += xob[i];
        sum_x2 += pow(yob[i], 2);
      }
      
      a = (count * sum_xy - sum_x * sum_y) / (count * sum_x2 - pow(sum_x, 2));
      b = (sum_x2 * sum_y - sum_xy * sum_x) / (count * sum_x2 - pow(sum_x, 2));

                if(a<0){
                  ysel=ymax;
                 // puts("left");
                }else{
                  ysel=ymin;
                 // puts("right");
                }

                }

                a=(yob[ysel]-yob[xmin])/(xob[ysel]-xob[xmin]);
                b=(yob[ysel]-yob[xmin])/(xob[ysel]-xob[xmin])*xob[xmin]+yob[ysel];
                printf("a:%f\n",a);
                if(a<0.3&&a>-0.3){
                  Spur_stop_line_FS( ((xob[xmin]+xob[ysel])/2), ( (yob[xmin]+yob[ysel])/2), 0 );
                }
                if(a<0){
                    Spur_stop_line_FS( ((xob[xmin]+xob[ysel])/2)-0.2, ( (yob[xmin]+yob[ysel])/2)-0.3, 0 );
                    puts("-");
                }
                else{
                    Spur_stop_line_FS( ((xob[xmin]+xob[ysel])/2)-0.2, ( (yob[xmin]+yob[ysel])/2)+0.3, 0 );
                    puts("+");
                }

        
                flag=1;
                //Spur_get_pos_LC(&x,&y,&th);
                //Spur_stop_line_LC((xob[xmin]+xob[ysel])/2,(yob[xmin]+yob[ysel])/2,th);
                //Spur_stop_line_FS( ((xob[xmin]+xob[ysel])/2)-0.4, (yob[xmin]+yob[ysel])/2, 0 );
                printf("xmin: x:%f y:%f\n",xob[xmin],yob[xmin]);
                printf("ysel: x:%f y:%f\n",xob[ysel],yob[ysel]);

                printf("x:%f y:%f \n",(xob[xmin]+xob[ysel])/2,(yob[xmin]+yob[ysel])/2);
                usleep(8000000);
                  step++;
                }

                        break;

      case 9:
                Spur_get_pos_LC(&x,&y,&th);
             //   if(xob[ymin]>xob[ymax]){
                Spur_spin_LC(-atan(b/(-b/a))-M_PI/2);
              //}else{
                //Spur_spin_LC(th+atan((yob[xmin]-yob[ymin])/(xob[xmin]-xob[ymin])));
              //}//while(Spur_near_ang_GL(th+atan((yob[xmin]-yob[ymax])/(xob[xmin]-xob[ymax])),M_PI/64)||Spur_near_ang_GL(th+atan((yob[xmin]-yob[ymin])/(xob[xmin]-xob[ymin])),M_PI/64)){
               // puts("spin");
                 // usleep(100000);}
                usleep(4000000);
                  step++;
                
                
      break;

                  
        default:Spur_stop();
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
