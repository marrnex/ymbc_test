#include <cstdio>
#include <cmath>

#include <unistd.h>
#include <signal.h>
#include <scip2awd.h>

int gIsShuttingDown;

void ctrl_c(int aStatus) {
  /* exit loop */
  gIsShuttingDown = 1;
  signal(SIGINT, NULL);
}

int main(int argc, char **argv) {
  /* variables */
  int ret;
  S2Port   *urg_port;   /* port */
  S2Sdd_t   urg_buff;   /* buffer */
  S2Scan_t *urg_data;   /* pointer to buffer */
  S2Param_t urg_param;  /* parameter */

  /* set Ctrl-c function */
  signal(SIGINT, ctrl_c);

  /* check argument */
  if( argc < 2 ) {
    fprintf(stderr, "ERORR: missing device operand\n");
    fprintf(stderr, "USAGE: %s <device>\n", argv[0]);
    fprintf(stderr, " e.g.: %s /dev/ttyACM0\n", argv[0]);
    return 1;
  }

  /* open port */
  urg_port = Scip2_Open(argv[1], B115200);
  if(urg_port == 0) {
    fprintf(stderr, "ERORR: cannot open %s\n", argv[1]);
    return 1;
  }
  fprintf(stdout, "port opened\n");

  /* init buffer */
  S2Sdd_Init(&urg_buff);

  /* get paramerter */
  Scip2CMD_PP(urg_port, &urg_param);

  /* scan start */
  // Scip2CMD_StartMS(urg_port, 44, 725, 1, 0, 0, &urg_buff, SCIP2_ENC_3BYTE);
  ret = Scip2CMD_StartMS(urg_port, urg_param.step_min, urg_param.step_max, 1, 0, 0, &urg_buff, SCIP2_ENC_3BYTE );
  if(ret == 0) {
    fprintf(stderr, "ERROR: Scip2CMD_StartMS\n");
    return 1;
  }
  fprintf(stdout, "scan started\n");

  /* open pipe to gnuplot */
  FILE *fp = popen("gnuplot -noraise", "w");
  if(fp == NULL) {
    fprintf(stderr, "ERROR: popen\n");
    return 1;
  }
  fprintf(stdout, "pipe opened\n");

  fputs("set terminal x11\n", fp);   /* drawing destination */
  fputs("set grid\n", fp);  /* draw grid */
  fputs("set mouse\n", fp);  /* use mouse */
  fputs("set xlabel \"y [m]\"\n", fp);  /* label of x-axis */
  fputs("set ylabel \"x [m]\"\n", fp);  /* label of y-axis */
  fputs("set xrange [-6:6] reverse\n", fp);  /* range of x-axis */
  fputs("set yrange [-6:6]\n", fp);  /* range of y-axis */
  fputs("set size ratio -1\n", fp);  /* aspect ratio */
  fputs("unset key\n", fp);  /* hide graph legends */

  int count=0;
  int i;
  double xob[500],yob[500];
  double a = 0, b = 0;
  double sum_xy = 0, sum_x = 0, sum_y = 0, sum_x2 = 0;
  int xmin=0,ymin=0,ymax=0;
  for(i=0;i<500;i++){
    xob[i]=0.0;
    yob[i]=0.0;
  }
  /* main loop */
  gIsShuttingDown = 0;
  while(!gIsShuttingDown) {
  //    int i;
    double x, y, rad;
  for(i=0;i<500;i++){
    xob[i]=0.0;
    yob[i]=0.0;
  } 
  sum_x=0;
  sum_x2=0;
  sum_xy=0;
  sum_y=0;
    /* lock buffer */
    ret = S2Sdd_Begin(&urg_buff, &urg_data);
    count=0;
    if(ret > 0) {
      fputs("plot '-'\n", fp);
      for(i = 0; i < urg_data->size; ++i) {
        if(urg_data->data[i] < 20) { /* error code */
          continue;
        }
        if((i>231)&&(i<447)&&(urg_data->data[i]<1500)){//前方障害物
      	 rad = (2 * M_PI / urg_param.step_resolution) * (i + urg_param.step_min - urg_param.step_front);
          x = urg_data->data[i] * cos(rad) / 1000.0;
          y = urg_data->data[i] * sin(-rad) / 1000.0;
         xob[count] = urg_data->data[i] * cos(rad) / 1000.0;
          yob[count] = urg_data->data[i] * sin(-rad) / 1000.0;
         //fprintf(fp, "%f %f\n", xob[count], yob[count]);
          //fprintf(fp, "%f %f\n", y, x);
         count++;
      }
      }
      int xmin2=0;
      xmin=count-1;
      ymin=0;
      ymax=0;
      for(i=count-1;i>(count/2);i--){
        if(xob[xmin]>xob[i]&&xob[i]!=0.0)
          xmin=i;
      }
      for(i=0;i<((count/2)-1);i++){
        if(xob[xmin2]>xob[i])
          xmin2=i;
      }
      printf("xmin: x:%f y:%f\n",xob[count],yob[count]);
      printf("xmin2: x:%f y:%f\n",xob[xmin2],yob[xmin2]);
      /*
      for(i=1;i<count;i++){
        if(xob[xmin]>xob[i])
          xmin=i;
        if(yob[ymin]>yob[i])
          ymin=i;
        if(yob[ymax]<yob[i])
          ymax=i;
      }*/
      if(xob[ymin]>xob[ymax]){
        //fprintf(fp, "%f %f\n", yob[ymax], xob[ymax]);
        a=(yob[ymax]-yob[xmin])/(xob[ymax]-xob[xmin]);
        b=-xob[xmin]*(yob[ymax]-yob[xmin])+yob[xmin];
        printf("y:%f x:%f\n",yob[ymax], xob[ymax]);
      }else{
        //fprintf(fp, "%f %f\n", yob[ymin], xob[ymin]);
        a=(yob[ymin]-yob[xmin])/(xob[ymin]-xob[xmin]);
        b=-xob[xmin]*(yob[ymin]-yob[xmin])+yob[xmin];
        printf("y:%f x:%f\n",yob[ymin], xob[ymin]);
      }
        //fprintf(fp, "%f %f\n", yob[xmin], xob[xmin]);
        printf("y:%f x:%f\n",yob[xmin], xob[ymin]);
        puts("");


      for (i=0; i<count; i++) {
        sum_xy += yob[i] * xob[i];
        sum_x += yob[i];
        sum_y += xob[i];
        sum_x2 += pow(yob[i], 2);
       // fprintf(fp, "%f %f\n", yob[i], xob[i]);
      }
      
      a = (count * sum_xy - sum_x * sum_y) / (count * sum_x2 - pow(sum_x, 2));
      b = (sum_x2 * sum_y - sum_xy * sum_x) / (count * sum_x2 - pow(sum_x, 2));
      if(a<0.2&&a>-0.2)
        puts("center");
    else if(a<=0.2){
          puts("left");
          //fprintf(fp, "%f %f\n", yob[ymin], xob[ymin]);
        }
    else { 
          puts("right");
          //fprintf(fp, "%f %f\n", yob[ymax], xob[ymax]);
        }
  //    fprintf(fp, "%f %f\n",yob[xmin],xob[xmin] );
      printf("cou:%d ",count);
      //for(i=0;i<count;i++)
        //printf("x:%f y:%f \n",xob[i],yob[i]);
      puts("");
      printf("a:%f b:%f \n",a,b);

      for(i=-100;i<100;i++){
        /*
        if(a<0.2&&a>-0.08){
          fprintf(fp, "%f %f\n", yob[ymin], xob[ymin]);
          fprintf(fp, "%f %f\n", yob[ymax], xob[ymax]);
        }
        else if(a>=0.2){//left
          fprintf(fp, "%f %f\n", yob[ymin], xob[ymin]);
          fprintf(fp, "%f %f\n", yob[xmin], xob[xmin]);
        }else if(a<=-0.08){
          fprintf(fp, "%f %f\n", yob[ymax], xob[ymax]);
          fprintf(fp, "%f %f\n", yob[xmin], xob[xmin]);
        
        }
        */
        fprintf(fp, "%f %f\n", yob[xmin2], xob[xmin2]);
          fprintf(fp, "%f %f\n", yob[xmin], xob[xmin]);
         // fprintf(fp, "%f %f\n", yob[ymax], xob[ymax]);
          //fprintf(fp, "%f %f\n", yob[ymin], xob[ymin]);
          //fprintf(fp, "%f %f\n", yob[xmin], xob[xmin]);
        //fprintf(fp, "%f %f\n", i*0.1, a*(i*0.1)+b);
      }
      fputs("e\n", fp);
      printf("atan:%f[rad] %f[deg] \n",atan(b/(-b/a)),atan(b/(-b/a))*180/3.1415);
      /* unlock buffer */
      S2Sdd_End(&urg_buff);

      /* wait 90 ms (URG-04LX 1scan = 100 ms) */
      usleep(90000);
    } else if(ret == -1) {
      fprintf(stderr, "ERROR: S2Sdd_Begin\n");
      break;
    } else {
      usleep(100);
    }
  }

  /* close pipe */
  pclose(fp);
  fprintf(stdout, "pipe closed\n");

  /* scan stop */
  ret = Scip2CMD_StopMS(urg_port, &urg_buff);
  if(ret == 0) {
    fprintf(stderr, "ERROR: Scip2CMD_StopMS\n");
    return 1;
  }
  fprintf(stdout, "scan stopped\n");

  /* destruct buffer */
  S2Sdd_Dest(&urg_buff);

  /* close port */
  Scip2_Close(urg_port);
  fprintf(stdout, "port closed\n");
}
