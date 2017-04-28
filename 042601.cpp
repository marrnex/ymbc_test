#include <cstdio>
#include <cmath>

#include <unistd.h>
#include <signal.h>
#include <scip2awd.h>
//#include <ypspur.h>

int gIsShuttingDown;

void ctrl_c(int aStatus) {
  /* exit loop */
  gIsShuttingDown = 1;
  signal(SIGINT, NULL);
}

int main(int argc, char **argv) {
  /* variables */
  int ret;

  int obs=0;

  S2Port   *urg_port;   /* port デバイスの識別子*/
  S2Sdd_t   urg_buff;   /* bufferデータを保存 */
  S2Scan_t *urg_data;   /* pointer to bufferバッファへのアクセス用 */
  S2Param_t urg_param;  /* parameter*/

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
  Scip2CMD_PP(urg_port, &urg_param);//URGのパラメータの取得

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

  /* main loop */
  gIsShuttingDown = 0;
  while(!gIsShuttingDown) {//多分ここで描画とかが起きてる
    int i;
    double x, y, rad;
    obs=0;
    /* lock buffer */
    ret = S2Sdd_Begin(&urg_buff, &urg_data);//データ拾ってくるretが0以上でデータ取得成功

    if(ret > 0) {//成功
      fputs("plot '-'\n", fp);
      for(i = 0; i < urg_data->size; ++i) {//取得できたデータの個数だけループ
        if(urg_data->data[i] < 20) { /* error code */
          continue;
        }
        if(urg_data->data[i]<300)
          obs=1;
      	rad = (2 * M_PI / urg_param.step_resolution) * (i + urg_param.step_min - urg_param.step_front);
        x = urg_data->data[i] * cos(rad) / 1000.0;
        y = urg_data->data[i] * sin(rad) / 1000.0;
        fprintf(fp, "%f %f\n", y, x);//ファイルに書いてgnuplotで描画？
      }
      fputs("e\n", fp);

      /* unlock buffer */
      S2Sdd_End(&urg_buff);

      /* wait 90 ms (URG-04LX 1scan = 100 ms) */
      //usleep(90000);
     // Spur_stop_line_GL( 0.5, 0, 0 );//
      //while( !Spur_over_line_GL( 0.5 - 0.005, 0.0, 0.0 ) )//0.5m進むまで
        usleep( 200000 );//待機
      //if(obs!=0)
        //Spur_stop();
        printf("%d\n",obs);

    } else if(ret == -1) {//データ取得失敗
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
