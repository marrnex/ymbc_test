/*

　山彦セミナー(点群処理：Point Cloud Library)用　サンプルプログラム
　2013.4.3　MTM 作成

　---
　URGデータを読み込んでPCLのPointCloudに保存し、Viewerで表示するプログラムです。
　実行には
　・Point Cloud Library
　・scip2awd
　の2つがインストールされている必要があります。
　また、コンパイルにはcmakeが必要です。

*/


//--------------------------------------------
//include
//--------------------------------------------
#include<cstdio> //stdio.hのc++版
#include<cmath> //math.hのc++版
#include<csignal> //signal.hのc++版
//PCL
#include<pcl/point_types.h> //pcl::PointXYZクラスを使う
#include<pcl/point_cloud.h> //pcl::PointCloudクラスを使う
#include<pcl/visualization/cloud_viewer.h> //cloudViewerクラスを使う
//RPP
#include"scip2awd.h" //ロボ研のURGドライバ(NewPCでインストール済み)

#include <unistd.h>
#include <signal.h>
#include <scip2awd.h>
#include <ypspur.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//--------------------------------------------
//ctrl-cで停止させるための記述
//--------------------------------------------
bool gIsShuttingDown = false;

void ctrl_c( int aStatus )
{
    gIsShuttingDown = true;
    signal(SIGINT, NULL);
}

//--------------------------------------------
//本体
//--------------------------------------------

double perp(double x1,double y1,double x2,double y2,double x3,double y3){
  double a,b;
  a=-(x2-x1)/(y2-y1);
  b=(x2-x1)/(y2-y1)*x3+y3;

  return a;
}

double perp2(double x1,double y1,double x2,double y2,double x3,double y3){
  double a,b;
  a=-(x2-x1)/(y2-y1);
  b=(x2-x1)/(y2-y1)*x3+y3;

  return b;
}

int main(int argc, char **argv) {
  /* variables */
  int ret;
  int str=0;
  int flag=0;
  int obs=0,obsr=0,obsl=0,obslf=0,obslb=0,obsrf=0,obsrb=0, obsld=0,lobslf=0,lobsrf=0;
  double maxobs=0;//前方に存在する中
  int test1=0,test2=0;
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

////////////

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
   //Scip2CMD_StartMS(urg_port, 44, 725, 1, 0, 0, &urg_buff, SCIP2_ENC_3BYTE);
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



    //+++++++
    //PCL関係変数の宣言
    //+++++++
    //PointCloudの宣言
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    //CloudViewerの宣言
    //pcl::visualization::CloudViewer viewer("PCL URG Viewer"); //クラウド表示のためのクラスを定義
  pcl::visualization::CloudViewer viewer("cluster viewer");

  /* main loop */
  gIsShuttingDown = 0;
  int step=0;
  int leftob=0,rightob=0;
  double x=0.0, y=0.0,xp=0.0,yp=0.0,rad,th=0.0;
  double xob[500],yob[500];
  double a = 0, b = 0;
  double pole;
  int i;
  int count=0;
  int xmin,ymin,ymax,ysel;
  int flag1=0;
  int loopf=0;
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
      fputs("plot '-'\n", fp);
      maxobs=0.0;
      for(i = 0; i < urg_data->size; ++i) {//取得できたデータの個数だけループ
        cloud2->clear();  //ポイントクラウドの中身を全部クリアする
        if(urg_data->data[i] < 20) { /* error code */
          continue;
        }
        if((i>281)&&(i<427)&&(urg_data->data[i]<550))//前方障害物
          obs=1;
        if((i>631)&&(i<649)&&(urg_data->data[i]<600))//左障害物
          obsl=1;
        if((i>119)&&(i<137)&&(urg_data->data[i]<800))//右障害物
          obsr=1;
        if((i>205)&&(i<223)&&(urg_data->data[i]<800))//右前障害物
          obsrf=1;
        if((i>205)&&(i<223)&&(urg_data->data[i]<1000))//右前障害物
          lobsrf=1;
        if((i>0)&&(i<53)&&(urg_data->data[i]<800))//右後ろ障害物
          obsrb=1;
        if((i>545)&&(i<563)&&(urg_data->data[i]<600))//左前障害物
          obslf=1;
        if((i>545-50)&&(i<563-50)&&(urg_data->data[i]<1000))//左前障害物
          lobslf=1;
        if((i>710)&&(i<725)&&(urg_data->data[i]<600))//左後ろ障害物
          obslb=1;
        if((i>631)&&(i<649)&&(urg_data->data[i]<600))//左障害物
          obsld=1;
          if((i>281)&&(i<427)&&(urg_data->data[i]<2000)){//前方障害物
              if(maxobs<=urg_data->data[i]){//前方の中で最も距離のある物体を導出
            obslf=1;
            maxobs=urg_data->data[i];
                rad = (2 * M_PI / urg_param.step_resolution) * (i + urg_param.step_min - urg_param.step_front);
                xp = urg_data->data[i] * cos(rad)/1000.0;
                yp = urg_data->data[i] * sin(rad) / 1000.0;
              }
          }
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

         count++;
    }


      }
printf("step:%d front:%d left:%d right:%d  lf:%d lb:%d rf:%d rb:%d\n",step,obs,obsl,obsr,obslf,obslb,obsrf,obsrb);
      xmin=0;
      ymin=0;
      ymax=0;
      

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
    

            for(i=0; i < urg_data->size; ++i) //URGから取得したすべての点について行う
            {
              if((i>281)&&(i<427)&&(urg_data->data[i]<1200)){
                double rad = ((double)urg_param.step_min + i - urg_param.step_front) * M_PI * 2 / urg_param.step_resolution; //今のステップの角度を計算
                
                pcl::PointXYZ p;  //１つの点pを定義
                p.x = (double)urg_data->data[i] * sin(rad) / 1000; //点pのX座標をURGのデータから計算(1000で割ってミリ表記からメートル表記に直す)
                p.y = (double)urg_data->data[i] * cos(rad) / 1000; //点pのY座標をURGのデータから計算(1000で割ってミリ表記からメートル表記に直す)
                p.z = 0.0; //点pのZ座標を計算(常に0としておく)

                cloud2->points.push_back( p ); //作成した点pをcloudに追加する
                }
            }

        S2Sdd_End(&urg_buff); //バッファのアンロック(読み込み終了)
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (5);
        ec.setMaxClusterSize (25000);
        //ec.setSearchMethod (tree);
        ec.setInputCloud( cloud2);
        ec.extract (cluster_indices);

        int j = 0,cou=0;
        float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud2, *cloud_cluster);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
          cloud_cluster->points[*pit].r = colors[j%6][0];
          cloud_cluster->points[*pit].g = colors[j%6][1];
          cloud_cluster->points[*pit].b = colors[j%6][2];
          cou++;
            }
            std::cout << " point count: " << cou << " data points." << std::endl;
            std::cout << "PointCloud representing the Cluster: " << j << " data points." << std::endl;
            std::stringstream ss;
            //ss << "cloud_cluster_" << j << ".pcd";
            //writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
            j++;
        }
        viewer.showCloud (cloud_cluster); 

      
      switch(step){
        case 0://壁に角度合わせる
            th=0;
            while(loopf==0){
              ret = S2Sdd_Begin(&urg_buff, &urg_data);
              if(ret > 0) {
                maxobs=0;
                xp=0;
                Spur_get_pos_GL(&x,&y,&th);
                for(i = 0; i < urg_data->size; ++i) {
                  if((i>=325)&&(i<=327)&&(urg_data->data[i]<2000)&&(urg_data->data[i]>1700)){//前方障害物
                        if(maxobs<=urg_data->data[i]){//前方の中で最も距離のある物体を導出
                    obslf=1;                
                    maxobs=urg_data->data[i];
                        rad = (2 * M_PI / urg_param.step_resolution) * (i + urg_param.step_min - urg_param.step_front);
                        xp = urg_data->data[i] * cos(rad)/1000.0;
                        yp = urg_data->data[i] * sin(rad) / 1000.0;
                        }
                    }
                  }

            if(th>M_PI/2){
              th-=M_PI/32;
              Spur_spin_FS(-M_PI/32);
            }
            else if(th<-M_PI/2)
              th+=M_PI/32;
              Spur_spin_FS(M_PI/32);
            if(xp!=0){
              loopf=1;
            }
            
            printf("xp:%f th:%f\n",xp,th);
            usleep(100000);
          }else{
            puts("error");
          }
          S2Sdd_End(&urg_buff);
            }
            step++;
            printf("xp:%f th:%f\n",xp,th);
            S2Sdd_End(&urg_buff);
            loopf=0;
            //Spur_stop();
            break;

        case 1: //壁まで直進
                //Spur_get_pos_FS(&x,&y,&th);
                Spur_stop_line_FS( xp-0.4, 0, 0 );//前方にある最も距離のある物体xpより45cm手前まで移動
                //Spur_orient_FS(th);
                //while( !Spur_over_line_FS( xp-0.6, 0, 0 ) )
                    usleep( 5000000 );
                step++;
                
                break;

        case 2: //90度回転
            if(flag1==0){//初回に一度だけ実行
                  //前方の物体から回帰直線作成//
                  for (i=0; i<count; i++) {
                    sum_xy += yob[i] * xob[i];
                    sum_x += yob[i];
                    sum_y += xob[i];
                    sum_x2 += pow(yob[i], 2);
                  } 
      
                  a = (count * sum_xy - sum_x * sum_y) / (count * sum_x2 - pow(sum_x, 2));//傾き
                  b = (sum_x2 * sum_y - sum_xy * sum_x) / (count * sum_x2 - pow(sum_x, 2));//切片
                  printf("atan:%f[rad] %f[deg] \n",atan(b/(-b/a)),atan(b/(-b/a))*180/3.1415);
                  flag1=1;
                }
                Spur_set_vel(0.1);
                Spur_spin_FS(-atan(b/(-b/a))+(-M_PI/2));//
                usleep(2000000);
                Spur_set_vel(0.3);
                step++;
                flag1=0;
                break;

        case 3://右壁最後まで直進
          while(loopf==0){
            ret = S2Sdd_Begin(&urg_buff, &urg_data);
            xp=0;
            if(ret > 0) {
              maxobs=0;
              xp=0;
              Spur_get_pos_GL(&x,&y,&th);
              for(i = 0; i < urg_data->size; ++i) {
                if((i>=325)&&(i<=327)&&(urg_data->data[i]<2500)&&(urg_data->data[i]>2000)){//前方障害物
                      if(maxobs<=urg_data->data[i]){//前方の中で最も距離のある物体を導出
                    obslf=1;                
                    maxobs=urg_data->data[i];
                        rad = (2 * M_PI / urg_param.step_resolution) * (i + urg_param.step_min - urg_param.step_front);
                        xp = urg_data->data[i] * cos(rad)/1000.0;
                        yp = urg_data->data[i] * sin(rad) / 1000.0;
                      }
                  }
                }
          if(xp!=0){
            loopf=1;
            
          }else{
            Spur_stop();
          }
          printf("xp:%f th:%f\n",xp,th);
          usleep(100000);
          }else{
            Spur_stop();
            puts("error");
        }
        S2Sdd_End(&urg_buff);
        }
                loopf=0;
                step++;
                printf("xp:%f th:%f\n",xp,th);
                S2Sdd_End(&urg_buff);
                break;
         case 4://前方やまびこ検知から見失うまで待機
                Spur_stop_line_FS( xp+1.8, 0, 0 );
        usleep(8000000);
        step++;
                break;
        case 5: //やまびこ位置からちょっと前まで移動
              Spur_spin_FS(-M_PI/2);//
              usleep(2000000);
              step++;  
                                break;
        case 6://45度回転して探索、もとの角度に
                
                break; 
        case 7://探索エリアに半分進んで90度回転、探索、戻る
                break;
        case 8://探索エリア端まで行って135度回転、探索、元の角度に
                break;
        case 9:           
            break;     
        default:Spur_stop();usleep(100000); break;
      }


      /* unlock buffer */
      //S2Sdd_End(&urg_buff);


usleep(90000);
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